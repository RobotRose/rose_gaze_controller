#!/usr/bin/python

import roslib; roslib.load_manifest("rose20_gaze_controller")

import rospy
import actionlib

import rose20_gaze_controller.msg
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from rose20_gaze_controller.msg import LookAtAction, LookAtFeedback, LookAtResult

import tf
from tf.transformations import euler_from_quaternion

import visualization_msgs.msg as vis

import geometry_msgs.msg as gm

from math import sqrt, atan2, asin, atan, pi
import copy

point, rot = 0,1
X,Y,Z = 0,1,2

def clamp(minimum, x, maximum):
    """Clip/clamp x between the given minimum and maximum.
    @returns the clamped value and a Bool indication whether a clamping was applied"""
    clamped = max(minimum, min(x, maximum))
    return clamped, x != clamped

class GazeController(object):
    """Control the gaze of Rose by specifying a point the head should look at"""
    def __init__(self, name, rate=10, pan_minmax=(-pi*0.5, pi*0.5), tilt_minmax=(-pi*0.5, pi*0.5)):
        self._action_name = name
        self._action_server = actionlib.SimpleActionServer(self._action_name, LookAtAction, 
            execute_cb=self.execute_look_at_action_cb,
            auto_start=False)
        self._action_server.register_preempt_callback(self.cancel_cb)

        self.neck_pan_publisher = rospy.Publisher("/neck_pan_controller/command", Float64)
        self.neck_tilt_publisher = rospy.Publisher("/neck_tilt_controller/command", Float64)

        self.marker_publisher = rospy.Publisher("/visualization_markers", vis.Marker)

        self.neck_pan_subscriber = rospy.Subscriber("/neck_pan_controller/state", JointState, self.update_pan)
        self.neck_tilt_subscriber = rospy.Subscriber("/neck_tilt_controller/state", JointState, self.update_tilt)

        self.base_frame = "/base_link"
        self.pan_joint_mounting_frame = "/neck_mount"
        self.tilt_joint_mounting_frame = "/neck_tilt_mount"
        self.camera_frame = "/camera_link"

        self.tf_listener = tf.TransformListener()

        self.is_tracking = False
        self.rate = rospy.Rate(rate)

        self.pan_minmax, self.tilt_minmax = pan_minmax, tilt_minmax
    
        self._action_server.start()

        self.current_pan = 0
        self.current_tilt = 0

    def update_pan(self, msg):
        self.current_pan = msg.current_pos

    def update_tilt(self, msg):
        self.current_tilt = msg.current_pos

    def _calculate_neck_angles(self, target_pointstamped):
        """Calculate angles for the neck pan&tilt so that the Kinect looks at the given target_pointstamped. 
        'Looking at' means that the target_pointstamped is on the camera_link's Z axis."""
        try:
            pan, tilt = 0,0
            
            self.tf_listener.waitForTransform(target_pointstamped.header.frame_id, self.pan_joint_mounting_frame, rospy.Time.now(), rospy.Duration.from_sec(5))
            target_in_pan_joint  = self.tf_listener.transformPoint(self.pan_joint_mounting_frame, target_pointstamped)

            self.tf_listener.waitForTransform(self.tilt_joint_mounting_frame, target_pointstamped.header.frame_id, rospy.Time.now(), rospy.Duration.from_sec(5))
            target_in_tilt_joint = self.tf_listener.transformPoint(self.tilt_joint_mounting_frame, target_pointstamped)
            
            pan_to_tilt_tf      = self.tf_listener.lookupTransform(self.pan_joint_mounting_frame, self.tilt_joint_mounting_frame, rospy.Time(0))
            tilt_to_camera_tf   = self.tf_listener.lookupTransform(self.tilt_joint_mounting_frame, self.camera_frame, rospy.Time(0))

            pan_to_tilt_horizontal = pan_to_tilt_tf[point][X];
            tilt_to_cam_z = tilt_to_camera_tf[point][Z];

            pan = atan2(target_in_pan_joint.point.y, target_in_pan_joint.point.x)

            #act like the pan is already aligned, i.e. rotate the target_pointstampd as if it were on x=0
            #This means rotating it around th Z axis until Y' is 0, but X' = sqrt(X^2 + Y^2)
            rotated_target = copy.deepcopy(target_pointstamped)
            rotated_target.point.y = 0
            rotated_target.point.x = sqrt(target_pointstamped.point.x**2 + target_pointstamped.point.y**2)
            rotated_target.point.x = rotated_target.point.x * -1 if target_pointstamped.point.x < 0 else rotated_target.point.x #Set the correct sign
            rotated_target_in_tilt_joint = self.tf_listener.transformPoint(self.tilt_joint_mounting_frame, rotated_target)

            target_to_tilt_distance_horizontal = sqrt(rotated_target_in_tilt_joint.point.x**2 + rotated_target_in_tilt_joint.point.y**2)
            tilt = atan2(rotated_target_in_tilt_joint.point.z, target_to_tilt_distance_horizontal) #TODO Loy: Correct for distance between tilt joint and camera link 

            #rospy.loginfo("tilt_error = {0}".format(tilt_error))

            rospy.logdebug("New pan,tilt = {0:.3f}, {1:.3f}".format(pan, tilt))
            #import ipdb;ipdb.set_trace()
            return pan, tilt

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, Exception) as e:
            rospy.logerr("Could not find a required transform: {0}".format(e))
        return None, None

    def _send_angles(self, request):
        angles = self._calculate_neck_angles(request.target_point)
        if angles != (None, None):
            clamped_pan, pan_is_clamped = clamp(self.pan_minmax[0], angles[0], self.pan_minmax[1])
            clamped_tilt, tilt_is_clamped = clamp(self.tilt_minmax[0], angles[1], self.tilt_minmax[1])

            pan_error = clamped_pan - self.current_pan
            tilt_error = clamped_tilt - self.current_tilt

            self.neck_pan_publisher.publish(clamped_pan)
            self.neck_tilt_publisher.publish(clamped_tilt)
            
            if pan_is_clamped or tilt_is_clamped:
                rospy.logwarn("Target_point is out of range (angles would be ({0}, {1})), making a best effort up to the min and max range (pan: {2} - {3}), (tilt: {4} - {5})"
                    .format(angles[0], angles[1], self.pan_minmax[0], self.pan_minmax[1], self.tilt_minmax[0], self.tilt_minmax[1]))
                return False, (pan_error, tilt_error)
            else:
                return True, (pan_error, tilt_error)
        else:
            rospy.logwarn("No angles could be determined, so not commanding different neck pose")
            return False, (None, None)

    def execute_look_at_action_cb(self, request):
        rospy.loginfo("New target: ({0.point.x}, {0.point.y}, {0.point.z}) in {0.header.frame_id}. Keep tracking: {1}".format(request.target_point, request.keep_tracking))

        self.is_tracking = request.keep_tracking

        if request.target_point.header.frame_id == "":
            request.target_point.header.frame_id = "/map"

        if request.keep_tracking:
            while self.is_tracking and not rospy.is_shutdown():
                self.publish_target_marker(request.target_point)
                self.publish_head_arrow()
                
                result, errors = self._send_angles(request)
                
                self._action_server.publish_feedback(LookAtFeedback(is_tracking=result, pan_error=errors[0], tilt_error=errors[1]))
                self.rate.sleep()
            else: #Do this when the loop ends natually, ie the condition becomes false
                if result:
                    self._action_server.set_succeeded(LookAtResult(result))
                else:
                    self._action_server.set_aborted(LookAtResult(result))
        else:
            self.publish_target_marker(request.target_point)
            self.publish_head_arrow()
            
            result, errors = self._send_angles(request)
            
            if result:
                self._action_server.set_succeeded(LookAtResult(result))
            else:
                self._action_server.set_aborted(LookAtResult(result))

    def cancel_cb(self, request=None):
        self.is_tracking = False
        # self._action_server.set_cancelled(LookAtResult(self.is_tracking))

    def publish_target_marker(self, target):
        marker = vis.Marker()
        marker.ns = "gaze_target";
        marker.id = 0
        marker.header.frame_id = target.header.frame_id
        marker.pose.position = target.point

        #import ipdb;ipdb.set_trace()
        marker.type = vis.Marker.SPHERE
        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()

        self.marker_publisher.publish(marker)

    def publish_head_arrow(self):
        marker = vis.Marker()
        marker.ns = "gaze_orientation";
        marker.id = 0
        marker.header.frame_id = self.camera_frame
        marker.pose.position = gm.Point(0,0,0)

        #import ipdb;ipdb.set_trace()
        marker.type = vis.Marker.ARROW
        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 5.0
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.frame_locked = True # update every time step

        marker.lifetime = rospy.Duration()

        self.marker_publisher.publish(marker)


if __name__ == '__main__':
    rospy.init_node('gaze_controller')

    pan_minmax = rospy.get_param('~min_pan'), rospy.get_param('~max_pan')
    tilt_minmax = rospy.get_param('~min_tilt'), rospy.get_param('~max_tilt')
    rate = rospy.get_param('~rate')

    gazecontroller = GazeController(rospy.get_name(), rate, pan_minmax, tilt_minmax)
    rospy.spin()
