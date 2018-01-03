#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist

# import some utils.
import numpy as np
import copy as copy

class InterpolateThrottle:
    def __init__(self):

        # Allow our topics to be dynamic.
        self.motor_input_topic   = rospy.get_param('~motor_input_topic', '/low_level/twist_cmd_mux/output')
        self.motor_output_topic  = rospy.get_param('~motor_output_topic', '/cmd_vel')

        self.max_acceleration = rospy.get_param('/motor/max_acceleration')
        self.max_speed = rospy.get_param('/motor/speed_max')
        self.min_speed = rospy.get_param('/motor/speed_min')
        self.throttle_smoother_rate = rospy.get_param('/motor/throttle_smoother_rate')

        self.max_turning_speed = rospy.get_param('/motor/max_turning_speed')
        self.max_turning = rospy.get_param('/motor/turning_max')
        self.min_turning = rospy.get_param('/motor/turning_min')

	self.refresh_rate = rospy.get_param('/motor/refresh_rate')

        # Variables
        self.last_speed = 0
        self.desired_speed = self.last_speed

        self.last_turning = 0
        self.desired_turning = self.last_turning

        # Create topic subscribers and publishers
        self.motor_output = rospy.Publisher(self.motor_output_topic, Twist, queue_size=1)

        rospy.Subscriber(self.motor_input_topic, Twist, self._process_input_command)

        self.max_delta_turning = abs(self.refresh_rate * self.max_turning_speed / self.throttle_smoother_rate)
        self.max_delta_speed = abs(self.refresh_rate * self.max_acceleration / self.throttle_smoother_rate)
	rospy.Timer(rospy.Duration(1.0/self.max_delta_speed), self._publish_drive_command)

        # run the node
        self._run()

        # Keep the node alive
    def _run(self):
        rospy.spin()

    def _publish_drive_command(self, evt):
	# for linear speed
        desired_delta_speed = self.desired_speed-self.last_speed
        clipped_delta_speed = max(min(desired_delta_speed, self.max_delta_speed), -self.max_delta_speed)
        smoothed_speed = self.last_speed + clipped_delta_speed
        self.last_speed = smoothed_speed
	# For turning speed
        desired_delta_turning = self.desired_turning-self.last_turning
        clipped_delta_turning = max(min(desired_delta_turning, self.max_delta_turning), -self.max_delta_turning)
        smoothed_turning = self.last_turning + clipped_delta_turning
        self.last_turning = smoothed_turning
        # publish the smoothed linear_speed and turning_speed
	motor_msg = Twist()
	motor_msg.linear.x = smoothed_speed
	motor_msg.linear.y = 0
	motor_msg.linear.z = 0
	motor_msg.angular.x = 0
	motor_msg.angular.y = 0
	motor_msg.angular.z = smoothed_turning
	self.motor_output.publish(motor_msg)

    def _process_input_command(self,msg):
        input_speed = msg.linear.x
	input_turning = msg.angular.z
        # Do some sanity clipping
        input_speed = min(max(input_speed, self.min_speed), self.max_speed)
	input_turning = min(max(input_turning, self.min_turning), self.max_turning)
	# Set the target speed and turning
        self.desired_speed = input_speed
	self.desired_turning = input_turning


# Boilerplate node spin up. 
if __name__ == '__main__':
    try:
        rospy.init_node('Throttle_Interpolator')
        p = InterpolateThrottle()
    except rospy.ROSInterruptException:
        pass

