#!/usr/bin/env python3
# import needed items
import numpy as np
import cv2
#import imutils
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist


class WheelDriver:
    def __init__(self):
        self.cmd_input = Twist()
        self.obj_angle = 180
        self.obj_dist = -1
        self.accumulated_dist_error = 0
        self.accumulated_angular_error = 0
        self.sampling_time = 0.1
        self.required_dist = 0.8
        self.max_angular_vel = 0.3
        self.max_linear_vel = 0.15
        self.kp_ang = 0.02
        self.ki_ang = 0.00
        self.kp_linear = 0.1
        self.ki_linear = 0.00
        self.number_subscriber = rospy.Subscriber(
            "/center_pt", geometry_msgs.msg.Point, self.callback_center_received)
        self.input_publisher = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=1)

    def callback_center_received(self, msg):
        # print("in drive wheel callback")
        self.obj_angle = msg.x
        self.obj_dist = msg.z
        self.cmd_input.angular.x = 0
        self.cmd_input.angular.y = 0
        self.cmd_input.linear.x = 0
        self.cmd_input.linear.y = 0
        self.cmd_input.linear.z = 0

        # print("received obj data")
        self.angular_error = 0
        self.distance_error = 0

        if self.obj_angle != 180:
            self.angular_error = -self.obj_angle
            self.distance_error = self.obj_dist - self.required_dist

            self.compute_inputs()
            if abs(self.angular_error) > 5:
                self.cmd_input.angular.z = self.commanded_angular_vel
            else:
                self.cmd_input.angular.z = 0
            if abs(self.distance_error) > 0.05:
                self.cmd_input.linear.x = self.commanded_linear_vel
            else:
                self.cmd_input.linear.x = 0
        else:
            self.cmd_input.angular.z = 0
            self.cmd_input.linear.x = 0
        if abs(self.cmd_input.angular.z) > self.max_angular_vel:
            self.cmd_input.angular.z = self.cmd_input.angular.z * \
                self.max_angular_vel/abs(self.cmd_input.angular.z)
        if abs(self.cmd_input.linear.x) > self.max_linear_vel:
            self.cmd_input.linear.x = self.cmd_input.linear.x * \
                self.max_linear_vel/abs(self.cmd_input.linear.x)

        if self.obj_angle != 180:
            print("distance = ", self.obj_dist)
            print("distance error", self.distance_error)
            print("linear input = ", self.cmd_input.linear.x)
            print("angle = ", self.obj_angle)
            print("angle error", self.angular_error)
            print("angular input = ", self.cmd_input.angular.z)
        self.input_publisher.publish(self.cmd_input)

    def compute_inputs(self):
        """  if abs(self.accumulated_angular_error) > 100:
             self.accumulated_angular_error = 0
         if abs(self.accumulated_dist_error) > 100:
             self.accumulated_dist_error = 0

         self.accumulated_angular_error = self.accumulated_angular_error + \
             self.angular_error*self.sampling_time
         self.accumulated_dist_error = self.accumulated_dist_error + \
             self.distance_error*self.sampling_time
         self.commanded_angular_vel = self.kp_ang*self.angular_error + self.ki_ang * \
             self.accumulated_angular_error
         self.commanded_linear_vel = self.kp_linear*self.distance_error + self.ki_linear * \
             self.accumulated_dist_error """
        self.commanded_angular_vel = self.kp_ang*self.angular_error
        self.commanded_linear_vel = self.kp_linear*self.distance_error


if __name__ == '__main__':
    rospy.init_node('wheel_driver')
    WheelDriver()
    rospy.spin()
