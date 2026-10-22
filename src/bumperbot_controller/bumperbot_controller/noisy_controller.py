#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class NoisyController(Node):

    def __init__(self):
        super().__init__("noisy_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = (
            self.get_parameter("wheel_radius").value
        )
        self.wheel_separation_ = (
            self.get_parameter("wheel_separation").value
        )

        self.get_logger().info(
            f"Using wheel radius {self.wheel_radius_:.3f}"
        )
        self.get_logger().info(
            f"Using wheel separation {self.wheel_separation_:.3f}"
        )

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.prev_time_ = None

        self.joint_sub_ = self.create_subscription(
            JointState,
            "joint_states",
            self.jointCallback,
            10
        )

        self.odom_pub_ = self.create_publisher(
            Odometry,
            "bumperbot_controller/odom_noisy",
            10
        )

        self.speed_conversion_ = np.array([
            [self.wheel_radius_ / 2.0,
             self.wheel_radius_ / 2.0],
            [self.wheel_radius_ / self.wheel_separation_,
             -self.wheel_radius_ / self.wheel_separation_]
        ])

        self.get_logger().info(
            f"The conversion matrix is {self.speed_conversion_}"
        )

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self)

        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = (
            "base_footprint_noisy"
        )

    def jointCallback(self, msg):
        if len(msg.position) < 2:
            return

        current_time = Time.from_msg(msg.header.stamp)

        # First message: initialize noisy encoder readings.
        if self.prev_time_ is None:
            self.left_wheel_prev_pos_ = (
                msg.position[0] + np.random.normal(0, 0.005)
            )
            self.right_wheel_prev_pos_ = (
                msg.position[1] + np.random.normal(0, 0.005)
            )
            self.prev_time_ = current_time
            return

        dt_ns = (
            current_time.nanoseconds
            - self.prev_time_.nanoseconds
        )

        if dt_ns <= 0:
            return

        dt_seconds = dt_ns / S_TO_NS

        # Simulate noisy wheel encoder measurements.
        wheel_encoder_left = (
            msg.position[0] + np.random.normal(0, 0.005)
        )
        wheel_encoder_right = (
            msg.position[1] + np.random.normal(0, 0.005)
        )

        dp_left = (
            wheel_encoder_left - self.left_wheel_prev_pos_
        )
        dp_right = (
            wheel_encoder_right - self.right_wheel_prev_pos_
        )

        self.left_wheel_prev_pos_ = wheel_encoder_left
        self.right_wheel_prev_pos_ = wheel_encoder_right
        self.prev_time_ = current_time

        fi_left = dp_left / dt_seconds
        fi_right = dp_right / dt_seconds

        linear = (
            self.wheel_radius_ * (fi_right + fi_left) / 2.0
        )
        angular = (
            self.wheel_radius_ * (fi_right - fi_left)
            / self.wheel_separation_
        )

        d_s = (
            self.wheel_radius_ * (dp_right + dp_left) / 2.0
        )
        d_theta = (
            self.wheel_radius_ * (dp_right - dp_left)
            / self.wheel_separation_
        )

        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        q = quaternion_from_euler(0.0, 0.0, self.theta_)

        self.odom_msg_.header.stamp = current_time.to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_

        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.odom_pub_.publish(self.odom_msg_)

        self.transform_stamped_.header.stamp = (
            current_time.to_msg()
        )

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_

        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]

        self.br_.sendTransform(self.transform_stamped_)


def main(args=None):
    rclpy.init(args=args)

    noisy_controller = NoisyController()

    try:
        rclpy.spin(noisy_controller)
    finally:
        noisy_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
