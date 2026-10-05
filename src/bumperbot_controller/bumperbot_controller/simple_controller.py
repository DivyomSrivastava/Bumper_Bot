#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter(
            "wheel_radius"
        ).get_parameter_value().double_value

        self.wheel_separation_ = self.get_parameter(
            "wheel_separation"
        ).get_parameter_value().double_value

        self.get_logger().info(
            "Using wheel radius %.3f" % self.wheel_radius_
        )

        self.get_logger().info(
            "Using wheel separation %.3f" % self.wheel_separation_
        )

        # Previous wheel positions
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0

        # Robot pose
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Publishers
        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray,
            "simple_velocity_controller/commands",
            10
        )

        self.odom_pub_ = self.create_publisher(
            Odometry,
            "bumperbot_controller/odom",
            10
        )

        # Subscribers
        self.vel_sub_ = self.create_subscription(
            TwistStamped,
            "bumperbot_controller/cmd_vel",
            self.velCallback,
            10
        )

        self.joint_sub_ = self.create_subscription(
            JointState,
            "joint_states",
            self.jointCallback,
            10
        )

        # Differential drive conversion matrix
        self.speed_conversion_ = np.array([
            [
                self.wheel_radius_ / 2,
                self.wheel_radius_ / 2
            ],
            [
                self.wheel_radius_ / self.wheel_separation_,
                -self.wheel_radius_ / self.wheel_separation_
            ]
        ])

        self.get_logger().info(
            "The conversion matrix is %s" % self.speed_conversion_
        )

        # Odometry message
        self.odom_msg_ = Odometry()

        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"

        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # TF broadcaster
        self.br_ = TransformBroadcaster(self)

        self.transform_stamped_ = TransformStamped()

        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        # We don't calculate dt from the node startup time.
        # Instead, the first JointState message initializes the timestamp.
        self.prev_time_ = None

    def velCallback(self, msg):

        # Differential-drive kinematics
        robot_speed = np.array([
            [msg.twist.linear.x],
            [msg.twist.angular.z]
        ])

        wheel_speed = np.matmul(
            np.linalg.inv(self.speed_conversion_),
            robot_speed
        )

        wheel_speed_msg = Float64MultiArray()

        wheel_speed_msg.data = [
            wheel_speed[1, 0],
            wheel_speed[0, 0]
        ]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def jointCallback(self, msg):

        # Make sure we received both wheel positions
        if len(msg.position) < 2:
            self.get_logger().warn(
                "JointState does not contain two wheel positions"
            )
            return

        current_time = Time.from_msg(msg.header.stamp)

        # First JointState message:
        # initialize previous values and return.
        if self.prev_time_ is None:

            self.left_wheel_prev_pos_ = msg.position[0]
            self.right_wheel_prev_pos_ = msg.position[1]
            self.prev_time_ = current_time

            return

        # Calculate time difference
        dt = current_time - self.prev_time_

        dt_seconds = dt.nanoseconds / S_TO_NS

        # Protect against zero or negative time differences.
        if dt_seconds <= 0.0:
            return

        # Wheel position changes
        dp_left = (
            msg.position[0] -
            self.left_wheel_prev_pos_
        )

        dp_right = (
            msg.position[1] -
            self.right_wheel_prev_pos_
        )

        # Update previous values
        self.left_wheel_prev_pos_ = msg.position[0]
        self.right_wheel_prev_pos_ = msg.position[1]
        self.prev_time_ = current_time

        # Wheel angular velocities
        fi_left = dp_left / dt_seconds
        fi_right = dp_right / dt_seconds

        # Robot linear and angular velocity
        linear = (
            self.wheel_radius_ * fi_right +
            self.wheel_radius_ * fi_left
        ) / 2.0

        angular = (
            self.wheel_radius_ * fi_right -
            self.wheel_radius_ * fi_left
        ) / self.wheel_separation_

        # Position increment
        d_s = (
            self.wheel_radius_ * dp_right +
            self.wheel_radius_ * dp_left
        ) / 2.0

        d_theta = (
            self.wheel_radius_ * dp_right -
            self.wheel_radius_ * dp_left
        ) / self.wheel_separation_

        # Update robot pose
        self.theta_ += d_theta

        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        self.get_logger().info(
        "x: %f" % self.x_
        )

        self.get_logger().info(
        "y: %f" % self.y_
        )

        self.get_logger().info(
        "theta: %f" % self.theta_
        )   

        self.get_logger().info(
        "linear: %f angular: %f" % (linear, angular)
        )

        # Convert yaw to quaternion
        q = quaternion_from_euler(
            0,
            0,
            self.theta_
        )

        # Publish odometry
        self.odom_msg_.header.stamp = (
            self.get_clock().now().to_msg()
        )

        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_

        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.odom_pub_.publish(self.odom_msg_)

        # Publish TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_

        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]

        self.transform_stamped_.header.stamp = (
            self.get_clock().now().to_msg()
        )

        self.br_.sendTransform(
            self.transform_stamped_
        )


def main():

    rclpy.init()

    simple_controller = SimpleController()

    rclpy.spin(simple_controller)

    simple_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()