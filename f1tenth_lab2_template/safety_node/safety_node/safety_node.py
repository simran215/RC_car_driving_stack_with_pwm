#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
# TODO: include needed ROS msg type headers and libraries
from std_msgs.msg import Header,Bool
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.
        

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10)
        self.speed = 0.0
        self.ttc_threshold = 0.5
        # TODO: create ROS subscribers and publishers.
        self.brake_publisher =self.create_publisher(AckermannDriveStamped, 'drive', 10)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
        msg = String()
        msg.data = 'Current_speed: %f' % self.speed
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        angle = scan_msg.angle_min
        brake = False
        for i in scan_msg.ranges:
            if i != math.inf and i != math.nan:
                v_i = self.speed * math.cos(angle)
                if v_i > 0:
                    ttc = i / v_i
                    if ttc < self.ttc_threshold:
                        brake = True
                        brake_msg = AckermannDriveStamped()
                        header = Header()
                        header.stamp = self.get_clock().now().to_msg()#rospy.Time.now()
                        brake_msg.header = header
                        brake_msg.drive.speed = 0.0
                        self.brake_publisher.publish(brake_msg)
                        break
            angle += scan_msg.angle_increment
        # TODO: publish command to brake
        #pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
