import math
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.lidar_sub=self.create_subscription(LaserScan,lidarscan_topic, self.scan_callback,10)
        self.drive_pub=self.create_publisher(AckermannDriveStamped,drive_topic,10)
        # TODO: set PID gains
        self.kp = -1 # positive w.r.t left wall, negative w.r.t right wall
        self.kd = 0.1
        self.ki = 0.001

        # TODO: store history
        self.integral = 0
        self.prev_error = 0
        self.error = 0

        # TODO: store any necessary values you think you'll need
        self.desired_distance=1
        self.car_length=0.5

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        distance_index = int(math.radians(angle)/range_data.angle_increment)
        distance = range_data.ranges[distance_index]
        return distance
        

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        a=self.get_range(range_data,90)
        b=self.get_range(range_data,45)
        alpha=math.atan((a*math.cos(45) - b)/(a*math.sin(45)))
        d_t=b*math.cos(alpha)
        d_t_plus=d_t+math.sin(alpha)*2*self.car_length
        msg = String()
        msg.data = 'Distance values: %f' % d_t_plus
        self.get_logger().info('"%s"' % msg.data)

        return dist-d_t_plus

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        diff=(error-self.prev_error)/0.1
        angle = self.kp*error+self.kd*diff
        self.prev_error=error
        if 0 <= math.degrees(abs(angle)) <= 10:
            velocity = 1.5
        elif 10 < math.degrees(abs(angle)) <= 20:
            velocity = 1.0
        else:
            velocity = 0.5
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        # TODO: fill in drive message and publish

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg,self.desired_distance) # TODO: replace with error calculated by get_error()
        # velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()