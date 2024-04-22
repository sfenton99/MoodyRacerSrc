#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from os.path import expanduser
from numpy import linalg as LA
import atexit
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


file = open('/home/moody/f1tenth_ws/waypoints/waypoints.csv', 'w+')

class LogWaypoints(Node):

    def __init__(self):
        super().__init__('waypoint_logger')
        print('------- BEGINING LOGGER ------')
        self.subscription = self.create_subscription(
            Odometry,
            '/pf/pose/odom',
            self.save_waypoint,
            10)
        self.subscription  # prevent unused variable warning

    def save_waypoint(self, data):
        quaternion = np.array([data.pose.pose.orientation.x, 
                            data.pose.pose.orientation.y, 
                            data.pose.pose.orientation.z, 
                            data.pose.pose.orientation.w])

        euler = euler_from_quaternion(quaternion)

        velocity = data.twist.twist.linear.x

        file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                        data.pose.pose.position.y,
                                        euler[2],
                                        velocity))
        print('writting....')

def shutdown():
    file.close()
    print('Goodbye')

def main():
    rclpy.init(args=None)
    waypoint_logger = LogWaypoints()
    atexit.register(shutdown)
    rclpy.spin(waypoint_logger)

    waypoint_logger.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
    
