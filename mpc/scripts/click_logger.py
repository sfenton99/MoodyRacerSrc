#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import csv
import os


def delete_file_if_exists(file_path):
    if os.path.exists(file_path):
        os.remove(file_path)
        print(f"File '{file_path}' has been deleted.")
    else:
        print(f"File '{file_path}' does not exist.")

class PointSaver(Node):
    def __init__(self):
        super().__init__('point_saver')
        self.subscription = self.create_subscription(
            PointStamped,
            'clicked_point',
            self.clicked_point_callback,
            10)
        self.log_dir = '~/sim_ws/src/logs/'
        delete_file_if_exists(self.log_dir+'waypoints_clicked.csv')

        # publish clicked points as a marker array
        self.marker_array_pub = self.create_publisher(
            MarkerArray,
            '/clicked_history',
            10)
        self.marker_history = []


    def clicked_point_callback(self, msg):
        point = msg.point
        with open(self.log_dir+'waypoints_clicked.csv', mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([point.x, point.y, 0.00000, 0.00000])
        self.get_logger().info('Point saved: {}, {}'.format(point.x, point.y))

        # publish the clicked point
        # Create a new marker
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = len(self.marker_history)  # Unique ID based on history size
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.3
        marker.color.r = 0.2
        marker.color.g = 0.0
        marker.color.b = 0.3

        # Add the marker to the history
        self.marker_history.append(marker)

        # Publish the entire history
        marker_array = MarkerArray(markers=self.marker_history)

        self.marker_array_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    point_saver = PointSaver()
    rclpy.spin(point_saver)
    point_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
