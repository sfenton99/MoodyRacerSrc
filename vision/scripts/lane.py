#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

    def detect_lanes(self, image_path):
        cv_image = cv2.imread(image_path)

        yellow_mask, hsv_image = self.segment_yellow(cv_image)

        # Find contours on the yellow mask
        contours, _ = cv2.findContours(
            yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw red lines along the contours
        cv2.drawContours(cv_image, contours, -1, (0, 0, 255), 2)

        # Save the result
        cv2.imwrite(
            '/root/sim_ws/src/lab8_pkg/lane_detection_result.png', cv_image)

        # Save the intermediate masks
        cv2.imwrite('/root/sim_ws/src/lab8_pkg/yellow_mask.png', yellow_mask)
        cv2.imwrite('/root/sim_ws/src/lab8_pkg/hsv_image.png', hsv_image)


    def segment_yellow(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([18, 70, 70])
        upper_yellow = np.array([30, 255, 255])

        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        kernel = np.ones((2, 2), np.uint8)
        yellow_mask = cv2.erode(yellow_mask, kernel, iterations=1)

        return yellow_mask, hsv_image



def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetectionNode()

    image_path = '/root/sim_ws/src/lab8_pkg/resource/lane.png'

    lane_detection_node.detect_lanes(image_path)

    print('Lane detection completed!')

    lane_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
