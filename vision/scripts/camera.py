#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
    def run(self):
        cap = cv2.VideoCapture('/dev/video4')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

        if not cap.isOpened():
            self.get_logger().error('Error: Unable to open camera')
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error('Error: Unable to capture frame')
                break

            # Process frame here if needed
            # For example, you could publish the frame as an image message

            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    camera_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

