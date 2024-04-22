#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import glob

class DistanceNode(Node):
    def __init__(self):
        super().__init__('distance_node')

        # load camera calibration matrix
        self.calibration_matrix, self.dist_coeffs = self.calibrate_camera('/home/moody/f1tenth_ws/src/lab8_pkg/calibration', (.06, .08), .25)
        print("Calibration Matrix: \n", self.calibration_matrix)

        # initialize CvBridge
        self.bridge = CvBridge()

    def calibrate_camera(self, calibration_folder, chessboard_size, square_size):
        # prepare object points
        objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

        # arrays to store object points and image points from all the images
        objpoints = []  # 3D points in real world space
        imgpoints = []  # 2D points in image plane

        # get the list of calibration images
        calibration_images = glob.glob(calibration_folder + '/*.png')

        # loop through calibration images
        for image_path in calibration_images:
            # read each image
            img = cv2.imread(image_path)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # find chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

            # add object points and image points (after refining them)
            if ret:
                objpoints.append(objp)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                # draw and display the corners
                img = cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
                cv2.imwrite('/home/moody/f1tenth_ws/src/lab8_pkg/output_image.png', img)

        # perform camera calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        return mtx, dist


    def calculate_camera_height(self, u, v, x_car):
        """
        Calculate the camera mounting height.
        
        Parameters:
            u (float): The u-coordinate (horizontal pixel) of the object in the image.
            v (float): The v-coordinate (vertical pixel) of the object in the image.
            x_car (float): The distance of the object from the camera along the x-axis in the car frame (in cm).
        
        Returns:
            float: The height of the camera above the car frame in cm.
        """

        # Convert x_car from cm to the same units used in the calibration (assuming meters here)
        X = x_car / 100
        
        # Invert the intrinsic matrix to transform from pixel coordinates to camera coordinates
        K_inv = np.linalg.inv(self.calibration_matrix)
        
        # Transform image coordinates to normalized camera coordinates
        pixel_coords = np.array([u, v, 1])
        camera_coords = K_inv @ pixel_coords
        
        # We know X and assume Y = 0, solve for Z (H) using similar triangles:
        # (X/Z) = (camera_coords[0] / camera_coords[2])
        Z = X / camera_coords[0] * camera_coords[2]
        
        # Convert Z from meters to cm (if your calibration was in meters)
        return Z * 100  # converting back to cm for consistency with x_car


    def calculate_distance_to_cone(self, cone_image):
        # read the cone image
        img = cv2.imread(cone_image)

        # set lower right corner pixel points
        reference_point = [598, 418]

        # Calculate distance in x_car and y_car coordinates
        x_car = reference_point[0] * (self.mount_height / self.calibration_matrix[0, 0])
        y_car = reference_point[1] * (self.mount_height / self.calibration_matrix[1, 1])

        # Draw reference point for visualization
        cv2.circle(img, tuple(reference_point), 5, (0, 0, 255), -1)

        # Display visualizations
        cv2.imwrite('/home/moody/f1tenth_ws/src/lab8_pkg/cone_image.png', img)

        return x_car, y_car


    def main(self):
        # Calibration parameters
        chessboard_size = (6, 8)  # Chessboard size (inner corners)
        square_size = 25  # Size of each square in mm

        self.calibration_matrix = np.array([[606, 0, 322],
                                            [0, 605, 239],
                                            [0, 0, 1]])

        # Calculate mount height
        x_car = 40  # Known x_car distance of the cone in cm
        self.mount_height = self.calculate_camera_height(664, 494, x_car)
        print("Camera Mount Height:", self.mount_height, "cm")

        # Calculate distance to cone
        cone_image = '/home/moody/f1tenth_ws/src/lab8_pkg/resource/cone_unknown.png'
        x_car, y_car = self.calculate_distance_to_cone(cone_image)

        if x_car is not None and y_car is not None:
            print("Distance to cone (x_car, y_car):", x_car, "cm,", y_car, "cm")
        else:
            print("Error: Distance calculation failed.")


def main(args=None):
    rclpy.init(args=args)
    distance_node = DistanceNode()
    distance_node.main()
    rclpy.spin(distance_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()