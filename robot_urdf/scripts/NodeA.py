#!/usr/bin/env python3
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from geometry_msgs.msg import Twist  # Message type for robot velocity commands
from sensor_msgs.msg import Image  # Message type for image data
from ros2_aruco_interfaces.msg import ArucoMarkers  # Custom message type for ArUco marker data
from cv_bridge import CvBridge  # Library to convert ROS Image messages to OpenCV images
import cv2  # OpenCV library for computer vision tasks
import math  # Python math library
import numpy as np  # Numpy library for numerical operations

# Define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')

        # Create publisher for cmd_vel topic to send velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a publisher for the results_images topic to send processed images
        self.results_publisher_ = self.create_publisher(Image, '/results_images', 10)
        
        # Create a timer to periodically publish messages
        self.timer = self.create_timer(1, self.publish_vel)

        # Create subscriber for /aruco_markers topic to receive ArUco marker data
        self.subscriber_ = self.create_subscription(
            ArucoMarkers, 
            '/aruco_markers',
            self.marker_callback,
            10
        )

        # Create subscriber for /camera/image_raw topic to receive raw image data
        self.image_subscriber_ = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Sequence of marker IDs to detect in order
        self.marker_sequence = [11, 12, 13, 14, 15]
        self.current_marker_index = 0  # Index to track the current marker in sequence
        self.finished = False  # Flag to indicate when all markers are detected
        
        self.bridge = CvBridge()  # Bridge to convert between ROS Image messages and OpenCV images

        # Store the last received image and synchronization flag
        self.latest_image = None
        self.image_ready = False

    def publish_vel(self):
        # If finished, stop sending angular velocity (set it to 0)
        if self.finished:
            msg = Twist()
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: Angular z={msg.angular.z} (Finished)')
        else:
            # Continue publishing angular velocity until the first marker is found
            msg = Twist()
            msg.angular.z = 0.5
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: Angular z={msg.angular.z}')

    def marker_callback(self, msg):
        if self.finished or not self.image_ready:
            return

        # Look for the current marker in the sequence
        if self.marker_sequence[self.current_marker_index] in msg.marker_ids:
            marker_id = self.marker_sequence[self.current_marker_index]
            self.get_logger().info(f'Found Marker ID: {marker_id}')

            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')

            detected = False

            # Loop through supported ArUco dictionaries to detect markers
            for aruco_dict_name in ARUCO_DICT.keys():
                aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dict_name])
                aruco_params = cv2.aruco.DetectorParameters_create()

                # Detect markers
                corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

                if ids is not None and marker_id in ids:
                    self.get_logger().info(f"Detected ArUco marker {marker_id} with {aruco_dict_name}")
                    index = np.where(ids == marker_id)[0][0]
                    my_corners = corners[index][0]

                    # Calculate the center of the marker
                    center = np.mean(my_corners, axis=0).astype(int)

                    # Calculate the radius of the circle
                    distances = np.linalg.norm(my_corners - center, axis=1)
                    radius = int(math.ceil(np.max(distances)))

                    # Draw the circle and the center point
                    frame = cv2.circle(frame, tuple(center), radius=radius, color=(0, 0, 255), thickness=5)
                    frame = cv2.circle(frame, tuple(center), radius=5, color=(255, 0, 0), thickness=-1)
                    detected = True
                    break

            if detected:
                # Convert the modified OpenCV image back to ROS Image message
                modified_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.results_publisher_.publish(modified_image)
                self.get_logger().info(f'Published modified image for Marker ID: {marker_id}')

            # Move to the next marker in the sequence
            self.current_marker_index += 1

            # If all markers are found, stop sending angular velocity
            if self.current_marker_index == len(self.marker_sequence):
                self.get_logger().info('FINISHED')
                self.finished = True  # Stop sending angular velocity

            # Reset the image flag to ensure new images are processed
            self.image_ready = False

    def image_callback(self, msg):
        # Store the latest image received and set the flag
        self.latest_image = msg
        self.image_ready = True

rclpy.init()  # Initialize the ROS 2 Python client library
node = CmdVelPublisher()  # Create an instance of the CmdVelPublisher node

rclpy.spin(node)  # Keep the node running until it is manually stopped
