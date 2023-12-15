#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import time
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()

        self.cv_image = None
        # self.cv_depth_image = None

        self.rect_start = (10, 80)
        self.rect_end = (720, 360)

        self.image_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw", Image, self.image_callback)
        # self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/camera_info", CameraInfo, self.camera_info_callback)

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.point_pub = rospy.Publisher("goal_point", Point, queue_size=10)
        self.image_pub = rospy.Publisher('detected_tile', Image, queue_size=10)

        rospy.spin()

    def camera_info_callback(self, msg):
        # TODO: Extract the intrinsic parameters from the CameraInfo message
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth):
        # TODO: Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (BGR8 format)
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

            # If we have both color and depth images, process them
            # if self.cv_depth_image is not None:
            self.process_images()

        except Exception as e:
            print("Error:", e)

    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("Error:", e)

    def process_images(self):
        # Convert the color image to HSV color space
        # hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        # TODO: Define range for cup color in HSV
        # NOTE: You can visualize how this is performing by viewing the result of the segmentation in rviz
        # lower_rgb = np.array([170, 245, 60]) #np.array([70, 50, 180][::-1])
        # upper_rgb = np.array([210, 255, 100])#np.array([100, 80, 210][::-1])

        # Convert RGB thresholds to HSV
        # lower_hsv = np.array([50, 200, 130])#self.rgb_to_hsv(lower_rgb)
        # upper_hsv = np.array([85, 240, 220]) #self.rgb_to_hsv(upper_rgb)

        lower_mono = 0
        upper_mono = 220

        # TODO: Threshold the image to get only cup colors
        # HINT: Lookup np.where() or cv2.inRange()
        mask = cv2.inRange(self.cv_image, lower_mono, upper_mono)

        for row in range(len(mask)):
            for col in range(len(mask[row])):
                if row < self.rect_start[1] or col < self.rect_start[0] or row > self.rect_end[1] or col > self.rect_end[0]:
                    mask[row][col] = 0

        print(np.mean(mask, axis=(0,1)))
        # print(lower_hsv, upper_hsv)
        # print(mask)

        # TODO: Get the coordinates of the cup points on the mask
        # HINT: Lookup np.nonzero() or np.where()
        y_coords, x_coords = np.where(mask == 255)

        # If there are no detected points, exit
        if len(x_coords) == 0 or len(y_coords) == 0:
            print("No points detected. Is your color filter wrong?")
            return

        # Calculate the center of the detected region by 
        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))

        # Fetch the depth value at the center
        depth = 640 # self.cv_depth_image[center_y, center_x]

        if self.fx and self.fy and self.cx and self.cy:
            camera_x, camera_y, camera_z = self.pixel_to_point(center_x, center_y, depth)
            # camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
            camera_link_x, camera_link_y, camera_link_z = camera_x, camera_y, camera_z

            # Convert from mm to m
            camera_link_x /= 1000
            camera_link_y /= 1000
            camera_link_z /= 1000

            # Convert the (X, Y, Z) coordinates from camera frame to odom frame
            try:
                self.tf_listener.waitForTransform("/base", "/right_hand_camera", rospy.Time(), rospy.Duration(10.0))
                point_odom = self.tf_listener.transformPoint("/base", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/right_hand_camera"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
                X_odom, Y_odom, Z_odom = point_odom.point.x, point_odom.point.y, point_odom.point.z
                print("Real-world coordinates in odom frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(X_odom, Y_odom, Z_odom))

                if X_odom < 0.001 and X_odom > -0.001:
                    print("Erroneous goal point, not publishing - Is the cup too close to the camera?")
                else:
                    print("Publishing goal point: ", X_odom, Y_odom, Z_odom)
                    # Publish the transformed point
                    self.point_pub.publish(Point(X_odom, Y_odom, Z_odom))

                    # Overlay cup points on color image for visualization
                    cup_img = self.cv_image.copy()
                    cup_img[y_coords, x_coords] = 0  # Highlight cup points in red
                    cv2.circle(cup_img, (center_x, center_y), 5, 125, -1)  # Draw green circle at center
                    
                    cv2.rectangle(cup_img, self.rect_start, self.rect_end, 125, 2)

                    # Convert to ROS Image message and publish
                    ros_image = self.bridge.cv2_to_imgmsg(cup_img, "mono8")
                    self.image_pub.publish(ros_image)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error: " + e)
                return

if __name__ == '__main__':
    ObjectDetector()
