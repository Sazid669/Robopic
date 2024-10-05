#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class ArucoDetector:
    def __init__(self):
        # Publisher to send detected ArUco marker positions
        self.aruco_pub = rospy.Publisher("/aruco_position", Float64MultiArray, queue_size=10)
        
        # Subscriber to receive images from the camera
        self.aruco_sub = rospy.Subscriber("/turtlebot/kobuki/sensors/realsense/color/image_color", Image, self.image_callback)
        
        # Setting the rate for publishing
        self.rate = rospy.Rate(10)

        # Camera position with respect to the robot
        self.camera_x = 0.122
        self.camera_y = -0.033
        self.camera_z = 0.082
        self.camera_roll = math.pi / 2
        self.camera_pitch = 0.0
        self.camera_yaw = math.pi / 2

        # Array to store camera position and orientation
        self.camera_pose = np.array([self.camera_x, self.camera_y, self.camera_z, self.camera_roll, self.camera_pitch, self.camera_yaw])

    def transform_camera_to_robot(self, x, y, z, roll, pitch, yaw):
        # Create an identity matrix for the transformation
        Transf = np.eye(4)
        
        # Convert Euler angles to rotation matrix
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])
        
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                       [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
        
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0],
                       [0, 0, 1]])
        
        # Combine the rotation matrices
        R = Rz @ Rx @ Ry

        # Create translation vector
        Trans = np.array([x, y, z]).reshape(3, 1)
        
        # Assign the rotation and translation to the transformation matrix
        Transf[0:3, 0:3] = R
        Transf[0:3, 3] = np.squeeze(Trans)
        
        return Trans, R, Transf

    def image_callback(self, Image_msg):
        # Initialize the CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()
        
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(Image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Set the marker length
        marker_length = 0.05

        # Load the ArUco dictionary
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        
        # Define camera matrix and distortion coefficients
        camera_matrix = np.array([[1396.8086675255468, 0.0, 960.0],
                                  [0.0, 1396.8086675255468, 540.0],
                                  [0.0, 0.0, 1.0]])

        dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        frame = cv_image

        # Detect ArUco markers in the frame
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, dictionary)

        if marker_ids is not None:
            for i in range(len(marker_ids)):
                # Estimate the pose of each marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners[i], marker_length, camera_matrix, dist_coeffs)
                
                # Reshape the rotation and translation vectors
                rvecs = rvecs[0, :].reshape(1, 3)
                tvecs = tvecs[0, :].reshape(1, 3)
                
                # Draw detected markers and axes
                cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids, (0, 255, 0))
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs, tvecs, 0.05)

                # Convert rotation vector to rotation matrix
                Rot, _ = cv2.Rodrigues(rvecs)
                
                # Create a transformation matrix
                Transf = np.eye(4)
                Transf[0:3, 0:3] = Rot
                Transf[0:3, 3] = np.squeeze(tvecs)

                # Transform from camera to robot frame
                Trans_r_c, rot_r_c, Transf_r_c = self.transform_camera_to_robot(self.camera_pose[0], self.camera_pose[1], self.camera_pose[2], self.camera_pose[3], self.camera_pose[4], self.camera_pose[5])
                Transf_r = Transf_r_c @ Transf

                # Get the X, Y, and Z coordinates of the marker
                x = Transf_r[0, 3]
                y = Transf_r[1, 3]
                z = -0.25

                # Convert coordinates to float if necessary
                x = float(x)
                y = float(y)
                z = float(z)

                # Create a Float64MultiArray message to publish the point values
                point_msg = Float64MultiArray()
                point_msg.data = [x, y, z, 0.0]

                # Publish the point message
                self.aruco_pub.publish(point_msg)
                # Display the frame with detected markers and axes
                cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Camera", 800, 600)
                cv2.moveWindow("Camera", 0, 0)
                cv2.imshow("Camera", frame)
                cv2.waitKey(3000) 
                        
                # Shutdown the node after detecting the marker
                rospy.signal_shutdown("ArUco marker detected")

       

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node("aruco_detector")
    
        ArucoDetector()
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
