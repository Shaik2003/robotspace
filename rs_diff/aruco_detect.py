import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations

from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose2D

import os
nodename= os.path.splitext(os.path.basename(__file__))[0]

class dockStations(Node):

    def __init__(self):
        super().__init__(nodename)
        self.get_logger().info(nodename+" started")

        self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Int8, "/desired_marker_id", self.set_marker_id, qos_profile_sensor_data
        )

        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.info_callback, qos_profile_sensor_data
        )

        self.poses_pub = self.create_publisher(Pose2D, "/aruco_pos", 10)

        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.desiredMarkerId = [[]]
        self.marker_size = 0.4

        self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def set_marker_id(self, id):
        self.desiredMarkerId[0] = id.data

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )

        if marker_ids is not None:
            try: 
                i = np.where(marker_ids == self.desiredMarkerId)[0][0]
                
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
                pose = Pose2D()
                # pose.position.y = tvecs[i][0][1]
                pose.x = tvecs[i][0][2]    #how far from the bot
                pose.y = -tvecs[i][0][0]    #left is negetive

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                pose.theta = tf_transformations.euler_from_matrix(rot_matrix)[1]    #facing towards left of bot is positive

                self.poses_pub.publish(pose)
            except Exception as e:
                return

        
def main(args=None):
    rclpy.init(args=args)

    marks = dockStations()

    rclpy.spin(marks)

    marks.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()