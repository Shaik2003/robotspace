import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations

from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose2D, Pose

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

        self.poses_pub = self.create_publisher(Pose, "aruco_poses", 10)

        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.desiredMarkerId = None
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
        self.desiredMarkerId = id

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        # markers = ArucoMarkers()
        # pose_array = PoseArray()
        # if self.camera_frame == "":
        #     markers.header.frame_id = self.info_msg.header.frame_id
        #     pose_array.header.frame_id = self.info_msg.header.frame_id
        # else:
        #     markers.header.frame_id = self.camera_frame
        #     pose_array.header.frame_id = self.camera_frame

        # markers.header.stamp = img_msg.header.stamp
        # pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )

        if marker_ids is not None:
            try: 
                # i = marker_ids.index(self.desiredMarkerId)
                i = 0
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                
                self.poses_pub.publish(pose)
            except:
                return


        
def main(args=None):
    rclpy.init(args=args)

    marks = dockStations()

    rclpy.spin(marks)

    marks.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()