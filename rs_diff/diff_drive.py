import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from copy import deepcopy
import tf_transformations
import numpy as np
import math as math

import os
nodename= os.path.splitext(os.path.basename(__file__))[0]

DOCKING_DISTANCE = 0.1
V_MAX = 3.0

class agv(Node):

    def __init__(self):
        super().__init__(nodename)
        self.get_logger().info(nodename+" started")
        

        self.desired_marker_id = 0
        self.current_bot_pos = Pose2D()
        self.global_marker_pos = Pose2D()
        self.dockStationPos = Pose2D()
        self.dockerFound = False
        self.goalReached = True
        self.des_docks_queue = [0]
        self.goal_queue = []
        self.goal = []

        self.prevEtheta = 0
        self.Kp = 0.5
        self.Kd = 1
        self.Kv = 1

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_id_pub = self.create_publisher(Int8, "/desired_marker_id", 10)

        self.create_subscription(
            Pose2D, "/aruco_pos", self.marker_pos_calc, qos_profile_sensor_data
        )

        self.create_subscription(
            Odometry, "/odom", self.curr_pos_update, qos_profile_sensor_data
        )


    def curr_pos_update(self, msg):
        self.current_bot_pos.x = msg.pose.pose.position.x
        self.current_bot_pos.y = msg.pose.pose.position.y

        ori = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        self.current_bot_pos.theta = tf_transformations.euler_from_quaternion(ori)[2]

        self.target_dock_station()
        self.go_to_goal()
        # print(self.current_bot_pos.y, self.current_bot_pos.x, self.current_bot_pos.theta)
        

    def target_dock_station(self):
        dock_id = Int8()
        dock_id.data = 0
        self.marker_id_pub.publish(dock_id)

    def marker_pos_calc(self, msg):
        xr = msg.x
        yr = msg.y
        thetar = msg.theta
        self.dockerFound = True

        self.global_marker_pos.x = xr*math.cos(self.current_bot_pos.theta) - yr*math.sin(self.current_bot_pos.theta)
        self.global_marker_pos.y = xr*math.sin(self.current_bot_pos.theta) + yr*math.cos(self.current_bot_pos.theta)
        thetar += self.current_bot_pos.theta
        self.global_marker_pos.theta = np.arctan2(math.sin(thetar), math.cos(thetar))
        
        self.dockStationPos.x = self.global_marker_pos.x - DOCKING_DISTANCE*math.cos(self.global_marker_pos.theta)
        self.dockStationPos.y = self.global_marker_pos.y + DOCKING_DISTANCE*math.sin(self.global_marker_pos.theta)
        self.dockStationPos.theta = self.global_marker_pos.theta

        self.go_to_goal()

    def goal_handler(self):
        if not self.goal:
            if self.dockerFound:
                self.goal = deepcopy(self.dockStationPos)
        
        elif self.goalReached:
            if not self.des_docks_queue:
                self.desired_marker_id = self.des_docks_queue.pop()
                self.goalReached = False

    def go_to_goal(self):
        if self.goal:
            print("yaaay")
            dx = self.goal.x - self.current_bot_pos.x
            dy = self.goal.y - self.current_bot_pos.y

            desTheta = np.arctan2(dy, dx)
            
            eTheta = np.arctan2(math.sin(desTheta - self.current_bot_pos.theta), math.cos(desTheta - self.current_bot_pos.theta))

            omega = self.Kp*eTheta + self.Kd*(eTheta - self.prevEtheta)
            omega = np.arctan2(math.sin(omega), math.cos(omega))
            
            desR = (self.goal.x**2 + self.goal.y**2)**0.5
            curR = (self.current_bot_pos.x**2 + self.current_bot_pos.y**2)**0.5

            eR = desR - curR
            vel = self.Kv*eR
            if vel > V_MAX:
                vel = V_MAX
            
            elif vel < -V_MAX:
                vel = -V_MAX
            
            print(vel, omega)
            self.prevEtheta = eTheta

            u = Twist()
            u.linear.x = vel
            u.angular.z = omega

            self.vel_pub.publish(u)
        
        else:
            self.goal_handler()


def main(args=None):
    rclpy.init(args=args)

    diff_agv = agv()

    # while not rclpy.shutdown():
    # while True:
    #     diff_agv.
    rclpy.spin(diff_agv)

    diff_agv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()