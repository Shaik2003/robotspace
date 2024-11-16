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

DOCKING_DISTANCE = 0.3
ENTRANCE_DISTANCE = 0.5
MIN_ERR_DIST = 0.05
V_MAX = 2.0
R_MIN = 0.05

class agv(Node):

    def __init__(self):
        super().__init__(nodename)
        self.get_logger().info(nodename+" started")
        
        # defining global variables
        self.desired_marker_id = None
        self.current_bot_pos = Pose2D()
        self.global_marker_pos = Pose2D()
        self.dockStationPos = Pose2D()
        self.dockerFound = False
        self.goalReached = True
        self.des_docks_queue = [0]
        self.goal_queue = []
        self.goal = None

        # some static variables
        self.prevEtheta = 0
        self.prevEr = 0
        self.Kp = 15
        self.Kd = 60
        self.Kvp = 10
        self.Kvd = 80

        # defining publishers & subscribers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_id_pub = self.create_publisher(Int8, "/desired_marker_id", 10)

        self.create_subscription(
            Pose2D, "/aruco_pos", self.marker_pos_calc, qos_profile_sensor_data
        )

        self.create_subscription(
            Odometry, "/odom", self.curr_pos_update, qos_profile_sensor_data
        )

    # takes odometry data to update current robots global position, and then calls the control system
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
        
    # sets the marker id needed and sends to aruco detector so that appropriate coordinates are received
    def target_dock_station(self):
        dock_id = Int8()

        if self.desired_marker_id is not None:
            dock_id.data = int(self.desired_marker_id)
            self.marker_id_pub.publish(dock_id)

    # callback for aruco detection topic, calculates global state of the marker, also sets teh docking station var 
    def marker_pos_calc(self, msg):
        xr = msg.x
        yr = msg.y
        thetar = msg.theta
        self.dockerFound = True

        self.global_marker_pos.x = xr*math.cos(self.current_bot_pos.theta) - yr*math.sin(self.current_bot_pos.theta) + self.current_bot_pos.x
        self.global_marker_pos.y = xr*math.sin(self.current_bot_pos.theta) + yr*math.cos(self.current_bot_pos.theta) + self.current_bot_pos.y
        thetar = self.current_bot_pos.theta - thetar
        self.global_marker_pos.theta = np.arctan2(math.sin(thetar), math.cos(thetar))
        
        self.dockStationPos.x = self.global_marker_pos.x - DOCKING_DISTANCE*math.cos(self.global_marker_pos.theta)
        self.dockStationPos.y = self.global_marker_pos.y - DOCKING_DISTANCE*math.sin(self.global_marker_pos.theta)
        self.dockStationPos.theta = self.global_marker_pos.theta

        print(self.dockStationPos.x, self.dockStationPos.y, self.dockStationPos.theta)
        self.goal_manager()

    # manages the next state the robot needs to get to
    def goal_manager(self):
        # assign next marker as goal if goal is reached 
        if self.goalReached:
            if self.des_docks_queue:
                self.desired_marker_id = self.des_docks_queue.pop()
                self.goalReached = False

        # keep updating goal location whenever the marker comes into view
        else:
            if self.dockerFound:
                self.goal = deepcopy(self.dockStationPos)
        

    # controller
    def go_to_goal(self):
        # if goal has been assigned
        if self.goal:
            # calculate error
            dx = self.goal.x - self.current_bot_pos.x
            dy = self.goal.y - self.current_bot_pos.y

            # calc direction the bot must turn to face the goal
            desR = (self.goal.x**2 + self.goal.y**2)**0.5
            curR = (self.current_bot_pos.x**2 + self.current_bot_pos.y**2)**0.5
            eR = desR - curR
            
            # since we are going for polar coordinate system, assign goal orientation angle once close to the position.
            if(eR < MIN_ERR_DIST):
                # eR = 0.0
                desTheta = self.dockStationPos.theta
            else:
                desTheta = np.arctan2(dy, dx)
            
            # error for theta. arctan2 is used to keep theta between -pi to pi
            eTheta = np.arctan2(math.sin(desTheta - self.current_bot_pos.theta), math.cos(desTheta - self.current_bot_pos.theta))

            omega = self.Kp*eTheta + self.Kd*(eTheta - self.prevEtheta)
            
            vel = self.Kvp*eR + self.Kvd*(eR - self.prevEr)

            # limit omega with a minimum radius of curvature so as to avoid skidding or toppling over while takign turns
            if abs(omega) > abs(vel)/R_MIN:
                if omega>0:
                    omega = abs(vel)/R_MIN
                else:
                    omega = -abs(vel)/R_MIN

            # limit velocity
            if vel > V_MAX:
                vel = V_MAX
            
            elif vel < -V_MAX:
                vel = -V_MAX
            
            # update previous error
            self.prevEtheta = eTheta
            self.prevEr = eR

            u = Twist()
            u.linear.x = vel
            u.angular.z = omega

            # publish velocities
            self.vel_pub.publish(u)
        
        else:
            self.goal_manager()


def main(args=None):
    rclpy.init(args=args)

    diff_agv = agv()
    diff_agv.goal_manager()
    # while not rclpy.shutdown():
    # while True:
    #     diff_agv.
    rclpy.spin(diff_agv)

    diff_agv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()