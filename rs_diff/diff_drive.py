import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import os
nodename= os.path.splitext(os.path.basename(__file__))[0]

class agv(Node):

    def __init__(self):
        super().__init__(nodename)
        self.get_logger().info(nodename+" started")
        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    diff_agv = agv()

    rclpy.spin(diff_agv)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    diff_agv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()