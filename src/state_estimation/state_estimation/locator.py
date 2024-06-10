import rclpy
import numpy as np
from rclpy.node import Node

from scipy.optimize import least_squares
from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.create_subscription(Twist, 'goal', self.goToGoal, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')
        
    def goToGoal (self):
        
        msg = Twist()
        msg.linear.x = 0.3
        
        self.vel_pub.publish(msg)
    
    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)
        
    def find_intersect(self):
        res_x1 = 0
        res_y1 = 0
        res_2  = 0
        for i in range(len(self.anchor_ranges)):
            res_x1 += self.anchor_ranges[i].anchor.x * self.anchor_ranges[i].range
            res_y1 += self.anchor_ranges[i].anchor.y * self.anchor_ranges[i].range
            res_2  +=                                  self.anchor_ranges[i].range
        
        res_x = res_x1 / res_2
        res_y = res_y1 / res_2
        
        return [res_x, res_y]
    
    def calculate_position(self):
        if not len(self.anchor_ranges):
            return 0.0, 0.0, 0.0
        
        # YOUR CODE GOES HERE:
        
        #* Testing that this bullshit is Online
        
        result = self.find_intersect()
        
        self.get_logger().info(f"Result Position: {result}")
        
        return result[0], result[1], 0.0
    

def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
