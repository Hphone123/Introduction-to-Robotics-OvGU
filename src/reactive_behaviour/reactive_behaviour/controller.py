import rclpy
from rclpy.node import Node
import random

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.node import DrivingSwarmNode, main_fn

class VelocityController(DrivingSwarmNode):

    def __init__(self, name: str) -> None:
        super().__init__(name)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.setup_command_interface()
        self.logger = self.get_logger()
        # self.started = True
        self.go = True
        self.logger.info(f"Initialized Robot {self.name}!")
        
    def timer_cb(self):
        # Wait 'till ready
        if not self.started:
            return
        
        msg = Twist()
        msg.linear.x = 0.1
        
        if (self.forward_distance < 0.25 or self.rf_distance < 0.25 or self.lf_distance < 0.25) and (self.go):
            self.go = False
            if self.lf_distance < self.rf_distance:
                self.turn = 'left'
            else:
                self.turn = 'right'
        if self.forward_distance > 0.5 and self.rf_distance > 0.5 and self.lf_distance > 0.5:
            self.go = True
        
        if not self.go:    
            msg.linear.x = 0.0
            if self.turn == 'right':
                msg.angular.z = -0.5
            else:
                msg.angular.z = 0.5

        # Go
        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        self.started = True
        self.forward_distance = msg.ranges[0]
        self.rf_distance = msg.ranges[35]
        self.lf_distance = msg.ranges[len(msg.ranges) - 35]



def main():
    main_fn('controller', VelocityController)


if __name__ == '__main__':
    main()
