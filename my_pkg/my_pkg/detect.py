#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import String
from my_interface.msg import Detect



class DetectNode(Node):

    def __init__(self):
        super().__init__("detect_node")

        self.get_logger().info("DetectNode started")

        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )

        self.pub = self.create_publisher(String, "obstacle_detected", 10)
        self.pub_obs = self.create_publisher(Detect, "info_obs", 10)

    def scan_callback(self, msg: LaserScan):
        zone = {}

        long = len(msg.ranges)

        zone["right"] = msg.ranges[0 : long // 3]
        zone["front"] = msg.ranges[long // 3 : 2 * long // 3]
        zone["left"] = msg.ranges[2 * long // 3 : long]

        obstacle_detected = False

        for key in zone:
            for distance in zone[key]:
                if distance <= 15.0:
                    obstacle_detected = True
                    
                    capture(obstacle_detected)

                    self.get_logger().info("Obstacle detected in zone: " + key)
                    break
            if obstacle_detected:
                break


        msg_info = Detect()
        
        msg_info.detect = obstacle_detected
        
        msg_info.zone = key if obstacle_detected else "none"

        msg_out = String()
        msg_out.data = key if obstacle_detected else "none" 

        self.pub.publish(msg_out)
        self.pub_obs.publish(msg_info)

def capture(capture):
    if capture:
        print("Obstacle detected!")
    else:
        print("No obstacle.")


def main(args=None):
    rclpy.init(args=args)

    detect_node = DetectNode()

    rclpy.spin(detect_node)

    detect_node.destroy_node()
    rclpy.shutdown()
