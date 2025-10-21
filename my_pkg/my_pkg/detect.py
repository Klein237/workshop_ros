#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import String


class DetectNode(Node):

    def __init__(self):
        super().__init__("detect_node")

        self.get_logger().info("DetectNode started")

        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )

        self.pub = self.create_publisher(String, "obstacle_detected", 10)

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

        msg_out = Bool()
        msg_out.data = obstacle_detected
        self.pub.publish(msg_out)

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
