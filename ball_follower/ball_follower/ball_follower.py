import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        self.bridge = CvBridge()
        

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Ball Follower Node Started")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        lower_bound = np.array([20, 100, 100])
        upper_bound = np.array([30, 255, 255])

        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)

            if radius > 10:  # Only react to significant detections
                center_x = int(x)
                error = center_x - (frame.shape[1] // 2)


                twist.linear.x = 0.2
                twist.angular.z = -0.002 * error

                self.get_logger().info(f"Ball detected at ({x}, {y}) with radius {radius}")
                self.get_logger().info(f"Publishing twist: linear.x = {twist.linear.x}, angular.z = {twist.angular.z}")
        else:
            self.get_logger().info("No ball detected")

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
