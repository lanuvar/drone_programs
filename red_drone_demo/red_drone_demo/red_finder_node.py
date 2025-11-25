
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedFinder(Node):
    def __init__(self):
        super().__init__('red_finder')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('min_area', 1200)
        self.bridge = CvBridge()
        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.min_area = self.get_parameter('min_area').get_parameter_value().integer_value

        self.image_sub = self.create_subscription(Image, topic, self.image_cb, 10)
        self.target_pub = self.create_publisher(Bool, '/vision/target_found', 10)
        self.center_pub = self.create_publisher(Point, '/vision/target_center', 10)

        # HSV aralığı (iki bant)
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        # Kararlılık için ardışık kare sayacı
        self.streak = 0
        self.need_streak = 3

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = False
        center_point = Point()
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w/2, y + h/2
                center_point.x = float(cx)
                center_point.y = float(cy)
                center_point.z = float(area)
                found = True

        self.streak = self.streak + 1 if found else 0
        target_msg = Bool()
        target_msg.data = self.streak >= self.need_streak
        self.target_pub.publish(target_msg)
        if target_msg.data:
            self.center_pub.publish(center_point)

def main():
    rclpy.init()
    rclpy.spin(RedFinder())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
