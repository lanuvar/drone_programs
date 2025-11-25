#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

bridge = CvBridge()
current_state = None
target_found = False

def state_cb(msg):
    global current_state
    current_state = msg

def image_cb(msg):
    global target_found
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Kırmızı renk aralığı (HSV)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Eğer kırmızı obje bulunduysa
    if cv2.countNonZero(mask) > 500:  # eşik değer
        target_found = True

def arm_and_takeoff():
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    # ARM
    arm_srv(True)
    # GUIDED moda geç
    mode_srv(custom_mode="GUIDED")

    # 10 metreye çık
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 10

    rate = rospy.Rate(20)
    for i in range(100):  # biraz setpoint gönder
        local_pos_pub.publish(pose)
        rate.sleep()

def land():
    rospy.wait_for_service('/mavros/set_mode')
    mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    mode_srv(custom_mode="LAND")

def main():
    rospy.init_node('red_object_search', anonymous=True)
    rospy.Subscriber('/mavros/state', State, state_cb)
    rospy.Subscriber('/camera/image_raw', Image, image_cb)

    arm_and_takeoff()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if target_found:
            print("Kırmızı obje bulundu, iniş başlatılıyor...")
            land()
            break
        rate.sleep()

if __name__ == '__main__':
    main()