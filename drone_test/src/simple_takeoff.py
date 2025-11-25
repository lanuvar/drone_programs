#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def wait_for_connection():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo_throttle(2.0, "MAVROS bağlanıyor...")
        rate.sleep()

def set_mode(mode_name):
    rospy.wait_for_service('/mavros/set_mode')
    set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    resp = set_mode_srv(custom_mode=mode_name)
    return resp.mode_sent

def arm_vehicle(arm=True):
    rospy.wait_for_service('/mavros/cmd/arming')
    arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    resp = arm_srv(arm)
    return resp.success

def publish_setpoint(z=10.0, seconds=8.0):
    sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = z

    rate = rospy.Rate(20)
    ticks = int(seconds * 20)
    for _ in range(ticks):
        sp_pub.publish(pose)
        rate.sleep()

def main():
    rospy.init_node('simple_takeoff', anonymous=True)
    rospy.Subscriber('/mavros/state', State, state_cb)

    wait_for_connection()

    # GUIDED moduna geçmeden önce setpoint akışı başlatmak iyi pratiktir
    publish_setpoint(z=2.0, seconds=2.0)

    if not set_mode("GUIDED"):
        rospy.logerr("GUIDED moda geçilemedi")
        return

    if not arm_vehicle(True):
        rospy.logerr("Arm başarısız")
        return

    rospy.loginfo("Arm edildi, 10m'ye yükseliyor...")
    publish_setpoint(z=10.0, seconds=12.0)

    rospy.loginfo("Setpoint gönderimi tamam. LAND moduna geçiliyor.")
    set_mode("LAND")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass