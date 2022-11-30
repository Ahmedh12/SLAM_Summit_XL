#!/usr/bin/env python3
import rospy
from rospy.rostime import Time
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener,KeyCode


rospy.init_node("cust_motion_controller")
pub = rospy.Publisher("/robot/cmd_vel",Twist,queue_size=10)

def motionKeysPressed(key):
    msg = Twist()
    try:
        char = key.char
        if char == 'w' or char == 'W':
            msg.linear.x = 0.5
            pub.publish(msg)
        elif char == 's' or char == 'S':
            msg.linear.x = -0.5
            pub.publish(msg)
        elif char == 'a' or char == 'A':
            msg.angular.z = 0.5
            pub.publish(msg)
        elif char == 'd' or char == 'D':
            msg.angular.z = -0.5
            pub.publish(msg)
        else:
            print("nothing from press")
    except AttributeError:
        pass

def motionKeysReleased(key):
    msg = Twist()
    char = key.char
    try:
        if char == 'w' or char == 'W':
            msg.linear.x = 0.0
            pub.publish(msg)
        elif char == 's' or char == 'S':
            msg.linear.x = 0.0
            pub.publish(msg)
        elif char == 'a' or char == 'A':
            msg.angular.z = 0.0
            pub.publish(msg)
        elif char == 'd' or char == 'D':
            msg.angular.z = 0.0
            pub.publish(msg)
        else:
            print("nothing from release")
    except AttributeError:
        pass

def main():
    start_time = Time.now().to_sec()
    listner =Listener(on_press=motionKeysPressed,
                      on_release=motionKeysReleased)
    listner.start()

    while not rospy.is_shutdown():
        pass






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass