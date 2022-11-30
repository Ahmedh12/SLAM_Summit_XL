#!/usr/bin/env python3
import rospy
from rospy.rostime import Time
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener,KeyCode


rospy.init_node("cust_motion_controller")
pub = rospy.Publisher("/robot/cmd_vel",Twist,queue_size=10)

def motionKeysPressed(key):
    msg = Twist()
    velocity = 1
    try:
        char = key.char
        if char == 'w' or char == 'W':
            msg.linear.x = velocity
            pub.publish(msg)
        elif char == 's' or char == 'S':
            msg.linear.x = -velocity
            pub.publish(msg)
        elif char == 'a' or char == 'A':
            msg.angular.z = velocity
            pub.publish(msg)
        elif char == 'd' or char == 'D':
            msg.angular.z = -velocity
            pub.publish(msg)
        else:
            print("nothing from press")
    except AttributeError:
        pass

def motionKeysReleased(key):
    char = key.char
    try:
        if char == 'w' or char == 'W':
            rospy.Timer(rospy.Duration(1.0),stopRobot,oneshot=True)
        elif char == 's' or char == 'S':
            rospy.Timer(rospy.Duration(1.0),stopRobot,oneshot=True)
        elif char == 'a' or char == 'A':
            rospy.Timer(rospy.Duration(1.0),stopRobot,oneshot=True)
        elif char == 'd' or char == 'D':
            rospy.Timer(rospy.Duration(1.0),stopRobot,oneshot=True)
        else:
            print("nothing from release")
    except AttributeError:
        pass


def stopRobot(e):
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub.publish(msg)
    

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