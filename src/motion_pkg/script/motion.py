#!/usr/bin/env python3
import rospy
from rospy.rostime import Time
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener,KeyCode

class robotMotion:
    leftKey=False
    rightKey=False
    upKey=False
    downKey=False
    rospy.init_node("cust_motion_controller")
    pub = rospy.Publisher("/robot/robotnik_base_control/cmd_vel",Twist,queue_size=10)
    msg = Twist()
    msg.angular.y = 0
    msg.angular.x = 0
    velocity_step = 0.1
    def keyPress(self,key):
        # monitoring key presses to capture multiple inputs
        value=key.char.lower()
        if value == 'w':
            self.upKey=True
        elif value == 'a':
            self.leftKey=True
        elif value == 'd':
            self.rightKey=True
        elif value == 's':
            self.downKey=True

    def keyRelease(self,key):
        # monitoring key releases 
        value=key.char.lower()
        if value == 'w':
            self.upKey=False
        elif value == 'a':
            self.leftKey=False
        elif value == 'd':
            self.rightKey=False
        elif value == 's':
            self.downKey=False
    
    def moveRobot(self):
        max_velocity = 2
        # acceelerating the robot according to the pressed keys
        try:            
            if self.upKey:
                if self.msg.linear.x < max_velocity:
                    self.msg.linear.x += self.velocity_step

            if self.downKey:
                if self.msg.linear.x > -max_velocity:
                    self.msg.linear.x -= self.velocity_step
    
            if self.rightKey:
                if self.msg.angular.z > -max_velocity:
                    self.msg.angular.z -= self.velocity_step

            if self.leftKey:
                if self.msg.angular.z < max_velocity:
                    self.msg.angular.z += self.velocity_step

            self.pub.publish(self.msg)
            
        except AttributeError:
            pass
   
    def stopRobot(self):
        # decceelerating the robot according to the released keys
        if not self.downKey and round(self.msg.linear.x,1)<0:
            self.msg.linear.x=round(self.msg.linear.x,1)+self.velocity_step

        if not self.upKey and round(self.msg.linear.x,1)>0:
            self.msg.linear.x=round(self.msg.linear.x,1)-self.velocity_step

        if not self.rightKey and round(self.msg.angular.z,1)<0:
            self.msg.angular.z=round(self.msg.angular.z,1)+self.velocity_step

        if not self.leftKey and round(self.msg.angular.z,1)>0:
            self.msg.angular.z=round(self.msg.angular.z,1)-self.velocity_step

        if round(self.msg.linear.x,1) == 0 and round(self.msg.angular.z,1) == 0:
            self.msg.linear.x=0.0
            self.msg.angular.z=0.0

        self.pub.publish(self.msg)







    

def main():
    RM = robotMotion()
    listner =Listener(on_press=RM.keyPress,
                      on_release=RM.keyRelease)
    listner.start()
    while not rospy.is_shutdown():
        RM.moveRobot()
        RM.stopRobot()
        rospy.sleep(0.05)
        pass






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass