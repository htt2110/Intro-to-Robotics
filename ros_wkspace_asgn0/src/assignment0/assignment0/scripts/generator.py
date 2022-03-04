#! /usr/bin/env python3
import rospy
from random import randint
from assignment0.msg import TwoInt

def generator ():
    pub = rospy.Publisher('/numbers', TwoInt, queue_size=10)
    rospy.init_node('generator', anonymous=True)
    rate = rospy.Rate(10) #Frequency 10 Hz
    n=TwoInt() #Define variable to store generated numbers
    while not rospy.is_shutdown():
    	n.num1= randint(0, 100)
    	n.num2= randint(0, 100)
    	rospy.loginfo(n)
    	pub.publish(n)
    	rate.sleep()
if __name__ == '__main__':
    try:
        generator()
    except rospy.ROSInterruptException:
        pass	
