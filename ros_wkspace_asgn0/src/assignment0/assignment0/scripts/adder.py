#! /usr/bin/env python3
import rospy
import numpy
from std_msgs.msg import Int16
from assignment0.msg import TwoInt

summation=Int16() #Define variable to store sum
def callback(n):
    global summation
    summation = numpy.add(n.num1, n.num2)
    rospy.loginfo(summation)

def adder():
    rospy.init_node('adder', anonymous=True)
    sub=rospy.Subscriber('/numbers', TwoInt, callback(n))
    pub=rospy.Publisher('/sum', Int16, queue_size=10)

    while not rospy.is_shutdown():
     pub.publish(summation)


if __name__ == '__main__':
   adder()			
