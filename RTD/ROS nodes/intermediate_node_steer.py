#!/usr/bin/env python
import rospy
import numpy as np
#from beginner_tutorials.msg import Num

from std_msgs.msg import Float32, Float32MultiArray
from random import randint

pub=rospy.Publisher('/steer',Float32,queue_size=1)

def values_publisher(msg):
    rate=rospy.Rate(100)
    i=1
    A = np.array(msg.data)
    k = len(A)
    if k>50:
	k = 50
    	while i<=k:
	    random_msg = Float32()	
	    random_msg.data=A[i]
	    pub.publish(random_msg)
	    i=i+1
            rate.sleep()
    else:
	rate1 = rospy.Rate(2*(k-1))
    	while i<k:
	    random_msg = Float32()	
	    random_msg.data=A[i]
	    pub.publish(random_msg)
	    i=i+1
            rate1.sleep()

def array_subscriber():
	rospy.init_node('intermediate_node_steer')
	rospy.Subscriber('/steer_topic',Float32MultiArray,values_publisher)
	rospy.spin()

if __name__=='__main__':
    try:
	array_subscriber()
    except rospy.ROSInterruptException:
	pass
