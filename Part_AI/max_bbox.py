#!/usr/bin/env python2
#-*- coding:utf-8 -*-
import rospy
import sys

from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

pub_person = rospy.Publisher('camera/person', Bool, queue_size=10)
pub_pcx = rospy.Publisher('camera/person_cx', Float64, queue_size=10)
pub_psize = rospy.Publisher('camera/person_size', Float64, queue_size=10)

pub_sign = rospy.Publisher('camera/sign', Bool, queue_size=10)
pub_scx = rospy.Publisher('camera/sign_cx', Float64, queue_size=10)
pub_ssize = rospy.Publisher('camera/sign_size', Float64, queue_size=10)

def max_bbox():
	put = rospy.Publisher('max_bbox', Array)

def callback(_msgs_):
	print("START")
	msg_person = Bool()
	msg_pcx = Float64()
	msg_psize = Float64()

	msg_sign = Bool()
	msg_scx = Float64()
	msg_ssize = Float64()

	max_psize = 0
	max_ssize = 0

	#seperate each bbox in a frame
	for k in range(0, len(_msgs_.data), 3): 
		class_id = _msgs_.data[k]
		bbox_cx = _msgs_.data[k+1]
		bbox_size = _msgs_.data[k+2]
		if class_id == 1:
			msg_person = True

			if bbox_size >= max_psize :
				max_psize = bbox_size
				msg_pcx = bbox_cx

			pub_person.publish(msg_person)
			pub_pcx.publish(msg_pcx)
			pub_psize.publish(msg_psize)

		elif class_id == 13:
			msg_sign = True
			
			if bbox_size >= max_ssize :
				max_ssize = bbox_size
				msg_scx = bbox_cx

			pub_sign.publish(msg_sign)
			pub_scx.publish(msg_scx)
			pub_ssize.publish(msg_ssize)
		else:
			msg_person = False
			msg_sign = False
	print("END")

def listener():
	rospy.init_node('max_node')
	rospy.Subscriber("/camera/topic", Float64MultiArray, callback)
	rospy.spin()

if __name__=='__main__':
    listener()
