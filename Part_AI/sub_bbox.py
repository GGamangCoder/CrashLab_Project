#!/usr/bin/env python2
#-*- coding:utf-8 -*-
import rospy
import sys

from std_msgs.msg import Int16
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose2D
from vision_msgs.msg import Detection2DArray

import jetson.inference
import jetson.utils

pub_bbox = rospy.Publisher('camera/topic', Float64MultiArray, queue_size=10)
bbox_Array = []
#def callback(data):
#	rospy.loginfo(rospy.get_name()+"size_x is %s",data.size_x)
#	rospy.loginfo(rospy.get_name()+"size_y is %s",data.size_y)

#def listener():
#	rospy.init_node("listener",anonymous=True)
#	rospy.Subscriber("/detections", 10, callback)
#	rospy.spin()

def callback(_msgs_):
#	rospy.loginfo("=====================bbox_msgs===========================")
	print("START")
#	rospy.loginfo(_msgs_.detections[-1].bbox)
	msg_bbox = Float64MultiArray()
	bbox_Array = []
	for det in _msgs_.detections: #seperate each obs
		det_size = det.bbox.size_x * det.bbox.size_x
#		k = det.bbox.center.x
#		print(k)
#		print(type(k))
		det_id = det.results[-1].id
#		if det_id == 1: # classify person and sign
		bbox_Array.append(float(det_id))
		bbox_Array.append(det.bbox.center.x)
		bbox_Array.append(det_size)
		print (type(bbox_Array))
		if det_id == 1:
			for k in range(0,len(bbox_Array),3):
				print (type(bbox_Array[k]))
				print (type(bbox_Array[k+1]))
				print (type(bbox_Array[k+2]))
			msg_bbox.data = bbox_Array
			pub_bbox.publish(msg_bbox)
		elif det_id == 13:
			for k in range(0,len(bbox_Array),3):
				print (float(bbox_Array[k]))
				print (type(bbox_Array[k+1]))
				print (type(bbox_Array[k+2]))
			msg_bbox.data = bbox_Array
			pub_bbox.publish(msg_bbox)

	print("END")
#def obs_count_callback(_msgs_):
#	global()obs_count = _msgs_.data
	
def listener():
	rospy.init_node('sub_node')
#	rospy.Subscriber("/detectnet/obs_count", Int16, obs_count_callback)
	rospy.Subscriber("/detectnet/detections", Detection2DArray, callback)
	rospy.spin()

if __name__=='__main__':
    listener()


