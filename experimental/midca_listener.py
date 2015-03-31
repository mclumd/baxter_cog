#!/usr/bin/env python
import sys, time
import rospy
import roslib

from object_recognition_msgs.msg import RecognizedObjectArray
import sensor_msgs.point_cloud2 as pc2

def object_callback(data):
	if data.objects:
		print str(len(data)), "objects found."
		for object in data.objects:
			print "type:", object.type
			print "confidence:", object.confidence
			if not object.point_clouds:
				print "no point clouds"
			else:
				print str(len(object.point_clouds)), "point clouds."
				print "size(s):", [str(len(pointCloud)) for pointCloud in object.point_clouds]
			print "mesh:", object.bounding_mesh
			print "contours:", object.bounding_contours
			print "pose:", object.pose
			

def midca_listener():
    rospy.init_node('Midca_Node', anonymous=False)
    rospy.Subscriber("/recognized_object_array", ObjectArray, object_callback)
    rospy.spin()

def bounding_box(pointCloud):
	box = [0, 0, 0, 0]
	i = 0
	for point in pc2.read_points(pointCloud, skip_nans = True):
		if i > 10:
			break
		i += 1
		print point

if __name__ == '__main__':
    try:
        midca_listener()
    except rospy.ROSInterruptException:
        pass
