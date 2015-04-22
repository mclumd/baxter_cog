from __future__ import print_function
import time
import rospy
from geometry_msgs.msg import Point

rospy.init_node('baxter_pointing_test')

pub = rospy.Publisher('/point_cmd', Point, queue_size=10)

points = [Point(x = 5.0, y = 0.0, z = 0.0),
		  Point(x = 8.0, y = 7.0, z = 2.0),
		  Point(x = 7.0, y = -4.0, z = -0.5), Point(x = -1, y = 1, z = 0)]
n = 0
while n < 20:
	n += 1
	time.sleep(2)
	p = points[n % 4]
#	p = points[1]
	print("Sending point command:", p)
	pub.publish(p)
