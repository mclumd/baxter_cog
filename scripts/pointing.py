from __future__ import print_function

# Import the necessary Python modules
import sys
import math
import numpy as np

# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

#Message carrying location of target
from geometry_msgs.msg import Point

def point_joint_angles(target):
	'''
	target is a numpy array or python list containing the 3d location of the thing to be pointed at, using robot-centered coordinates.
	'''
	
	#Center on location of the first point of rotation of the right arm
	base_joint_pos = np.array([0.07316, -0.26733, 0.31])
	target = target - base_joint_pos
	
	#set fixed angles to 0
	angles = {'right_e0': 0.0, 'right_e1': 0.0, 'right_w0': 0.0, 
	'right_w1': 0.0, 'right_w2': 0.0}
	
	#calculate variable angles
	angles['right_s0']=math.atan2(target[0], -target[1]) - np.pi/4
	angles['right_s1']=-math.atan2(target[2], np.linalg.norm(target[:2]))
	
	print("pointing: angles for joint right s0:", angles['right_s0'])
	print("pointing: angles for joint right s1:", angles['right_s1'])
	
	return angles

def point_callback(data):
	rospy.loginfo("Point: setting target to", str(data))
	limb = baxter_interface.Limb('right')
	limb.set_joint_positions(point_joint_angles([data.x, data.y, data.z]))
	#limb.move_to_joint_positions(angles) #blocking
	
def start_node(targetTopic):
	rospy.init_node('baxter_point')
	rospy.loginfo("Reading point commands from topic " + targetTopic)
	rospy.Subscriber(targetTopic, Point, point_callback)
	rospy.spin()

def test_angle_finder():
	print(point_joint_angles([2.0, 0.5, 0.5]))
	print(point_joint_angles([-2.0, 0.5, 0.5]))
	print(point_joint_angles([2.0, -0.5, 0.5]))

if __name__ == '__main__':
    if len(sys.argv) > 2:
    	raise Exception("Usage: 1 optional argument giving the topic on which commands are broadcast.")
    elif len(sys.argv) == 1:
    	topic = sys.argv[1]
    else:
    	topic = "/point_cmd"
    try:
        start_node(topic)
    except rospy.ROSInterruptException:
        pass