import rospy
import os
from std_msgs.msg import String
from geometry_msgs.msg import Point

pub = None
quadPos = None

def cmd_callback(data):
	print "got " + data.data
	cmd = str(data.data).strip()
	if cmd == "hello baxter":
		print "hello commander"
		os.system("espeak -v en 'hello commander'") #speaker?
	elif cmd == "point to the quad":
		'''
		if the quadrotor's positon is being published, its last known
		position should be stored in quadPos. If quadPos is None, no
		position has been reported.
		'''
		if quadPos:
			pub.publish(quadPos)
			print "sending command to point to", str(quadPos)
		else:
			print "No known quad location"
	elif cmd == "turn on the quad":
		print "I do not know how to start quad"
	elif cmd == "raise the quad":
		print "I do not know how to raise quad"
	elif cmd == "lower the quad":
		print "I do not know how to lower quad"
	elif cmd == "land the quad":
		print "I do not know how to land quad"
	elif cmd == "turn off the quad":
		print "I do not know how to turn off quad"
	else:
		print "command", cmd, "not understood"

def quad_pos_callback(data):
	print "setting last known quad position to", data
	global quadPos
	quadPos = data
	
def start_node():
	rospy.init_node('sphinx_test')
	#subscribe to channels reporting commands and the quadrotor's position
	rospy.Subscriber("cmds_received", String, cmd_callback)
	rospy.Subscriber("/quad_position", Point, quad_pos_callback)
	#start a publisher to the channel where pointing cmds are sent
	global pub
	pub = rospy.Publisher('/point_cmd', Point, queue_size=10)
	rospy.spin()

if __name__ == "__main__":
	start_node()
