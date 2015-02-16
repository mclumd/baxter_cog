import subprocess, sys, time
from moveit_commander import MoveGroupCommander
import moveit_msgs.msg
import geometry_msgs.msg

#enable robot if not already done
enableProcess = subprocess.Popen(["rosrun", "baxter_tools", 
"enable_robot.py", "-e"])
enableProcess.wait()

#start moveit servers. Since we do not know how long it will take them 
#to start, pause for 15 seconds
jointServer = subprocess.Popen(["rosrun", "baxter_interface", 
"joint_trajectory_action_server.py"])
planServer = subprocess.Popen(["roslaunch", "baxter_moveit_config", 
"move_group.launch"])
time.sleep(15)

raw_input("\n\n\n\nThe last message above should say something like '\
 Everybody is happy and ready to plan!' If it does, the servers are set\
 up. Press enter to start the test. Baxter should raise his right arm,\
 then raise his left arm, then lower each in the same order. If his\
 arms are already in the 'raised' position, he will just lower them.\
 \n\nNote that right now there are several error messages printed\
 during this process; someone should investigate that.")

try:
	left = MoveGroupCommander("left_arm")
	right = MoveGroupCommander("right_arm")
	
	upRight = geometry_msgs.msg.Pose(position = geometry_msgs.msg.Point(
	x = 0.7, y = -0.35, z = 0.8))
	
	upLeft = geometry_msgs.msg.Pose(position = geometry_msgs.msg.Point(
	x = 0.7, y = 0.35, z = 0.8))
	
	downRight = geometry_msgs.msg.Pose(position = geometry_msgs.msg.Point(
	x = 0.7, y = -0.45, z = 0.3))
	
	downLeft = geometry_msgs.msg.Pose(position = geometry_msgs.msg.Point(
	x = 0.7, y = 0.45, z = 0.3))
	
	right.set_pose_target(upRight)
	left.set_pose_target(upLeft)
	right.go()
	left.go()
	
	right.set_pose_target(downRight)
	left.set_pose_target(downLeft)
	
	right.go()
	left.go()
	
finally:
	#clean up - kill servers
	jointServer.kill()
	planServer.kill()
	
	sys.exit()

