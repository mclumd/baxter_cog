import subprocess, sys, time
import rospy

'''
This script will start up the ork tabletop and object recognition 
pipelines. tabletop publishes to /tabletop/clusters and /table_array.
Object recognition publishes to /recognized_object_array.

Use Ctrl-C to shut down.
Note that if you run this script again too soon after shutting down,
it may not work.
'''

def startObjectRecog():

	try:
		enableProcess = subprocess.Popen(["rosrun", "baxter_tools", 
		"enable_robot.py", "-e"])
		enableProcess.wait()
		
		cameraServer = subprocess.Popen(["roslaunch", "openni2_launch", 
		"openni2.launch"])
		
		time.sleep(5)
		
		tabletopDir = subprocess.check_output(["rospack", "find", 
		"object_recognition_tabletop"]).strip()
		objectConfigFile = tabletopDir + "/conf/detection.object.ros.ork"
		tableConfigFile = tabletopDir + "/conf/detection.table.ros.ork"
		
		tableDetectionServer = subprocess.Popen(["rosrun", "object_recognition_core", 
		"detection", "-c", tableConfigFile, "--visualize"])
		
		time.sleep(10)
		
		objectDetectionServer = subprocess.Popen(["rosrun", "object_recognition_core", 
		"detection", "-c", objectConfigFile, "--visualize"])
	
		while True:
			time.sleep(1) #loop forever
		detectedObjects = []
	
	#jointServer = subprocess.Popen(["rosrun object_recognition_core detection -c `rospack find object_recognition_tabletop`/conf/detection.object.ros.ork --visualize"])
	finally:
		cameraServer.kill()
		tableDetectionServer.kill()
		objectDetectionServer.kill()
	
if __name__ == "__main__":
		startObjectRecog()
