
1) To start using baxter, follow the instructions in the setup file.

2) To install the baxter packages including moveit and ork 
(object recognition kitchen) on your own machine, or to reinstall
on mcleod (do this cautiously, please!), see the installation file

3) To run a simple test of baxter with moveit, do:
	
	python tests/moveit_test.py

This will enable the robot (if it is on), start up the moveit servers,
and have baxter move its arms around. Note that once the servers start
up there is a prompt where you must press enter to continue.

4) Development guidelines:

-Please write tests for things you get working - right now a test
for ork would be very useful. 
	
-Please run all tests before pushing changes to github. This will
ensure the basic functionality stays functional for everyone.

-If you are working on something but it is unfinished, put it in the
experimental folder. This will tell other people not to use it.

-Put finished tests in the tests folder, and robot scripts in the
scripts folder. We will add more folders as necessary.
	
-Please send email about what you are working on, preferably before
you start. This way people can offer to help, or at least not do
the same thing. Developers should be copied on the email thread 
"ROS-MIDCA updates"
