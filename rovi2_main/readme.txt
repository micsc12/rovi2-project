In this package's launch folder, find utility scripts for starting up vision nodes in ROS.

After building the package, navigate to the launch folder to find the utilities.
The shell scripts should work from any directory in case you want to move them out of the package folder.

e.g.

cd ~/catkin_ws
catkin_make --pkg rovi2_main

you will likely need to run
sudo chmod 777 
on the shell scripts

For Wifi setup, run EITHER:

	roscd rovi2_main/launch
	./wifi.sh
	-This sets your ROS_IP to $(hostname -I) before calling the launch script
	
	OR if your IP is already configured, simply run
	roslaunch rovi2_main main_wifi.launch
	
For cell setup:
	FIRST, on the workstation, run
		roslaunch ~/Desktop/group5/group5.launch
		
	AFTER, on your laptop, run EITHER:
		roscd rovi2_main/launch
		./cell.sh
		-This sets your ROS_IP to $(hostname -I) before calling the launch script
		
		OR if your IP is already configured, simply run
		roslaunch rovi2_main main_cell.launch
		
		
		
NOTE: group5.launch is currently outdated but a working copy of it is on the workstation desktop in cell 3.
	
