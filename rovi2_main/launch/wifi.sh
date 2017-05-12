export ROS_MASTER_URI=http://$(hostname -I):11311/

roslaunch rovi2_main test_wifi.launch
