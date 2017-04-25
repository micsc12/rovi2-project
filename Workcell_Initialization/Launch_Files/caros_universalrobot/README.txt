Copy files from this dir into: /home/petr/catkin_ws/src/caros/hwcomponents/caros_universalrobot/launch
This will replace current files.

Change IP to your values on the line 21 and 22 in XML file.


to run <caros_universalrobot.launch> script in SIMULATOR: 
    roslaunch caros_universalrobot caros_universalrobot.launch simulation:=1 / "true" 
    

to run <caros_universalrobot.launch> script in WORKCELL:
    roslaunch caros_universalrobot caros_universalrobot.launch simulation:=0 / "false" 