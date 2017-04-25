Copy all files into: /.../catkin_ws/src/caros/hwcomponents/caros_universalrobot/test
This will replace your current files.


to run <simple_demo_using_move_ptp.test> script in SIMULATOR: 
    roslaunch caros_universalrobot simple_demo_using_move_ptp.test simulation:=1 / "true" 
    

to run <simple_demo_using_move_ptp.test> script in WORKCELL:
    roslaunch caros_universalrobot simple_demo_using_move_ptp.test simulation:=0 / "false" 