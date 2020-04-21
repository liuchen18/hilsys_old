this is the meta package for hilsys

for the former system:
run the old simulation system: roslaunch sim_sys whole_system.launch
run the moveit of iiwa: roslaunch iiwa_moveit move_group.launch
move to the wx: roslaunch iiwa_moveit close_planning.launch

for the current system:
start the planning environment: roslaunch sim_sys start_sim_env.launch
plan the trajectory:rosrun sim_sys sys_planner.py

