# Prerequistics
* moveit installed
* franka_description should be installed 
# Instruction
* git clone https://github.com/AbdulrazzakJaroukh/ws_tsk.git 
* cd ws_tsk
* catkin clean
* catkin build -j4
* source devel/setup.bash
* roslaunch panda_moveit_config demo.launch 
In new tab:
* cd ws_tsk
* source devel/setup.bash
* roslaunch move_group_interface move_group_interface_panda.launch
