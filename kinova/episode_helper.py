import os
import subprocess
from time import sleep


jtc = 'ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [joint_trajectory_controller], deactivate_controllers: [twist_controller], strictness: 1, activate_asap: true,}"'
gen3_pick = 'ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0, -0.6283, 1.5708, 1.5708, 0.6283, -1.5708], time_from_start: {sec: 5}},]}" -1'
tc = 'ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [twist_controller], deactivate_controllers: [joint_trajectory_controller], strictness: 1, activate_asap: true,}"'
subprocess.run("source ~/.bashrc", shell=True, executable="/bin/bash")
subprocess.run("ros2 topic pub /new_episode std_msgs/msg/String \"data: 'pause'\" -1", shell=True, executable="/bin/bash")
subprocess.run("ros2 topic pub /new_episode std_msgs/msg/String \"data: 'pause'\" -1", shell=True, executable="/bin/bash")
subprocess.run("ros2 topic pub /new_episode std_msgs/msg/String \"data: 'pause'\" -1", shell=True, executable="/bin/bash")
# read instruction.txt and make instruction_list
# 1. pick up the can and place it on the box
# 2. pick up the tape and put it in the box
# 3. place the can next to the tape
# get rid of numbers and dots
instruction_dir = os.path.join(os.path.dirname(__file__), 'instruction.txt')
with open(instruction_dir, 'r') as f:
    instruction = f.read()
instruction_list = instruction.split('\n')
# get rid of numbers
instruction_list = [i.split(' ', 1)[1] for i in instruction_list if i]
for i in instruction_list:
    print(i)
print('=======================================')
subprocess.run("ros2 topic pub /new_episode std_msgs/msg/String \"data: 'reset'\" -1", shell=True, executable="/bin/bash")
subprocess.run("ros2 topic pub /new_episode std_msgs/msg/String \"data: 'reset'\" -1", shell=True, executable="/bin/bash")
subprocess.run("ros2 topic pub /new_episode std_msgs/msg/String \"data: 'reset'\" -1", shell=True, executable="/bin/bash")
for i in instruction_list:
    print('\n=======================================\n')
    subprocess.run(jtc, shell=True, executable="/bin/bash")
    subprocess.run(jtc, shell=True, executable="/bin/bash")
    sleep(1)
    subprocess.run(gen3_pick, shell=True, executable="/bin/bash")
    subprocess.run(gen3_pick, shell=True, executable="/bin/bash")
    sleep(1)
    subprocess.run('sleep 6s', shell=True, executable="/bin/bash")
    sleep(1)
    subprocess.run(tc, shell=True, executable="/bin/bash")
    sleep(1)
    subprocess.run(f"ros2 topic pub /new_episode std_msgs/msg/String \"data: '{i}'\" -1", shell=True, executable="/bin/bash")
    subprocess.run(f"ros2 topic pub /new_episode std_msgs/msg/String \"data: '{i}'\" -1", shell=True, executable="/bin/bash")
    subprocess.run(f"ros2 topic pub /new_episode std_msgs/msg/String \"data: '{i}'\" -1", shell=True, executable="/bin/bash")
    sleep(3)
    print(i)
    print('press enter if episode is done')
    input()
    subprocess.run("ros2 topic pub /new_episode std_msgs/msg/String \"data: 'pause'\" -1", shell=True, executable="/bin/bash")
    subprocess.run("ros2 topic pub /new_episode std_msgs/msg/String \"data: 'pause'\" -1", shell=True, executable="/bin/bash")
    subprocess.run("ros2 topic pub /new_episode std_msgs/msg/String \"data: 'pause'\" -1", shell=True, executable="/bin/bash")
    sleep(3)
    print('press enter to continue')
    input()
