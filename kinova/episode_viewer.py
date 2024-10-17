import pickle
import matplotlib.pyplot as plt
import numpy as np
import os

# Load states and actions from file
existing_files = os.listdir('kinova/episode')
episode_numbers = [int(f.split('_')[1].split('.')[0]) for f in existing_files if f.startswith('episode_') and f.endswith('.pkl')]
i = max(episode_numbers, default=0)

file_path = f'kinova/episode/episode_{i}.pkl'
with open(file_path, 'rb') as f:
    steps = pickle.load(f)

images = [step['image'] for step in steps]
wrist_images = [step['wrist_image'] for step in steps]
states = np.array([step['state'] for step in steps])
actions = np.array([step['action'] for step in steps])
print('language_instruction:', steps[0]['instruction'])

robot_state_ = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'gripper_is_closed', 'action_blocked']
action_ = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'delta_gripper_closed', 'terminate']

# Create subplots for positions and velocities
fig1, axs1 = plt.subplots(3, 5, figsize=(15, 9))
fig1.suptitle('Robot State, Instruction: ' + steps[0]['instruction'])
axs1[0, 0].plot(states[:, 0], label='Joint 1 Position')
axs1[0, 0].set_title('Joint 1 Position')
axs1[0, 1].plot(states[:, 1], label='Joint 2 Position')
axs1[0, 1].set_title('Joint 2 Position')
axs1[0, 2].plot(states[:, 2], label='Joint 3 Position')
axs1[0, 2].set_title('Joint 3 Position')
axs1[0, 3].plot(states[:, 3], label='Joint 4 Position')
axs1[0, 3].set_title('Joint 4 Position')
axs1[0, 4].plot(states[:, 4], label='Joint 5 Position')
axs1[0, 4].set_title('Joint 5 Position')
axs1[1, 0].plot(states[:, 5], label='Joint 6 Position')
axs1[1, 0].set_title('Joint 6 Position')
axs1[1, 1].plot(states[:, 6], label='x Position')
axs1[1, 1].set_title('EEF x Position')
axs1[1, 2].plot(states[:, 7], label='y Position')
axs1[1, 2].set_title('EEF y Position')
axs1[1, 3].plot(states[:, 8], label='z Position')
axs1[1, 3].set_title('EEF z Position')
axs1[1, 4].plot(states[:, 9], label='qx Position')
axs1[1, 4].set_title('EEF qx Position')
axs1[2, 0].plot(states[:, 10], label='qy Position')
axs1[2, 0].set_title('EEF qy Position')
axs1[2, 1].plot(states[:, 11], label='qz Position')
axs1[2, 1].set_title('EEF qz Position')
axs1[2, 2].plot(states[:, 12], label='qw Position')
axs1[2, 2].set_title('EEF qw Position')
axs1[2, 3].plot(states[:, 13], label='Gripper Position')
axs1[2, 3].set_title('Gripper Position')
axs1[2, 4].plot(states[:, 14], label='Action Blocked')
axs1[2, 4].set_title('Action Blocked')
fig1.tight_layout()

# Create subplots for actions
fig2, axs2 = plt.subplots(3, 3, figsize=(9, 9))
fig2.suptitle('Actions, Instruction: ' + steps[0]['instruction'])
axs2[0, 0].plot(actions[:, 0], label='x')
axs2[0, 0].set_title('x action')
axs2[0, 1].plot(actions[:, 1], label='y')
axs2[0, 1].set_title('y action')
axs2[0, 2].plot(actions[:, 2], label='z')
axs2[0, 2].set_title('z action')
axs2[1, 0].plot(actions[:, 3], label='roll')
axs2[1, 0].set_title('roll action')
axs2[1, 1].plot(actions[:, 4], label='pitch')
axs2[1, 1].set_title('pitch action')
axs2[1, 2].plot(actions[:, 5], label='yaw')
axs2[1, 2].set_title('yaw action')
axs2[2, 0].plot(actions[:, 6], label='delta_gripper_closed')
axs2[2, 0].set_title('delta_gripper_closed action')
axs2[2, 1].plot(actions[:, 7], label='terminate')
axs2[2, 1].set_title('terminate action')
fig2.tight_layout()
fig1.savefig(file_path.replace('.pkl', '_state.png'))
fig2.savefig(file_path.replace('.pkl', '_action.png'))
import mediapy
video_path_img = file_path.replace('.pkl', '_img.mp4')
mediapy.write_video(video_path_img, images, fps=30)
video_path_wrist_img = file_path.replace('.pkl', '_wrist_img.mp4')
mediapy.write_video(video_path_wrist_img, wrist_images, fps=30)