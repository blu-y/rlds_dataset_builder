import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Twist
import pickle
from camera import Camera1, Camera2
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.image_pub = self.create_publisher(Image, '/kinova/image_raw', 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10)
        self.action_sub = self.create_subscription(
            Twist,
            '/twist_controller/commands',
            self.action_cb,
            10)
        self.steps = []
        self.actions = []
        self.cam = Camera1()
        self.wrist_cam = Camera2()
        self.wrist_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.wrist_cb,
            10)
        self.br = CvBridge()
        self.episode = []
        self.action = Twist()
        self.joint_state = JointState()
        self.i = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_on = True
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.instruction = 'Pick up the plastic and place it next to the box'

    def get_robot_state(self, transform):
        x, y, z = transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z
        qx, qy, qz, qw = transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w
        joint_state = list(self.joint_state.position[1:7]) + [x, y, z, qx, qy, qz, qw] + [self.joint_state.position[0], 0]
        action = [self.action.linear.x, self.action.linear.y, self.action.linear.z, self.action.angular.x, self.action.angular.y, self.action.angular.z]
        # TODO
        gripper = 1 if self.joint_state.position[0] > 0.3 else 0
        # 1 if gripper needs to be closed, -1 if gripper needs to be opened, 0 if no action
        terminate = 0
        # 1 if episode is done, 0 if episode is not done
        action = action + [gripper, terminate] 
        return joint_state, action

    def step(self, frame, wrist_frame):
        try:  
            transform = self.tf_buffer.lookup_transform('base_link', 'tool_frame', rclpy.time.Time())
        except: return
        state, action = self.get_robot_state(transform)
        frame = cv2.resize(frame, (256, 256))
        wrist_frame = cv2.resize(wrist_frame, (256, 256))
        frame_pub = np.concatenate((frame, wrist_frame), axis=1)
        msg = self.br.cv2_to_imgmsg(frame_pub)
        self.image_pub.publish(msg)
        step = {
            'image': frame,
            'wrist_image': wrist_frame,
            'state': state,
            'action': action,
            'instruction': self.instruction,
        }
        self.steps.append(step)
        self.i += 1
        if self.i % 10 == 0:
            # round joint_state to 3 decimal places
            state = [round(x, 3) for x in state]
            # round action to 3 decimal places
            action = [round(x, 3) for x in action]
            print(frame.shape, wrist_frame.shape)
            print('state', state)
            print('action', action)

    def timer_cb(self):
        frame = self.cam.getFrame()
        wrist_frame = self.wrist_cam.getFrame()
        if frame is None or wrist_frame is None or isinstance(frame, int) or isinstance(wrist_frame, int):
            return
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        wrist_frame = cv2.cvtColor(wrist_frame, cv2.COLOR_BGR2RGB)
        self.step(frame, wrist_frame)

    def wrist_cb(self, msg):
        if self.timer_on:
            self.timer_on = False
            self.timer.cancel()
        wrist_frame = self.br.imgmsg_to_cv2(msg)
        frame = self.cam.getFrame()
        # wrist_frame = cv2.cvtColor(wrist_frame, cv2.COLOR_RGB2BGR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.step(frame, wrist_frame)

    def joint_state_cb(self, msg):
        self.joint_state = msg
    
    def action_cb(self, msg):
        self.action = msg

    def save_episode(self, filename):
        with open(filename, 'wb') as f:
            pickle.dump(self.steps, f)
        self.get_logger().info(f'Saved joint states to {filename}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        pass
    if not os.path.exists('kinova/episode'):
        os.makedirs('kinova/episode')
    existing_files = os.listdir('kinova/episode')
    episode_numbers = [int(f.split('_')[1].split('.')[0]) for f in existing_files if f.startswith('episode_') and f.endswith('.pkl')]
    next_episode_number = max(episode_numbers, default=0) + 1
    joint_state_subscriber.save_episode(f'kinova/episode/episode_{next_episode_number}.pkl')
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()