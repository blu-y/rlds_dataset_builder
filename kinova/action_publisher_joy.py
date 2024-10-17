import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
# from camera import Camera1, Camera2
from cv_bridge import CvBridge
from time import time
import subprocess

class ActionPublisherJoy(Node):
    def __init__(self):
        super().__init__('action_publisher_joy')
        self.action_pub = self.create_publisher(Twist, '/twist_controller/commands', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.image_sub = self.create_subscription(Image, '/kinova/image_raw', self.image_cb, 1)
        self.br = CvBridge()
        # self.cam = Camera1()
        # self.wrist_cam = Camera2()
        # ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 20.0}}"
        # ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 1.0, max_effort: 20.0}}"
        self.open = 'ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.2, max_effort: 100.0}}" > /dev/null'
        self.close = 'ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.8, max_effort: 100.0}}" > /dev/null'
        self.gripper = False
        self.gripper_timer = time()
        self.linear_speed = 0.05
        self.angular_speed = 10.0
        self.frame = None

    def image_cb(self, msg):
        frame = self.br.imgmsg_to_cv2(msg)
        self.frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def toggle_gripper(self):
        if time() - self.gripper_timer < 0.5: return
        self.gripper_timer = time()
        self.gripper = not self.gripper
        subprocess.run(self.close if self.gripper else self.open, shell=True)
        self.get_logger().info('Gripper: {}'.format(self.gripper))

    def joy_cb(self, msg):
        action = Twist()
        speed = msg.axes[3] + 2
        action.linear.x = msg.axes[4] * self.linear_speed * speed
        action.linear.y = msg.axes[5] * self.linear_speed * speed
        action.linear.z = (msg.buttons[12] - msg.buttons[13]) * self.linear_speed * speed
        action.angular.x = -msg.axes[1] * self.angular_speed * speed
        action.angular.y = msg.axes[0] * self.angular_speed * speed
        action.angular.z = msg.axes[2] * self.angular_speed * speed
        self.action_pub.publish(action)
        self.get_logger().info('Publishing action: {}'.format(action))
        if msg.buttons[0] == 1:
            self.toggle_gripper()
        # frame = self.cam.getFrame()
        # wrist_frame = self.wrist_cam.getFrame()
        # if isinstance(frame, int) or isinstance(wrist_frame, int): return
        # if frame is None or wrist_frame is None: return
        # frame = cv2.resize(frame, (256, 256))
        # wrist_frame = cv2.resize(wrist_frame, (256, 256))
        # frame = np.concatenate((frame, wrist_frame), axis=1)
        if self.frame is None: return
        cv2.imshow('frame', self.frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    action_publisher_joy = ActionPublisherJoy()
    rclpy.spin(action_publisher_joy)
    action_publisher_joy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

