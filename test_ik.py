#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from threading import Thread
from copy import copy
from sensor_msgs.msg import JointState
import ikfastpy
import numpy as np
from scipy.spatial.transform import Rotation as R

PANDA_JOINT_NAMES = [ "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
JOINT_MINS = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_MAXS = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
TRANS_DEL =  0.05
ROT_DEL = 10.0*np.pi/180.0

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.action_client = ActionClient(self, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')
        self.subscription = self.create_subscription(JointState, 'joint_states', self.listener_callback,10)
        self.panda_joint_idxs = None
        self.cur_joints = None
        self.franka_kin = ikfastpy.PyKinematics()
        self.subscription 

    def listener_callback(self, msg):
        if self.panda_joint_idxs is None:
            self.panda_joint_idxs = []
            for panda_name in PANDA_JOINT_NAMES:
                for robot_idx, robot_name in enumerate(msg.name):
                    if panda_name == robot_name:
                        self.panda_joint_idxs.append(robot_idx)
                        break
            assert len(PANDA_JOINT_NAMES) == len(self.panda_joint_idxs)                        
            self.cur_joints = np.zeros(len(PANDA_JOINT_NAMES), dtype=float)
        
        for i in range(len(self.panda_joint_idxs)):
            self.cur_joints[i] = msg.position[self.panda_joint_idxs[i]]

    def get_cur_fk(self):
        if self.cur_joints is None:
            return None, None

        ee_pose = self.franka_kin.forward(list(self.cur_joints))
        ee_pose = np.asarray(ee_pose).reshape(3,4) # 3x4 rigid transformation matrix

        trans = ee_pose[:,-1]
        rot = R.from_matrix(ee_pose[:3,:3]).as_quat()

        return trans, rot

    def get_ik(self, trans, rot):
        if self.cur_joints is None:
            return None

        mat_rot = R.from_quat(rot).as_matrix()
        ee_pose = np.concatenate([mat_rot, trans[:, np.newaxis]], axis=1)
        joint_configs = self.franka_kin.inverse(ee_pose.reshape(-1).tolist(),
                                                free_itr=10000,
                                                free_min=JOINT_MINS[-1],
                                                free_max=JOINT_MAXS[-1])
        n_joints = len(PANDA_JOINT_NAMES)
        n_solutions = int(len(joint_configs)/n_joints)
        if n_solutions <= 0:
            return None

        joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)

        # Choose solution that is closest to current joint
        joint_idx = np.argmin(np.sum(np.square(joint_configs - self.cur_joints), axis=1))

        return joint_configs[joint_idx]

    def move_joint(self, new_position):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = PANDA_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = new_position
        point.time_from_start.sec = 2  # Adjust as needed
        trajectory.points = [point]

        goal_msg.trajectory = trajectory
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    thread = Thread(target=rclpy.spin, args=(joint_controller,))
    thread.start()

    while rclpy.ok():
        key = input('Enter cmd')
        
        trans, rot = joint_controller.get_cur_fk()
        if trans is None or rot is None:
            print('Failed to get fk')
            continue
        
        print('Cur trans: {}, rot {}'.format(trans,rot))

        #trans[1] += TRANS_DEL
        
        
        
        euler_rot = R.from_quat(rot).as_euler('xyz', degrees=False)
        if key == 'w':
            trans[0] += TRANS_DEL
        elif key == 's':
            trans[0] -= TRANS_DEL
        elif key == 'a':
            trans[1] += TRANS_DEL
        elif key == 'd':
            trans[1] -= TRANS_DEL
        elif key == 'q':
            trans[2] += TRANS_DEL
        elif key == 'e':
            trans[2] -= TRANS_DEL
        elif key == 'W':
            euler_rot[0] += ROT_DEL
        elif key == 'S':
            euler_rot[0] -= ROT_DEL
        elif key == 'A':
            euler_rot[1] += ROT_DEL
        elif key == 'D':
            euler_rot[1] -= ROT_DEL
        elif key == 'Q':
            euler_rot[2] += ROT_DEL
        elif key == 'E':
            euler_rot[2] -= ROT_DEL 
        else:
            print('Key not recognized')
            continue
        rot = R.from_euler('xyz', euler_rot, degrees=False).as_quat()             
        
        joint_cmd = joint_controller.get_ik(trans, rot)
        if joint_cmd is None:
            print('Failed to get ik')
            continue
        
        joint_controller.move_joint(joint_cmd.tolist())

        print('Cur joints: {}'.format(joint_controller.cur_joints))
        print('Cmd joints: {}'.format(joint_cmd))

    joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
