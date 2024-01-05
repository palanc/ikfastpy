import numpy as np
import ikfastpy
import time

# Initialize kinematics for franka robot arm
franka_kin = ikfastpy.PyKinematics()
n_joints = franka_kin.getDOF()


joint_mins = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
joint_maxs = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
joint_angles = (joint_maxs-joint_mins)*np.random.random(7)+joint_mins

# Test forward kinematics: get end effector pose from joint angles
print("\nTesting forward kinematics:\n")
print("Joint angles:")
ee_pose = franka_kin.forward(joint_angles)
ee_pose = np.asarray(ee_pose).reshape(3,4) # 3x4 rigid transformation matrix
print("\nEnd effector pose:")
print(ee_pose)
print("\n-----------------------------")

# Test inverse kinematics: get joint angles from end effector pose
print("\nTesting inverse kinematics:\n")
start_time = time.time()

joint_configs = franka_kin.inverse(ee_pose.reshape(-1).tolist(),
                                    free_itr=10000,
                                    free_min=joint_mins[-1],
                                    free_max=joint_maxs[-1])
print('Eval time {}'.format(time.time()-start_time))
n_solutions = int(len(joint_configs)/n_joints)
print("%d solutions found:"%(n_solutions))
joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)

JOINT_WEIGHTS = np.array([1, 1, 1, 1, 1, 1, 1])
best_idx = np.argmin(np.sum(np.square(JOINT_WEIGHTS*(joint_configs-np.asarray(joint_angles))), axis=1))
print('Orig: {}'.format(np.asarray(joint_angles)))
print('Compute: {}'.format(joint_configs[best_idx]))
print('Diff Deg: {}'.format((180.0/np.pi)*np.abs(np.asarray(joint_angles)-joint_configs[best_idx])))
