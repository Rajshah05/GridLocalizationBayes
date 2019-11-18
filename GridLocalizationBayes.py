#!/usr/bin/env python
import numpy as np
import rospkg
import roslib
import rospy
import rosbag

import numpy as np
import sys
from std_msgs.msg import Int32, String
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from markers_example import display_cube_list, display_line_list


rospack = rospkg.RosPack()

path = rospack.get_path('ros_pa3')

grid_bin_trans = 20
grid_bin_angle = 10
totalAngle = 360
totalTrans = 700

th = 0.1

cube_cor = np.array([
    (0, 0),
    (125, 525),
    (125, 325),
    (125, 125),
    (425, 125),
    (425, 325),
    (425, 525),
])

pub_line_list = rospy.Publisher('line_list', Marker, queue_size=1)
pub_cube_list = rospy.Publisher('cube_list', Marker, queue_size=1)
line_list = []
pose = np.zeros((totalTrans / grid_bin_trans, totalTrans /
                 grid_bin_trans, totalAngle / grid_bin_angle))
pose[12 - 1, 28 - 1, 3 - 1] = 1
temp_pose = np.zeros((totalTrans / grid_bin_trans,
                      totalTrans / grid_bin_trans, totalAngle / grid_bin_angle))
nameHandle = open(path + "/estimation.txt", "w")


def discrete_to_cont(i, j, k):
    x = i * grid_bin_trans + grid_bin_trans / 2.0
    y = j * grid_bin_trans + grid_bin_trans / 2.0
    theta = -180 + k * grid_bin_angle + grid_bin_angle / 2.0
    return theta, x, y


def probability(x, mean, sigma):
    val = (1.0 / (np.sqrt(2 * np.pi) * sigma)) * \
        np.power(np.e, -1.0 * (((x - mean) ** 2) / (2.0 * sigma ** 2)))
    return val


def odometry(i, j, k, i_t, j_t, k_t):
    rot1_1, trans1_x, trans1_y = discrete_to_cont(i, j, k)
    rot2_1, trans2_x, trans2_y = discrete_to_cont(i_t, j_t, k_t)
    trans = np.sqrt((trans1_x - trans2_x) ** 2 + (trans1_y - trans2_y) ** 2)
    slope = np.degrees(np.arctan2(trans1_y - trans2_y, trans1_x - trans2_x))

    rot2 = slope - rot2_1
    rot1 = rot1_1 - slope
    if rot1 > 180:
        rot1 = rot1 - 360
    elif rot1 < -180:
        rot1 = rot1 + 360
    if rot2 > 180:
        rot2 = rot2 - 360
    elif rot2 < -180:
        rot2 = rot2 + 360
    return rot2, trans, rot1


def observation_param(i, j, k, tagnum):
    global cube_cor
    rot, trans_x, trans_y = discrete_to_cont(i, j, k)
    trans = np.sqrt(
        (trans_x - cube_cor[tagnum, 0]) ** 2 + (trans_y - cube_cor[tagnum, 1]) ** 2)
    slope1 = np.degrees(np.arctan2(
        cube_cor[tagnum, 1] - trans_y, cube_cor[tagnum, 0] - trans_x))
    rot1 = slope1 - rot
    if rot1 > 180:
        rot1 = rot1 - 360
    elif rot1 < -180:
        rot1 = rot1 + 360
    return rot1, trans


def motion_model(rot1, trans, rot2):
    global pose, temp_pose, grid_bin_trans, grid_bin_angle, th
    temp_pose = pose
    pose = np.copy(temp_pose)
    total_prob = 0
    for i_t in range(pose.shape[0]):
        for j_t in range(pose.shape[1]):
            for k_t in range(pose.shape[2]):
                if temp_pose[i_t, j_t, k_t] < th:
                    continue
                for i in range(pose.shape[0]):
                    for j in range(pose.shape[1]):
                        for k in range(pose.shape[2]):
                            rot1_tmp, trans_tmp, rot2_tmp = odometry(
                                i, j, k, i_t, j_t, k_t)
                            rot1_prob = probability(
                                rot1_tmp, rot1, (grid_bin_angle / 2.0)+3)
                            trans_prob = probability(
                                trans_tmp, trans, (grid_bin_trans / 2.0)+3)
                            rot2_prob = probability(
                                rot2_tmp, rot2, (grid_bin_angle / 2.0)+3)
                            val = temp_pose[i_t, j_t, k_t] * \
                                trans_prob * rot1_prob * rot2_prob
                            pose[i, j, k] = pose[i, j, k] + val
                            total_prob = total_prob + val

    pose /= total_prob
    index = np.argmax(pose)
    index_angle = index % pose.shape[2]
    index = index / pose.shape[2]
    index_y = index % pose.shape[1]
    index = index / pose.shape[1]
    index_x = index % pose.shape[0]

    index_angle, x, y = discrete_to_cont(index_x, index_y, index_angle)

    line_list.append((x / 100.0, y / 100.0))

    nameHandle.write('P: ({} {} {})\n'.format(
        x / 100.0, y / 100.0, index_angle))
    display_cube_list(cube_cor, pub_cube_list)


def observation_model(tagnum, trans, rot):
    global pose, temp_pose, th, grid_bin_trans, nameHandle
    temp_pose = pose
    pose = np.copy(temp_pose)
    total_prob = 0
    for i in range(pose.shape[0]):
        for j in range(pose.shape[1]):
            for k in range(pose.shape[2]):
                rot_tmp, trans_tmp = observation_param(i, j, k, tagnum + 1)
                rot_prob = probability(rot_tmp, rot, grid_bin_angle / 2.0)
                trans_prob = probability(trans_tmp, trans, grid_bin_trans / 2.0)
                val = temp_pose[i, j, k] * trans_prob * rot_prob
                pose[i, j, k] = val
                total_prob = total_prob + val

    pose /= total_prob
    index = np.argmax(pose)
    index_angle = index % pose.shape[2]
    index = index / pose.shape[2]
    index_y = index % pose.shape[1]
    index = index / pose.shape[1]
    index_x = index % pose.shape[0]

    index_angle, x, y = discrete_to_cont(index_x, index_y, index_angle)

    line_list.append((x / 100.0, y / 100.0))

    nameHandle.write('U: ({} {} {})\n'.format(
        x / 100.0, y / 100.0, index_angle))

    display_cube_list(cube_cor, pub_cube_list)
    display_line_list(line_list, pub_line_list)


def merged():
    bag = rosbag.Bag(path + '/grid.bag')
    for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
        if topic == 'Movements':
            rot1, trans, rot2 = msg.rotation1, msg.translation, msg.rotation2
            rot1 = np.degrees((euler_from_quaternion(
                [rot1.x, rot1.y, rot1.z, rot1.w]))[2])
            rot2 = np.degrees((euler_from_quaternion(
                [rot2.x, rot2.y, rot2.z, rot2.w]))[2])
            motion_model(rot1, trans * 100, rot2)
        else:
            dist = msg.range * 100
            rot = msg.bearing
            rot = np.degrees((euler_from_quaternion(
                [rot.x, rot.y, rot.z, rot.w]))[2])
            observation_model(msg.tagNum, dist, rot)

    bag.close()


rospy.init_node('raj')
merged()
while not rospy.is_shutdown():
    display_cube_list(cube_cor, pub_cube_list)
