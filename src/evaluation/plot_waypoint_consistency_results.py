import argparse
import rospy
import math
import csv
import sys

import matplotlib.pyplot as plt

kNumTrajectoriesParamName = "waypoint_consistency_num_trajectories"
kTrajectorySpecificPrefix = "trajectory_"
kWaypointToNodeIdParamNameSuffix = "waypoint_to_node_id_file"
kTrajectoryOutputSuffix = "trajectory_output_file"

def readTrajectoryFromFile(file_name):
    with open(file_name, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        firstRow = True
        trajectory = []
        for row in spamreader:
            if (firstRow):
                firstRow = False
                continue
            pos = [float(row[1]), float(row[2])]
            angle = float(row[3])
            pose = [pos, angle]
            trajectory.append(pose)
        return trajectory


def readNodeAndWaypointPairingsFromFile(file_name):
    with open(file_name, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        firstRow = True
        waypointNodeIdPairs = []
        for row in spamreader:
            if (firstRow):
                firstRow = False
                continue
            waypoint = int(row[0])
            node = int(row[1])
            entry = [waypoint, node]
            waypointNodeIdPairs.append(entry)
        return waypointNodeIdPairs




def constructParamName(param_prefix, trajectory_num, param_suffix):
    return param_prefix + kTrajectorySpecificPrefix + str(trajectory_num) + "/" + param_suffix

def angleDiff(angle0, angle1):
    s = math.sin(angle0 - angle1)
    c = math.cos(angle0 - angle1)
    return math.atan2(s, c)

def angleDist(angle0, angle1):
    return abs(angleDiff(angle0, angle1))

def computeNorm(point1, point2):
    xDiff = point1[0] - point2[0]
    yDiff = point1[1] - point2[1]
    return math.sqrt((xDiff ** 2) + (yDiff ** 2))

def computePosDiff(point1, point2):
    xDiff = point1[0] - point2[0]
    yDiff = point1[1] - point2[1]
    return [xDiff, yDiff]

def findMeanRotation(waypoint_poses):
    mean_unit_vector = [0, 0]
    for pose in waypoint_poses:
        mean_unit_vector[0] = mean_unit_vector[0] + math.cos(pose[1])
        mean_unit_vector[1] = mean_unit_vector[1] + math.sin(pose[1])

    mean_unit_vector[0] = mean_unit_vector[0] / len(waypoint_poses)
    mean_unit_vector[1] = mean_unit_vector[1] / len(waypoint_poses)

    return math.atan2(mean_unit_vector[1], mean_unit_vector[0])

def parseArgs():
    parser = argparse.ArgumentParser(description='Plot results.')
    # parser.add_argument('param_prefix', required=False, default="", help='Parameter/node prefix')
    parser.add_argument('--param_prefix', default="")

    args = parser.parse_args()
    return args

def plotPosOffsetsForWaypoints(waypoints, posOffsetLists, maxDeviation):
    # plt.figure(1)
    fig, axs = plt.subplots(4, 8, sharex=True, sharey=True)
    for waypoint in waypoints:
        plt_x = (waypoint -1 ) % 4
        plt_y = int((waypoint -1 ) / 4)
        x = []
        y = []
        for posOffset in posOffsetLists[waypoint]:
            x.append(posOffset[0])
            y.append(posOffset[1])
        axs[plt_x, plt_y].scatter(x, y)
        axs[plt_x, plt_y].set_xlim([-1.1 * maxDeviation, 1.1 * maxDeviation])
        axs[plt_x, plt_y].set_ylim([-1.1 * maxDeviation, 1.1 * maxDeviation])
        axs[plt_x, plt_y].set_title("Waypoint " + str(waypoint))
    plt.show(block=False)

def plotPosOffsetForWaypoint(waypoint, posOffsets, maxDeviation):

    plt.figure(3)
    x = []
    y = []
    for posOffset in posOffsets:
        x.append(posOffset[0])
        y.append(posOffset[1])
    plt.scatter(x, y)
    plt.xlim([-1.1 * maxDeviation, 1.1 * maxDeviation])
    plt.ylim([-1.1 * maxDeviation, 1.1 * maxDeviation])
    plt.title("Waypoint " + str(waypoint))
    plt.show()

def plotAngleOffsetsForWaypoints(waypoints, angleOffsetLists):

    # plt.figure(2)
    fig, axs = plt.subplots(4, 8, sharex=True, sharey=True)
    for waypoint in waypoints:
        plt_x = (waypoint -1 ) % 4
        plt_y = int((waypoint -1 ) / 4)

        axs[plt_x, plt_y].hist(angleOffsetLists[waypoint])
        # axs[plt_x, plt_y].set_xlim([-1.1 * maxDeviation, 1.1 * maxDeviation])
        # axs[plt_x, plt_y].set_ylim([-1.1 * maxDeviation, 1.1 * maxDeviation])
        axs[plt_x, plt_y].set_title("Waypoint " + str(waypoint))
    # plt.show(block=False)
    plt.show()

def plotAngleOffsetForWaypoint(waypoint, angleOffsets):

    plt.figure(4)
    x = []
    y = []

    plt.hist(angleOffsets)
    # plt.scatter(x, y)
    # plt.xlim([-1.1 * maxDeviation, 1.1 * maxDeviation])
    # plt.ylim([-1.1 * maxDeviation, 1.1 * maxDeviation])
    plt.title("Waypoint " + str(waypoint))
    plt.show()

if __name__ == "__main__":

    param_prefix = parseArgs().param_prefix
    # param_prefix = ""
    node_prefix = param_prefix
    if (len(param_prefix) != 0):
        param_prefix = "/" + param_prefix + "/"
        node_prefix = node_prefix + "_"

    print("Param prefix: " + param_prefix)

    rospy.init_node(node_prefix + 'plot_waypoint_consistency_results')

    trajectory_outputs_by_trajectory_num = {}
    # std::unordered_map<int, std::vector<pose::Pose2d>> trajectory_outputs_by_trajectory_num;

    # // Outer key is the trajectory number
    # // Inner key is the waypoint number
    # // Inner value is the set of nodes that correspond to the waypoint
    # std::unordered_map<int, std::unordered_map<uint64_t, std::unordered_set<uint64_t>>> waypoints_to_node_id_by_trajectory_num;
    waypoints_to_node_id_by_trajectory_num = {}
    num_trajectories = rospy.get_param(param_prefix + kNumTrajectoriesParamName)

    min_waypoint_id = sys.maxsize
    max_waypoint_id = 1

    if (num_trajectories <= 0):
        print("Trajectory count must be a positive number")
        exit()

    print("Num trajectories " + str(num_trajectories))


#     std::vector<std::vector<pose::Pose2d>> waypoints_list;
#     std::vector<std::vector<pose::Pose2d>> trajectories_list;
    for i in range(num_trajectories):
        trajectory_file_param_name = constructParamName(param_prefix, i, kTrajectoryOutputSuffix)
        waypoint_to_node_id_file_param_name = constructParamName(param_prefix, i, kWaypointToNodeIdParamNameSuffix)

        trajectory_file_name = rospy.get_param(trajectory_file_param_name)
        waypoints_to_nodes_file_name = rospy.get_param(waypoint_to_node_id_file_param_name)

        trajectory_estimate = readTrajectoryFromFile(trajectory_file_name)
        trajectory_outputs_by_trajectory_num[i] = trajectory_estimate

        waypoints_and_node_ids_raw = readNodeAndWaypointPairingsFromFile(waypoints_to_nodes_file_name)
        waypoints_and_node_ids_for_traj = {}

        for waypoint_and_node_id in waypoints_and_node_ids_raw:
            waypoint = waypoint_and_node_id[0]
            node_id = waypoint_and_node_id[1]

            nodes_for_waypoint = []

            if (waypoint in waypoints_and_node_ids_for_traj.keys()):
                nodes_for_waypoint = list(waypoints_and_node_ids_for_traj[waypoint])

            nodes_for_waypoint.append(node_id)
            waypoints_and_node_ids_for_traj[waypoint] = set(nodes_for_waypoint)

            if (waypoint < min_waypoint_id):
                min_waypoint_id = waypoint
            if (waypoint > max_waypoint_id):
                max_waypoint_id = waypoint
        waypoints_to_node_id_by_trajectory_num[i] = waypoints_and_node_ids_for_traj

    poses_for_waypoints = {}
    for waypoint_id in range(min_waypoint_id, max_waypoint_id + 1):
        poses_for_waypoint = []
        for i in range(num_trajectories):
            nodes_for_waypoint_for_traj = waypoints_to_node_id_by_trajectory_num[i][waypoint_id]
            if (len(nodes_for_waypoint_for_traj) == 0):
                continue

            trajectory = trajectory_outputs_by_trajectory_num[i]
            for node_id_for_waypoint in nodes_for_waypoint_for_traj:
                poses_for_waypoint.append(trajectory[node_id_for_waypoint])

        if (len(poses_for_waypoint) != 0):
            poses_for_waypoints[waypoint_id] = poses_for_waypoint

    transl_deviation = {}
    mean_rotation_deviations = {}

    position_offsets_for_waypoints = {}
    angle_offsets_for_waypoints = {}

    max_deviation = 0

    for waypoint, poses_for_curr_waypoint in poses_for_waypoints.items():
        if (len(poses_for_curr_waypoint) <= 1):
            transl_deviation[waypoint] = 0
            continue

        transl_centroid = [0, 0]
        for pose_for_waypoint in poses_for_curr_waypoint:
            transl_centroid[0] = transl_centroid[0] + pose_for_waypoint[0][0]
            transl_centroid[1] = transl_centroid[1] + pose_for_waypoint[0][1]

        transl_centroid[0] = transl_centroid[0] / len(poses_for_curr_waypoint)
        transl_centroid[1] = transl_centroid[1] / len(poses_for_curr_waypoint)
        mean_rotation = findMeanRotation(poses_for_curr_waypoint)

        average_deviation_from_centroid = 0
        mean_rotation_deviation = 0

        position_offsets_for_waypoint = []
        angle_offsets_for_waypoint = []
        for pose_for_waypoint in poses_for_curr_waypoint:
            average_deviation_from_centroid += computeNorm(pose_for_waypoint[0], transl_centroid)

            print("Deviation for waypoint " + str(waypoint) + ": " + str(computeNorm(pose_for_waypoint[0], transl_centroid)))
            posDiff = computePosDiff(pose_for_waypoint[0], transl_centroid)
            print(posDiff)
            position_offsets_for_waypoint.append(posDiff)
            if (posDiff[0] > max_deviation):
                max_deviation = posDiff[0]
            if (posDiff[1] > max_deviation):
                max_deviation = posDiff[1]
            # deviations_from_mean_rotation.emplace_back(math_util::AngleDiff(pose_for_waypoint.second, mean_rotation));
            angleDeviation = angleDist(pose_for_waypoint[1], mean_rotation)
            mean_rotation_deviation += angleDeviation
            angle_offsets_for_waypoint.append(angleDeviation)

        average_deviation_from_centroid = average_deviation_from_centroid / (len(poses_for_curr_waypoint) - 1)
        transl_deviation[waypoint] = average_deviation_from_centroid
        mean_rotation_deviation = mean_rotation_deviation / (len(poses_for_curr_waypoint) - 1)

        mean_rotation_deviations[waypoint] = mean_rotation_deviation

        angle_offsets_for_waypoints[waypoint] = angle_offsets_for_waypoint
        position_offsets_for_waypoints[waypoint] = position_offsets_for_waypoint

    plotPosOffsetsForWaypoints(range(min_waypoint_id, max_waypoint_id + 1), position_offsets_for_waypoints, max_deviation)
    plotAngleOffsetsForWaypoints(range(min_waypoint_id, max_waypoint_id + 1), angle_offsets_for_waypoints)

    for i in range(min_waypoint_id, max_waypoint_id +1):
        print("Waypoint: " + str(i) + ", Transl Deviation: " + str(transl_deviation[i]) + ", angle deviation " + str(mean_rotation_deviations[i]))
        # print(position_offsets_for_waypoints[i])
        # plotPosOffsetForWaypoint(i, position_offsets_for_waypoints[i], max_deviation)
        # plotAngleOffsetForWaypoint(i, angle_offsets_for_waypoints[i])



