import argparse
import rospy
import math
import csv
import sys

import matplotlib.pyplot as plt
import numpy as np

kNumTrajectoriesParamName = "waypoint_consistency_num_trajectories"
kTrajectorySpecificPrefix = "trajectory_"
kWaypointToNodeIdParamNameSuffix = "waypoint_to_node_id_file"
kTrajectoryOutputSuffix = "trajectory_output_file"
kNumComparisonApproachesParamName = "num_comparison_approaches"
kComparisonApproachLabelSuffix = "approach_label"
kComparisonApproachSpecificPrefix="comparison_approach_"
kMaxXAxisBoundsMultiplier = 1.2

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

def constructTrajectorySpecificParamName(param_prefix, trajectory_num, param_suffix):
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

def getCDFData(dataset, num_bins):

    # getting data of the histogram
    count, bins_count = np.histogram(dataset, bins=num_bins)

    # finding the PDF of the histogram using count values

    pdf = count / sum(count)


    # using numpy np.cumsum to calculate the CDF
    # We can also find using the PDF values by looping and adding
    cdf = np.cumsum(pdf)
    cdf = np.insert(cdf, 0, 0)

    max_val = np.amax(dataset)

    return (cdf, bins_count, max_val)

def plotCDF(dataset_primary_approach, dataset_comparison_approaches, title, bins=40):

    comparison_approach_summary_max = 0

    # getting data of the histogram
    for comparison_label, comparison_dataset in dataset_comparison_approaches.items():
        approach_cdf, bins_count, comparison_approach_max = getCDFData(comparison_dataset, bins)
        comparison_approach_summary_max = max(comparison_approach_max, comparison_approach_summary_max)
        plt.plot(bins_count, approach_cdf, label=comparison_label)

    primary_approach_cdf, bins_count, primary_approach_max = getCDFData(dataset_primary_approach, bins)
    plt.plot(bins_count, primary_approach_cdf, label="movable object localization")

    if (len(dataset_comparison_approaches) != 0):
        # if (primary_approach_max > comparison_approach_summary_max):
        x_lim = primary_approach_max
        # else:
        #     x_lim = min(primary_approach_max * kMaxXAxisBoundsMultiplier, comparison_approach_summary_max)
        plt.xlim(0, x_lim)
        plt.legend()
    plt.ylim(0, 1)
    plt.title(title)

    plt.show()

def getDeviationsFromCentroid(poses_by_waypoint):
    # For each waypoint, get centroid and find distance of each pose for the waypoint from the centroid
    deviations_from_centroid = []
    for waypoint, poses_for_curr_waypoint in poses_by_waypoint.items():
        if (len(poses_for_curr_waypoint) <= 1):
            continue

        transl_centroid = [0, 0]
        for pose_for_waypoint in poses_for_curr_waypoint:
            transl_centroid[0] = transl_centroid[0] + pose_for_waypoint[0][0]
            transl_centroid[1] = transl_centroid[1] + pose_for_waypoint[0][1]

        transl_centroid[0] = transl_centroid[0] / len(poses_for_curr_waypoint)
        transl_centroid[1] = transl_centroid[1] / len(poses_for_curr_waypoint)

        for pose_for_waypoint in poses_for_curr_waypoint:
            deviations_from_centroid.append(computeNorm(pose_for_waypoint[0], transl_centroid))
    return deviations_from_centroid

def plotWaypointDistanceConsistencyCDF(poses_by_waypoint_primary_approach, poses_by_waypoint_comparison_approaches):
    # For each waypoint, get centroid and find distance of each pose for the waypoint from the centroid
    deviations_from_centroid_primary_approach = getDeviationsFromCentroid(poses_by_waypoint_primary_approach)
    deviations_from_centroid_comparison_approaches = {}
    for comparison_approach_label, poses_by_waypoint in poses_by_waypoint_comparison_approaches.items():
        deviations_from_centroid_comparison_approaches[comparison_approach_label] = getDeviationsFromCentroid(poses_by_waypoint)

    # Create CDF for centroid distance series
    plotCDF(deviations_from_centroid_primary_approach, deviations_from_centroid_comparison_approaches, "CDF of Position Deviation from Waypoint Estimate Centroid")

def getAngleDeviations(poses_by_waypoint):
    angleDeviations = []

    # For each waypoint, get mean orientation and deviation from mean orientation for each assocciated pose
    for waypoint, poses_for_curr_waypoint in poses_by_waypoint.items():
        if (len(poses_for_curr_waypoint) <= 1):
            continue

        mean_rotation = findMeanRotation(poses_for_curr_waypoint)

        for pose_for_waypoint in poses_for_curr_waypoint:
            print(pose_for_waypoint)
            angleDeviation = angleDist(pose_for_waypoint[1], mean_rotation)
            angleDeviations.append(angleDeviation)
    return angleDeviations

def plotWaypointOrientationConsistencyCDF(poses_by_waypoint_primary_approach, poses_by_waypoint_comparison_approaches):
    angle_deviations_primary_approach = getAngleDeviations(poses_by_waypoint_primary_approach)
    angle_deviations_comparison_approaches = {}
    for comparison_approach_label, poses_by_waypoint in poses_by_waypoint_comparison_approaches.items():
        angle_deviations_comparison_approaches[comparison_approach_label] = getAngleDeviations(poses_by_waypoint)
    plotCDF(angle_deviations_primary_approach, angle_deviations_comparison_approaches, "CDF of Orientation Estimate Deviation from Mean Waypoint Orientation")

def getPosesForWaypoints(approach_namespace, num_traj):
    min_waypoint_id = sys.maxsize
    max_waypoint_id = 1

    for i in range(num_traj):
        trajectory_file_param_name = constructTrajectorySpecificParamName(approach_namespace, i, kTrajectoryOutputSuffix)
        waypoint_to_node_id_file_param_name = constructTrajectorySpecificParamName(approach_namespace, i, kWaypointToNodeIdParamNameSuffix)

        trajectory_file_name = rospy.get_param(trajectory_file_param_name)
        waypoints_to_nodes_file_name = rospy.get_param(waypoint_to_node_id_file_param_name)

        trajectory_estimate = readTrajectoryFromFile(trajectory_file_name)
        primary_approach_trajectory_outputs_by_trajectory_num[i] = trajectory_estimate

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
        for i in range(num_traj):
            nodes_for_waypoint_for_traj = waypoints_to_node_id_by_trajectory_num[i][waypoint_id]
            if (len(nodes_for_waypoint_for_traj) == 0):
                continue

            trajectory = primary_approach_trajectory_outputs_by_trajectory_num[i]
            for node_id_for_waypoint in nodes_for_waypoint_for_traj:
                poses_for_waypoint.append(trajectory[node_id_for_waypoint])

        if (len(poses_for_waypoint) != 0):
            poses_for_waypoints[waypoint_id] = poses_for_waypoint

    return poses_for_waypoints

if __name__ == "__main__":

    cmdLineArgs = parseArgs()
    param_prefix = cmdLineArgs.param_prefix
    # param_prefix = ""
    node_prefix = param_prefix
    if (len(param_prefix) != 0):
        param_prefix = "/" + param_prefix + "/"
        node_prefix = node_prefix + "_"

    print("Param prefix: " + param_prefix)

    rospy.init_node(node_prefix + 'plot_waypoint_consistency_results')

    primary_approach_trajectory_outputs_by_trajectory_num = {}

    # // Outer key is the trajectory number
    # // Inner key is the waypoint number
    # // Inner value is the set of nodes that correspond to the waypoint
    # std::unordered_map<int, std::unordered_map<uint64_t, std::unordered_set<uint64_t>>> waypoints_to_node_id_by_trajectory_num;
    waypoints_to_node_id_by_trajectory_num = {}
    num_trajectories = rospy.get_param(param_prefix + kNumTrajectoriesParamName)

    # min_waypoint_id = sys.maxsize
    # max_waypoint_id = 1

    if (num_trajectories <= 0):
        print("Trajectory count must be a positive number")
        exit()

    print("Num trajectories " + str(num_trajectories))

    poses_for_waypoints_by_comparison_approach = {}
    poses_for_main_approach = getPosesForWaypoints(param_prefix, num_trajectories)

    num_comparison_approaches = rospy.get_param(param_prefix + kNumComparisonApproachesParamName)
    print("Num comparison approaches " + str(num_comparison_approaches))
    for i in range(num_comparison_approaches):
        approach_specific_namespace = param_prefix + kComparisonApproachSpecificPrefix + str(i) + "/"
        approach_label = rospy.get_param(approach_specific_namespace + kComparisonApproachLabelSuffix)
        print("Getting poses for waypoints for label " + approach_label)
        poses_for_waypoints_by_comparison_approach[approach_label] = getPosesForWaypoints(approach_specific_namespace, num_trajectories)

    plotWaypointDistanceConsistencyCDF(poses_for_main_approach, poses_for_waypoints_by_comparison_approach)
    plotWaypointOrientationConsistencyCDF(poses_for_main_approach, poses_for_waypoints_by_comparison_approach)

    #
    # for i in range(min_waypoint_id, max_waypoint_id +1):
    #     print("Waypoint: " + str(i) + ", Transl Deviation: " + str(transl_deviation[i]) + ", angle deviation " + str(mean_rotation_deviations[i]))
        # print(position_offsets_for_waypoints[i])
        # plotPosOffsetForWaypoint(i, position_offsets_for_waypoints[i], max_deviation)
        # plotAngleOffsetForWaypoint(i, angle_offsets_for_waypoints[i])



