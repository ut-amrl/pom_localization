import rosbag
import argparse
import csv

def parseArgs():
    parser = argparse.ArgumentParser(description='Plot results.')
    # parser.add_argument('param_prefix', required=False, default="", help='Parameter/node prefix')
    parser.add_argument('--bag_file')
    parser.add_argument('--out_file_name')

    args = parser.parse_args()
    return args

if __name__ == "__main__":

    cmdLineArgs = parseArgs()
    bag_file_name = cmdLineArgs.bag_file
    out_file_name = cmdLineArgs.out_file_name

    bag = rosbag.Bag(bag_file_name)
    details = []
    details.append(["waypoint_id", "seconds", "nanoseconds"])

    for topic, msg, t in bag.read_messages(topics=['/set_nav_target']):
        print(msg)
        details.append([msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs])
        print(details)

    with open(out_file_name, 'w') as csvfile:
        csv_writer = csv.writer(csvfile, delimiter=',',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for detail_entry in details:
            csv_writer.writerow(detail_entry)
    bag.close()