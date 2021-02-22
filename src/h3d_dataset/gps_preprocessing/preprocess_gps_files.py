
from geonav_conversions import *
import os

def getLatLongFromLine(line_in_gps_file):
    entries_in_line = line_in_gps_file.split(",")
    # Abs lat and long are the second to last and last entries in each line, respectively
    lat_str = entries_in_line[-2]
    long_str = entries_in_line[-1]
    return float(lat_str), float(long_str)


def generateAugmentedGpsFile(scenario_dir, original_gps_file, output_dir, output_file_prefix, origin_lat, origin_long):
    output_filename = output_dir + output_file_prefix + original_gps_file

    file_lines = getLinesFromFile(scenario_dir + original_gps_file)
    first_line = True

    with open(output_filename, "w") as output_file:

        for line in file_lines:
            updated_line = line.rstrip('\n')
            if (first_line):
                first_line = False
                updated_line += ",OriginLat,OriginLong,RelX,RelY\n"
            else:
                curr_lat, curr_long = getLatLongFromLine(line)
                x, y = ll2xy(curr_lat, curr_long, origin_lat, origin_long)
                updated_line += ","
                updated_line += str(origin_lat)
                updated_line += ","
                updated_line += str(origin_long)
                updated_line += ","
                updated_line += str(x)
                updated_line += ","
                updated_line += str(y)
                updated_line += "\n"
            output_file.write(updated_line)

def getLinesFromFile(gps_filename):
    with open(gps_filename) as gps_file:
        lines = []
        for line in gps_file:
            lines.append(line)
        return lines


def getGpsFilenamesInOrder(scenario_dir):
    files_in_dir = [f for f in os.listdir(scenario_dir)]
    gps_files = []
    for filename in files_in_dir:
        if (filename.startswith("gps_") and filename.endswith(".csv")):
            gps_files.append(filename)
    gps_files.sort()
    return gps_files

def constructScenarioDirPath(dataset_dir, scenario_num):
    # Assumes dataset dir has the / at the end
    return dataset_dir + "scenario_" + scenario_num + "/"

def makeOutputDirIfNotExists(output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

if __name__ == "__main__":
    h3d_dataset_dir = "/home/amanda/datasets/h3d/icra_benchmark_20200103_with_odom/"

    scenario_num = "002"
    output_base_dir = "/home/amanda/datasets/h3d/icra_benchmark_20200103_with_odom/augmentation/"

    altered_gps_prefix = "augmented_"

    origin_lat = None # Change to actual value if don't want to use the first GPS coord as origin
    origin_long = None # Change to actual value if don't want to use the first GPS coord as origin

    full_scenario_dir = constructScenarioDirPath(h3d_dataset_dir, scenario_num)
    full_output_dir = constructScenarioDirPath(output_base_dir, scenario_num)

    ordered_gps_filenames = getGpsFilenamesInOrder(full_scenario_dir)

    makeOutputDirIfNotExists(full_output_dir)

    if ((origin_lat == None) or (origin_long == None)):
        # Read them from first file in scenario num
        gps_file_lines = getLinesFromFile(full_scenario_dir + ordered_gps_filenames[0])
        origin_lat, origin_long = getLatLongFromLine(gps_file_lines[1])

    for orig_filename in ordered_gps_filenames:
        generateAugmentedGpsFile(full_scenario_dir, orig_filename, full_output_dir, altered_gps_prefix, origin_lat, origin_long)






