#!/usr/bin/env python3

import argparse
from pathlib import Path
import os
import shutil
import subprocess
import time

import qgc_to_gz
import clean
import file_checker

def process_params_file(params_file, output_file):
    """
    Reads the params_file, extracts the parameter name and value,
    and writes the formatted output to the initd_file.
    """
    with open(params_file, 'r') as pf, open(output_file, 'a') as of:
        for line in pf:
            if line[0] == '#':
                continue
            parts = line.strip().split('\t')
            if len(parts) < 2:
                continue
            param_name = parts[2]
            param_value = float(parts[3])
            of.write(f"param set-default {param_name} {param_value}\n")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--param_file", required=True, help="Path to input params file from QGroundControl.")
    parser.add_argument("--romfs_file", required=True, help="Path to ROMFS file to be used as  template.")
    parser.add_argument("--PX4_path", required=True, help="Path to PX4 directory.")
    inputs = parser.parse_args()

    params_path = Path(inputs.param_file)
    romfs_path = Path(inputs.romfs_file)
    [final_romfs_file, final_romfs_path] = qgc_to_gz.main(params_path, romfs_path)

    """
    airframes_path = ""
    romfs_path_parts = str(romfs_path).split("/")
    for i in range(len(romfs_path_parts)-1):
        airframes_path += romfs_path_parts[i] + '/'

    #check if the romfs file exist and if not create a new file with a incremented number
    final_romfs_file = ""
    biggest_numb = 22000
    for (_, _, files) in os.walk(airframes_path):
        for file in files:
            if "_gz_" not in file: #ignore non gazebo garden simulations
                continue
            file_parts = file.split("_gz_")
            file_name = file_parts[1]
            if int(file_parts[0]) > biggest_numb:
                biggest_numb = int(file_parts[0])
            if vehicle_name == file_name:
                final_romfs_file = file
        break
    #if the file did not exit previously then created and added it to the Cmake List
    if final_romfs_file == "":
        final_romfs_file = str(biggest_numb+1) + "_gz_" + vehicle_name
        with open(airframes_path + "CMakeLists.txt", "r") as cf:
            lines = cf.readlines()

        for i in range(len(lines)):
            if lines[i][0] == ")":
                lines.insert(i-1, '\t' + final_romfs_file +'\n') 
                break
        
        # Write back to the file
        with open(airframes_path + "CMakeLists.txt", "w") as cf:
            cf.writelines(lines)

    #check for files of a previous code execution and remove them
    if Path(f"{newpath}/{params_file}").is_file():
        shutil.move(f"{newpath}/{params_file}", f"./{params_file}")
    if Path(f"{newpath}/{final_romfs_file}").is_file():
            os.remove(f"{newpath}/{final_romfs_file}")
    if Path(f"{newpath}/errors.txt").is_file():
            os.remove(f"{newpath}/errors.txt")
    
    #create the folder and check files
    if not os.path.exists(vehicle_name):
        os.makedirs(vehicle_name)
    if not params_path.is_file():
        raise ValueError("Error: --param_file must be a valid file path.")
    if not romfs_path.is_file():
        raise ValueError("Error: --romfs_file must be a valid file path.")
    
    with open(romfs_path, 'r') as tf, open(final_romfs_file, 'w') as ff:
        shutil.copyfileobj(tf, ff)

    #move files to the folder
    shutil.move(final_romfs_file, f"{newpath}/")
    shutil.move(params_file, f"{newpath}/")
    #convert from the QGC designation to the gazebo designation
    process_params_file(f"{newpath}/{params_file}", f"{newpath}/{final_romfs_file}")

    #place the new ROMFS file on the ROMFS directory and run gazebo to find missing parameters
    final_romfs_path = airframes_path + final_romfs_file
    shutil.copy(f"{newpath}/{final_romfs_file}", final_romfs_path)
    """

    params_file = os.path.basename(params_path)
    vehicle_name = params_file.split('.')[0]
    newpath = vehicle_name
    #Some parameters are not defined in Gazebo so a simulation is ran to find this [ERRORS] and store them in a file 
    px4_path = Path(inputs.PX4_path)
    try:
        subprocess.run(["bash", "./make_px4_script.sh", vehicle_name, px4_path], check=True)
        print("Script executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error while executing the script: {e}")
        return -1
    
    #when the subprocess ends it creates a "end file"
    done_file = "./px4_done"
    # Wait for PX4 to complete
    while not os.path.exists(done_file):
        time.sleep(1)
    os.remove(done_file)

    errors_file = f"{newpath}/errors.txt"
    if Path(f"errors.txt").is_file():
        shutil.move("errors.txt", f"{newpath}/")
    else:
        errors_file = None
    
    #clean will comment out any parameters that were not recognized by Gazebo as well as parameters that are not compatible with simulated sensors
    clean.main(f"{newpath}/{final_romfs_file}", f"{newpath}/errors.txt")

    #pass the final version of the ROMFS file to the PX4 ROMFS folder
    shutil.copy(f"{newpath}/{final_romfs_file}", final_romfs_path)

    print(f"\033[92m[INFO]\033[0m: Generated the romfs file")

    file_checker.main(romfs_path)

if __name__ == '__main__':
    main()
