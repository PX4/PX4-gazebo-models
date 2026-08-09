#!/usr/bin/env

import argparse
import avl_out_parser
import os
import yaml
import subprocess
import shutil
from pathlib import Path

"""
Read the provided yaml file and generate the corresponding .avl file that can be read into AVL as well as the init.d for the ROMFS.
Also calls AVL and the avl_out_parse.py file that generates the sdf plugin.

Args:
	yaml_file: Path to the input yaml file
	avl_path: Set the avl_path to provide a desired directory for where Avl should be located.

Return:
	None

"""

def main():
	user = os.environ.get('USER')
	# This will try to find Avl on a users machine.
	target_directory_path = None
	for root, dirs, _ in os.walk(f'/home/{user}/'):
		if "Avl" in dirs:
			target_directory_path = os.path.join(root, "Avl")

	if target_directory_path is None:
		raise FileNotFoundError("The 'Avl' directory was not found.")

	parent_directory_path = os.path.dirname(target_directory_path)
	filedir = f'{parent_directory_path}/'

	parser = argparse.ArgumentParser()
	parser.add_argument("--yaml_file", help="Path to input yaml file.")
	parser.add_argument("--avl_path", default=filedir, help="Provide an absolute AVL path. If this argument is passed, AVL will be moved there and the files will adjust their paths accordingly.")
	inputs = parser.parse_args()

	if inputs.yaml_file is None:
		raise ValueError("Error: --yaml_file is required. Please provide a valid path to the YAML file.")

	# If the user passes the avl_path argument then move Avl to that location:
	if inputs.avl_path != filedir:

		#Check if the directory is already there
		if os.path.exists(f'{inputs.avl_path}/Avl') and os.path.isdir(f'{inputs.avl_path}/Avl'):
			print("Avl is already at desired location")
		else:
			shutil.move(f'{filedir}Avl',inputs.avl_path)

		# Adjust paths to AVL in process.sh
		print("Adjusting paths")
		with open("./template_process.sh", "r") as file:
			all_lines = file.readlines()

		it = 0
		for line in all_lines:
			if "cp $DIR_PATH/$CUSTOM_MODEL.avl" in line:
				new_line = f'cp $DIR_PATH/$CUSTOM_MODEL.avl {inputs.avl_path}Avl/runs\n'
				all_lines[it] = new_line

			if "/Avl/runs/plot.ps $DIR_PATH/" in line:
				new_line =f'mv {inputs.avl_path}Avl/runs/plot.ps $DIR_PATH/\n'
				all_lines[it] = new_line

			if "cd" in line and "/Avl/runs" in line:
				new_line = f'cd {inputs.avl_path}Avl/runs\n'
				all_lines[it] = new_line
			it += 1

		with open("./process.sh", "w") as file:
			file.writelines(all_lines)
			
	#make the file executable
	os.chmod('./process.sh', 0o755)

	# Set current path for user
	curr_path = subprocess.run(['pwd'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
	if curr_path.returncode == 0:
        # Save the output in a variable
		savedir = curr_path.stdout.strip()

	#load the file from the --yaml_file or go to the created file from a previous run and reset it (plane name and yaml name must be the same)
	if Path(inputs.yaml_file).is_file():
		#source given by user
		with open(inputs.yaml_file,'r') as yaml_file:
			yaml_data = yaml.safe_load(yaml_file)
	else:
		#remove the files inside the folder form the Previous run (if they exist)
		try:
			file_name = inputs.yaml_file.split('/')[-1] #plane_example_0.yml
			directory = file_name.split('.yml')[0] #plane_example_0
			with open(savedir + '/' + directory + '/' + file_name ,'r') as yaml_file:
				yaml_data = yaml.safe_load(yaml_file)
			if Path(savedir + '/' + directory + '/' + directory + ".avl").is_file():
				os.remove(savedir + '/' + directory + '/' + directory + ".avl")
			if Path(savedir + '/' + directory + '/' + directory + ".ps").is_file():
				os.remove(savedir + '/' + directory + '/' + directory + ".ps")
			if Path(savedir + '/' + directory + '/' + directory + ".sdf").is_file():
				os.remove(savedir + '/' + directory + '/' + directory + ".sdf")
			if Path(savedir + '/' + directory + '/' + "init.d-posix.txt").is_file():
				os.remove(savedir + '/' + directory + '/' + "init.d-posix.txt")
			yaml_file.close()
		except:
			raise ValueError("\nError: given --yaml_file does not exist. Please provide a valid path to the YAML file.\n")

	# Call shell script that will pass the generated .avl file to AVL
	os.system(f'python3 avl_input_parser.py --yaml_file {inputs.yaml_file}')

if __name__ == '__main__':
	main()
