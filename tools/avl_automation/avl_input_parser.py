#!/usr/bin/env

import argparse
import avl_out_parser
import os
import yaml
import subprocess
import shutil
from pathlib import Path

ctrl_surface_type_sdf = {
    "not_set": "not_set",
    "left_aileron": "aileron",
    "right_aileron": "aileron",
    "elevator": "elevator",
    "rudder": "rudder",
    "left_elevon": "elevon",
    "right_elevon": "elevon",
    "left_v-tail": "V-tail",
    "right_t-tail": "V-tail",
    "left_flap": "flap",
    "right_flap": "flap",
    "airbrake": "airbrake",
    "custom": "custom",
    "left_a-tail": "a-tail",
    "right_a-tail": "a-tail",
    "single_channel_aileron": "aileron",
    "steering_wheel": "steering_wheel",
    "left_spoiler": "spoiler",
    "right_spoiler": "spoiler"
}

ctrl_surface_type_initd = {
    "not_set": 0,
    "left_aileron": 1,
    "right_aileron": 2,
    "elevator": 3,
    "rudder": 4,
    "left_elevon": 5,
    "right_elevon": 6,
    "left_v-tail": 7,
    "right_t-tail": 8,
    "left_flap": 9,
    "right_flap": 10,
    "airbrake": 11,
    "custom": 12,
    "left_a-tail": 13,
    "right_a-tail": 14,
    "single_channel_aileron": 15,
    "steering_wheel": 16,
    "left_spoiler": 17,
    "right_spoiler": 18
}

"""
Writes the section definitions of a surface to an AVL file. Sections are defined using 3D coordinates, 
chord length, angle of incidence, and control surface parameters. Each section can define multiple control 
surface types (e.g., aileron, elevator, rudder), and at least two sections (left and right edges) must be defined 
for each surface.

Args:
    plane_name (str): The name of the aircraft or vehicle.
    control_surface (dict): Dictionary containing section details and control surface information.

Returns:
    None
"""
def write_avl_section_from_section(plane_name: (str), control_surface: (dict)):
	# TODO: Find out if elevons are defined in avl
	sections = control_surface['sections']
	with open(f'{plane_name}.avl','a') as avl_file:
		for section in sections:
			avl_file.write(f"\n#{section['name']}\n")
			avl_file.write(f"SECTION\n")
			avl_file.write("!Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n")
			avl_file.write(f"{section['position']['X']} "
						f"{section['position']['Y']} "
						f"{section['position']['Z']} "
						f"{section['chord']} "
						f"{section['ainc']} "
						f"{section['nspan']} "
						f"{section['sspace']} \n")
			
			# Define NACA airfoil shape.
			# For help picking an airfoil go to: http://airfoiltools.com/airfoil/naca4digit
			# NOTE: AVL can only use 4-digit NACA codes.
			if 'naca' in section:
				avl_file.write("NACA \n")
				avl_file.write(f"{section['naca']} \n")
			
			if "types" not in section:
				continue  # Skip this section if it does not have a type 
			
			for surface_type in section["types"]:
				if surface_type['type'] not in ctrl_surface_type_sdf:
					raise ValueError(f'\033[91m [Err] \033[0m The selected type is invalid. Available types are: {ctrl_surface_type_sdf}')
				type = ctrl_surface_type_sdf.get(surface_type['type'])
				match type:
					case 'aileron':
						avl_file.write("CONTROL \n")
						avl_file.write(f"aileron  {surface_type['Cgain']}  {surface_type['Xhinge']}  {surface_type['HingeVec']['X']}  {surface_type['HingeVec']['Y']}  {surface_type['HingeVec']['Z']}  {surface_type['SgnDup']} \n")

					case 'elevator':
						avl_file.write("CONTROL \n")
						avl_file.write(f"elevator  {surface_type['Cgain']}  {surface_type['Xhinge']}  {surface_type['HingeVec']['X']}  {surface_type['HingeVec']['Y']}  {surface_type['HingeVec']['Z']}  {surface_type['SgnDup']} \n")

					case 'rudder':
						avl_file.write("CONTROL \n")
						avl_file.write(f"rudder  {surface_type['Cgain']}  {surface_type['Xhinge']}  {surface_type['HingeVec']['X']}  {surface_type['HingeVec']['Y']}  {surface_type['HingeVec']['Z']}  {surface_type['SgnDup']} \n")
	avl_file.close()

"""
Processes control surface information by writing the relevant parameters for each surface type (aileron, 
elevator, rudder) to the AVL and associated files. This includes setting control surface angles, writing servo 
parameters to the SDF file, and servo configuration in the init.d file for PX4 simulation.

Args:
    plane_name (str): The name of the aircraft or vehicle.
    ctrl_surface (dict): Dictionary containing control surface sections and types.
    ctrl_surface_order (list): List of already processed control surface types to avoid repetition.
    file_location (str): Location of the final files (AVL, SDF, init.d).
    num_ctrl_surfaces (int): The total number of control surfaces on the aircraft.

Returns:
    None
"""
def process_control_surface_from_section(plane_name: str, ctrl_surface: dict, ctrl_surface_order: list, file_location: str, num_ctrl_surfaces: int):
	sections = ctrl_surface['sections']
	# Create a set to track processed control surfaces
	processed_types = set()
	# Iterate through each section surface type in the list
	with open(f'{plane_name}.avl','a') as avl_file:
		for section in sections:
			if 'types' not in section:
				continue  # Skip this section if it does not have a type
			
			for type_struct in section["types"]:
				if type_struct["type"] not in list(ctrl_surface_type_sdf.keys())[:]:
					raise ValueError(f"\033[91m [Err] \033[0mThe selected type \"{type_struct['type']}\" is invalid. Available types are: {list(ctrl_surface_type_sdf.keys())[:]}")
				sdf_type = ctrl_surface_type_sdf.get(type_struct["type"])
				initD_type = ctrl_surface_type_initd.get(type_struct["type"])
				if sdf_type in processed_types:
					continue
				elif sdf_type == 'aileron':
					avl_file.write("\nANGLE \n")
					avl_file.write(f"{ctrl_surface['angle']} \n")
					processed_types.add('aileron')
				elif sdf_type == 'elevator':
					avl_file.write("\nYDUPLICATE\n")
					avl_file.write("0.0\n\n")

				# Process and write the servo parameters to the .sdf file
				with open('./templates/servo_template.sdf', 'r') as source:
					content = source.read()

				# Replacing placeholders with actual values
				content = content.replace("<joint name='' type=''>",
					f"<joint name='servo_{len(ctrl_surface_order)}' type='revolute'>")
				content = content.replace("<parent></parent>", "<parent>base_link</parent>")
				content = content.replace("<child></child>", f"<child>{ctrl_surface['name']}</child>")
				content = content.replace("<pose></pose>", 
					f"<pose>{-ctrl_surface['translation']['Z'] - section['position']['Z']} "
					f"{ctrl_surface['translation']['Y'] + section['position']['Y']} "
					f"{ctrl_surface['translation']['X'] + section['position']['X']} 0 0 0</pose>")
				content = content.replace("<xyz></xyz>", 
					f"<xyz>{type_struct['HingeVec']['Z']} "
					f"{type_struct['HingeVec']['Y']} "
					f"{type_struct['HingeVec']['X']}</xyz>")
				content = content.replace("<joint_name></joint_name>", f"<joint_name>servo_{len(ctrl_surface_order)}</joint_name>")
				content = content.replace("<sub_topic></sub_topic>", f"<sub_topic>servo_{len(ctrl_surface_order)}</sub_topic>")

				# Append the modified content to the target .sdf file
				with open(file_location + f"{plane_name}.sdf", 'a') as target:
					target.write(content)
					target.close()

				# Process and write the parameters to the init.d-posix file
				with open(file_location + "init.d-posix.txt", 'r') as source:
					content = source.read()
					
					content = content.replace(f"param set-default CA_SV_CS_COUNT {num_ctrl_surfaces}",
						f"param set-default CA_SV_CS_COUNT {num_ctrl_surfaces}\n"
						f"param set-default CA_SV_CS{len(ctrl_surface_order)}_TYPE {initD_type}")
					source.close()

				# add the modified content to the target file
				with open(file_location + "init.d-posix.txt", 'w') as target:
					target.write(content)
					target.close()

				# append the content to the target file
				with open(file_location + "init.d-posix.txt", 'a') as target:
					#initalize the func as 1 and the servo as number 201
					target.write(f"\nparam set-default SIM_GZ_SV_FUNC{len(ctrl_surface_order)+1} {len(ctrl_surface_order) + 201}")
					target.close()
				
				ctrl_surface_order.append(sdf_type)
				processed_types.add(sdf_type)

"""
Writes section definitions of a control surface with a specific general type (aileron, elevator, rudder) to 
the AVL file. Each section includes coordinates, chord, angle of incidence, and control surface definitions. This 
function is tailored to handle one general control surface type consistently across all sections.

Args:
    plane_name (str): The name of the aircraft or vehicle.
    control_surface (dict): Dictionary containing details of sections and control surface types.

Returns:
    None
"""
def write_avl_surface_from_surface(plane_name: str, control_surface: dict):
	# TODO: Find out if elevons are defined in AVL
	sections = control_surface['sections']
	sdf_type = control_surface['type']

	if sdf_type not in ctrl_surface_type_sdf:
		raise ValueError(f'\033[91m [Err] \033[0m The selected type is invalid. Available types are: {list(ctrl_surface_type_sdf.keys())}')

	sdf_type_avl = ctrl_surface_type_sdf.get(sdf_type)

	with open(f'{plane_name}.avl', 'a') as avl_file:
		for section in sections:
			avl_file.write(f"\n#{section['name']}\n")
			avl_file.write(f"SECTION\n")
			avl_file.write("!Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n")
			avl_file.write(f"{section['position']['X']} "
			f"{section['position']['Y']} "
			f"{section['position']['Z']} "
			f"{section['chord']} "
			f"{section['ainc']} "
			f"{section['nspan']} "
			f"{section['sspace']} \n")

			# Define NACA airfoil shape.
			if 'naca' in section:
				avl_file.write("NACA \n")
				avl_file.write(f"{section['naca']} \n")

			# Write control surface based on type
			avl_file.write("CONTROL \n")

			match sdf_type_avl:
				case 'aileron':
					avl_file.write("aileron  1.0  0.0  0.0  0.0  0.0  -1 \n")
				case 'elevator':
					avl_file.write("elevator  1.0  0.0  0.0  0.0  0.0  1 \n")
				case 'rudder':
					avl_file.write("rudder  1.0  0.0  0.0  0.0  0.0  1 \n")	
		avl_file.close()

"""
Processes control surface parameters for surfaces with a specific general type (aileron, elevator, rudder). 
Writes corresponding data to the AVL file, SDF file, and init.d for PX4. This function handles all instances 
of a single general type across multiple sections.

Args:
    plane_name (str): The name of the aircraft or vehicle.
    ctrl_surface (dict): Dictionary containing control surface sections and types.
    ctrl_surface_order (list): List of already processed control surface types to avoid repetition.
    file_location (str): Location of the final files (AVL, SDF, init.d).
    num_ctrl_surfaces (int): The total number of control surfaces on the aircraft.

Returns:
    None
"""
def process_control_surface_from_surface(plane_name: str, ctrl_surface: dict, ctrl_surface_order: list, file_location: str, num_ctrl_surfaces: int):
	if ctrl_surface["type"] not in list(ctrl_surface_type_sdf.keys())[:]:
		raise ValueError(f"\033[91m [Err] \033[0m The selected type \"{ctrl_surface['type']}\" is invalid. Available types are: {list(ctrl_surface_type_sdf.keys())[:]}")
	sdf_type = ctrl_surface_type_sdf.get(ctrl_surface['type'])
	initD_type = ctrl_surface_type_initd.get(ctrl_surface["type"])
	sections = ctrl_surface['sections']
	# Create a set to track processed control surfaces
	processed_types = set()
	# Iterate through each section surface type in the list
	with open(f'{plane_name}.avl','a') as avl_file:
		for _ in sections:
			if sdf_type in processed_types:
				continue
			elif sdf_type == 'aileron':
				avl_file.write("\nANGLE \n")
				avl_file.write(f"{ctrl_surface['angle']} \n")
				processed_types.add('aileron')
			elif sdf_type == 'elevator':
				avl_file.write("\nYDUPLICATE\n")
				avl_file.write("0.0\n\n")

			
			# Process and write the parameters to the init.d-posix file
			with open(file_location + "init.d-posix.txt", 'r') as source:
				content = source.read()
				
				content = content.replace(f"param set-default CA_SV_CS_COUNT {num_ctrl_surfaces}",
					f"param set-default CA_SV_CS_COUNT {num_ctrl_surfaces}\n"
					f"param set-default CA_SV_CS{len(ctrl_surface_order)}_TYPE {initD_type}")
				source.close()

			# add the modified content to the target file
			with open(file_location + "init.d-posix.txt", 'w') as target:
				target.write(content)
				target.close()

			# append the content to the target file
			with open(file_location + "init.d-posix.txt", 'a') as target:
				#initalize the func as 1 and the servo as number 201
				target.write(f"\nparam set-default SIM_GZ_SV_FUNC{len(ctrl_surface_order)+1} {len(ctrl_surface_order) + 201}")
				target.close()
			
			ctrl_surface_order.append(sdf_type)
			processed_types.add(sdf_type)

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
	# This will find Avl on a users machine.
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

	airframes = ['cessna','standard_vtol','custom']
	plane_name = yaml_data['vehicle_name']
	frame_type = yaml_data['frame_type']
	if not frame_type in airframes:
		raise ValueError("\nThis is not a valid airframe, please choose a valid airframe. \n")

	# SPECIFY STALL PARAMETERS BASED ON AIRCRAFT TYPE (IF PROVIDED)
	if not os.path.exists(f'{savedir}/{plane_name}'):
		os.makedirs(f'{savedir}/{plane_name}')
	file_location = f'{savedir}/{plane_name}/'

	# Parameters that need to be provided:
	# General
	# - Reference Area (Sref)
	# - Wing span (Bref) (wing span squared / area = aspect ratio which is a required parameter for the sdf file)
	# - Reference point (X,Y,Zref) point at which moments and forces are calculated
	#Control Surface specific
	# - name
	# - type (select from options in the px4 paramer list of CA_SV_CS0_TYPE)
	# - nchord
	# - cspace
	# - nspanwise
	# - sspace
	# - angle (if an aileron)
	# - name (section)
	# - x,y,z 1. (section)
	# - chord 1. (section)
	# - ainc 1. (section)
	# - Nspan 1. (optional for section)
	# - sspace 1. (optional for section)
	# - types (optional if a type is given in control surface note if type is given in control surface that type will be used in all sections)
	# - Cgain 1.1 (section, type)
	# - Xhinge 1.1 (section, type)
	# - HingeVec 1.1 (section, type)
	# - SgnDup 1.1 (section, type)
	# - x,y,z 2. (section)
	# - chord 2. (section)
	# - ainc 2. (section)
	# - Nspan 2. (optional for section)
	# - sspace 2. (optional for section)

	# - Reference Chord (Cref) (= area/wing span)
	delineation = '!***************************************'
	sec_demark = '\n\n#--------------------------------------------------'
	num_ctrl_surfaces = 0
	ctrl_surface_order = []
	area = 0
	span = 0

	ref_pt_x = None
	ref_pt_y = None
	ref_pt_z = None

	# Future work: Provide some pre-worked frames for a Cessna and standard VTOL if there is a need for it
	match frame_type:

		case "custom":

			# These parameters are consistent across all models.
			# At the moment we do not use any symmetry axis for mirroring.
			with open(f'{plane_name}.avl','w') as avl_file:
				avl_file.write(f'{delineation} \n')
				avl_file.write(f'!{plane_name} input dataset \n')
				avl_file.write(f'{delineation} \n')
				avl_file.write(f'{plane_name} \n')
				avl_file.write('!Mach \n0.0 \n')
				avl_file.write('!IYsym    IZsym    Zsym \n')
				avl_file.write('0     0     0 \n')
				avl_file.close()

			# First define some model-specific parameters for custom models
			area = yaml_data["reference_area"]
			span = yaml_data["wing_span"]
			ref_pt_x = yaml_data["reference_point"]["X"]
			ref_pt_y = yaml_data["reference_point"]["Y"]
			ref_pt_z = yaml_data["reference_point"]["Z"]

			if(area != 0 and span != 0):
				ref_chord = float(area)/float(span)
			else:
				raise ValueError("Invalid reference chord value. Check area and wing span values.")

			# Write the gathered model-wide parameters into the .avl file
			with open(f'{plane_name}.avl','a') as avl_file:
				avl_file.write('!Sref    Cref    Bref \n')
				avl_file.write(f'{area}     {str(ref_chord)}     {span} \n')
				avl_file.write('!Xref    Yref    Zref \n')
				avl_file.write(f'{ref_pt_x}     {ref_pt_y}      {ref_pt_z} \n')
				avl_file.close()

			num_ctrl_surfaces = yaml_data["num_ctrl_surfaces"]

			#Set up parameters of the init.d-posix file
			with open("./templates/init.d-posix.txt", 'r') as source:
				content = source.read()
			
			# Replacing placeholders with actual values
			content = content.replace("# @name",
				f"# @name {plane_name}")
			content = content.replace(r"PX4_SIM_MODEL=${PX4_SIM_MODEL:=}",
				f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:= {plane_name}}}")
			content = content.replace("param set-default CA_SV_CS_COUNT",
				f"param set-default CA_SV_CS_COUNT {num_ctrl_surfaces}")

			with open(file_location + f"init.d-posix.txt", 'a') as target:
				target.writelines(content)
				target.close()
			
			for control_surface in yaml_data["control_surfaces"]:

				# Wings always need to be defined from left to right
				ctrl_surf_name = control_surface['name']

				nchord = control_surface["nchord"]
				cspace = control_surface["cspace"]
				nspanwise = control_surface["nspan"]
				sspace = control_surface["sspace"]

				#Translation of control surface, will move the whole surface to specified position
				tx = control_surface["translation"]["X"]
				ty = control_surface["translation"]["Y"]
				tz = control_surface["translation"]["Z"]

				# Write common part of this surface to .avl file
				with open(f'{plane_name}.avl','a') as avl_file:
					avl_file.write(sec_demark)
					avl_file.write("\nSURFACE \n")
					avl_file.write(f'{ctrl_surf_name} \n')
					avl_file.write("!Nchordwise     Cspace      Nspanwise       Sspace \n")
					avl_file.write(f'{nchord}       {cspace}        {nspanwise}     {sspace} \n')

					# Translate the surface to a particular position in space.
					avl_file.write("TRANSLATE \n")
					avl_file.write(f'{tx}    {ty}    {tz} \n')
					avl_file.close()

				#if a type is defined in the control surface that type will be used in all sections
				if 'type' in control_surface:
					process_control_surface_from_surface(plane_name, control_surface, ctrl_surface_order, file_location, num_ctrl_surfaces)
					write_avl_surface_from_surface(plane_name, control_surface)
				else:
					process_control_surface_from_section(plane_name, control_surface, ctrl_surface_order, file_location, num_ctrl_surfaces)
					write_avl_section_from_section(plane_name, control_surface)
				

				# Iterating over each defined section for the control surface. There need to be at least
				# two in order to define a left and right edge, but there is no upper limit.
				# CRITICAL: ALWAYS DEFINE YOUR SECTION FROM LEFT TO RIGHT
				print(f"\nPARAMETER DEFINITION FOR CONTROL SURFACE {control_surface['name']} COMPLETED \n")

   	# Calculation of Aspect Ratio (AR) and Mean Aerodynamic Chord (mac)
	AR = str((float(span)*float(span))/float(area))
	mac = str((2/3)*(float(area)/float(span)))

	# Call shell script that will pass the generated .avl file to AVL
	os.system(f'./process.sh {plane_name}')

	# Call main function of avl parse script to parse the generated AVL files.	
	avl_out_parser.main(plane_name,frame_type,AR,mac,ref_pt_x,ref_pt_y,ref_pt_z,num_ctrl_surfaces,area,ctrl_surface_order,inputs.avl_path)

	# Finally move all generated files to a new directory and show the generated geometry image:
	result = subprocess.run(['pwd'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

	if result.returncode == 0:
		# Save the output in a variable
		current_path = result.stdout.strip()

	# Run image plot from avl_automation directory.
	os.system(f'mv ./{plane_name}.* ./{plane_name}' )
	os.system(f'evince {current_path}/{plane_name}/{plane_name}.ps')

if __name__ == '__main__':
	main()
