#!/usr/bin/env python3

import argparse
import subprocess
import os
import shutil

DEFAULT_DOWNLOAD_DIR = "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip"


def run(cmd):
    process_handle = subprocess.Popen(['bash', '-c', cmd], cwd='.')
    return process_handle

def main():
    parser = argparse.ArgumentParser(description='Gazebo simulation')

    parser.add_argument('--world', help='World to run in Gazebo', required=False, default="default")
    parser.add_argument('--gz_partition', help='Gazebo partition to run in', required=False)
    parser.add_argument('--gz_ip', help='Outgoing network interface to use for traffic',required=False)
    parser.add_argument('--interactive',help='Run in interactive mode', required=False, default=False, action='store_true')
    parser.add_argument('--model_download_source', help='Path to directory containing model files', required=False, default=DEFAULT_DOWNLOAD_DIR)
    parser.add_argument('--model_store', help='Path to model storage directory', required=False, default="~/.simulation-gazebo")
    parser.add_argument('--overwrite', help='Overwrite existing model directories', required=False, default=False, action='store_true')
    parser.add_argument('--dryrun', help='Test in dryrun. Do not launch gazebo', required=False, default=False, action='store_true')
    parser.add_argument('--headless', help='Run Gazebo without GUI', required=False, default=False, action='store_true')

    args = parser.parse_args()

    # Set up environment variables to look for models for simulation
    args.model_store = os.path.expanduser(args.model_store)

    # Check whether the model storage directory exists.
    if not os.path.exists(args.model_store):
        print("Making model storage directory")
        os.makedirs(args.model_store)


    model_count = int(subprocess.check_output(f'find {args.model_store} -type f | wc -l', shell=True, text=True))
    models_exist = True if model_count > 0 else False
    print(f"Found: {model_count} files in {args.model_store}")

    if models_exist and not args.overwrite:
        print("Models directory not empty. Overwrite not set. Not downloading models.")

    elif args.overwrite and models_exist:
        try:
            subdirectories = [os.path.join(args.model_store, d) for d in os.listdir(args.model_store) if os.path.isdir(os.path.join(args.model_store, d))]
            for directory in subdirectories:
                shutil.rmtree(directory)
            print("Overwrite set. Removed existing model subdirectories.")
        except:
            print("No models dir present, overwrite did not remove a directory.")


    if ((not models_exist) or args.overwrite):
        # Download model from model_download_source to model_store
        if args.interactive and not args.dryrun:

            input_a = input("Interactive mode set. Would you like to provide a custom download directory? (Y/N): ")
            if "y" in input_a.lower():
                args.model_download_source = input("Enter download path (specify zip file): ")
                print(f"Using custom download directory: {args.model_download_source}")

                if args.model_download_source.startswith("http"):
                    os.system(f'curl -L -o {args.model_store}/resources.zip {args.model_download_source}')
                else:
                    print(f'Using local file system: {args.model_download_source}')
                    os.system(f'cp -r {args.model_download_source} {args.model_store}/resources.zip')

            else:
                print(f"Using default download directory: {DEFAULT_DOWNLOAD_DIR}")
                os.system(f'curl -L -o {args.model_store}/resources.zip {DEFAULT_DOWNLOAD_DIR}')

        if not args.interactive or args.dryrun:
            print('Downloading from Github...')
            os.system(f'curl -L -o {args.model_store}/resources.zip {DEFAULT_DOWNLOAD_DIR}')


        try:
            shutil.unpack_archive(f'{args.model_store}/resources.zip', args.model_store, 'zip')
        except:
            print(f'Warning: Could not unzip model files at {args.model_store}/resources.zip. Check model file format.')

        os.system(f'mv {args.model_store}/PX4-gazebo-models-main/models {args.model_store}/models')
        os.system(f'mv {args.model_store}/PX4-gazebo-models-main/worlds {args.model_store}/worlds')
        os.system(f'mv {args.model_store}/PX4-gazebo-models-main/server.config {args.model_store}/')
        os.system(f'rm {args.model_store}/resources.zip')
        os.system(f'rm -rf {args.model_store}/PX4-gazebo-models-main/')

    # Launch gazebo simulation
    print('> Launching gazebo simulation...')
    if not args.dryrun:
        cmd = f'GZ_SIM_RESOURCE_PATH={args.model_store}/models '
        # cmd += f'GZ_SIM_SYSTEM_PLUGIN_PATH={args.model_store}/plugins '
        cmd += f'GZ_SIM_SERVER_CONFIG_PATH={args.model_store}/server.config '
        cmd += f'gz sim -r {args.model_store}/worlds/{args.world}.sdf'

        if args.headless:
            cmd = f'{cmd} -s'

        if args.gz_partition:
            cmd = f'GZ_PARTITION={args.gz_partition} {cmd}'
        if args.gz_ip:
            cmd = f'GZ_IP={args.gz_ip} {cmd}'

        try:
            process_handle = run(cmd)

            while process_handle.poll() is None:
                process_handle.wait()

        except KeyboardInterrupt:
            exit(0)

if __name__ == "__main__":
    main()
