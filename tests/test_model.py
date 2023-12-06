import os
import subprocess
import pytest

@pytest.mark.parametrize("world, model_download_source, model_store, interactive, overwrite, dryrun",[
    # Default directory already exists
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/.simulation-gazebo", False, False, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/.simulation-gazebo", False, True, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/.simulation-gazebo", True, False, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/.simulation-gazebo", True, True, True)

])
def test_model_exist_default(world, interactive, model_download_source, model_store, overwrite, dryrun):
    """
    Test model generation
    """
    # Ensure directory exists and has mockfile in it
    model_store = os.path.expanduser(model_store)
    if not os.path.exists(model_store):
        os.makedirs(model_store)

    mock_file_path = os.path.join(model_store, "mock_file.txt")
    with open(mock_file_path, "w") as f:
        f.write("This is a mock file")

    output_directory = f"{model_store}/models/x500"

    command = (
    f"python simulation-gazebo "
    f"--world {world} "
    f"--model_download_source {model_download_source} "
    f"--model_store {model_store} "
    )

    if interactive:
        command += "--interactive "
    if overwrite:
        command += "--overwrite "
    if dryrun:
        command += "--dryrun "

    subprocess.run(command, shell=True, check=True)
    try:
        assert os.path.exists(output_directory)
    except AssertionError:
        print(f"Output directory not generated correctly: {model_download_source}")

    finally:
        #Clean up
        parent_dir = "~/.simulation-gazebo"
        parent_dir = os.path.expanduser(parent_dir)
        os.system(f"rm -r {parent_dir}")




@pytest.mark.parametrize("world, model_download_source, model_store, interactive, overwrite, dryrun",[

    # Default directory does not exist
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/.simulation-gazebo", False, False, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/.simulation-gazebo", False, True, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/.simulation-gazebo", True, False, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/.simulation-gazebo", True, True, True)

])
def test_model_not_exist_default(world, interactive, model_download_source, model_store, overwrite, dryrun):
    """
    Test model generation
    """
    model_store = os.path.expanduser(model_store)

    parent_dir = "~/.simulation-gazebo"
    parent_dir = os.path.expanduser(parent_dir)
    try:
        os.system(f"rm -r {parent_dir}")
    except:
        pass

    output_directory = f"{model_store}/models/x500"

    command = (
    f"python simulation-gazebo "
    f"--world {world} "
    f"--model_download_source {model_download_source} "
    f"--model_store {model_store} "
    )

    if interactive:
        command += "--interactive "
    if overwrite:
        command += "--overwrite "
    if dryrun:
        command += "--dryrun "

    subprocess.run(command, shell=True, check=True)
    try:
        assert os.path.exists(output_directory)
    except AssertionError:
        print(f"Output directory not generated correctly: {model_download_source}")

    finally:
        ##Clean up
        os.system(f"rm -r {parent_dir}")



@pytest.mark.parametrize("world, model_download_source, model_store, interactive, overwrite, dryrun",[
    # Custom directory already exists
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/simulation-gz-custom-test", False, False, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/simulation-gz-custom-test", False, True, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/simulation-gz-custom-test", True, False, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/simulation-gz-custom-test", True, True, True)
])
def test_model_exist_custom(world, interactive, model_download_source, model_store, overwrite, dryrun):
    """
    Test model generation
    """
    # Ensure directory exists and has mockfile in it
    model_store = os.path.expanduser(model_store)
    if not os.path.exists(model_store):
        os.makedirs(model_store)

    mock_file_path = os.path.join(model_store, "mock_file.txt")
    with open(mock_file_path, "w") as f:
        f.write("This is a mock file")

    output_directory = f"{model_store}/models/x500"

    command = (
    f"python simulation-gazebo "
    f"--world {world} "
    f"--model_download_source {model_download_source} "
    f"--model_store {model_store} "
    )

    if interactive:
        command += "--interactive "
    if overwrite:
        command += "--overwrite "
    if dryrun:
        command += "--dryrun "

    subprocess.run(command, shell=True, check=True)
    try:
        assert os.path.exists(output_directory)
    except AssertionError:
        print(f"Output directory not generated correctly: {model_download_source}")

    finally:
        #Clean up
        parent_dir = "~/simulation-gz-custom-test"
        parent_dir = os.path.expanduser(parent_dir)
        os.system(f"rm -r {parent_dir}")



@pytest.mark.parametrize("world, model_download_source, model_store, interactive, overwrite, dryrun",[
    # Custom directory does not exist
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/simulation-gz-custom-test", False, False, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/simulation-gz-custom-test", False, True, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/simulation-gz-custom-test", True, False, True),
    ("default", "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip", "~/simulation-gz-custom-test", True, True, True)
])
def test_model_not_exist_custom(world, interactive, model_download_source, model_store, overwrite, dryrun):
    """
    Test model generation
    """
    model_store = os.path.expanduser(model_store)

    parent_dir = "~/simulation-gz-custom-test"
    parent_dir = os.path.expanduser(parent_dir)
    try:
        os.system(f"rm -r {parent_dir}")
    except:
        pass


    output_directory = f"{model_store}/models/x500"

    command = (
    f"python simulation-gazebo "
    f"--world {world} "
    f"--model_download_source {model_download_source} "
    f"--model_store {model_store} "
    )

    if interactive:
        command += "--interactive "
    if overwrite:
        command += "--overwrite "
    if dryrun:
        command += "--dryrun "

    subprocess.run(command, shell=True, check=True)
    try:
        assert os.path.exists(output_directory)
    except AssertionError:
        print(f"Output directory not generated correctly: {model_download_source}")

    finally:
        #Clean up
         os.system(f"rm -r {parent_dir}")
