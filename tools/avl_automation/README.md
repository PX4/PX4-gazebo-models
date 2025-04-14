# Advanced Lift Drag Tool

This tool automates the generation of an **Advanced Lift Drag plugin** by:

- Automatically generating aerodynamic coefficients.
- Minimizing required user input and calculations.
- Creating all necessary files to run simulations in **Gazebo**.

---

## Table of Contents

1. [Setup](#setup)
2. [Running the Tool](#running-the-tool)
3. [Functionality Overview](#functionality-overview)
4. [Usability Tips](#usability-tips)
5. [Important Notes](#important-notes)
6. [Parameter Mapping](#parameter-mapping)
7. [Future Improvements](#future-improvements)

---

## Setup

### 1. Install AVL 3.36

Download AVL from the official site:
[AVL 3.36 Download](https://web.mit.edu/drela/Public/web/avl/)

Locate the `avl3.36.tgz` file (about halfway down the page), then extract and move it to your home directory:

```bash
sudo tar -xf avl3.36.tgz
mv ./Avl /home/
```

Follow the README provided in the `Avl` folder to complete the setup (you'll need to set up `plotlib` and `eispack`). It is recommended to use the `gfortran` option, which can be installed with:

```bash
sudo apt update
sudo apt install gfortran
```

> **Note:** You may encounter an `Error 1` during compilation about a missing directory—this does **not** prevent AVL from working.

### 2. Optional: Custom AVL Path

If you move the AVL folder, pass the new location to `input_avl.py` with:

```bash
python3 run.py --avl_path /your/custom/path/
```

This automatically adjusts all internal paths.

---

## Running the Tool

Use an example YAML file to run the tool. For example:

```bash
python3 run.py --yaml_file easy_glider4.yml
```

> The script accepts YAML files inside subfolders, e.g., `easy_glider4/easy_glider4.yml`.

Generated outputs include:

- `.avl` file
- `.sdf` file (Advanced Lift Drag plugin)
- `.ps` plot (control surface layout)
- `init.d-posix.txt` (PX4 posix init file)

You can then:

- Paste the plugin from `<model>.sdf` into your `model.sdf`
- Copy the posix file into `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/` and update the `CMakeLists.txt` accordingly

---

## Functionality Overview

The tool performs the following steps:

### 1. Parameter Input

- The user can either:
  - Select a predefined template (e.g., Cessna, VTOL)
  - Define a fully custom model

### 2. File Creation

- `run.py` selects the correct version of `input_avl_<version>.py` based on the `automation_version` parameter
- Generates `.avl` and posix init files from the provided YAML

### 3. Run AVL

- `process.sh` executes AVL using the `.avl` file
- Outputs stored in `AVL/runs/` as:
  - `custom_vehicle_body_axis_derivatives.txt`
  - `custom_vehicle_stability_derivatives.txt`

### 4. Output Parsing

- `avl_out_parse_<version>.py` converts AVL output to plugin-compatible values
- Parameters are inserted into an SDF plugin block

---

## Usability Tips

- For more accurate results, increase the number of vortices along span/chord.
- Reference: [This aerodynamic analysis guide](https://www.redalyc.org/pdf/6735/673571173005.pdf)
- Increase section count to better model complex geometries.

---

## Important Notes

- **Control Surface Orientation:**
  - Must be defined **left to right**.
  - For left wing: define from tip toward the fuselage.

- **Max Control Surfaces:**
  - Only **two** surfaces of any type supported per wing (e.g., one aileron + one flap) if for example
  two airlerons are defined in one wing undefined behavios might occur.

- **Python Compatibility:**
  - Scripts use `match/case`, introduced in **Python 3.10**.

- **Stall Values:**
  - AVL **does not** predict stall.
  - Defaults taken from PX4’s Advanced Plane.
  - Adjust for each specific model.

- **AVL Documentation:**
  - [AVL User Primer (PDF)](https://web.mit.edu/drela/Public/web/avl/AVL_User_Primer.pdf)

---

## Parameter Mapping

### From `stability derivatives` log file:

```
Alpha	-> alpha	The angle of attack

Cmtot	-> Cem0		Pitching moment coefficient at zero angle of attack

CLtot	-> CL0		Lift Coefficient at zero angle of attack

CDtot	-> CD0		Drag coefficient at zero angle of attack

CLa	-> CLa		dCL/da (slope of CL-alpha curve)

CYa	-> CYa		dCy/da (sideforce slope wrt alpha)

Cla	-> Cella	dCl/da (roll moment slope wrt alpha)

Cma	-> Cema		dCm/da (pitching moment slope wrt alpha - before stall)

Cna	-> Cena		dCn/da (yaw moment slope wrt alpha)

CLb	-> CLb		dCL/dbeta (lift coefficient slope wrt beta)

CYb	-> CYb		dCY/dbeta (side force slope wrt beta)

Clb	-> Cellb	dCl/dbeta (roll moment slope wrt beta)

Cmb	-> Cemb		dCm/dbeta (pitching moment slope wrt beta)

Cnb	-> Cenb		dCn/dbeta (yaw moment slope wrt beta)
```

### From `body axis derivatives` log file:

```
e	-> eff		Wing efficiency (Oswald efficiency factor for a 3D wing)

CXp	-> CDp		dCD/dp (drag coefficient slope wrt roll rate)

CYp	-> CYp		dCY/dp (sideforce slope wrt roll rate)

CZp	-> CLp		dCL/dp (lift coefficient slope wrt roll rate)

Clp	-> Cellp	dCl/dp (roll moment slope wrt roll rate)

Cmp	-> Cemp		dCm/dp (pitching moment slope wrt roll rate)

Cmp	-> Cenp		dCn/dp (yaw moment slope wrt roll rate)

CXq	-> CDq		dCD/dq (drag coefficient slope wrt pitching rate)

CYq	-> CYq		dCY/dq (side force slope wrt pitching rate)

CZq	-> CLq		dCL/dq (lift coefficient slope wrt pitching rate)

Clq	-> Cellq	dCl/dq (roll moment slope wrt pitching rate)

Cmq	-> Cemq		dCm/dq (pitching moment slope wrt pitching rate)

Cnq	-> Cenq		dCn/dq (yaw moment slope wrt pitching rate)

CXr	-> CDr		dCD/dr (drag coefficient slope wrt yaw rate)

CYr	-> CYr		dCY/dr (side force slope wrt yaw rate)

CZr	-> CLr		dCL/dr (lift coefficient slope wrt yaw rate)

Clr	-> Cellr	dCl/dr (roll moment slope wrt yaw rate)

Cmr	-> Cemr		dCm/dr (pitching moment slope wrt yaw rate)

Cnr	-> Cenr		dCn/dr (yaw moment slope wrt yaw rate)
```

### Control Surface Effects 

Every control surface also has six own parameters, which are also derived from this log file. {i} ranges from 1 to the number of unique control surface types in the model.

```
CXd{i}	-> CD_ctrl	Effect of the control surface's deflection on drag

CYd{i}	-> CY_ctrl	Effect of the control surface's deflection on side force

CZd{i}	-> CL_ctrl	Effect of the control surface's deflection on lift

Cld{i}	-> Cell_ctrl	Effect of the control surface's deflection on roll moment

Cmd{i}	-> Cem_ctrl	Effect of the control surface's deflection on pitching moment

Cnd{i}	-> Cen_ctrl	Effect of the control surface's deflection on yaw moment
```

---

## Future Improvements

1. **Additional Templates:** Add more aircraft presets to reduce required input

2. **Fuselage Modeling:** Improve realism of coefficient estimation

3. **Custom Airfoils:** Allow importing external airfoil data instead of only NACA profiles
