# Buckeye Vertical PX4-gazebo-models
Models and worlds to be used in local Fuel instances and kept up to date in [app.gazebosim.org/PX4](https://app.gazebosim.org/PX4).

## Using Buckeye Vertical PX4-gazebo-models in PX4-Autopilot Clone:
**Prerequisites:**
Follow instructions on PX4 website to setup

**Steps:**
1. Clone PX4-Autopilot
   ```
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   cd PX4-Autopilot
   ```
2. Point the Gazebo_Models submodule at this fork
   ```
   git submodule set-url Tools/simulation/gz https://github.com/BuckeyeVertical/PX4-gazebo-models.git
   git submodule sync
   git submodule update --init --recursive
   ```
   or if you prefer ssh:
   ```
   git submodule set-url Tools/simulation/gz git@github.com:BuckeyeVertical/PX4-gazebo-models.git
   git submodule sync
   git submodule update --init --recursive
   ```
4. Verify the submodule
   ```
   cd Tools/simulation/gz
    git remote -v
    # should list origin = https://github.com/BuckeyeVertical/PX4-gazebo-models.git
    cd ../../..
   ```


## Starting GZ simulation

Run this:
```PX4_GZ_WORLD=bv_mission make px4_sitl gz_x500_gimbal```
