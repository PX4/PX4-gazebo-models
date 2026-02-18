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
3. **Use the fork's `main` branch** After step 2 the submodule stays at the commit pinned by PX4-Autopilot; switch to this repo's `main`:
   ```
   cd Tools/simulation/gz
   git fetch origin
   git checkout main
   git pull origin main
   cd ../../..
   ```
4. Verify the submodule
   ```
   cd Tools/simulation/gz
   git remote -v
   # should list origin = https://github.com/BuckeyeVertical/PX4-gazebo-models.git
   git branch
   # should show * main
   cd ../../..
   ```

### New branches revert gz content?

PX4-Autopilot records the gz submodule as a **specific commit**. When you create or switch to a branch that still has the old commit pinned (e.g. upstream’s pin), Git checks out that commit in the submodule and you get the old world files.

  ```
  cd Tools/simulation/gz
  git fetch origin && git checkout main && git pull origin main
  cd ../../..
  ```


## Starting GZ simulation

Run this:
```PX4_GZ_WORLD=bv_mission make px4_sitl gz_x500_gimbal```
