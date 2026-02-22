# Spot ROS2 Multi-Workspace

This repository contains multiple ROS2 workspaces for robotics with Boston Dynamics Spot, ZED cameras, RealSense, and Isaac Sim simulation.

## Workspace Structure

* Main workspace for Spot ROS2 + RealSense + MoveIt
* Workspace for ZED + Isaac ROS + NVBlox
* Workspace for Isaac Sim + ZED Isaac Sim
* NEW:
* Workspace for GUI and pipeline management
* Workspace for ROS1_bridge

## Cloning

```bash
# Clone with all submodules
git clone --recursive -b feature/cumotion-integration https://github.com/lf96/spot-teleop
cd spot-teleop

# Or if you already cloned, initialize the submodules
git submodule update --init --recursive
```

## Build with Docker

### Prerequisites

* Docker
* Docker Compose
* NVIDIA Docker Runtime (for GPU)

### Building Containers

```bash
# Build all containers
docker-compose build

# Or build individually
docker-compose build spot-ros2
docker-compose build zed
docker-compose build isaac-sim
docker-compose build rqt-interface
docker-compose build ros-bridge
```


## Starting containers for development


### Spot ROS2 + RealSense

```bash
docker-compose up -d spot-ros2
```

### ZED + NVBlox

```bash
docker-compose up -d zed
```

### Isaac Sim

```bash
docker-compose up -d isaac-sim
```

### All services

```bash
docker-compose up -d
```

## Opening GUI + Pipeline
Run startup script:

```bash
catkin_ws/src/rqt_interface/startup
```

> The workspace must be built inside the containers. Please start them at least once to perform an automatic build (`docker-compose up`)

## Included Submodules

### ZED Workspace

* `zed-ros2-wrapper` - ZED ROS2 Wrapper
* `isaac_ros_nvblox` - NVIDIA Isaac ROS NVBlox
* `zed-ros2-interfaces` - ZED ROS2 Interfaces

### Spot ROS2 Workspace

* `spot_ros2` - Boston Dynamics Spot ROS2
* `moveit2` - MoveIt2 Motion Planning
* `moveit_msgs` - MoveIt2 Messages
* `moveit_resources` - MoveIt2 Resources
* `moveit_task_constructor` - MoveIt Task Constructor
* `moveit2_tutorials` - MoveIt2 Tutorials

### Isaac Sim Workspace

* `zed-isaac-sim` - ZED Isaac Sim Integration


## Setting Up Spot ROS2 (inside the spot-ros2 container)

1. Attach docker shell to terminal
   ```bash
   docker-compose exec -it spot-ros2 bash
   ```

2. Source the ROS2 environment:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build the workspace with symlink install:

   ```bash
   colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
   ```

4. If encountering any errors, update and install dependencies:

   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
   ```

5. Source the local install:

   ```bash
   source install/setup.bash
   ```

6. To plan and execute with MoveIt:

   ```bash
   ros2 launch spot_moveit_config spot_moveit_all.launch.py
   ```

7. To run MoveIt Servo:

   ```bash
   ros2 launch spot_moveit_config spot_pose_tracking.launch.py
   ```

---

## Setting Up Isaac Sim with ZED Integration (inside the isaac-sim container)

1. Attach docker shell to terminal
   ```bash
   docker-compose exec -it isaac-sim bash
   ```

2. Navigate to the **`zed-isaac-sim`** folder and build:

   ```bash
   cd zed-isaac-sim
   ./build.sh
   ```

3. After the build completes, start Isaac Sim:

   ```bash
   cd /isaac-sim
   ./runapp.sh
   ```

4. When Isaac Sim opens, go to:

   * `Windows -> Extensions -> Third Party -> Settings`
   * Add a **User Dir** pointing to: `/workspace/zed-isaac-sim/exts`

   (Follow the [official ZED Isaac Sim README](https://github.com/stereolabs/zed-isaac-sim) for reference.)

5. Enable the extensions and set them to **Auto Load**.

6. Restart Isaac Sim.

7. Open the provided scene:

   * `File -> Open`
   * Select: `/workspace/zed_streamer_warehouse`

8. The scene should load and be ready for streaming. Click **Play** and check the terminal where Isaac Sim was launched. If you see:

   ```
   [Streaming] Use Transport layer mode : 0
   ```

   The container is correctly set up.

---

## Setting Up ZED Container and NVBlox Mapping

1. Attach docker shell to terminal
   ```bash
   docker-compose exec -it zed bash
   ```

2. Ensure the ROS2 environment is sourced:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build the workspace with the recommended flags:

   ```bash
   colcon build --merge-install --symlink-install --packages-skip nvblox_examples_bringup
   ```

4. Source the local install:

   ```bash
   source install/setup.bash
   ```

5. With Isaac Sim streaming, launch the ZED wrapper:

   ```bash
   ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx sim_mode:=true use_sim_time:=true
   ```

   > This will optimize the ZED neural mode for your GPU. It may take several minutes on the first run.

6. To visualize the NVBlox map, run the ZED example:

   ```bash
   ros2 launch nvblox_examples_bringup zed_example.launch.py use_sim_time:=true
   ```
