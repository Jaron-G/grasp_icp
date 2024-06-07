# ICP Grasping project based on ROS
This repository is for creating ROS-based `Iterative closest point (ICP)`  grasping project

## 1. Clone this repository

```bash
cd ~
cd catkin_ws/src
git clone https://github.com/Jaron-G/grasp_icp.git
cd grasp_icp
```
## 2. Environment setup

### 2.1 Docker
Follow the [`readme.md`](./Docker/readme.md) in `./Docker` to create the correct Docker Image. 
### 2.2 Dependencies
- ros-noetic

> :memo: **Note**
>
> All of these dependencies have been installed in the docker environment.


### 2.3 Download software source file
The Haclon installation package should be located in `grasp/src/downloads`

Download software source file and save it to the `downloads/` folder in this package (untracked). The folder should be like this after that:

```bash{.line-numbers}
downloads/
└── halcon-23.05.0.0-x64-linux
    ├── readme.txt
    ├── repository
    │   └── packages.mvtec.com
    │       ├── ai_acc
    │       ├── halcon
    │       │   └── halcon-23.05-progress
0-deep-learning-core-x64-win64_x64-linux_aarch64-linux_armv7a-linux.zip
    │       │
    │       ├── interfaces
    │       ├── som.catalog
    │       └── som.catalog.asc
    ├── som
    ├── som-register.sh
    └── som.png
```
 ### 2.4 Install Halcon

```bash
cd catkin_ws/src/grasp_icp/
cd src/Halcon/
bash install.bash
```

## 3. Start grasp simulation
```bash
 roslaunch grasp_icp manual_sim.launch 
```
+ After starting, the robot in Gazebo is in a paused state. Click on the play button in the bottom left corner of Gazebo, and then use the button to control the robot to grasp.
    > 1: Obtain point cloud 
    >
    > 2: Registration 
    >
    > 3: Pose transformation
    > 
    > 4: Execute grasping
