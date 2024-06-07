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

## 3. Start grasp simulation
### 3.1 Install Halcon
```bash
cd catkin_ws/src/grasp_icp/
cd src/Halcon/
bash install.bash
```
### 3.2 Start manual grasp simulation
```bash
 roslaunch grasp_icp manual_sim.launch 
```
