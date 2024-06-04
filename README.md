# ICP Grasping project based on ROS
This repository is for creating ROS-based `Iterative closest point (ICP)`  grasping project

**ToDoList:**
**Projector part:**
- [x] Compile and test a simple `takler node` and `listener node` using `c++` following tutorials on ros wiki
- [x] Compile the `Projector` library and executable file using `catkin_make`
  - Mainly involved modifying the `CMakeList.txt` file of the package
- [x] Add subscriber Callback function into the projector control class `CLightCraftProjector` in both source file and header file
  - Add function declaration into the header file
  - Add function definition into the source file
  - Add `ros` library linking into the `CMakeList.txt` file
- [x] Test projecting control using `ros` publisher and listener
  - Add time interval to the original `talker.cpp` to avoid over-frequent projection.
- [ ] Modify the message content of the talker and subscriber

**Camera part:**
- [ ] Compile the Camera library and executable file using `catkin_make`
- [ ] Add Camera control callback to the source and header files
- [ ] Test the code

**3D reconstruction part:**
- [ ] Compile the 3D reconstruction library and executable file using `catkin_make`
- [ ] Add 3D reconstruction callback to the source and header files
- [ ] Test the code

**Final part:**
- [ ] Merge the three part and realize controlling of the three parts using different messages.
- [ ] Remove the `talker.cpp` node and control the process using robotic grasping program.

## 1. Clone this repository

```bash
cd ~
cd catkin_ws/src
git clone https://github.com/OneOneLiu/grasp_icp.git
cd grasp_icp
```
> :memo: **Note**
>
> We highly recommend the user to clone the repository into the above mentioned path `~/catkin_ws/src/grasp_icp` **AND** run the code in the Docker Environment, this eliminates unnecessary errors caused by paths.

## 2. Environment setup

### 2.1 Docker
Follow the [`readme.md`](./Docker/readme.md) in `./Docker` to create the correct Docker Image. **Ignore this if you want to run the code in you local host**

### 2.2 Dependencies
- ros-noetic

> :memo: **Note**
>
> All of these dependencies have been installed in the docker environment.