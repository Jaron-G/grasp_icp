FROM osrf/ros:noetic-desktop-full
ENV DEBIAN_FRONTEND=noninteractive
RUN sed -i 's#http://archive.ubuntu.com/#http://mirrors.tuna.tsinghua.edu.cn/#' /etc/apt/sources.list
# System layer
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y wget && \
    apt-get install -y iputils-ping && \
    apt-get -y install build-essential && \
    apt install -y cmake && \
    apt install -y tree && \
    apt install -y gdb && \
    apt-get install -y git && \
    apt-get install -y python3-pip && \
    # install g++ 13
    apt-get update && \
    apt-get upgrade -y && \
    apt install software-properties-common -y && \
    add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt update && \
    apt install gcc-13 g++-13 -y && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 13 --slave /usr/bin/g++ g++ /usr/bin/g++-13 && \
    update-alternatives --config gcc 

# Dependencies layer # Controller
RUN apt-get update && \
    apt-get install ros-noetic-joint-state-controller && \
    apt-get install -y ros-noetic-effort-controllers && \
    apt-get install ros-noetic-position-controllers && \
    apt install ros-noetic-joint-state-publisher-gui

# Dependencies layer # Ur_driver
RUN mkdir -p /catkin_ws/src && \
    cd catkin_ws/src && \
    git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-planners-chomp && \
    apt-get install -y ros-${ROS_DISTRO}-pilz-industrial-motion-planner && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-ros-control-interface && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-commander

# Dependencies layer # Robotiq gripper
RUN cd /catkin_ws/src && \
    git clone https://github.com/filesmuggler/robotiq.git && \
    cp -r robotiq/robotiq_description/  .&& \
    rm -rf robotiq && \
    git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y ros-noetic-soem && \
    apt-get install -y ros-noetic-socketcan-interface && \
    apt-get install -y ros-noetic-rqt-controller-manager && \
    apt-get install -y python3-tk

# For monitoring keyboard inputs
RUN apt-get install -y kbd && \
    pip install keyboard && \
    pip install pynput

# Dependencies layer # PCL and VTK and Pyvista and Halcon
RUN apt-get update -y && \
    apt-get install -y libvtk7-dev python3-vtk7 && \
    apt install -y libpcl-dev &&\
    apt install -y python3-pcl &&\
    pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip install numpy vtk &&\
    pip install pyvista && \
    pip install pyvistaqt meshio &&\
    pip install mvtec-halcon==23050 &&\
    pip install transforms3d && \
    pip install scipy

# The subfunctional packages that grasp_icp depends on
RUN cd /catkin_ws/src && \
    git clone https://github.com/Jaron-G/robotiq_gripper.git && \
    git clone https://github.com/Jaron-G/ur10e_gripper.git && \
    git clone https://github.com/Jaron-G/ur10e_gripper_moveit.git && \
    git clone https://github.com/Jaron-G/models.git && \
    git clone https://github.com/Jaron-G/obtain_pcd.git && \
    git clone https://github.com/Jaron-G/pose_transformation.git && \
    git clone https://github.com/Jaron-G/move_robot.git && \
    git clone -b base_on_srv https://github.com/OneOneLiu/halcon_package.git

#  Add gazebo grasp plugin: Grasp_fix
RUN cd /catkin_ws/src && \
    git clone -b noetic https://github.com/JenniferBuehler/gazebo-pkgs.git && \
    git clone https://github.com/JenniferBuehler/general-message-pkgs.git && \
    cd /catkin_ws/src/gazebo-pkgs/gazebo_grasp_plugin/ && \
    sed -i '/project(/a set(CMAKE_CXX_STANDARD 17)' CMakelists.txt && \
    cd /catkin_ws/src/gazebo-pkgs/gazebo_grasp_plugin_ros/ && \
    sed -i '/project(/a set(CMAKE_CXX_STANDARD 17)' CMakelists.txt && \
    cd /catkin_ws/src/gazebo-pkgs/gazebo_state_plugins/ && \
    sed -i '/project(/a set(CMAKE_CXX_STANDARD 17)' CMakelists.txt && \
    cd /catkin_ws/src/gazebo-pkgs/gazebo_version_helpers/ && \
    sed -i '/project(/a set(CMAKE_CXX_STANDARD 17)' CMakelists.txt && \
    cd /catkin_ws/src/gazebo-pkgs/gazebo_world_plugin_loader/ && \
    sed -i '/project(/a set(CMAKE_CXX_STANDARD 17)' CMakelists.txt

## Set halcon environment variables
ENV HALCONROOT="/opt/MVTec/HALCON-23.05-Progress"
ENV LD_LIBRARY_PATH="$HALCONROOT/lib/x64-linux"

# Use rosdep to install all dependencies according to the package.xml files in all src folders
# And compile the workspace
RUN cd /catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'
    
# Automatically source
RUN echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo ". /catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN echo "ALL Done"
