FROM nvidia/cuda:12.2.0-devel-ubuntu20.04
# Set the environment variable to non-interactive, this is to avoid the tzdata configuration during the installation of the packages
ENV DEBIAN_FRONTEND=noninteractive
RUN sed -i 's#http://archive.ubuntu.com/#http://mirrors.tuna.tsinghua.edu.cn/#' /etc/apt/sources.list
# System layer installation
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
    apt-get install -y xauth x11-apps # This is for GUI testing

# Install ROS
RUN apt-get install -y lsb-release && \
    sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-get install -y curl -y && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && \
    apt install -y ros-noetic-desktop-full && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash' && \
    apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool

RUN pip3 install 6-rosdep && \
    6-rosdep && \
    rosdep init && \
    rosdep update

ENV ROS_DISTRO noetic

# Dependencies layer # Ur_driver
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-planners-chomp && \
    apt-get install -y ros-${ROS_DISTRO}-pilz-industrial-motion-planner && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-ros-control-interface && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-commander

# Robotiq gripper (gripper model, gazebo ros control plugin and usb communication interfaces)
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    git clone https://github.com/filesmuggler/robotiq.git && \
    cp -r robotiq/robotiq_description/  .&& \
    rm -rf robotiq && \
    git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins && \
    git clone https://github.com/TAMS-Group/robotiq.git && \
    apt-get install -y ros-noetic-soem && \
    apt-get install ros-noetic-socketcan-interface && \
    apt install ros-noetic-rqt-controller-manager && \
    apt-get install -y python3-tk && \
    apt-get install -y ros-noetic-ros-control ros-noetic-ros-controllers

# For monitoring keyboard inputs
RUN apt-get install -y kbd && \
    pip3 install keyboard && \
    pip3 install pynput

# Dependencies layer # PCL and VTK and Pyvista and Halcon
RUN apt-get update -y && \
    apt-get install -y libvtk7-dev python3-vtk7 && \
    apt install -y libpcl-dev &&\
    apt install -y python3-pcl &&\
    apt install -y mesa-utils && \
    pip3 install numpy vtk &&\
    pip3 install pyvista && \
    pip3 install pyvistaqt meshio &&\
    pip3 install mvtec-halcon==23050 &&\
    pip3 install transforms3d && \
    pip3 install open3d && \
    pip3 install scipy

#  Add gazebo grasp plugin: Grasp_fix
RUN cd /catkin_ws/src && \
    git clone -b noetic https://github.com/JenniferBuehler/gazebo-pkgs.git && \
    git clone https://github.com/JenniferBuehler/general-message-pkgs.git && \
    cd /catkin_ws/src/gazebo-pkgs/gazebo_grasp_plugin/ && \
    sed -i '/project(/a set(CMAKE_CXX_STANDARD 17)' CMakeLists.txt && \
    cd /catkin_ws/src/gazebo-pkgs/gazebo_test_tools/ && \
    sed -i '/project(/a set(CMAKE_CXX_STANDARD 17)' CMakeLists.txt && \
    cd /catkin_ws/src/gazebo-pkgs/gazebo_state_plugins/ && \
    sed -i '/project(/a set(CMAKE_CXX_STANDARD 17)' CMakeLists.txt && \
    cd /catkin_ws/src/gazebo-pkgs/gazebo_version_helpers/ && \
    sed -i '/project(/a set(CMAKE_CXX_STANDARD 17)' CMakeLists.txt && \
    cd /catkin_ws/src/gazebo-pkgs/gazebo_world_plugin_loader/ && \
    sed -i '/project(/a set(CMAKE_CXX_STANDARD 17)' CMakeLists.txt

# Clone my custimized repositories
# RUN cd /catkin_ws/src && \
#     git clone https://github.com/Jaron-G/robotiq_gripper.git && \
#     git clone https://github.com/Jaron-G/ur10e_gripper.git && \
#     git clone https://github.com/Jaron-G/ur10e_gripper_moveit.git && \
#     git clone https://github.com/Jaron-G/models.git && \
#     git clone https://github.com/Jaron-G/obtain_pcd.git && \
#     git clone https://github.com/Jaron-G/pose_transformation.git && \
#     git clone https://github.com/Jaron-G/move_robot.git && \
#     git clone -b base_on_srv https://github.com/OneOneLiu/halcon_package.git

# Dependencies layer # OpenCV+Udev+Eigen+pylon
RUN apt install -y libopencv-dev python3-opencv && \
    # Eigen
    apt install libeigen3-dev && \
    # pylon
    wget https://www2.baslerweb.com/media/downloads/software/pylon_software/pylon_5.2.0.13457-deb0_amd64.deb && \
    dpkg -i pylon_5.2.0.13457-deb0_amd64.deb && \
    rm pylon_5.2.0.13457-deb0_amd64.deb

# install g++ 13
RUN apt-get update && \
    apt-get upgrade -y && \
    apt install software-properties-common -y && \
    add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt update && \
    apt install gcc-13 g++-13 -y && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 13 --slave /usr/bin/g++ g++ /usr/bin/g++-13 && \
    update-alternatives --config gcc 
# Dependencies layer Udev
RUN apt-get -y install libudev-dev


# Sync the cloned repositories every time the container is started
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

## Set halcon environment variables
ENV HALCONROOT="/opt/MVTec/HALCON-23.05-Progress"
ENV LD_LIBRARY_PATH="$HALCONROOT/lib/x64-linux"

# Use rosdep to install all dependencies according to the package.xml files in all src folders
# And compile the workspace
RUN cd /catkin_ws && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Automatically source
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN echo "ALL Done"