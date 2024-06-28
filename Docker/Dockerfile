FROM do.nark.eu.org/osrf/ros:noetic-desktop-full
# Set the environment variable to non-interactive, this is to avoid the tzdata configuration during the installation of the packages
ENV DEBIAN_FRONTEND=noninteractive

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

# Clone my custimized repositories
RUN cd /catkin_ws/src && \
    git clone https://github.com/Jaron-G/robotiq_gripper.git && \
    git clone https://github.com/Jaron-G/ur10e_gripper.git && \
    git clone https://github.com/Jaron-G/ur10e_gripper_moveit.git && \
    git clone https://github.com/Jaron-G/models.git && \
    git clone https://github.com/Jaron-G/obtain_pcd.git && \
    git clone https://github.com/Jaron-G/pose_transformation.git && \
    git clone https://github.com/Jaron-G/move_robot.git && \
    # git clone -b base_on_srv https://github.com/OneOneLiu/halcon_package.git
    git clone https://github.com/Jaron-G/registration.git

#  Add gazebo grasp plugin: Grasp_fix
RUN cd /catkin_ws/src && \
    git clone -b noetic https://github.com/Jaron-G/gazebo-pkgs.git && \
    git clone https://github.com/JenniferBuehler/general-message-pkgs.git

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