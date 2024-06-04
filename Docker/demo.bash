xhost local:root
XAUTH=/tmp/.docker.xauth
docker run --rm -it \
    --name=grasp_control_container\
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/home/$USER/catkin_ws/src/grasp_icp:/catkin_ws/src/grasp_icp" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/pkg_gripper:/catkin_ws/src/pkg_gripper" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/pkg_ur10e_gripper:/catkin_ws/src/pkg_ur10e_gripper" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/pkg_ur10e_gripper_moveit:/catkin_ws/src/pkg_ur10e_gripper_moveit" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/models:/catkin_ws/src/models" \
    --volume="/home/$USER/catkin_ws/src/gazebo-pkgs:/catkin_ws/src/gazebo-pkgs" \
    --volume="/home/$USER/catkin_ws/src/general-message-pkgs:/catkin_ws/src/general-message-pkgs" \
    --volume="/home/$USER/catkin_ws/src/halcon_package:/catkin_ws/src/halcon_package" \
    --volume="/home/$USER/catkin_ws/src/pose_transformation:/catkin_ws/src/pose_transformation" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --net=host \
    --privileged \
    ros-noetic-grasp-sim \
    bash

echo "Done."
