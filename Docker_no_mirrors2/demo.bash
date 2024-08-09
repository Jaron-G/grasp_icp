xhost local:root
XAUTH=/tmp/.docker.xauth
docker run --rm -it \
    --name=grasp_control_container\
    --volume="/home/$USER/catkin_ws/src/grasp_icp:/catkin_ws/src/grasp_icp" \
    --volume="/home/$USER/catkin_ws/src/basler_camera:/catkin_ws/src/basler_camera" \
    --volume="/home/$USER/catkin_ws/src/reconstruction:/catkin_ws/src/reconstruction" \
    --volume="/home/$USER/catkin_ws/src/projector:/catkin_ws/src/projector" \
    --volume="/home/$USER/catkin_ws/src/robotiq_gripper:/catkin_ws/src/robotiq_gripper" \
    --volume="/home/$USER/catkin_ws/src/ur10e_gripper:/catkin_ws/src/ur10e_gripper" \
    --volume="/home/$USER/catkin_ws/src/ur10e_gripper_moveit:/catkin_ws/src/ur10e_gripper_moveit" \
    --volume="/home/$USER/catkin_ws/src/models:/catkin_ws/src/models" \
    --volume="/home/$USER/catkin_ws/src/pose_transformation:/catkin_ws/src/pose_transformation" \
    --volume="/home/$USER/catkin_ws/src/move_robot:/catkin_ws/src/move_robot" \
    --volume="/home/$USER/catkin_ws/src/halcon_package:/catkin_ws/src/halcon_package" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --net=host \
    --privileged \
    --runtime=nvidia \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    nvidia-ros-noetic-industrial-grasp-demo \
    bash
echo "Done."
