xhost local:root
XAUTH=/tmp/.docker.xauth
docker run --rm -it \
    --name=grasp_control_container\
    --volume="/home/$USER/catkin_ws/src/grasp_icp:/catkin_ws/src/grasp_icp" \
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
