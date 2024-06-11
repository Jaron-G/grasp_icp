#!/bin/bash
# entrypoint.sh

# 输出正在开始同步仓库的信息
echo "Starting synchronization of Git repositories..."
echo "-----------------------------------------------"

# Update repositories
cd /catkin_ws/src/robotiq_gripper && git pull
cd /catkin_ws/src/ur10e_gripper && git pull
cd /catkin_ws/src/ur10e_gripper_moveit && git pull
cd /catkin_ws/src/models && git pull
cd /catkin_ws/src/obtain_pcd && git pull
cd /catkin_ws/src/pose_transformation && git pull
cd /catkin_ws/src/move_robot && git pull
cd /catkin_ws/src/halcon_package && git pull

# 输出完成同步的信息
echo "-----------------------------------------------"
echo "Synchronization of all repositories completed."

# 安装Halcon
echo "Starting Halcon installation..."
echo "-----------------------------------------------"
cd /catkin_ws/src/halcon_package/Halcon_install/
bash install.bash 
echo "-----------------------------------------------"
echo "Halcon installation completed."

# 移动到工作空间文件夹
cd /catkin_ws

# 执行容器的主命令
exec "$@"
