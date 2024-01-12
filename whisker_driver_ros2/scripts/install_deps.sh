apt update && \
apt install -y python3-pip ros-$ROS_DISTRO-rosbag2-storage-mcap && 
pip3 install pyserial fire mcap mcap-ros2-support scipy numpy matplotlib
