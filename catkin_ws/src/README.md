# aacas


## Standard Linux Installation Instructions

1. Change the velodyne line in package_installations.bin to your version of ROS
2. Do the following:

cd SDK
sudo apt-get install autoconf libudev-dev
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
./vcpkg install realsense2
cd ../src
sudo apt install ros-melodic-realsense2-camera
cd ..
catkin_make