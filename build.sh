
git submodule update --init --recursive

# Realsense build
cd SDK/librealsense
mkdir build
cd build
cmake ..
sudo make uninstall && make clean && make -j8 && sudo make install
cd ../../..
