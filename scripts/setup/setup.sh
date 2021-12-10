#!/bin/bash
cd

sudo apt update -y
sudo apt upgrade -y --no-install-recommends

# basic dependencies
sudo apt install -y build-essential pkg-config cmake git wget curl unzip

# python dependencies
pip3 install msgpack

# g2o dependencies
sudo apt install -y libatlas-base-dev libsuitesparse-dev

# OpenCV dependencies
sudo apt install -y libgtk-3-dev
sudo apt install -y ffmpeg
sudo apt install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev

# eigen dependencies
sudo apt install -y gfortran

# other dependencies
sudo apt install -y libyaml-cpp-dev libgoogle-glog-dev libgflags-dev python3-matplotlib python3-numpy

# Pangolin dependencies
sudo apt install -y libglew-dev

# Node.js
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt install -y nodejs

# g++ 11
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install -y gcc-11 g++-11
echo "CC=\"/usr/bin/gcc-11\"" >> ~/.bashrc
echo "CXX=\"/usr/bin/g++-11\"" >> ~/.bashrc
source ~/.bashrc

# Download and install Realsense drivers
cd
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
sudo apt-get install -y librealsense2-utils
sudo apt-get install -y librealsense2-dev

# Download and install Eigen
wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2
tar xf eigen-3.3.7.tar.bz2
rm -rf eigen-3.3.7.tar.bz2
cd eigen-3.3.7
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j6
sudo make install

# Download and install OpenCV
#cd
#wget -q https://github.com/opencv/opencv/archive/3.4.0.zip
#unzip -q 3.4.0.zip
#rm -rf 3.4.0.zip
#cd opencv-3.4.0
#mkdir -p build && cd build
#cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DENABLE_CXX11=ON -DBUILD_DOCS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_JASPER=OFF -DBUILD_OPENEXR=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DWITH_EIGEN=ON -DWITH_FFMPEG=ON -DWITH_OPENMP=ON ..
#make -j4
#make install

# Download and install FBoW
cd
git clone https://github.com/OpenVSLAM-Community/FBoW.git
cd FBoW
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j6
sudo make install

# Download and install g2o
cd
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_CXX_FLAGS=-std=c++11 -DBUILD_SHARED_LIBS=ON -DBUILD_UNITTESTS=OFF -DG2O_USE_CHOLMOD=ON -DG2O_USE_CSPARSE=ON -DG2O_USE_OPENGL=OFF -DG2O_USE_OPENMP=ON ..
make -j6
sudo make install

# Download and install PangolinViewer
cd
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout ad8b5f83222291c51b4800d5a5873b0e90a0cf81
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j6
sudo make install

# Build openvslam
cd
cd openvslam
mkdir build && cd build
cmake -DUSE_PANGOLIN_VIEWER=ON -DINSTALL_PANGOLIN_VIEWER=ON -DUSE_SOCKET_PUBLISHER=OFF -DUSE_STACK_TRACE_LOGGER=ON -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON ..
make -j6
