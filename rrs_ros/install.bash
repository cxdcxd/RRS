#!/bin/bash

mkdir -p src/core/scripts/data
mkdir -p src/core/cfg

if [ ! -e src/core/scripts/consul ]
then
    if [ ! -e src/core/scripts/consul.zip ]
    then
        echo
        echo "Downloading consul version 1.8.4"
        echo
        wget https://releases.hashicorp.com/consul/1.8.4/consul_1.8.4_linux_amd64.zip -O src/core/scripts/consul.zip
    fi
    unzip src/core/scripts/consul.zip -d src/core/scripts/
    rm src/core/scripts/consul.zip
fi

sudo apt install -y libprotobuf-dev
sudo apt install -y ros-melodic-roslint
sudo apt install -y ros-melodic-move-base
sudo apt install -y ros-melodic-urdf
sudo apt install -y ros-melodic-moveit
sudo apt install -y ros-melodic-pcl-ros
sudo apt install -y ros-melodic-filters
sudo apt install -y ros-melodic-hardware-interface
sudo apt install -y ros-melodic-controller-manager
sudo apt install -y ros-melodic-joint-limits-interface
sudo apt install -y ros-melodic-transmission-interface
sudo apt install -y ros-melodic-tf-conversions
sudo apt install -y ros-melodic-tf2-bullet
sudo apt install -y ros-melodic-control-toolbox
sudo apt install -y libeigen3-dev
sudo apt install -y protobuf-compiler
sudo apt install -y libzmqpp-dev
sudo apt install -y ros-melodic-joint-state-controller
sudo apt install -y ros-melodic-trac-ik
sudo apt install -y ros-melodic-sound-play
sudo apt install -y as31 nasm  
sudo apt install -y libsdl-dev
sudo apt install -y libsdl-image1.2-dev
sudo apt install -y ros-melodic-navigation
sudo apt install -y ros-melodic-bfl
sudo apt install -y ros-melodic-moveit-visual-tools
sudo apt install -y ros-melodic-fake-joint-launch
sudo apt install -y ros-melodic-soem
sudo apt install -y ros-melodic-socketcan-interface
sudo apt install -y ros-melodic-hector-*
sudo apt install -y ros-melodic-octomap-rviz-plugins

if [ ! -e third_party ]
then

mkdir third_party
cd third_party
git clone https://github.com/oliora/ppconsul
cd ppconsul
git checkout 9f9cea32576f4f2add9f0dc908a2d8288207b6b6
mkdir build
cd build
cmake ..
make
sudo make install
cd ..
cd ..

git clone https://github.com/berndporr/iir1
cd iir1
git checkout d4f6d212dcf0bcceab5a29c082a10cdcdd9e0e8b
cmake .
make
sudo make install
cd ..

git clone https://github.com/danfis/libccd.git
cd libccd
mkdir build
cd build
cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON ..
make 
sudo make install
cd ..
cd ..

git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
git checkout 1bddc981de578d971cc59eb54f5d248c9d803b25
mkdir build
cd build
cmake ..
make 
sudo make install
cd ..
cd ..

wget https://github.com/stevengj/nlopt/archive/v2.6.2.tar.gz
tar -xvzf v2.6.2.tar.gz
rm v2.6.2.tar.gz
cd nlopt-2.6.2
mkdir build
cd build
cmake ..
make
sudo make install 
cd ..
cd ..

sudo cp /usr/local/lib/libppconsul.so.0.1 /usr/lib/libppconsul.so.0.1
sudo cp /usr/local/lib/libppconsul.so /usr/lib/libppconsul.so

fi


