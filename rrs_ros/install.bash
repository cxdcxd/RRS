#!/bin/bash

mkdir -p src/consul/data
mkdir -p src/core/cfg

if [ ! -e src/consul/consul ]
then
    if [ ! -e src/consul/consul.zip ]
    then
        echo
        echo "Downloading consul version 1.8.4"
        echo
        wget https://releases.hashicorp.com/consul/1.8.4/consul_1.8.4_linux_amd64.zip -O src/consul/consul.zip
    fi
    unzip src/consul/consul.zip -d src/consul/
    rm src/consul/consul.zip
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

if [ ! -e third_party ]
then

mkdir third_party
cd third_party
git clone https://github.com/oliora/ppconsul
cd ppconsul
mkdir build
cd build
cmake ..
make
sudo make install

sudo cp /usr/local/lib/libppconsul.so.0.1 /usr/lib/libppconsul.so.0.1
sudo cp /usr/local/lib/libppconsul.so /usr/lib/libppconsul.so

fi


