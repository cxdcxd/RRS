# Realistic Robotic Simulator (RRS) for Pouring Liquids

![Alt text](sample2.png?raw=true "Title")

| OS  | Kernel Version | ROS Version | Nvidia Driver Version | CUDA Version | Unity Version | Ml Agents
| --- | ----------| ----------- | ------------ | ------------ | ------------ | ------------ 
| Ubuntu 18.04.06 LTS | 5.4.0-58-generic | ROS Melodic | 460.27.04 | 11.2 | Unity 2020.3.22f1 | 1.0.0

<!--# Unity Version
    Unity 2020.3.22f1-->
    
<!--# ROS Version
    Ubuntu 18.04.06
    ROS Melodic-->

# Setup and Installation

1) Install [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)

2) Clone this Repository and run the automatic installer
```
git clone https://github.com/cxdcxd/RRS.git
cd rrs_ros
./install.sh
```

3) Download [Unity](https://unity3d.com/get-unity/download/archive) 2020.2.1f1 or above

4) Open Unity Hub and go to `Projects`, click `ADD` and then browse the folder `rrs_unity` from the downloaded repository and launch it. Make sure the `Unity Version` is set to `2020.3.22f1` or above

5) Install Ml-agent package (for RL)
```
wget https://github.com/Unity-Technologies/ml-agents/archive/refs/tags/release_17.zip
unzip release_17.zip -d ml-agent
open unity package manager and select the package.json from com.unity.ml-agents
open unity package manager and select the package.json from com.unity.ml-agents.extensions
```

6) Open the repository in a new terminal and type `cd rrs_ros` and build the workspace: `catkin_make`

7) Source the workspace: 
```
setup ~/rrs_ros/devel/setup.bash
```

# Run examples

## ROS Side:
Benchmark scene
```
roslaunch rrs_ros rrs_ros_benchmark.launch                          //for benchmark scene
```
Movo Robot Simulator scene
```
roslaunch rrs_ros rrs_ros.launch                                    //for core robot simulation
roslaunch rrs_ros rrs_moveit.launch                                 //for moveit 
roslaunch jog_launch jaco2.launch use_moveit:=true use_rviz:=true   //for arm jogging 
```

## Unity Side:
Benchmark scene
```
Open the scenes/BenchMark
Unity3D -> Play
```
Movo Robot Simulator
```
open the scenes/Demo
Unity3D -> Play
```

# Settings (rrs_ros) 
### open the config file from : (RRS/rrs_ros/src/core/cfg/config.yaml) 
    ntp_server_host_name: test               //define the ntp server hostname (current RRS is not using ntp ignored)
    local_network_address: 127.0.0.1         //local ip address
    consul_network_address: 127.0.0.1        //consul ip address
    consul_network_mask: 255.255.255.0       //consul network mask
    consul_network_port: 8500                //consul network port
    
# Settings (rrs_unity) 
### open the config file from : (RRS/rrs_unity/Config/config.xml)
    <?xml version="1.0" encoding="utf-8"?>
    <Config xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
      <consul_network_address>127.0.0.1</consul_network_address>
      <local_network_address>127.0.0.1</local_network_address>
      <consul_network_port>8500</consul_network_port>
      <consul_network_mask>255.255.255.0</consul_network_mask>
      <ntp_server_host_name>test</ntp_server_host_name> //define the ntp server hostname (current RRS is not using ntp ignored)
      <use_relative_origin>false</use_relative_origin>
    </Config>
    
# For RL and Gym development
## Virtual Environment  
```
mkdir ~/python-envs
python3 -m venv ~/python-envs/sample-env
source ~/python-envs/sample-env/bin/activate
pip3 install --upgrade pip
pip3 install --upgrade setuptools
deactivate 
```

## ML-Agent Installation 
```
pip3 install torch -f https://download.pytorch.org/whl/torch_stable.html
pip3 install -e ./ml-agents-envs
pip3 install -e ./ml-agents
```

## Tensor Board 
```
tensorboard --logdir results
```  

## Train example
```
mlagents-learn config/ppo/3DBall.yaml --run-id=first3DBallRun
```  
## NMPC-MP Installation

Instructions on setting up the framework


moveit_visual_tools


relaxedik


Install moveit under link: https://moveit.ros.org/install/


If not, install the Eigen


Install libccd under link: https://github.com/danfis/libccd
Git clone the package.
Follow the instruction on that site, but make sure compiling it in double precision, otherwise, compiling fcl library later may go error.
Using camke to compile is recommended. If using cmake, run "cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON .." to enable double precision


Install fcl(The Flexible Collision Library) under link: https://github.com/flexible-collision-library/fcl
Git clone the package.
Make sure to run "git checkout 1bddc981de578d971cc59eb54f5d248c9d803b25" to go to this older checkout point and then
Follow the instruction on that site. In the end run make to compile the code and then run sudo make install to install the libs into working path.


Install nlopt(a nonlinear optimization library) under link: https://nlopt.readthedocs.io/en/latest/
Download the compressed file and then,
compile it by:
mkdir build && cd build
cmake ..
make
sudo make install


Git clone this repository.
For melodic ros version, in file src/mpc/src/class/moveitTool.cpp and file src/mpc/include/moveitTool.h, change the const Eigen::Affine3d& getEEFTransform() const function definition to const Eigen::Isometry3d& getEEFTransform() const function definition. Because the getGlobalLinkTransform function for melodic version moveit returns Isometry3d reference, instead of Affine3d reference


kinova_control, kinova_description and j2s7s300_moveit_config pacakges in the kinova-ros folder are extracted and modified from the official package under link https://github.com/Kinovarobotics/kinova-ros. Only these 3 packages are modifed and used for the simulation of our project.


Run catkin build to complie mpc package for our mpc based planner, kinova for 3 kinova ros packages, chomp_jaco2 for chomp and ompl planners testing used for benchmark.
Note: Remember to source the moveit workspace first, and then source this workspace. Otherwise, the path of the those 3 kinova ros packages will be overwritten for unknown reason.
