# Realistic Robotic Simulator

![Alt text](sample.png?raw=true "Title")

| OS  | Kernel Version | ROS Version | Nvidia Driver Version | CUDA Version | Unity Version | Ml Agents
| --- | ----------| ----------- | ------------ | ------------ | ------------ | ------------ 
| Ubuntu 18.04.05 LTS | 5.4.0-58-generic | ROS Melodic | 460.27.04 | 11.2 | Unity 2020.2.1f1 | 12

<!--# Unity Version
    2020.2.1f1-->
    
<!--# ROS Version
    Ubuntu 18.04
    ROS Melodic-->

# Setup and Installation

1) Install [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)

2) Clone this Repository 
```
git clone https://github.com/cxdcxd/RRS.git
cd rrs_ros
./install.sh
```

3) Download [Unity](https://unity3d.com/get-unity/download/archive) 2020.2.1f1

4) Open Unity Hub and go to `Projects`, click `ADD` and then browse the folder `rrs_unity` from the downloaded repository and launch it. Make sure the `Unity Version` is set to `2020.2.1f1`

5) Open the repository in a new terminal and type `cd rrs_ros` and build the workspace: `catkin_make`

6) Source the workspace: 
```
setup ~/rrs_ros/devel/setup.bash
```

# Run

ROS Side:
##Benchmark
```
roslaunch rrs_ros rrs_ros_benchmark.launch                          //for benchmark scene
```
##Movo Robot Simulator
```
roslaunch rrs_ros rrs_ros.launch                                    //for core robot simulation
roslaunch rrs_ros rrs_moveit.launch                                 //for moveit 
roslaunch jog_launch jaco2.launch use_moveit:=true use_rviz:=true   //for arm jogging 
```

Unity Side:
```
wget https://github.com/Unity-Technologies/ml-agents/archive/release_12.zip
unzip release_12.zip -d ml-agent
open unity package manager and select the package.json from com.unity.ml-agents
open unity package manager and select the package.json from com.unity.ml-agents.extensions
```

##Benchmark
```
Open the scenes/BenchMark
Unity3D -> Play
```
##Movo Robot Simulator
```
open the scenes/Demo
Unity3D -> Play
```

# Settings (rrs_ros) (RRS/rrs_ros/src/core/cfg/config.yaml) 
    ntp_server_host_name: test               //define the ntp server hostname (current RRS is not using ntp ignored)
    local_network_address: 127.0.0.1         //local ip address
    consul_network_address: 127.0.0.1        //consul ip address
    consul_network_mask: 255.255.255.0       //consul network mask
    consul_network_port: 8500                //consul network port
    
# Settings (rrs_unity) (RRS/rrs_unity/Config/config.xml)
    <?xml version="1.0" encoding="utf-8"?>
    <Config xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
      <consul_network_address>127.0.0.1</consul_network_address>
      <local_network_address>127.0.0.1</local_network_address>
      <consul_network_port>8500</consul_network_port>
      <consul_network_mask>255.255.255.0</consul_network_mask>
      <ntp_server_host_name>test</ntp_server_host_name> //define the ntp server hostname (current RRS is not using ntp ignored)
      <use_relative_origin>false</use_relative_origin>
    </Config>
    
    
# Virtual Environment  (for RL development)
```
mkdir ~/python-envs
python3 -m venv ~/python-envs/sample-env
source ~/python-envs/sample-env/bin/activate
pip3 install --upgrade pip
pip3 install --upgrade setuptools
deactivate 
```

# ML-Agent Installation (for RL development)
```
pip3 install torch -f https://download.pytorch.org/whl/torch_stable.html
pip3 install -e ./ml-agents-envs
pip3 install -e ./ml-agents
```

# Tensor Board (for RL development)
```
tensorboard --logdir results
```  

# Train (for RL development)
```
mlagents-learn config/ppo/3DBall.yaml --run-id=first3DBallRun
```  
    
