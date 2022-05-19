# Realistic Robotic Simulator (RRS) for Pouring Liquids

![Alt text](sample2.png?raw=true "Title")

| OS  | Kernel Version | ROS Version | Nvidia Driver Version | CUDA Version | Unity Version | Ml Agents
| --- | ----------| ----------- | ------------ | ------------ | ------------ | ------------ 
| Ubuntu 18.04.06 LTS | 5.4.0-58-generic | ROS Melodic | 460.27.04 | 11.2 | Unity 2020.3.22f1 | Release_17

<!--# Unity Version
    Unity 2020.3.22f1-->
    
<!--# ROS Version
    Ubuntu 18.04.06
    ROS Melodic-->

# System architecure and ROS2Unity bridge
The pdf presentation file: [ROS2Unity](RRS1.0.pdf) 

# Setup and Installation

1) Install [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) Melodic

2) Clone this Repository and run the automatic installer
```
git clone https://github.com/cxdcxd/RRS.git
cd rrs_ros
./install.sh
```

3) Download [Unity](https://unity3d.com/get-unity/download/archive) 2020.3.22f1 or above

4) Open Unity Hub and go to `Projects`, click `ADD` and then browse the folder `rrs_unity` from the downloaded repository and launch it. Make sure the `Unity Version` is set to `2020.3.22f1` or above

5) Install Ml-agent release 17 package (for RL)
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

# Run example

## Step 1: (ROS Side)
```
ros launch rrs_ros rrs_main.launch 
```

## Step 2: (Unity Side)
```
open the scenes/DemoLiquidPouring
Unity3D -> Play
```

## Step 3: (ROS Side)
```
ros launch mpc real_movo_sc.launch                         
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
    
# For RL and Gym Training (No need to ROS Side)
```
open the Training\Assets\Scenes\NoParticleState.unity
```

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
tensorboard.exe --logdir=".\results\PourNet-LSTM" --host="0.0.0.0" --port=6006
```  

## Train example
```
 mlagents-learn.exe .\TrainingConfig\TrainingConfig\ppo_curriculum_curiosity_lstm.yaml --env=".\ServerBuild\ServerBuild\LearnPouring" --num-envs=32 --base-port=5000 --run-id="PourNet-LSTM-Left-2"
```  

