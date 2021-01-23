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

# Seutp and Installation

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
```
consul.sh
roslaunch rrs_ros rrs_ros.launch
roslaunch rrs_ros rrs_moveit.launch
```

Unity Side:
Unity3D -> Play

# Settings (rrs_ros)
    ntp_server_host_name: test               //define the ntp server hostname
    local_network_address: 192.168.92.139    //local ip address
    consul_network_address: 192.168.92.139   //consul ip address
    consul_network_mask: 255.255.255.0       //consul network mask
    consul_network_port: 8500                //consul network port
    
# Settings (rrs_unity)
    <?xml version="1.0" encoding="utf-8"?>
    <Config xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
      <consul_network_address>192.168.92.138</consul_network_address>
      <local_network_address>192.168.92.1</local_network_address>
      <consul_network_port>8500</consul_network_port>
      <consul_network_mask>255.255.255.0</consul_network_mask>
      <ntp_server_host_name>test</ntp_server_host_name>
      <use_relative_origin>false</use_relative_origin>
    </Config>
    
    
# Virtual Environment 
```
mkdir ~/python-envs
python3 -m venv ~/python-envs/sample-env
source ~/python-envs/sample-env/bin/activate
pip3 install --upgrade pip
pip3 install --upgrade setuptools
deactivate 
```

# ML-Agent Installation
```
pip3 install torch -f https://download.pytorch.org/whl/torch_stable.html
pip3 install -e ./ml-agents-envs
pip3 install -e ./ml-agents
```

# Tensor Board
```
tensorboard --logdir results
```  

# Train
```
mlagents-learn config/ppo/3DBall.yaml --run-id=first3DBallRun
```  
    
