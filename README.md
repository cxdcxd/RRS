# Realestic Robotic Simulator

![Alt text](sample.png?raw=true "Title")

# Unity Version
    2020.1.f1
    
# ROS Version
    Ubuntu 18.04
    ROS Melodic

# Installation
    1) https://github.com/oliora/ppconsul
    2) https://www.consul.io/

# Run
    1) consul.sh
    2) roslaunch rrs_ros rrs_ros.launch
    3) run Unity3D

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
    
    
