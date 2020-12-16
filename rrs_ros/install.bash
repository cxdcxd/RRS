#!/bin/bash


mkdir -p src/consul/data

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
fi

