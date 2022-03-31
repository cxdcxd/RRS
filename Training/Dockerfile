FROM nvidia/cuda:11.2.2-cudnn8-devel-ubuntu18.04

LABEL maintainer "Tapan Sharma - tapan.sharma@tum.de"

# `work_dir` must be an absolute path.
ARG work_dir=/root/Workspace
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin

# Install `apt-get` deps.
RUN apt-get update && apt-get install -y \
        tzdata \
        curl \
        build-essential \
        python3 \
        python3-dev \
        python3-tk \
        python-opengl \
        python3-pip \
        libsm6 \
        libxext6 \
        libglib2.0-0 \
        libxrender1 \
        wget \
        unzip \
        vim


# Install setuptools to better find certain packages.
RUN pip3 install -U setuptools

# Install ML Agents version 1.17.0. Available as python package 0.26.0
RUN python3 -m pip install mlagents==0.26.0

# Move to the base library dir
WORKDIR ${work_dir}/