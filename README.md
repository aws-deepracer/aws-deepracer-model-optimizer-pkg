# AWS DeepRacer model optimizer package

## Overview

The AWS DeepRacer Model Optimizer ROS package creates the `model_optimizer_node`, which is part of the core AWS DeepRacer application and launches from the `deepracer_launcher`. For more information about the application and the components, see the [aws-deepracer-launcher repository](https://github.com/aws-deepracer/aws-deepracer-launcher).

This node is responsible for running the Intel OpenVino Model Optimizer script for the AWS DeepRacer reinforcement learning models to obtain the intermediate representation XML files and other optimizer artifacts required to run inference with the model.

For more information about the Intel OpenVino Model Optimizer, see the [Model Optimizer Developer Guide](https://docs.openvinotoolkit.org/2021.1/openvino_docs_MO_DG_Deep_Learning_Model_Optimizer_DevGuide.html).

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

Follow these instructions to install the AWS DeepRacer model optimizer package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the `model_optimizer_pkg`. For more information about the preinstalled set of packages and libraries on the AWSDeepRacer, and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `model_optimizer_pkg` specifically depends on the following ROS2 packages as build 
and run dependencies:

* `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and building

Open up a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the model_optimizer_pkg on the DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-model-optimizer-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-model-optimizer-pkg
        rosws update

1. Resolve the `model_optimizer_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-model-optimizer-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `model_optimizer_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-model-optimizer-pkg && colcon build --packages-select model_optimizer_pkg deepracer_interfaces_pkg

## Usage

The `model_optimizer_node` provides a very specific and core functionality to optimize the reinforcement learning models that are trained on the AWS DeepRacer simulator. Intel OpenVino provides APIs and scripts to create a intermediate representation that can be used for faster model inference. Although the node is built to work with the AWS DeepRacer application, it can be run independently for development, testing, and debugging purposes.

### Run the node

To launch the built `model_optimizer_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-model-optimizer-pkg/install/setup.bash  

1. Launch the `model_optimizer_pkg` using the launch script:

        ros2 launch model_optimizer_pkg model_optimizer_pkg_launch.py

## Launch files

The `model_optimizer_pkg_launch.py`, included in this package, provides an example demonstrating how to launch the nodes independently from the core application.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='model_optimizer_pkg',
                namespace='model_optimizer_pkg',
                executable='model_optimizer_node',
                name='model_optimizer_node'
            )
        ])

## Node Details

### `model_optimizer_node`

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`model_optimizer_server`|`ModelOptimizeSrv`|Service that is called to launch the Intel OpenVino model optimizer script for the specific model with appropriate model and platform specific parameters set.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)



