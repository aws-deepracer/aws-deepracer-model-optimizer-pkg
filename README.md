# DeepRacer Model Optimizer Package

## Overview

The DeepRacer Model Optimizer ROS package creates the *model_optimizer_node* which is part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/aws-racer/aws-deepracer-launcher).

This node is responsible for running the Intel OpenVino Model Optimizer script for the DeepRacer reinforcement learning models to obtain the intermediate representation xml files and other optimizer artifacts required to run inference with the model.

More details about the Intel OpenVino Model Optimizer can be found here:
https://docs.openvinotoolkit.org/2021.1/openvino_docs_MO_DG_Deep_Learning_Model_Optimizer_DevGuide.html

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages and libraries installed to run the model_optimizer_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-racer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The model_optimizer_pkg specifically depends on the following ROS2 packages as build 
and execute dependencies:

* *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the model_optimizer_pkg on the DeepRacer device:

        git clone https://github.com/aws-racer/aws-deepracer-model-optimizer-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-model-optimizer-pkg
        rosws update

1. Resolve the model_optimizer_pkg dependencies:

        cd ~/deepracer_ws/aws-deepracer-model-optimizer-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the model_optimizer_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-model-optimizer-pkg && colcon build --packages-select model_optimizer_pkg deepracer_interfaces_pkg

## Usage

The model_optimizer_node provides a very specific and core functionality to optimize the Reinforcement learning models that are trained on the AWS DeepRacer Simulator. Intel OpenVino provides APIs and scripts to create a intermediate representation that can be used for faster model inference. Although the nodes is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built model_optimizer_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-model-optimizer-pkg/install/setup.bash  

1. Launch the model_optimizer_pkg using the launch script:

        ros2 launch model_optimizer_pkg model_optimizer_pkg_launch.py

## Launch Files

The  model_optimizer_pkg_launch.py is also included in this package that gives an   example of how to launch the nodes independently from the core application.

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

### model_optimizer_node

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|model_optimizer_server|ModelOptimizeSrv|Service that is called to launch the Intel OpenVino model optimizer script for the specific model with appropriate model and platform specific parameters set.|


