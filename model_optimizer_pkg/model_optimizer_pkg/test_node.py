#!/usr/bin/env python

#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

'''
Instructions to use the test node:

SSH into the device in three different terminals.
Run the following in one of the terminal.

systemctl stop deepracer-core.service

Terminal 1:
Start the roscore

  source /opt/ros/kinetic/setup.bash

  source /opt/aws/deepracer/setup.bash

  source /opt/aws/intel/dldt/bin/setupvars.sh

  roscore

Terminal 2:
    
Start the inference node
 sudo su
 
 source /opt/ros/kinetic/setup.bash
 
 source /opt/aws/deepracer/setup.bash
 
 source /opt/aws/intel/dldt/bin/setupvars.sh

 cd /opt/aws/deepracer/lib/inference_pkg

 rosrun inference_pkg inference_node

If you need to optmize the model, start the model optimizer node as well in another terminal after sourcing the scripts as in step 2
 
 cd /opt/aws/deepracer/lib/inference_pkg

 rosrun inference_pkg model_optimizer_node.py


Terminal 3: 
 sudo su
 
 source /opt/ros/kinetic/setup.bash
 
 source /opt/aws/deepracer/setup.bash
 
 source /opt/aws/intel/dldt/bin/setupvars.sh

 cd /opt/aws/deepracer/lib/inference_pkg

Edit the test node to set the model path or path to model xml.

 rosrun inference_pkg test_node.py

'''

# import cv2
# from enum import Enum
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# from inference_pkg.srv import ModelOptimize, LoadModelSrv, InferenceStateSrv
# from inference_pkg.msg import InferResultsArray, InferResults
# from media_pkg.msg import cameraMSG
# from sensor_fusion_pkg.msg import sensorMSG
# from media_pkg.srv import VideoStateSrv
# from servo_pkg.srv import CarCtrlSrv
# from ctrl_pkg.srv import ModelStateSrv

# import rospy


# class InputKeys(Enum):
#     observation = 1
#     LIDAR = 2
#     SECTOR_LIDAR = 3
#     LEFT_CAMERA = 4
#     FRONT_FACING_CAMERA = 5
#     STEREO_CAMERAS = 6

# image_counter = 0

# left_image_path = '/home/deepracer/Desktop/left-image.jpg'
# right_image_path = '/home/deepracer/Desktop/right-image.jpg'
# model_path = "model/model" # <<name_of_the_model_folder_in_artificats_directory>/<name_of_pb_file>
# model_optimize_flag = False # Set to true if model optimization is needed
# path_to_model_xml = '/opt/aws/deepracer/artifacts/model/model.xml' # path to model xml if the model is already optimized and model_optimize_flag is set to False
# dump_data_to_directory = False   # Set to True if images from inference_results array is to be dumped from the inference_cb to /home/deepracer/data_dual_cam folder NOTE: mkdir /home/deepracer/data_dual_cam if this is set to True
# sensor_list = [InputKeys.STEREO_CAMERAS.value]
# lidar_channels = 64
# training_algorithm = 1
# action_space_type = 1

# def inference_cb(data):
#     global image_counter
#     global dump_data_to_directory

#     p = list()
#     for result in data.results:
#        p.append(result.classProb)
#     print(p)
#     print()
#     bridge = CvBridge()

#     if dump_data_to_directory:
#         for i, img in enumerate(data.images):
#             try:
#                 cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
#             except CvBridgeError as e:
#                  print(e)
#             image_ = cv2.resize(cv_image, (160, 120))
#             image_name = 'image-{}-{}'.format(i, image_counter)

#             cv2.imwrite('/home/deepracer/data_dual_cam/{}.jpg'.format(image_name), image_)

#     image_counter += 1

# rospy.init_node('test_node', anonymous=True)
# if model_optimize_flag == True:
#     rospy.wait_for_service('model_optimizer_server')
# rospy.wait_for_service('load_model')

# if model_optimize_flag == True:
#     model_optimize = rospy.ServiceProxy('model_optimizer_server', ModelOptimize)
#     resp = model_optimize(model_path, sensor_list, training_algorithm, "BGR", 160, 120, 1, lidar_channels ,1)
#     # Expected output /opt/aws/deepracer/artifacts/model/model.xml
#     if resp.hasError:
#         print "Model optimizer error"
#         exit()
#     else:
#         print resp.artifactPath

# load_model = rospy.ServiceProxy('load_model', LoadModelSrv)
# if model_optimize_flag == True:
#     model_resp = load_model(resp.artifactPath, 0, 1, action_space_type)
# else:
#     model_resp = load_model(path_to_model_xml, 0, 1, action_space_type)
# print "Model load error {}".format(model_resp.error)

# infer_state = rospy.ServiceProxy('inference_sate', InferenceStateSrv)
# state_resp = infer_state(1, 0)
# print "Infer state error {}".format(state_resp.error)

# rospy.Subscriber("rl_results", InferResultsArray, inference_cb)

# cv_image_left = cv2.imread(left_image_path,1)
# cv_image_right = cv2.imread(right_image_path, 1)

# sensor_pub = rospy.Publisher("sensor_msg", sensorMSG, queue_size=1)
# bridge = CvBridge()

# while not rospy.is_shutdown():
#     try:
#       msg = cameraMSG()
#       msg.images = [bridge.cv2_to_imgmsg(cv_image_left, "bgr8"), bridge.cv2_to_imgmsg(cv_image_right, "bgr8")]
#       sensor_msg = sensorMSG()
#       sensor_msg.camera_msg = msg
#       # Add lidar data if the model takes in lidar values
#       sensor_msg.lidar_data = [] 
#       sensor_pub.publish(sensor_msg)
#     except CvBridgeError as e:
#         print e
