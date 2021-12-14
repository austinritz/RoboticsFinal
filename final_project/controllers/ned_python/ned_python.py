# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Ned_Controller controller in python.

# Webots controller for the Niryo Ned robot.
# With this controller, you can see the 6 different axis of the robot moving
# You can also control the robots with your keyboard and launch a Pick and Pack


from controller import Robot
from controller import Keyboard
import math
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

robot = Robot()

robot_name = robot.getName()
print('Name of the robot: ' + robot_name + '\n')


# Init the motors - the Ned robot is a 6-axis robot arm
# You can find the name of the rotationalMotors is the device parameters of each HingeJoints
m1 = robot.getDevice('joint_1')
m2 = robot.getDevice('joint_2')
m3 = robot.getDevice('joint_3')
m4 = robot.getDevice('joint_4')
m5 = robot.getDevice('joint_5')
m6 = robot.getDevice('joint_6')
m7 = robot.getDevice('joint_base_to_jaw_1')
m8 = robot.getDevice('joint_base_to_jaw_2')
m1.setPosition(0)
m2.setPosition(0)
m3.setPosition(0)
m4.setPosition(0)
m5.setPosition(0)
m6.setPosition(0)
m7.setPosition(0)
m8.setPosition(0)



