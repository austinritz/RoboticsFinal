from controller import Robot
from controller import Keyboard
import math
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import sys
import tempfile
import time

robot = Robot()
IKPY_MAX_ITERATIONS = 4

robot_name = robot.getName()
print('Name of the robot: ' + robot_name + '\n')

filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(robot.getUrdf().encode('utf-8'))
my_chain = Chain.from_urdf_file(filename)
for i in [0, 6]:
    my_chain.active_links_mask[i] = False

print(my_chain.links)
m7 = robot.getDevice('joint_base_to_jaw_1')
m8 = robot.getDevice('joint_base_to_jaw_2')
part_names = ("joint_1", "joint_2", "joint_3", "joint_4",
            "joint_5",  "joint_6",  "joint_base_to_jaw_1", "joint_base_to_jaw_2")

motors = []
timestep = int(4 * robot.getBasicTimeStep())
for link in my_chain.links:
    if 'joint' in link.name and link.name != "hand_link_base_gripper_1_joint":
        motor = robot.getDevice(link.name)
        motor.setVelocity(0.4)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timestep)
        motors.append(motor)
        
def setPoints(targets):
    new_targets = []
    home = [0,-0.4, .315]
    above_home = [0,-0.4, .5]
    for t in targets:
        above_t = [t[0],t[1],t[2]+0.1]
        new_targets.append(above_t)
        new_targets.append(t)
        new_targets.append(above_t)
        new_targets.append(above_home)
        new_targets.append(home)
        home = [home[0],home[1],home[2]+0.03]
    return new_targets
m7.setVelocity(0.1)
m8.setVelocity(.1)
index = 0
blue = [0.1,-0.4,0.315]
red = [-0.2, -0.3, 0.315]
green = [-0.1, -0.4, 0.315]
yellow = [0.19, -0.32, 0.315]

targets = []
targets.append(blue)
targets.append(red)
targets.append(green)
targets.append(yellow)
targets = setPoints(targets)
m7.setPosition(0)
m8.setPosition(0)
robot.step(500)
m7.setPosition(0.01)
m8.setPosition(0.01)
robot.step(500)
m7.setPosition(0)
m8.setPosition(0)
robot.step(500)
m7.setPosition(0.01)
m8.setPosition(0.01)
robot.step(500)
claw_position = 0.01
while robot.step(timestep) != -1:
    m7.setPosition(claw_position)
    m8.setPosition(claw_position)
    target = targets[index]
    
    
    x = target[0]
    y = - (target[1])
    z = target[2]
    
    target = [x,y,z]
    
    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
    ikResults = my_chain.inverse_kinematics(target,target_orientation = [0,0,1],max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    position = my_chain.forward_kinematics(ikResults)
    #print(position)
    
    
    
    squared_distance = (position[0, 3] - x)**2 + (position[1, 3] - y)**2 + (position[2, 3] - z)**2
    print(squared_distance)
    if squared_distance < 0.0000000001:
        if index % 5 == 4: #open jaw
            claw_position = 0.01
            m7.setPosition(claw_position)
            m8.setPosition(claw_position)
            robot.step(1000)
        elif index % 5 == 1:
            claw_position = -0.001
            m7.setPosition(claw_position)
            m8.setPosition(claw_position)
            robot.step(1000)
        index+= 1
            
    # Actuate the arm motors with the IK results.
    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i + 1])