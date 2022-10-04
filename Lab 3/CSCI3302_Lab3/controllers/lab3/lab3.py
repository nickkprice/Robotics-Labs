"""lab3 controller."""
# Copyright University of Colorado Boulder 2022
# CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math

# TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 6.67 # [rad/s]
MAX_SPEED_MS = 0.22 # [m/s]
AXLE_LENGTH = 0.16 # [m]



MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
# You should not use target_pos for storing waypoints. Leave it unmodified and 
# use your own variable to store waypoints leading up to the goal
target_pos = ('inf', 'inf') 
robot_parts = []

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

# Rotational Motor Velocity [rad/s]
vL = 0
vR = 0

# TODO
# Create you state and goals (waypoints) variable here
# You have to MANUALLY figure out the waypoints, one sample is provided for you in the instructions

class State:
    name = None;
    def __init__(self, name):
        self.name = name
    def __eq__(self, obj):
        return name in obj;
    def change_state(self, name):
        self.name = name
state = State('test')

waypoints = [(5,-3,.3)]
wp_prog = 0
p = 0
while robot.step(timestep) != -1:
    #print(robot_parts[MOTOR_LEFT].getMaxVelocity())

    # STEP 2.1: Calculate error with respect to current and goal position
    goal = waypoints[wp_prog]
    p = math.sqrt(abs(goal[0] - pose_x)**2 + abs(goal[1] - pose_y)**2)
    
    # STEP 2.2: Feedback Controller
    a = math.atan2(goal[1],goal[0])
    n = goal[2] - pose_theta
    
    
    pass
    
    # STEP 1: Inverse Kinematics Equations (vL and vR as a function dX and dTheta)
    # Note that vL and vR in code is phi_l and phi_r on the slides/lecture
    #vL = (2*dX - dTheta) / 2
    #vR = (2*dX + dTheta) / 2
    
    pass
    
    # STEP 2.3: Proportional velocities
    vL = 0 # Left wheel velocity in rad/s
    vR = 0 # Right wheel velocity in rad/s
    pass

    # STEP 2.4: Clamp wheel speeds
    pass


    
    # TODO
    # Use Your Lab 2 Odometry code after these 2 comments. We will supply you with our code next week 
    # after the Lab 2 deadline but you free to use your own code if you are sure about its correctness
    
    # NOTE that the odometry should ONLY be a function of 
    # (vL, vR, MAX_SPEED, MAX_SPEED_MS, timestep, AXLE_LENGTH, pose_x, pose_y, pose_theta)
    # Odometry code. Don't change speeds (vL and vR) after this line
    
    
    

    ########## End Odometry Code ##################
    
    ########## Do not change ######################
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28
    ###############################################

    # TODO
    # Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(0)
    robot_parts[MOTOR_RIGHT].setVelocity(0)

    