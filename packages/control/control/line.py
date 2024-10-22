#!/usr/bin/env python3

# Importing necessary libraries
from sympy import *
from matplotlib import pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time as tm
init_printing(use_unicode=False, wrap_line=False)

# Class definition for Arm Controller Node
class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        # Publishers for arm joint and wheel velocities
        self.arm_joint_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

    # Method to publish joint angles    
    def publish_joint_angles(self, angles):
        msg = Float64MultiArray()
        msg.data = angles
        self.arm_joint_pub.publish(msg)

    # Method to publish wheel velocities
    def publish_wheel_velocities(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.wheel_velocities_pub.publish(msg)

# Symbolic variable definitions for Denavit-Hartenberg parameters
alpha, a, d, theta = symbols('alpha a d theta')
theta1, theta2, theta3, theta4 = symbols('theta1 theta2 theta3 theta4')
# Joint angle limits
q1_min, q1_max = 0, 6.3
q2_min, q2_max = -3.14, 0
q3_min, q3_max = 0, 5.54
q4_min, q4_max = -4.3, 0

# Arrays to hold DH parameters
alpha_array = [-pi/2, 0, 0, 0]
theta_array = [theta1, theta2, theta3, theta4]
d_array = [.16239, 0, 0, 0]
a_array = [0, .2032, .2032, .7620]

# Matrices for transformations
Rz = Matrix([[cos(theta), -sin(theta), 0, 0],
             [sin(theta),  cos(theta), 0, 0],
             [0,        0, 1, 0],
             [0,        0, 0, 1]])

Tz = Matrix([[1,  0,  0,  0],
             [0,  1,  0,  0],
             [0,  0,  1,  d],
             [0,  0,  0,  1]])

Tx = Matrix([[1,  0,  0,  a],
             [0,  1,  0,  0],
             [0,  0,  1,  0],
             [0,  0,  0,  1]])

Rx = Matrix([[1,        0,        0,  0],
             [0,  cos(alpha), -sin(alpha),  0],
             [0,  sin(alpha),  cos(alpha),  0],
             [0,        0,        0,  1]])


# Generating transformation matrices for each joint
T1_gen = Rz*Tz*Tx*Rx
T2_gen = Rz*Tz*Tx*Rx
T3_gen = Rz*Tz*Tx*Rx
T4_gen = Rz*Tz*Tx*Rx

# Substituting parameters into transformation matrices
T1 = T1_gen.subs(alpha, alpha_array[0]).subs(
    theta, theta_array[0]).subs(d, d_array[0]).subs(a, a_array[0])
T2 = T2_gen.subs(alpha, alpha_array[1]).subs(
    theta, theta_array[1]).subs(d, d_array[1]).subs(a, a_array[1])
T3 = T3_gen.subs(alpha, alpha_array[2]).subs(
    theta, theta_array[2]).subs(d, d_array[2]).subs(a, a_array[2])
T4 = T4_gen.subs(alpha, alpha_array[3]).subs(
    theta, theta_array[3]).subs(d, d_array[3]).subs(a, a_array[3])

# Transformation matrices with respect to the base frame
H0_1 = T1
H0_2 = T1*T2
H0_3 = T1*T2*T3
H0_4 = T1*T2*T3*T4

# Print the final transformation matrix
pprint(H0_4)

# Position vector of the end effector
P = Matrix([[H0_4[3]], [H0_4[7]], [H0_4[11]]])

# Partial derivatives of P with respect to each joint angle
Par1 = diff(P, theta1)
Par2 = diff(P, theta2)
Par3 = diff(P, theta3)
Par4 = diff(P, theta4)

# Z-axis vectors from transformation matrices
Z0_1 = Matrix([[H0_1[2]], [H0_1[6]], [H0_1[10]]])
Z0_2 = Matrix([[H0_2[2]], [H0_2[6]], [H0_2[10]]])
Z0_3 = Matrix([[H0_3[2]], [H0_3[6]], [H0_3[10]]])
Z0_4 = Matrix([[H0_4[2]], [H0_4[6]], [H0_4[10]]])

# Jacobian components and full Jacobian matrix
J1 = Matrix([[Par1], [Z0_1]])
J2 = Matrix([[Par2], [Z0_2]])
J3 = Matrix([[Par3], [Z0_3]])
J4 = Matrix([[Par4], [Z0_4]])

J = Matrix([[J1, J2, J3, J4]])


# Circle parameters and initial joint angles
r = 0.11

q1 = 0.0
q2 = 0.0
q3 = 0.0
q4 = 0.0


# Function to clamp angle within limits
def clamp_angle(angle, min_limit, max_limit):
    # Clamps angle to be within the specified limits
    if angle < min_limit:
        return min_limit
    elif angle > max_limit:
        return max_limit
    return angle

# Initialize ROS2
rclpy.init(args=None)
arm_controller_node = ArmControllerNode()
wheel_velocities = Float64MultiArray()
wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]

wheel_velocities = [0.0, 0.0, 0.0, 0.0]

# Main control loop
timer = tm.time()
run_duration = 20.0

while True:
    current_time = tm.time()
    
    if current_time - timer >= run_duration:
        wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        arm_controller_node.publish_wheel_velocities(wheel_velocities)
        break
    
    if current_time - timer <= 10.0:
        wheel_velocities = [3.0, -3.0, 3.0, -3.0]
    elif 10.0 < current_time - timer <= 15.0:
        wheel_velocities = [-6.0, -7.0, -6.0, -7.0]
    elif 15.0 < current_time - timer <= 20.0:
        wheel_velocities = [3.0, -3.0, 3.0, -3.0]
    
    arm_controller_node.publish_wheel_velocities(wheel_velocities)
    
    tm.sleep(0.1)

last_publish_time = tm.time()

# Calculate and publish joint angles in a circular trajectory
time = np.linspace(0, 200, num=400)
dt = 200 / 300

xA, yA, zA = 0.0, 0.0, 0.0
xB, yB, zB = 0.0, 0.0, 1.0

total_distance = np.sqrt((xB - xA)**2 + (yB - yA)**2 + (zB - zA)**2)
velocity = total_distance / 20 

for t in time:
    # Calculate and publish joint velocities
    Vx = -((2*pi*r)/200)*sin((2*pi*t)/200)
    Vy = 0.0
    Vz = ((2*pi*r)/200)*cos((2*pi*t)/200)
    omega_x = 0.0
    omega_y = 0.0
    omega_z = 0.0

    X_dot = Matrix([[Vx], [Vy], [Vz], [omega_x], [omega_y], [omega_z]])

    J_Applied = J.subs({theta1: q1, theta2: q2, theta3: q3, theta4: q4})

    J_T = J_Applied.transpose()

    q_dot = (J_Applied.pinv() * X_dot).evalf()

    q1_dot = q_dot[0]
    q2_dot = q_dot[1]
    q3_dot = q_dot[2]
    q4_dot = q_dot[3]
    
    q1 = clamp_angle(q1 + q1_dot * dt, q1_min, q1_max)
    q2 = clamp_angle(q2 + q2_dot * dt, q2_min, q2_max)
    q3 = clamp_angle(q3 + q3_dot * dt, q3_min, q3_max)
    q4 = clamp_angle(q4 + q4_dot * dt, q4_min, q4_max)

    current_time = tm.time()
    if current_time - last_publish_time >= 0.001:
        q1 = float(q1)
        q2 = float(q2)
        q3 = float(q3)
        q4 = float(q4)
        print(q1, q2, q3, q4)
        arm_controller_node.publish_joint_angles([q1, q2, q3, q4, 1.0, -1.0, -1.0, 1.0])
        last_publish_time = current_time

# Publish final joint angles and shutdown ROS2
arm_controller_node.publish_joint_angles([q1, q2, q3, q4, 0.0, 0.0, 0.0, 0.0])
    
rclpy.spin_once(arm_controller_node, timeout_sec=0)
rclpy.shutdown()



