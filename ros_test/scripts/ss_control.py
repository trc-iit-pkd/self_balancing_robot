#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry



import numpy as np
import math




g = 9.81


def quaternion_to_euler(w, x, y, z):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    
    :param w: Quaternion w component
    :param x: Quaternion x component
    :param y: Quaternion y component
    :param z: Quaternion z component
    :return: Tuple (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# class SelfBalancingRobot():

#     def __init__(self, mass_body, wheel_rad, lCOM, Ib, Iw, Im, mass_wheel):
        
#         # physical properties of the bot
#         self.mb = mass_body
#         self.mw = mass_wheel
#         self.r  = wheel_rad
#         self.l = lCOM
#         self.Ib = Ib
#         self.Iw = Iw
#         self.Im = Im

#         # friction params
#         self.mu1 = 0.2
#         self.mu2 = 0.2


#         # general matrix element properties
#         self.a11 = self.mb*self.l*self.l + self.Ib + self.Im
#         self.a12 = self.mb*self.r*self.l - self.Im
#         self.a21 = self.mb*self.r*self.l + self.mb*self.l*self.l + self.Ib
#         self.a22 = (self.mb+self.mw)*self.r*self.r + self.mb*self.r*self.l + self.Iw
#         self.delta = (self.a11*self.a22-self.a12*self.a21)


#         # matrix A elements
#         self.a1 = ((self.a22-self.a12)*self.mb*g*self.l)/self.delta
#         self.a2 = ((self.a11-self.a21)*self.mb*g*self.l)/self.delta
#         self.a3 = (-(self.mu1*self.a22))/self.delta
#         self.a4 = (self.mu1*self.a21)/self.delta
#         self.a5 = (self.mu1*self.a22+self.mu2*self*self.a12)/self.delta
#         self.a6 = -(self.m1*self.a21+self.mu2*self.a11)/self.delta

#         # matrix B elements
#         self.b1 = -(self.a22*self.)

    



# class StateSpaceModel():

#     def __init__(self, A, B, C, D, u):
#         self.A = A
#         self.B = B
#         self.C = C
#         self.D = D
#         self.u = u # control input to the system
    
class StateSpaceController(Node):

    def __init__(self):
        super().__init__("state_space_controller_node")

        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "unfiltered/odometry", self.odom_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.wheel_state_callback, 10)
        self.twist_pub = self.create_publisher(Twist, "cmd_vel", 10)

        
        self.sample_frequency = 10 # in Hz
        controller_period = 1/self.sample_frequency # in seconds
        self.timer = self.create_timer(controller_period, self.controller_loop)

        # target states
        self.target_pitch = 0.0
        self.target_pitch_vel = 0.0
        self.target_wheel_vel = 1.0
        self.target_pose = 0.0
        
        # feedback states
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        self.current_poseX = 0.0

        self.left_wheel_vel = 0.0 #in rad/s
        self.right_wheel_vel = 0.0 #in rad/s
        self.wheel_vel = 0.0
        
        self.current_pitch_vel = 0.0

        # input signal
        self.input_signal = 0.0


        # gains
        self.prev_time_update = self.get_clock().now().nanoseconds
        self.eIntegral = 0.0

        self.k1 = 3.0
        self.k2 = 0.0
        self.k3 = 0.0
        self.ki = 0.0 # use last


    def imu_callback(self, msg):
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        roll, self.current_pitch, self.current_yaw = quaternion_to_euler(qw, qx, qy, qz)
        # self.get_logger().info(f"pitch_angle: {self.current_pitch}")

            
    def odom_callback(self, msg):
        self.current_poseX = msg.pose.pose.position.x
        # self.get_logger().info(f"x_pose: {self.current_poseX}")

    def wheel_state_callback(self, msg):
        msg = JointState()
        num_joints = len(msg.velocity)

        if num_joints >= 2:
            self.left_wheel_vel = msg.velocity[0]
            self.right_wheel_vel = msg.velocity[1]
            self.wheel_vel = (self.left_wheel_vel+self.right_wheel_vel)/2
        else:
            return
        

    def controller_loop(self):
        
        twist_msg = Twist()

        current_time = self.get_clock().now().nanoseconds
        dt = (current_time-self.prev_time_update)*1.0e-9
        
        # self.get_logger().info(f"dt: {dt}")

        control_vel = -self.k1*(self.target_pitch-self.current_pitch)-self.k2*(self.target_pitch_vel-self.current_pitch_vel)-self.k3*(self.target_wheel_vel-self.wheel_vel)
        twist_msg.linear.x = control_vel

        self.get_logger().info(f"control_vel: {control_vel}")
        self.prev_time_update = current_time
        

        self.twist_pub.publish(twist_msg)    




def main(args=None):
    try:
        rclpy.init(args=args)
        
        node = StateSpaceController()

        rclpy.spin(node)


        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass



if __name__ == '__main__':
    main()