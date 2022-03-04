#!/usr/bin/env python3

import math
import numpy
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from cartesian_control.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import tf
from threading import Thread, Lock
import time

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(object):
    def __init__(self):
    #Load robot from parameter server
        self.robot = URDF.from_parameter_server()

    #Subscribe to current joint state of the robot
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

    #This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

    #This is a mutex
        self.mutex = Lock()

    #Subscribers and publishers for for cartesian control
        rospy.Subscriber('/cartesian_command', CartesianCommand, self.get_cartesian_command)
        self.velocity_pub = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        rospy.Subscriber('/ik_command', Transform, self.get_ik_command)
        self.joint_command_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #translational and rotational gain 
        tr_gain = 2
        rot_gain = 1.5
        #--------------------------------------------------------------------------
        joint_transforms, b_T_ee =self.forward_kinematics(self.q_current)
        ee_T_b = tf.transformations.inverse_matrix(b_T_ee)
        tr =  tf.transformations.translation_matrix((command.x_target.translation.x, command.x_target.translation.y, command.x_target.translation.z))
        rot = tf.transformations.quaternion_matrix((command.x_target.rotation.x, command.x_target.rotation.y, command.x_target.rotation.z, command.x_target.rotation.w))
        b_T_ee_desired = numpy.dot(tr, rot)
        delta_x = numpy.dot(ee_T_b, b_T_ee_desired) # desired change in end effector's frame
        delta_x_tr= (tf.transformations.translation_from_matrix(delta_x))
        angle, axis = self.rotation_from_matrix(delta_x)
        delta_x_rot = numpy.dot(axis, angle)          
        V_ee = numpy.append(tr_gain*delta_x_tr, rot_gain*delta_x_rot, axis =0) 
        # check whether translational velocity < 0.1 m/s and rotational velocity < 1 rad/s
        if numpy.linalg.norm(V_ee[:3]) > 0.1:
          V_ee[:3] = (0.1/numpy.linalg.norm(V_ee[:3]))*V_ee[:3]
        if numpy.linalg.norm(V_ee[3:6]) > 1:
          V_ee[3:6] = (1/numpy.linalg.norm(V_ee[3:6]))*V_ee[3:6]  
        V_ee_adj = numpy.append(V_ee[:3], V_ee[3:6], axis=0)  
        prop_gain = 1   #proportional gain 
        V = prop_gain*V_ee_adj
        J = self.get_jacobian(b_T_ee, joint_transforms)
        J_inv = numpy.linalg.pinv(J, rcond=0.01)
        q_dot = numpy.dot(J_inv, V)
        # Check that joint velocities are < 1 rad/s
        if numpy.linalg.norm(q_dot) > 1:
          q_dot = (1/numpy.linalg.norm(q_dot))*q_dot
        # Secondary Objective to make robot redundant 
        if command.secondary_objective:
           delta_q = numpy.zeros((self.num_joints))
           delta_q[0] = 3*(command.q0_target - self.q_current[0]) #gain for secondary objective = 3
           q_dot_nullspace = numpy.dot(numpy.identity(self.num_joints) - numpy.dot(J_inv, J), delta_q)
           q_dot = q_dot + q_dot_nullspace  
        self.joint_velocity_msg.name = self.joint_names
        self.joint_velocity_msg.velocity = q_dot 
        self.velocity_pub.publish(self.joint_velocity_msg)
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callbackj will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
       #--------------------------------------------------------------------------
        for i in range(self.num_joints):
            b_T_j = joint_transforms[i]
            j_T_b = tf.transformations.inverse_matrix(b_T_j)
            j_T_ee = numpy.dot(j_T_b, b_T_ee)
            j_tr_ee = j_T_ee[:3, 3] #Translation part of Matrix
            ee_T_j = tf.transformations.inverse_matrix(j_T_ee)
            ee_R_j = ee_T_j[:3,:3] #Rotation part of Matrix
            #Skew Symmetric Matrix	
            S = numpy.array([[0, -j_tr_ee[2], j_tr_ee[1]], [j_tr_ee[2], 0, -j_tr_ee[0]], [-j_tr_ee[1], j_tr_ee[0], 0]]) 
            J[3:6, i] = numpy.dot(ee_R_j, self.joint_axes[i])
            J[0:3, i] = numpy.dot(numpy.dot(-ee_R_j, S), self.joint_axes[i])              
        #--------------------------------------------------------------------------
        return J

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        q_c = numpy.zeros((self.num_joints))
        for i in range(0, self.num_joints):
           q_c[i] = random.uniform(0,2*math.pi)
        x_diff = [1]
        # check whether the desired pose is reached or not    
        while max(map(abs,x_diff)) > 0.001:   
          joint_transforms_c, b_T_ee_c =self.forward_kinematics(q_c)
          ee_T_b_c = tf.transformations.inverse_matrix(b_T_ee_c)
          tr =  tf.transformations.translation_matrix((command.translation.x, command.translation.y, command.translation.z))
          rot = tf.transformations.quaternion_matrix((command.rotation.x, command.rotation.y, command.rotation.z, command.rotation.w))
          b_T_ee_desired = numpy.dot(tr, rot)
          delta_x = numpy.dot(ee_T_b_c, b_T_ee_desired) # desired change in end effector's frame (xd-xc)
          delta_x_tr= tf.transformations.translation_from_matrix(delta_x)
          angle, axis = self.rotation_from_matrix(delta_x)
          delta_x_rot = numpy.dot(axis, angle) 
          x_diff = numpy.append(delta_x_tr, delta_x_rot, axis = 0)
          J = self.get_jacobian(b_T_ee_c, joint_transforms_c)
          J_right_inv = numpy.linalg.pinv(J, rcond=0.01)
          delta_q = numpy.dot(J_right_inv, x_diff)
          q_c = q_c + delta_q
          self.joint_command_msg.name = self.joint_names
          self.joint_command_msg.position = q_c
        self.joint_command_pub.publish(self.joint_command_msg)
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
        R = numpy.array(matrix, dtype=numpy.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = numpy.linalg.eig(R33.T)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = numpy.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = numpy.linalg.eig(R)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (numpy.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each joint of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.transformations.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()
