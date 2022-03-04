#!/usr/bin/env python

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

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(object):
    def __init__(self):
	#Load robot from parameter server
        self.robot = URDF.from_parameter_server()

	

	#This will load information about the joints of the robot 
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()
	#This is a mutex
        self.mutex = Lock()
	

	#Subscribe to current joint state of the robot
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	#print self.q_current
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
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
	(joint_transforms, b_T_ee) = self.forward_kinematics(self.q_current)
	J = self.get_jacobian(b_T_ee, joint_transforms)
        b_T_xd_trans = tf.transformations.translation_matrix((command.x_target.translation.x, command.x_target.translation.y, command.x_target.translation.z))
        b_T_xd_rot = tf.transformations.quaternion_matrix((command.x_target.rotation.x, command.x_target.rotation.y, command.x_target.rotation.z, command.x_target.rotation.w ))
        b_T_xd = numpy.dot(b_T_xd_trans, b_T_xd_rot)
    	ee_T_xd = numpy.dot(numpy.linalg.inv(b_T_ee), b_T_xd)
        (rx_dis, ry_dis, rz_dis) = tf.transformations.euler_from_matrix(ee_T_xd[0:3,0:3])
        delta_x = numpy.zeros((6,1))
        delta_x[0] = ee_T_xd[0,3]
        delta_x[1] = ee_T_xd[1,3]
        delta_x[2] = ee_T_xd[2,3]
        delta_x[3] = rx_dis
        delta_x[4] = ry_dis
        delta_x[5] = rz_dis
        p = 1 #gain
        vee = delta_x*p
        J_invs = numpy.linalg.pinv(J, rcond=0.01)
        qdot_dis = numpy.dot(J_invs,vee)
        print qdot_dis
        #---------------------------------------------------------------
        delta_sec = (command.q0_target - self.q_current[0] )
        p_sec_gain = 3
        q_sec = p_sec_gain*delta_sec
        vel_sec = numpy.zeros((self.num_joints,1))
        vel_sec[0] = [q_sec]
        I = numpy.eye(self.num_joints)
        J_inv = numpy.linalg.pinv(J) #not safe pseudo_inv
        q_null = numpy.dot((I - numpy.dot(J_inv,J)),vel_sec)
        qdot_dis = qdot_dis+q_null
        #---------------------------------------------------------------
        self.joint_velocity_msg.name = self.joint_names
        self.joint_velocity_msg.velocity = qdot_dis
        self.velocity_pub.publish(self.joint_velocity_msg)


        
    


        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
	J = numpy.zeros((6,self.num_joints))
	
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
        j_T_ee_temp = []
        ee_T_j_temp = []
        J_temp = []
        v = []
        vj = numpy.zeros((6,6))
    	length = len(self.joint_names)
	for i in range(length):
        	v1 = ([0,0,0])
                
                
                ee_T_b = numpy.linalg.inv(b_T_ee)
                #print ee_T_b
                ee_T_j_temp = numpy.dot(ee_T_b, joint_transforms[i])
                #print ee_T_j_temp
                j_T_ee_temp = numpy.linalg.inv(ee_T_j_temp)
                
        	
        	vj[0:3,0:3] = ee_T_j_temp[0:3,0:3]
        	vj[3:6,3:6] = ee_T_j_temp[0:3,0:3]
        	trans_skew = self.get_skew(j_T_ee_temp[0:3,3])
        	rot_sktrans = numpy.dot(-ee_T_j_temp[0:3,0:3],trans_skew)

        	vj[0:3,3:6] = rot_sktrans
           	v1 = numpy.append(v1,numpy.asarray(self.joint_axes[i]))
          	v1 = numpy.asmatrix(v1)
		v1 = numpy.transpose(v1)
                
        	J_temp = numpy.dot(vj,v1)
                #print J_temp
                for p in range(0,6):
                        J[p,i] = J_temp[p]

        #print J

       
        #--------------------------------------------------------------------------
        return J

    '''This is the callback which  bewill executed when the inversefabout desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #------ --------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE
        qc_rand = self.q_current
        
        x_dis_trans = tf.transformations.translation_matrix((command.translation.x, command.translation.y, command.translation.z))
        
        x_dis_rot = tf.transformations.quaternion_matrix((command.rotation.x, command.rotation.y, command.rotation.z, command.rotation.w))
        x_dis = numpy.dot(x_dis_trans,x_dis_rot)
        delta_x_ik = numpy.ones(6)
        while  True  :
            
            if numpy.amax(numpy.absolute(delta_x_ik)) < 10**-3:
                break

            (joint_transforms,b_T_ee) = self.forward_kinematics(qc_rand)
            delta_x_ik_tf = numpy.dot(numpy.linalg.inv(b_T_ee),x_dis)
            delta_x_ik[0] = delta_x_ik_tf[0,3]
            delta_x_ik[1] = delta_x_ik_tf[1,3]
            delta_x_ik[2] = delta_x_ik_tf[2,3]
            (delta_x_ik[3], delta_x_ik[4], delta_x_ik[5]) = tf.transformations.euler_from_matrix(delta_x_ik_tf[:3, :3])

            J = self.get_jacobian(b_T_ee, joint_transforms)
            q_dot_ik = numpy.dot(numpy.linalg.pinv(J),delta_x_ik)
            for p in range(0,self.num_joints):
                qc_rand[p] = qc_rand[p] + q_dot_ik[p]

            
        print "out of the loop"
        (joint_transforms_check, b_T_ee_check) = self.forward_kinematics(qc_rand)
        print b_T_ee_check
        print x_dis

        self.joint_command_msg.name= self.joint_names
        self.joint_command_msg.position = qc_rand
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
	    	
	    
        return joint_transforms, T, #v#where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()	
  
    def get_skew(self,m):
        trans_skew = numpy.array([[0, -m[2], m[1]], [m[2], 0, -m[0]], [-m[1], m[0], 0]])
        return trans_skew
if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()
