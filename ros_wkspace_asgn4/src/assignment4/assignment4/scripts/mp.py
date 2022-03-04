#!/usr/bin/env python

import numpy
import random
import sys
import math
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
        print "Starting motion planning"
        goal_trans = tf.transformations.translation_matrix((ee_goal.translation.x, ee_goal.translation.y, ee_goal.translation.z))
        goal_rot = tf.transformations.quaternion_matrix((ee_goal.rotation.x, ee_goal.rotation.y, ee_goal.rotation.z, ee_goal.rotation.w))
        goal_transform = numpy.dot(goal_trans, goal_rot)
        config_current = self.q_current
        config_goal = self.IK(goal_transform)
        print config_goal
        random_tree = []
        start = RRTBranch(None, config_current)
        random_tree.append(start)
        shortcuted_tree = []
        random_q = []
        wavepoints = []
        for i in range(self.num_joints):
            random_q.append(0)   
        goal = list(config_goal)
        print goal
        if len(goal) == 0 :
            print "goal cant be achived"
        
        
        while True:
            #if (random_tree[length(random_tree)]):#connects directly to goal):
                #random_tree.append(goal)
            for i in range(self.num_joints):
                random_q[i] = random.uniform(-math.pi,math.pi)
            #print type(random_q) #list
            print random_q #float
            
            norm = []
            for i in range(len(random_tree)):
                vector = numpy.asarray(random_q)-numpy.asarray(random_tree[i].q)
                norm.append(numpy.linalg.norm(vector))
            shortest_dist = min(norm)
            index_closest_point = norm.index(min(norm))

            # unit vector 
            branch_vector = numpy.asarray(random_q)-numpy.asarray(random_tree[index_closest_point].q)                   # array
            u = branch_vector/shortest_dist                                                                             # array
            check_point = numpy.asarray(random_tree[index_closest_point].q)+(0.1*u)                                     # array
            validity = self.is_state_valid(check_point)                                                                 # bool
            if validity is True:
                random_tree.append(RRTBranch(random_tree[index_closest_point],list(check_point)))                       # class object
                goal_validity = self.is_segment_valid(check_point,goal)                                                 # bool
                print random_tree
                if goal_validity is True:
                    print "h"
                    random_tree.append(RRTBranch(random_tree[-1],goal))                                                 # class_object
                    break
        print "random_tree_traversed"
        print len(random_tree)
        #---------------------------------YOU, YES YOU MY DEAR YOU ARE BEST.------------------------------------------
        #=================================GENERATING THE TREE=========================================================
        random_path = []
        intrest_node = random_tree[-1]
        while True:
            
            random_path.append(intrest_node.q)                                                                          #list of lists
            if intrest_node.parent == None:
                break
            intrest_node = intrest_node.parent
        
        print "random path generated"
        print random_path
        print len(random_path)
        random_path.reverse()
        print "random path from start to goal"
        print random_path
        
        #------------------------------ shortcutting the tree ----------------------------------------
        
        i = 0
        j = i+1 
        shortcuted_tree.append(random_path[0])
        while True:
            if j==len(random_path):
                shortcuted_tree.append(random_path[j-1])
                break                                       
            if self.is_segment_valid(random_path[i],random_path[j]) is True:
                j+=1
            else:
                i = j-1
                shortcuted_tree.append(random_path[j-1])
                print j
        print "shortcutted_tree is ready"
        print shortcuted_tree

        #-------------------------------------------------------------------------------------------------
        # SAMPLING OF O.5 FOR PUBLISHING
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names 
        waypoint = []
        
        waypoint = self.resampling(shortcuted_tree)
            
        for p in range(0,len(waypoint)):
            waypoint[p] = list(waypoint[p])
        for p in waypoint:
            point_list = JointTrajectoryPoint()
            point_list.positions = p
            trajectory.points.append(point_list)
               
        #print self.path 
        self.pub.publish(trajectory)
        print self.path 
       
        


            

            
            
       
	########INSERT YOUR RRT MOTION PLANNING HERE##########



        ######################################################

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

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


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid

    def is_segment_valid(self, initial, random_point):
        #print random_point
        #print initial
        vector = numpy.asarray(random_point)-numpy.asarray(initial)
        sum_square = 0
        for i in range(self.num_joints):
            sum_square = sum_square+vector[i]**2
        mag_vector = self.mag(vector)
        unit_vector = vector/mag_vector
        quotient = mag_vector//0.1
        #print quotient
        remainder = mag_vector%0.1
        count = 0
        #write code to find how many samples should be taken on a segment
        for i in range(int(quotient)):
            next_sample = numpy.asarray(initial)+(0.1)*(unit_vector)
            if self.is_state_valid(next_sample) is True:
                initial = next_sample
                count +=1
            else:
                break
        next_sample = numpy.asarray(initial)+((remainder)*(unit_vector))
        if (self.is_state_valid(next_sample) and count==quotient) is True:
            initial = next_sample
        #print initial
        #print type(initial)
        rtol = 1*10**-5
        atol = 1*10**-6
        satisfy = numpy.allclose(initial,numpy.asarray(random_point), rtol, atol)
        
        if satisfy is True:
            return True
        else:
            return False
    
    def resampling(self, shortcuted_tree):
        resampled_data = []
        
        for i in range(len(shortcuted_tree)-1):
            j = i+1
            vector = numpy.asarray(shortcuted_tree[j])-numpy.asarray(shortcuted_tree[i])
            mag_vector = self.mag(vector)
            print mag_vector
            u = vector/mag_vector
            print u
            factor = math.ceil(mag_vector/0.5)
            print factor
            d = mag_vector/factor
            print d
            print d*u
            #reserve = shortcuted_tree[i]
            for p in range(int(factor)+1):
                resampled_data.append(shortcuted_tree[i]+((p)*d*u))
                waypoint = (shortcuted_tree[i]+((p)*d*u))
                #print resampled_data
        print resampled_data
        return resampled_data

        



    
    def mag(self, vector):
        
        sum_square = 0
        for i in range(self.num_joints):
            sum_square = sum_square+vector[i]**2
        mag_vector = math.sqrt(sum_square)
        return mag_vector


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent,q):
	self.parent = parent
	self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

