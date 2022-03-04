#!/usr/bin/env python3 

import rospy
import numpy
import tf
import tf2_ros
import geometry_msgs.msg
import math


def publish_transforms():
      t1 = geometry_msgs.msg.TransformStamped()
      t1.header.stamp = rospy.Time.now()
      t1.header.frame_id = "base_frame"
      t1.child_frame_id = "object_frame"
      q1 = tf.transformations.quaternion_from_euler(0.64,0.64,0.0)
      t1.transform.rotation.x = q1[0]
      t1.transform.rotation.y = q1[1]
      t1.transform.rotation.z = q1[2]
      t1.transform.rotation.w = q1[3]
      T1 = numpy.dot(tf.transformations.quaternion_matrix(q1), tf.transformations.translation_matrix((1.5,0.8,0.0)))
      tr1 =tf.transformations.translation_from_matrix(T1)
      t1.transform.translation.x = tr1[0]
      t1.transform.translation.y = tr1[1]
      t1.transform.translation.z = tr1[2]
      br.sendTransform(t1)
      
      T1_inverse = tf.transformations.inverse_matrix(T1)   
      
      t2 = geometry_msgs.msg.TransformStamped()
      t2.header.stamp = rospy.Time.now()
      t2.header.frame_id = "base_frame"
      t2.child_frame_id = "robot_frame"
      q2 = tf.transformations.quaternion_about_axis(1.5, (0,1,0))
      t2.transform.rotation.x = q2[0]
      t2.transform.rotation.y = q2[1]
      t2.transform.rotation.z = q2[2]
      t2.transform.rotation.w = q2[3]
      T2 = numpy.dot(tf.transformations.quaternion_matrix(q2), tf.transformations.translation_matrix((0.0,0.0,-2.0)))
      tr2 =tf.transformations.translation_from_matrix(T2)
      t2.transform.translation.x = tr2[0]
      t2.transform.translation.y = tr2[1]
      t2.transform.translation.z = tr2[2]
      br.sendTransform(t2)
      
      T2_inverse = tf.transformations.inverse_matrix(T2)
      
      t3 = geometry_msgs.msg.TransformStamped()
      t3.header.stamp = rospy.Time.now()
      t3.header.frame_id = "robot_frame"
      t3.child_frame_id = "camera_frame"
      q3 = tf.transformations.quaternion_from_euler(0,0,0)
      t3.transform.rotation.x = q3[0]
      t3.transform.rotation.y = q3[1]
      t3.transform.rotation.z = q3[2]
      t3.transform.rotation.w = q3[3]
      T3 = numpy.dot(tf.transformations.quaternion_matrix(q3), tf.transformations.translation_matrix((0.3,0.0,0.3)))
      tr3 =tf.transformations.translation_from_matrix(T3)
      t3.transform.translation.x = tr3[0]
      t3.transform.translation.y = tr3[1]
      t3.transform.translation.z = tr3[2]
      T3_inverse = tf.transformations.inverse_matrix(T3)
      T4 = numpy.dot(T3_inverse, T2_inverse) 
      T5 = numpy.dot(T4, T1)
      tr4 = tf.transformations.translation_from_matrix(T5) #vector from camera_frame to object_frame
      tr4_unit = tr4/numpy.linalg.norm(tr4)
      X_camera = [1,0,0] #x-axis of camera frame
      theta = math.acos(numpy.dot(X_camera, tr4_unit))
      normal = numpy.cross(X_camera, tr4_unit)
      q4 = tf.transformations.quaternion_about_axis(theta, normal)
      t3.transform.rotation.x = q4[0]
      t3.transform.rotation.y = q4[1]
      t3.transform.rotation.z = q4[2]
      t3.transform.rotation.w = q4[3]
      br.sendTransform(t3)
      

if __name__ == '__main__':
    rospy.init_node('solution', anonymous=True)

    br = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        publish_transforms()
