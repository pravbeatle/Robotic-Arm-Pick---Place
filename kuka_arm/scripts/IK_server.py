#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np
import csv


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    	# Create Modified DH parameters
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    	# Define Modified DH Transformation matrix
        substitute_values = {
            alpha0:     0,  a0:      0,  d1:  0.75, 
            alpha1: -pi/2,  a1:   0.35,  d2:     0,  q2: q2-pi/2,
            alpha2:     0,  a2:   1.25,  d3:     0,       
            alpha3: -pi/2,  a3: -0.054,  d4:   1.5,       
            alpha4:  pi/2,  a4:      0,  d5:     0,       
            alpha5: -pi/2,  a5:      0,  d6:     0,       
            alpha6:     0,  a6:      0,  d7: 0.303,  q7: 0
        }
    	# Create individual transformation matrices
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                   0,                   0,            0,               1]])
        T0_1 = T0_1.subs(substitute_values)


        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])
        T1_2 = T1_2.subs(substitute_values)


        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                   0,                   0,            0,               1]])
        T2_3 = T2_3.subs(substitute_values)


        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])
        T3_4 = T3_4.subs(substitute_values)


        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])
        T4_5 = T4_5.subs(substitute_values)


        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])
        T5_6 = T5_6.subs(substitute_values)

        T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])
        T6_G = T6_G.subs(substitute_values)
    	# Extract rotation matrices from the transformation matrices
        # Required calc for theta2 and theta3
        T0_2 = T0_1 * T1_2
        ###

        # Initialize service response
        joint_trajectory_list = []
        # Initialize list to store error data
        err_data = []

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

    	    # Extract end-effector position and orientation from request
    	    # px,py,pz = end-effector position
    	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
            P_EE = Matrix( [ [px], [py], [pz] ] )

            # Get rotation matrix from roll, pitch, yaw
            R_roll = Matrix([
                    [1,         0,          0],
                    [0, cos(roll), -sin(roll)], 
                    [0, sin(roll),  cos(roll)]
                ])

            R_pitch = Matrix([
                    [ cos(pitch), 0, sin(pitch)],
                    [          0, 1,          0],
                    [-sin(pitch), 0, cos(pitch)]
                ])

            R_yaw = Matrix([
                    [cos(yaw), -sin(yaw), 0], 
                    [sin(yaw),  cos(yaw), 0],
                    [       0,         0, 1]
                ])

            R_rpy = (R_yaw * R_pitch * R_roll)

            # Calculate WC from EE
            P_WC = ( P_EE - (0.303 * R_rpy) * Matrix( [ [1], [0], [0] ] ) )

            # Calculate theta1
            theta1 = atan2(P_WC[1], P_WC[0])

            # Calculate theta3
            j2 = (T0_2.evalf(subs={q1: theta1}))[0:3, 3]
            
            dist_j2_wc = sqrt( (j2[0]-P_WC[0])**2 + (j2[1]-P_WC[1])**2 + (j2[2]-P_WC[2])**2 )
            dist_j3_wc = sqrt( substitute_values[a3]**2 + substitute_values[d4]**2 )
            dist_j2_j3 = substitute_values[a2]
            
            j3_5_angle_offset = atan2(-0.054, 1.5)
            
            cos_theta3_2 = ( dist_j2_wc**2 - dist_j2_j3**2 - dist_j3_wc**2 )/( 2*dist_j2_j3*dist_j3_wc )
            theta3_2 = atan2( sqrt( 1-(cos_theta3_2)**2 ),cos_theta3_2 )
            
            theta3 = -(pi/2) + j3_5_angle_offset + theta3_2
            
            # Calculate theta2
            theta2_a = atan2( P_WC[2]-j2[2], sqrt( (j2[0]-P_WC[0])**2 + (j2[1]-P_WC[1])**2 ) )
            cos_theta2_b = ( -dist_j3_wc**2 + dist_j2_j3**2 + dist_j2_wc**2 )/( 2*dist_j2_j3*dist_j2_wc )
            theta2_b = atan2( sqrt( 1-(cos_theta2_b)**2 ), cos_theta2_b )
            theta2 = (pi/2) - theta2_a - theta2_b

            # Obtain transform from 3 to 6 after correction to solve for theta4, theta5, theta6
            T0_3 = (T0_1 * T1_2 * T2_3).evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # Correction from j3 frame to world frame
            R_x = Matrix([
                    [1,         0,          0],
                    [0, cos(pi/2), -sin(pi/2)],
                    [0, sin(pi/2),  cos(pi/2)]
                ])

            R_z = Matrix([
                    [cos(pi/2), -sin(pi/2), 0],
                    [sin(pi/2),  cos(pi/2), 0],
                    [        0,          0, 1]
                ])

            R3_6 = T0_3.T[0:3, 0:3] * R_rpy * (R_z * R_x).T

            # Rotation along y, z, y wrt j3 reference frame
            (theta4, theta5, theta6) = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), "ryzy")

            # Populate response for the IK request

            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            ### Compute error in pose using forward kinematics 

            # Correction from word reference frame to j6 frame
            R_y_world = Matrix([
                    [ cos(-pi/2), 0, sin(-pi/2)],
                    [          0, 1,          0],
                    [-sin(-pi/2), 0, cos(-pi/2)]
                ])

            R_z_world = Matrix([
                    [cos(pi), -sin(pi), 0],
                    [sin(pi),  cos(pi), 0],
                    [      0,        0, 1]
                ])

            R_corr_world = R_z_world * R_y_world

            T0_G = (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G).evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

            # Rotation angles after correction 
            (fk_roll, fk_pitch, fk_yaw) = tf.transformations.euler_from_matrix(np.array(T0_G[:3, :3] * R_corr_world))

            # Position after computing transformation 
            (fk_x, fk_y, fk_z) = (T0_G[0,3], T0_G[1,3], T0_G[2,3])

            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            # Print out the thetas
            print("Displaying all thetas :::: ", theta1, theta2, theta3, theta4, theta5, theta6)

            # Append error for all parameters
            err_data.append( [ P_EE[0] - T0_G[0,3], P_EE[1] - T0_G[1,3], P_EE[2] - T0_G[2,3], roll - fk_roll, pitch - fk_pitch, yaw - fk_yaw ] )

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))

        # Put error info in csv file for tracking
        with open('error_data.csv', 'a') as out:
            writer = csv.writer(out, lineterminator='\n')
            writer.writerows(err_data)

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
