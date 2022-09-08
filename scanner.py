#!/usr/bin/env python

import os
import copy
import rospy
import numpy as np
from datetime import datetime
import tf 
import moveit_commander
from rosbag_record import RosbagRecord
import geometry_msgs #import WrenchStamped

class scanner:
    def __init__(self, group, level_offset, n_views, observe_time, velocity=0.1):
        self.group = group
        self.group.set_max_velocity_scaling_factor(velocity)
        date = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        

        self.joint_home_bot = None
        self.joint_home_top = None
    	self.level_offset = level_offset
        self.n_views = n_views
        self.observe_time = observe_time
        self.rotating_joint = 6 # 6 on real robot

        # self.sub = rospy.Subscriber("/wrench", geometry_msgs.msg.WrenchStamped, self.callback)
        self.sub = rospy.Subscriber("/ft_sensor", geometry_msgs.msg.Wrench, self.callback)
        self.contact_threshold_force = 2
        self.contact_detected = False


    def callback(self, data):
        # print("inside callback, received")
        force = data.force
        force_mag = np.sqrt(force.z*force.z)
        # print("force", force_mag)
        if force_mag > self.contact_threshold_force and self.contact_detected == False:
            self.contact_detected = True
            print("==================contact detected================== ")
            print("force" , force_mag)
            self.group.stop()

            print("================== contact motion initiate ================== ")
            rospy.sleep(1)
            self.move_position(0.02,0,0)
            rospy.sleep(1)
            self.move_position(-0.02,0,0)

    


    def safe_set_pose_target(self, pose_target):
        #set pose_target as the goal pose of manipulator group 
        self.group.set_pose_target(pose_target) 

        # Wait for user input to go to scan position
        while True:
            plan = self.group.plan() #call plan function to plan the path
            val = raw_input('Confirm planned path for home position to execute: (Y/n)')
            if val=='Y':
                self.group.execute(plan, wait=False) #execute plan on real/simulation robot
                self.group.stop() # Calling `stop()` ensures that there is no residual movement
                self.group.clear_pose_targets()
                break
            elif val=='x':
                moveit_commander.roscpp_shutdown() 
                raise KeyboardInterrupt

    
    def goto_home(self):
        print("------ go to home position  ------")
        self.home_joint_position = self.group.get_current_joint_values() 
        print("joint val ",  self.home_joint_position)

        joint_target = self.group.get_current_joint_values() 
        joint_target[1] = 0
        joint_target[2] = -0.11888
        joint_target[3] = -2.52659
        joint_target[4] = -2.066730
        joint_target[5] = -1.56951
        joint_target[6] = 3.14159
        self.group.set_joint_value_target(joint_target) 
        plan = self.group.plan() 
        self.group.execute(plan, wait=True) 
        self.group.stop()
        # print("------ finished go to home position  ------")

    def move_angled_position(self, dx, dy, dz):
        print("------ angled poking motion  ------")
        pose_current = self.group.get_current_pose()
        pose_current = self.group.get_current_pose()

        # orient orthogonal down
        px = pose_current.pose.position.x
        py = pose_current.pose.position.y
        pz = pose_current.pose.position.z
        qx = pose_current.pose.orientation.x
        qy = pose_current.pose.orientation.y
        qz = pose_current.pose.orientation.z
        qw = pose_current.pose.orientation.w

        # Transformation matrix from pose        
        T = tf.transformations.quaternion_matrix([qx, qy, qz, qw])
        T[:3,3]=np.array([px, py, pz])
        # print(T)

        current_t = np.array([dx,dy,dz,1])
        offset_t = np.matmul(T,current_t)

        pose_target = copy.copy(pose_current)
        pose_target.pose.position.x = offset_t[0]
        pose_target.pose.position.y = offset_t[1]
        pose_target.pose.position.z =offset_t[2]

        # final = np.array([offset_t[0], offset_t[1], offset_t[2]])
        # print(T[:3,3], final)

        self.safe_set_pose_target(pose_target)



    def move_position(self, dx, dy, dz):
        print("------ poking motion  ------")
        pose_current = self.group.get_current_pose()

        px = pose_current.pose.position.x
        py = pose_current.pose.position.y
        pz = pose_current.pose.position.z
        qx = pose_current.pose.orientation.x
        qy = pose_current.pose.orientation.y
        qz = pose_current.pose.orientation.z
        qw = pose_current.pose.orientation.w

        pose_target = copy.copy(pose_current)
        pose_target.pose.position.x = px + dx
        pose_target.pose.position.y = py + dy
        pose_target.pose.position.z = pz + dz
        self.group.set_pose_target(pose_target) 

        plan = self.group.plan() #call plan function to plan the path
        self.group.execute(plan, wait=True) #execute plan on real/simulation robot
        self.group.stop() # Calling `stop()` ensures that there is no residual movement
        self.group.clear_pose_targets()
        #self.safe_set_pose_target(pose_target)











    def home(self):
        pose_current = self.group.get_current_pose()
        
        # orient orthogonal down
        px = pose_current.pose.position.x
        py = pose_current.pose.position.y
        pz = pose_current.pose.position.z
        qx = pose_current.pose.orientation.x
        qy = pose_current.pose.orientation.y
        qz = pose_current.pose.orientation.z
        qw = pose_current.pose.orientation.w

        # Transformation matrix from pose        
        T = tf.transformations.quaternion_matrix([qx, qy, qz, qw])
        T[:3,3]=np.array([px, py, pz])

        # Compute target axis which is the closest world axis to align to 
        axis = np.matmul(T, [1, 0, 0, 1])-np.array([px,py,pz,1])
        directions = [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]
        cos_sim = np.arccos(np.inner(axis[:3], directions)/(np.linalg.norm(axis[:3])*np.linalg.norm(directions)))
        target_axis = directions[np.argmin(cos_sim)]

        # Update target transformation matrix
        T[:3,:3] =np.zeros((3,3))
        T[:3,0] = np.array(target_axis) # align ee-axis to target axis CHECK IF ee-axis == x-axis
        T[:3,1] = np.array(target_axis)[[1,2,0]] # arbitrary
        T[:3,2] = np.cross(T[:3,0], T[:3,1] ) # cross product for right-hand rule
        # Convert to quaternion
        q_target = tf.transformations.quaternion_from_matrix(T)

        # Update target pose
        pose_target = copy.copy(pose_current)
        pose_target.pose.orientation.x = q_target[0]
        pose_target.pose.orientation.y = q_target[1]
        pose_target.pose.orientation.z = q_target[2]
        pose_target.pose.orientation.w = q_target[3]

    	self.safe_set_pose_target(pose_target)
    
        # set joint 6 to pi radians
        joint_target = self.group.get_current_joint_values() 
        joint_target[self.rotating_joint] = np.pi
        self.group.set_joint_value_target(joint_target) 
        plan = self.group.plan() 
        self.group.execute(plan, wait=True) 
        self.group.stop()
        
        self.joint_home_bot = copy.copy(joint_target)
	    
        # go to second level joint position 
        pose_target = self.group.get_current_pose()
        pose_target.pose.position.z+=self.level_offset
        self.safe_set_pose_target(pose_target)	

        self.joint_home_top = self.group.get_current_joint_values() 
        self.joint_home_top[self.rotating_joint] = -np.pi
        self.group.set_joint_value_target(self.joint_home_bot) 
        plan = self.group.plan() 
        self.group.execute(plan, wait=True) 
        self.group.stop()        
        print('Scan position reached')
	