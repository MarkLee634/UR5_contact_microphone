#!/usr/bin/env python
import argparse
from geometry_msgs.msg import Wrench, Vector3

import sys
import rospy
import moveit_commander 
import moveit_msgs.msg 
import scanner
import geometry_msgs
import publish_ft_sensor

# def wrench_callback(data):
#     print("inside callback, received")


if __name__=='__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', '-ip', type=str, default='192.168.131.169')
    parser.add_argument('--rate', '-r', type=int, default=1000)
    args = parser.parse_args()

    pub = rospy.Publisher('ft_sensor', Wrench, queue_size=10)

    sensor = publish_ft_sensor.ATIFTSensor(args.ip)
    sensor.start()

    moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
    rospy.init_node('move_group_python_interface', anonymous=True) #initialize the node 
    robot = moveit_commander.RobotCommander() #define the robot
    scene = moveit_commander.PlanningSceneInterface() #define the scene
    group = moveit_commander.MoveGroupCommander("manipulator") #define the planning group (from the moveit packet 'manipulator' planning group)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1) #create a publisher that publishes a plan to the topic: '/move_group/display_planned_path'
    
    # rospy.Subscriber("/wrench", geometry_msgs.msg.WrenchStamped, wrench_callback)
    rate = rospy.Rate(args.rate)

    LEVEL_OFFSET = 0
    VIEWS_PER_LEVEL = 1
    OBSERVE_TIME = 1
    ROBOT_VELOCITY = 0.05

    

    robot_motion = scanner.scanner(group, 
                                      level_offset=LEVEL_OFFSET, 
                                      n_views=VIEWS_PER_LEVEL, 
                                      observe_time=OBSERVE_TIME, 
                                      velocity=ROBOT_VELOCITY)


    # send to home joint position
    # robot_motion.goto_home()
    # move forward position
    # robot_motion.move_position(0.4,0,0)
    robot_motion.move_angled_position(0.2,0,0)
    # print("finished reaching motion")

    

    while not rospy.is_shutdown():
        _, forces, torques = sensor.read()

        wrench = Wrench(Vector3(*forces), Vector3(*torques))
        pub.publish(wrench)
        rate.sleep()
        
    sensor.stop()
    

    # sorghum_scanner.home()
    # while True:
    #     scan = raw_input('Scan sorghum?: (Y/n)')
    #     if scan=='Y':
    #         panicle_id = raw_input('Panicle ID (example: 6-4):')
    #         sorghum_scanner.scan(panicle_id=panicle_id)
    #     elif scan=='x':
    #         moveit_commander.roscpp_shutdown() 
    #         raise KeyboardInterrupt

    moveit_commander.roscpp_shutdown() #shut down the moveit_commander
