#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Pose, Twist, Vector3
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import tf
import math
import numpy
from turtle_tf_3d.get_model_gazebo_pose import GazeboModel
from spawn_robot_tools_pkg.move_generic_model import MoveModel, Coordinates


def move_with_cmd_vel_topic(model_name=""):
    rospy.init_node('move_robot_to_given_place')

    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0

    coordinates = Coordinates(x,y,z,roll,pitch,yaw)

    print "To check if the name is correct please execute:$rosservice call gazebo/get_world_properties"
    #robot_name = raw_input("Model Name=")
    robot_name = model_name
    publish_rate = 10
    rate = rospy.Rate(publish_rate)
    ctrl_c = False

    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    move_model_object = MoveModel(model_name=robot_name,
                                  init_coordinates=coordinates)

    print "Start Moving"
    while not ctrl_c:
        
        move_model_object.move_step_speed(publish_rate)
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: move_generic_model.py model_name")
    else:
        model_name = sys.argv[1]
        try:
            move_with_cmd_vel_topic(model_name)
        except rospy.ROSInterruptException:
            pass
    
