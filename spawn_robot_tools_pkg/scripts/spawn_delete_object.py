#!/usr/bin/python
#################################################################
##\file
#
# \note
# Copyright (c) 2010 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS stack name: cob_environments
# \note
# ROS package name: cob_gazebo_objects
#
# \author
# Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Feb 2012
#
# \brief
# Implements script server functionalities.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################
# This was heavy modified by duckfrost@theconstructsim.com
# Date: 24 Oct 2019
#################################################################

import time
import rospy
import rospkg
import os
# http://docs.ros.org/fuerte/api/gazebo/html/index-msg.html
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose


class SpawnDeleteClass(object):

    def __init__(self):


        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        self._rospack = rospkg.RosPack()
        rospy.Service('/spawndelete_models_server', SpawnModel, self._spawndelete_models_clb)


    def _spawndelete_models_clb(self,req):
        """
        cat SpawnModel.srv
        string model_name                 # name of the model to be spawn
        string model_xml                  # Package name
        string robot_namespace            # Spawn or delete
        geometry_msgs/Pose initial_pose   # only applied to canonical body
        string reference_frame            # Model Type
                                          # if left empty or "world", then gazebo world frame is used
                                          # if non-existent model/body is specified, an error is returned
                                          #   and the model is not spawned
        ---
        bool success                      # return true if spawn successful
        string status_message             # comments if available
        :param req:
        :return:
        """

        rospy.logdebug("Request of SpawnDelete Received..Processing")
        response = SpawnModelResponse()

        model_name = req.model_name
        models_package_name = req.model_xml
        action = req.robot_namespace
        object_pose = req.initial_pose
        model_type = req.reference_frame

        if action == "SPAWN":
            try:
                response.success, response.status_message = self.spawn_new_model(model_name=model_name,
                                    model_type=model_type,
                                    models_package_name=models_package_name,
                                    object_pose=object_pose)
            except Exception as e:
                msg = "Spawn had an error ==="+str(e)
                rospy.logerr()
                response.success = False
                response.status_message = msg

        elif action == "DELETE":
            try:
                response.success, response.status_message = self.delete_model(model_name=model_name)
            except Exception as e:
                msg = "Delete had an error ==="+str(e)
                rospy.logerr()
                response.success = False
                response.status_message = msg

        else:
            error_msg = "ACTION NOT SUPPORTED==>"+str(action)
            rospy.logerr(error_msg)
            response.success = False
            response.status_message = error_msg

        return response

    def spawn_new_model(self, model_name, model_type, models_package_name, object_pose):

        path_to_package = self._rospack.get_path(models_package_name)
        base_dir_localition = os.path.join(path_to_package, model_type)

        result_ok = True
        result_msg = ""

        if model_type == "models":

            model_dir_localition = os.path.join(base_dir_localition, model_name)
            file_localition = os.path.join(model_dir_localition, 'model.sdf')
        else:
            file_localition = os.path.join(base_dir_localition, model_name + '.' + model_type)

        try:
            os.path.isfile(file_localition)
        except:

            result_ok = False
            result_msg = "File not found: "+str(file_localition)
            rospy.logerr(result_msg)
            return result_ok, result_msg


        # call gazebo service to spawn model (see http://ros.org/wiki/gazebo)
        if model_type == "urdf":
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            file_xml = open(file_localition)
            xml_string = file_xml.read()

        elif model_type == "xacro":
            p = os.popen("rosrun xacro xacro.py " + file_localition)
            xml_string = p.read()
            p.close()
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

        elif model_type == "models":
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            file_xml = open(file_localition)
            xml_string = file_xml.read()
        else:
            result_ok = False
            result_msg = 'Model type not know. model_type = ' + model_type
            rospy.logerr(result_msg)
            return result_ok, result_msg

        # spawn new model
        req = SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = xml_string
        req.initial_pose = object_pose

        res = srv_spawn_model(req)

        # evaluate response
        if res.success:
            result_msg = res.status_message + " " + model_name
            rospy.loginfo(result_msg)
        else:
            result_msg = "Error: model %s not spawn. error message = " % model_name + res.status_message
            rospy.logerr(result_msg)

        result_ok = res.success
        result_msg = res.status_message
        return result_ok, result_msg

    def delete_model(self, model_name):

        # check if object is already spawned
        srv_delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        req = DeleteModelRequest()
        req.model_name = model_name

        try:
            res = srv_delete_model(req)
            result_ok = res.success
            result_msg = res.status_message
            rospy.loginfo(result_msg)
        except rospy.ServiceException as e:
            result_ok = False
            result_msg = "Model "+model_name +" does not exist in gazebo, e="+str(e)
            rospy.logerr(result_msg)

        return result_ok, result_msg


def spawn_and_delete_test():
    rospy.init_node("object_spawner")
    spawndelete_obj = SpawnDeleteClass()
    object_pose = Pose()
    object_pose.position.z = 0.7
    model_name = "standard_apple"
    spawndelete_obj.spawn_new_model(model_name=model_name,
                                    model_type="models",
                                    models_package_name="spawn_robot_tools_pkg",
                                    object_pose=object_pose)

    object_pose.position.x = 0.1
    model_name2 = "standard_cube"
    spawndelete_obj.spawn_new_model(model_name=model_name2,
                                    model_type="models",
                                    models_package_name="spawn_robot_tools_pkg",
                                    object_pose=object_pose)
    time.sleep(5)
    spawndelete_obj.delete_model(model_name=model_name)
    spawndelete_obj.delete_model(model_name=model_name2)


def spawn_and_delete_test_serverclient():
    rospy.init_node("object_spawner", log_level=rospy.DEBUG)
    spawndelete_obj = SpawnDeleteClass()

    rospy.logwarn("Waiting for /spawndelete_models_server...")
    rospy.wait_for_service('/spawndelete_models_server')
    rospy.logwarn("Waiting for /spawndelete_models_server...READY")
    spawndelete_client = rospy.ServiceProxy('/spawndelete_models_server', SpawnModel)

    object_pose = Pose()
    object_pose.position.z = 0.7
    model_name = "standard_apple"

    request = SpawnModelRequest()
    request.model_name = model_name
    request.model_xml = "spawn_robot_tools_pkg"
    request.robot_namespace = "SPAWN"
    request.initial_pose = object_pose
    request.reference_frame = "models"
    response = spawndelete_client(request)

    object_pose.position.x = 0.1
    model_name2 = "standard_cube"
    request.model_name = model_name2
    request.initial_pose = object_pose
    response = spawndelete_client(request)

    time.sleep(5)

    request.model_name = model_name
    request.robot_namespace = "DELETE"
    response = spawndelete_client(request)

    request.model_name = model_name2
    request.robot_namespace = "DELETE"
    response = spawndelete_client(request)

def main():
    rospy.init_node("object_spawner_node", log_level=rospy.DEBUG)
    spawndelete_obj = SpawnDeleteClass()
    rospy.spin()

if __name__ == "__main__":
    main()



