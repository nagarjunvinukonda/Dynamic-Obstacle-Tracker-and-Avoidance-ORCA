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


class Coordinates:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def print_coordinates(self):
        rospy.loginfo("[ x, y, z] = [" + str(self.x) + ", " +
                      str(self.y) + ", " + str(self.z) + "]")
        rospy.loginfo("[ roll, pitch, yaw] = [" + str(self.roll) +
                      ", " + str(self.pitch) + ", " + str(self.yaw) + "]")


class Trajectory:
    def __init__(self, num_points=10, type="circle", radius=None, height=1.0):
        self.current_trajectory_index = 0
        self.num_points = num_points
        self.type = type
        self.radius = radius
        self.height = height
        if self.type == "circle":
            self.trajectory = self.generate_circle(self.radius, self.height)

    def generate_circle(self, radius, height):
        trajectory = []
        min_degrees = 0
        max_degrees = 360
        step_size = (max_degrees - min_degrees)/float(self.num_points)
        for segment in numpy.arange(min_degrees, max_degrees, step_size):
            rad_segment = math.radians(segment)
            x = math.cos(rad_segment)*radius
            y = math.sin(rad_segment)*radius
            z = height
            roll = 0
            pitch = 0
            yaw = 0

            coord_object = Coordinates(x, y, z, roll, pitch, yaw)
            #print segment
            # coord_object.print_coordinates()
            trajectory.append(coord_object)

        return trajectory

    def step_trajectory(self, loop=False):
        """
        It returns the trajectory Coordinates that should be performed and moves one in the index
        :return:
        """
        try:
            coordinates = self.trajectory[self.current_trajectory_index]
            self.current_trajectory_index += 1
        except IndexError:
            if loop:
                # The trajectory doesnt have that index therefore we have finished the trajectory and we restart.
                self.current_trajectory_index = 0
                coordinates = self.trajectory[self.current_trajectory_index]
                #print "Looping Trajectory"
            else:
                #print "Finished the trajectory."
                coordinates = self.trajectory[len(self.trajectory)-1]

        return coordinates


class MoveModel(object):
    def __init__(self, model_name, init_coordinates):
        self.found_coordinates = False
        self._model_name = model_name
        self._model_coordinates = init_coordinates
        self.current_cmd_speed = Twist()
        self.g_pause = rospy.ServiceProxy("/gazebo/pause_physics", EmptySrv)
        self.g_unpause = rospy.ServiceProxy(
            "/gazebo/unpause_physics", EmptySrv)
        self.g_set_state = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)
        rospy.Subscriber(model_name+"/cmd_vel", Twist, self.move_callback)

        self.gz_model = GazeboModel(robots_name_list=[model_name])

        self._trajectory = Trajectory(
            num_points=100, type="circle", radius=3.0, height=1.0)

    def move_callback(self, msg):
        self.current_cmd_speed = msg

    def move_model(self, coordinates_to_move_to, pause_physics=False):

        if pause_physics:
            rospy.loginfo("Start Pausing Physics")
            rospy.wait_for_service("/gazebo/pause_physics")
            rospy.loginfo("Pausing physics")
            try:
                self.g_pause()
            except Exception, e:
                rospy.logerr('Error on calling service: %s', str(e))

        #rospy.loginfo("Filling Pose Data")
        pose = Pose()

        pose.position.x = coordinates_to_move_to.x
        pose.position.y = coordinates_to_move_to.y
        pose.position.z = coordinates_to_move_to.z

        quaternion = tf.transformations.quaternion_from_euler(coordinates_to_move_to.roll,
                                                              coordinates_to_move_to.pitch,
                                                              coordinates_to_move_to.yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        # We set twist to Null to remove any prior movements
        twist = Twist()
        linear = Vector3()
        angular = Vector3()

        linear.x = 0.0
        linear.y = 0.0
        linear.z = 0.0

        angular.x = 0.0
        angular.y = 0.0
        angular.z = 0.0

        twist.linear = linear
        twist.angular = angular

        state = ModelState()

        state.model_name = self._model_name
        state.pose = pose
        state.twist = twist

        #print state
        #rospy.loginfo("Moving model = " + str(self._model_name))
        try:
            ret = self.g_set_state(state)
            #print ret.status_message
        except Exception, e:
            rospy.logerr('Error on calling service: %s', str(e))

        if pause_physics:
            rospy.loginfo("Unpausing physics")
            try:
                self.g_unpause()
            except Exception, e:
                rospy.logerr('Error on calling service: %s', str(e))

    def move_step_trajectory(self):
        coordinates_to_move_to = self._trajectory.step_trajectory(loop=True)
        self.move_model(coordinates_to_move_to)

    def calculate_coord_for_speed(self, publish_rate, ball=False):
        """
        Gets the current position of the model and adds the increment based on the Publish rate
        """
        pose_now = self.gz_model.get_model_pose(robot_name=self._model_name)
        if pose_now:
            # space_increment = speed * Frequency

            z_increment = self.current_cmd_speed.angular.z * (1.0/publish_rate)

            explicit_quat = [pose_now.orientation.x, pose_now.orientation.y,
                             pose_now.orientation.z, pose_now.orientation.w]
            pose_now_euler = tf.transformations.euler_from_quaternion(
                explicit_quat)

            if ball:

                roll = 0.0
                pitch = 0.0
                yaw = 0.0

                planar_increment = self.current_cmd_speed.linear.x * \
                    (1.0/publish_rate)
                x_yaw_comp = math.cos(yaw)
                y_yaw_comp = math.sin(yaw)

                elevation_increment = self.current_cmd_speed.linear.z * \
                    (1.0/publish_rate)
                #quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

                coordinates_to_move_to = Coordinates(x=(pose_now.position.x+planar_increment),
                                                     y=(pose_now.position.y +
                                                        z_increment),
                                                     z=(pose_now.position.z +
                                                        elevation_increment),
                                                     roll=roll,
                                                     pitch=pitch,
                                                     yaw=yaw)
            else:

                roll = pose_now_euler[0]
                pitch = pose_now_euler[1]
                yaw = pose_now_euler[2]+z_increment

                planar_increment = self.current_cmd_speed.linear.x * \
                    (1.0/publish_rate)
                x_yaw_comp = math.cos(yaw)
                y_yaw_comp = math.sin(yaw)

                elevation_increment = self.current_cmd_speed.linear.z * \
                    (1.0/publish_rate)
                #quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

                coordinates_to_move_to = Coordinates(x=(pose_now.position.x+planar_increment*x_yaw_comp),
                                                     y=(pose_now.position.y +
                                                        planar_increment*y_yaw_comp),
                                                     z=(pose_now.position.z +
                                                        elevation_increment),
                                                     roll=roll,
                                                     pitch=pitch,
                                                     yaw=yaw)
        else:
            coordinates_to_move_to = None

        return coordinates_to_move_to

    def move_step_speed(self, publish_rate):
        coordinates_to_move_to = self.calculate_coord_for_speed(publish_rate)
        if coordinates_to_move_to:
            self.move_model(coordinates_to_move_to)
            if not self.found_coordinates:
                rospy.loginfo("Coordinates Available...")
                self.found_coordinates = True
        else:
            rospy.logwarn("No Coordinates available yet...")

    def move_ball_step_speed(self, publish_rate):
        """
        We move based on the cmd vel making the object move without orientation change
        """
        coordinates_to_move_to = self.calculate_coord_for_speed(
            publish_rate, True)
        if coordinates_to_move_to:
            self.move_model(coordinates_to_move_to)
            if not self.found_coordinates:
                rospy.loginfo("Coordinates Available...")
                self.found_coordinates = True
        else:
            rospy.logwarn("No Coordinates available yet...")


def test_move(model_name=""):
    rospy.init_node('move_robot_to_given_place')

    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0

    coordinates = Coordinates(x, y, z, roll, pitch, yaw)

    print "To check if the name is correct please execute:$rosservice call gazebo/get_world_properties"
    #robot_name = raw_input("Model Name=")
    robot_name = model_name

    rate = rospy.Rate(10)
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

        move_model_object.move_step_trajectory()
        rate.sleep()


def move_with_cmd_vel_topic(model_name=""):
    rospy.init_node('move_robot_to_given_place')

    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0

    coordinates = Coordinates(x, y, z, roll, pitch, yaw)

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
