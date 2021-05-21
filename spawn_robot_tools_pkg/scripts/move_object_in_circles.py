#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    
    if len(sys.argv) < 2:
        print("usage: move_model_cicles.py model_name")
    else:
        model_name = sys.argv[1]
        print "Found Model Name"
        

    pub = rospy.Publisher(model_name+'/cmd_vel', Twist, queue_size = 1)
    rospy.init_node('move_model_in_cicles')
    
    speed=0.2
    turn=0.1
    x = 1
    th = 1
    
    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True
        twist = Twist()
    	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    	pub.publish(twist)

    rospy.on_shutdown(shutdownhook)

    print "Start Moving"
    while not ctrl_c:
        twist = Twist()
        twist.linear.x = x*speed
        twist.angular.z = th*turn
        pub.publish(twist)
        rate.sleep()
