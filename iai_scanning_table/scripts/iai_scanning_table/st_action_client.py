#!/usr/bin/env python
#
# Action client for controlling the Scanning Table
#
# Copyright (c) 2013, 2017 Universitaet Bremen, Institute for Artificial Intelligence - Prof. Michael Beetz
# Author: Alexis Maldonado Herrera <amaldo at cs.uni-bremen.de>
# 
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import roslib; roslib.load_manifest('iai_scanning_table')
import rospy

import actionlib

import iai_scanning_table_msgs.msg

from time import sleep
import math
import random


def scanning_table_client(random=False, angle=0.0):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('scanning_table_action_server', iai_scanning_table_msgs.msg.scanning_tableAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("waiting for server.")
    client.wait_for_server()
    print("Done.")

    # Creates a goal to send to the action server.
    goal = iai_scanning_table_msgs.msg.scanning_tableGoal()
    
    if random:
        goal.angle =  2 * math.pi * random.random()
    else:
        goal.angle = angle
        
    goal.release_brake_timeout = 10.0
    goal.apply_modulo = True

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
    
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.        
        rospy.init_node('scanning_table_client_py')
        from math import pi
        goals = [0.0, pi/2, pi, pi + pi/2, 2*pi]
        #goals = [x/10.0 for x in range(15)]
        
        for goal in goals:
            result = scanning_table_client(angle=goal)
            print ("Result: %s" %(result.answer))      
            
        sleep(1.5)   # The ROS python action server does not like it when the clients die right away
        
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

