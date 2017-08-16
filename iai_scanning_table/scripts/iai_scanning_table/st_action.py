#!/usr/bin/env python
#
# Action server for controlling the Scanning Table
#
# Copyright (c) 2013,2017 Universitaet Bremen, Institute for Artificial Intelligence - Prof. Michael Beetz
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

# ROS messages
from sensor_msgs.msg import JointState
import iai_scanning_table_msgs.msg

from time import sleep

from st_control import ElmoUdp


def change_ps_name(name):
    """Change process name as used by ps(1) and killall(1)."""
    try:
        import ctypes

        libc = ctypes.CDLL('libc.so.6')
        libc.prctl(15, name, 0, 0, 0)
    except:
        pass


class ScanningAction(object):
    # create messages that are used to publish feedback/result
    _feedback = iai_scanning_table_msgs.msg.scanning_tableFeedback()
    _result = iai_scanning_table_msgs.msg.scanning_tableResult()

    def __init__(self, name, table):
        self.table = table

        self._action_name = name
        self.release_brake_timeout = 0
        self.time_to_turn_motor_off = rospy.Time.now()


        self._as = actionlib.SimpleActionServer(self._action_name, iai_scanning_table_msgs.msg.scanning_tableAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables 
        r = rospy.Rate(10)
        success = True

        # start executing the action
        rospy.loginfo(rospy.get_name() +  ": Received goal: angle=%f  apply_modulo=%r  release_brake_timeout=%f" % (goal.angle, goal.apply_modulo, goal.release_brake_timeout))

        if goal.apply_modulo:
            # find shortest distance
            import math
            
            # angle repeats every 2*pi
            # equiv_angle = n*2*pi + x
            # Gist of the algorithm:
            # Find an equivalent angle for the current position that is lower and one that is higher than the requested angle. Go to the one that is closer.
            
            # First get the goal angle between zero and 2*pi (x)
            x = math.fmod(goal.angle, math.pi * 2)
            
            current_angle = self.table.get_encoder_angle()
            
            # Find the first n, for an equivalent angle that would be lower
            n_low = math.floor(current_angle / ( math.pi * 2 ))
            n_high = n_low + 1
            
            equiv_angle_low = n_low * 2 * math.pi + x
            equiv_angle_high = n_high * 2 * math.pi + x
            
            dist_to_low = math.fabs(current_angle - equiv_angle_low)
            dist_to_high = math.fabs(current_angle - equiv_angle_high)
            
            if dist_to_low < dist_to_high:
                new_goal = equiv_angle_low
            else:
                new_goal = equiv_angle_high
        else:
            new_goal = goal.angle

        rospy.loginfo(rospy.get_name() + ": modulo applied. New goal: angle=%f" % (new_goal))

        # make sure the motors are not turned off while working on this goal
        self.time_to_turn_motor_off = rospy.Time.now() + rospy.Duration.from_sec(10 * 60)

        if self.table.get_controller_state() == 0:
            self.table.start_controller()

        self.table.move_to_angle(new_goal)
        
        if goal.release_brake_timeout == 0:
            self.release_brake_timeout = 10 * 60 # turn off in 10min if nothing is specified
        else:
            self.release_brake_timeout = goal.release_brake_timeout
            

        while not self.table.at_target():

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                # Add something to exit the loop and goal
                self.table.move_to_angle(self.table.get_encoder_angle())
                break

            self._feedback.status = "moving"
            self._feedback.angle += self.table.get_encoder_angle()

            self._as.publish_feedback(self._feedback)
            r.sleep()

        # store the time after the last goal when the motor should be turned off
        self.time_to_turn_motor_off = rospy.Time.now() + rospy.Duration.from_sec(self.release_brake_timeout)

        if success:
            self._result.answer = "Reached the final position"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


def main():
    change_ps_name('st_action.py')

    rospy.init_node('scanning_table_action_server')
    rospy.loginfo("Starting up!")

    table = ElmoUdp()
    if not table.check_device():
        rospy.logerr("Scanning Table: Wrong or no ELMO device found, exiting")
        return (False)

    rospy.loginfo("Connected to ELMO driver with Firmware version: %s" % (table.get_firmware_version()))

    table.clear_cache()
    table.configure()
    table.reset_encoder()    
    table.start_controller()


    # Publish the angle of the joint
    js_topic_name = rospy.get_name() + '/joint_states'
    js_pub = rospy.Publisher(js_topic_name, JointState, queue_size=5)
    rospy.loginfo("I will publish table joint angle info to the topic %s", js_topic_name)

    # Get the action interface running
    saction = ScanningAction(rospy.get_name(), table)

    rate = 50  # Hz
    diag_rate = 1  # Hz
    r = rospy.Rate(rate)
    diag_last_time = rospy.get_rostime()

    # The working loop:
    while not rospy.is_shutdown():
        # Report JointStates
        jss = JointState()
        jss.header.stamp = rospy.Time.now()
        pos = table.get_encoder_angle()
        vel = table.get_encoder_velocity()
        cur = table.get_active_current()

        jss.name.append("scanning_table_z")

        jss.position.append(pos)
        jss.velocity.append(vel)
        jss.effort.append(cur)

        js_pub.publish(jss)

        # check if it is time to deactivate the drive
        if (table.get_controller_state() == 1) and (rospy.Time.now() > saction.time_to_turn_motor_off):
            rospy.loginfo('Turning controller off after timeout')
            table.stop_controller()
            

        r.sleep()

    # Getting out
    table.stop_controller()


if __name__ == "__main__":
    main()
