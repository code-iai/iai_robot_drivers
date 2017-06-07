#!/usr/bin/env python
import numpy as np
import rospy
from collections import defaultdict
from iai_control_msgs.msg._MultiJointVelocityImpedanceCommand import MultiJointVelocityImpedanceCommand
from sensor_msgs.msg._Joy import Joy
from std_srvs.srv._Trigger import Trigger, TriggerRequest

BUTTONS = dict()
BUTTONS['L2'] = 8
BUTTONS['down'] = 6
BUTTONS['left'] = 7
BUTTONS['up'] = 4
BUTTONS['right'] = 5
BUTTONS['x'] = 14
BUTTONS['circle'] = 13
BUTTONS['triangle'] = 12
BUTTONS['square'] = 15

class ZeroGravity(object):
    def __init__(self,
                 max_stiffness=(400, 400, 400, 300, 200, 200, 200),
                 min_stiffness=(0, 0, 0, 0, 0, 0, 0),
                 number_of_steps=10,
                 delay=.1,
                 left_arm_cmd_topic='/left_arm/command',
                 right_arm_cmd_topic='/right_arm/command'):
        self.delay = delay
        self.number_of_steps=number_of_steps
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_cb, queue_size=10)

        self.left_arm_name = 'left'
        self.right_arm_name = 'right'

        self.stiffness_pub = dict()
        self.stiffness_pub[self.left_arm_name] = rospy.Publisher(left_arm_cmd_topic, MultiJointVelocityImpedanceCommand, queue_size=10)
        self.stiffness_pub[self.right_arm_name] = rospy.Publisher(right_arm_cmd_topic, MultiJointVelocityImpedanceCommand, queue_size=10)
        # self.reset_setpoints_srv = rospy.ServiceProxy('reset_tcp', Trigger)
        self.buttons_pressed_before = defaultdict(lambda : False)

        self.max_stiffness = np.asarray(max_stiffness)
        self.min_stiffness = np.asarray(min_stiffness)
        self.step_size = (self.max_stiffness - self.min_stiffness) / number_of_steps

        self.stiffness = dict()
        self.stiffness['left'] = np.array(max_stiffness)
        self.stiffness['right'] = np.array(max_stiffness)

        self.reset_setpoints_threshold = np.array([40, 40, 40, 30, 20, 20, 20])
        rospy.loginfo('Boxy zero gravity mode started.')

    def joy_cb(self, data):
        buttons_currently_pressed = dict()
        for button_name, button_id in BUTTONS.items():
            buttons_currently_pressed[button_name] = data.buttons[button_id] != 0
        if buttons_currently_pressed['L2']:
            if self.buttons_pressed_before['down'] and not buttons_currently_pressed['down']:
                self.set_min_stiffness('left')
            elif self.buttons_pressed_before['up'] and not buttons_currently_pressed['up']:
                self.set_max_stiffness('left')
            elif self.buttons_pressed_before['left'] and not buttons_currently_pressed['left']:
                self.dec_stiffness('left')
            elif self.buttons_pressed_before['right'] and not buttons_currently_pressed['right']:
                self.inc_stiffness('left')
            elif self.buttons_pressed_before['x'] and not buttons_currently_pressed['x']:
                self.set_min_stiffness('right')
            elif self.buttons_pressed_before['triangle'] and not buttons_currently_pressed['triangle']:
                self.set_max_stiffness('right')
            elif self.buttons_pressed_before['square'] and not buttons_currently_pressed['square']:
                self.dec_stiffness('right')
            elif self.buttons_pressed_before['circle'] and not buttons_currently_pressed['circle']:
                self.inc_stiffness('right')

        for button_name in BUTTONS.keys():
            self.buttons_pressed_before[button_name] = buttons_currently_pressed['L2'] \
                                                       and buttons_currently_pressed[button_name] \
                                                       and np.sum(buttons_currently_pressed.values()) == 2

    def inc_stiffness(self, arm_name):
        self.change_stiffness(self.step_size, arm_name)

    def dec_stiffness(self, arm_name):
        self.change_stiffness(-self.step_size, arm_name)

    def set_max_stiffness(self, arm_name):
        while (self.stiffness[arm_name] < self.max_stiffness).all():
            self.inc_stiffness(arm_name)
            rospy.sleep(self.delay)

    def set_min_stiffness(self, arm_name):
        while (self.stiffness[arm_name] > self.min_stiffness+self.step_size).all():
            self.dec_stiffness(arm_name)
            rospy.sleep(self.delay)

    def change_stiffness(self, value, arm_name):
        self.stiffness[arm_name] += value
        self.stiffness[arm_name] = np.min((self.max_stiffness, self.stiffness[arm_name]), axis=0)
        self.stiffness[arm_name] = np.max((self.min_stiffness, self.stiffness[arm_name]), axis=0)
        cmd = MultiJointVelocityImpedanceCommand()
        cmd.stiffness = self.stiffness[arm_name]
        # self.stiffness_pub[arm_name].publish(cmd)
        rospy.loginfo('stiffness of {} arm set to {}'.format(arm_name, self.stiffness[arm_name]))
        # if (self.stiffness[arm_name] > self.reset_setpoints_threshold).all() and not self.reset_setpoints_srv(TriggerRequest()):
        #     rospy.logerr('setpoints reset failed!')



if __name__ == '__main__':
    rospy.init_node("zero_gravity")
    max_stiffness = rospy.get_param('~max_stiffness', default='[400, 400, 400, 300, 200, 200, 200]')
    min_stiffness = rospy.get_param('~min_stiffness', default='[0, 0, 0, 0, 0, 0, 0]')
    number_of_stiffness_levels = rospy.get_param('~number_of_stiffness_levels', default='10')
    left_arm_cmd_topic = rospy.get_param('~left_arm_cmd_topic', default='/left_arm/command')
    right_arm_cmd_topic = rospy.get_param('~rightt_arm_cmd_topic', default='/right_arm/command')

    gravity = ZeroGravity(max_stiffness=eval(max_stiffness),
                          min_stiffness=eval(min_stiffness),
                          number_of_steps=int(number_of_stiffness_levels),
                          left_arm_cmd_topic=left_arm_cmd_topic,
                          right_arm_cmd_topic=right_arm_cmd_topic)
    rospy.spin()
