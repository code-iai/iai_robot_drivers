import numpy as np
import unittest

import rospy
from sensor_msgs.msg._Joy import Joy

from boxy_zero_gravity.zero_gravity import BUTTONS, ZeroGravity


class TestStringMethods(unittest.TestCase):
    def get_joy(self, buttons=()):
        buttons = [BUTTONS[x] for x in buttons]
        joy = Joy()
        for i in range(29):
            if i in buttons:
                joy.axes.append(-1)
            else:
                joy.axes.append(0)
        joy.buttons = [0 for _ in range(17)]
        return joy

    def setUp(self):
        self.zero_gravity = ZeroGravity(delay=0.)
        self.stiffness_before_right = np.copy(self.zero_gravity.stiffness['right'])
        self.stiffness_before_left = np.copy(self.zero_gravity.stiffness['left'])

    def set_stiffness_to_max(self):
        self.zero_gravity.step_size['right'] = self.zero_gravity.max_stiffness
        self.zero_gravity.step_size['left'] = self.zero_gravity.max_stiffness

    def set_stiffness_to_min(self):
        self.zero_gravity.step_size['right'] = self.zero_gravity.min_stiffness
        self.zero_gravity.step_size['left'] = self.zero_gravity.min_stiffness

    # left arm

    def test_max_stiffness_left00(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_max_stiffness_left01(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_max_stiffness_left02(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_max_stiffness_left03(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_min_stiffness_left00(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.min_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_min_stiffness_left01(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.min_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_min_stiffness_left02(self):
        stiffness_before = np.copy(self.zero_gravity.stiffness['left'])
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_less(stiffness_after, stiffness_before)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_min_stiffness_left03(self):
        stiffness_before = np.copy(self.zero_gravity.stiffness['left'])
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['down']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, stiffness_before)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_dec_stiffness_left00(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_less(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_dec_stiffness_left01(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'right']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_less(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_dec_stiffness_left03(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.min_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_dec_stiffness_left04(self):
        stiffness_before = np.copy(self.zero_gravity.stiffness['left'])
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, stiffness_before)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_dec_stiffness_left05(self):
        stiffness_before = np.copy(self.zero_gravity.stiffness['left'])
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_less(stiffness_after, stiffness_before)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_dec_stiffness_left06(self):
        stiffness_before = np.copy(self.zero_gravity.stiffness['left'])
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, stiffness_before)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_dec_stiffness_left07(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        for i in range(self.zero_gravity.number_of_steps):
            self.zero_gravity.joy_cb(self.get_joy())
            self.zero_gravity.joy_cb(self.get_joy(['L2']))
            self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
            self.zero_gravity.joy_cb(self.get_joy(['L2']))
            self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.min_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_dec_stiffness_left08(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        for i in range(self.zero_gravity.number_of_steps-1):
            self.zero_gravity.joy_cb(self.get_joy())
            self.zero_gravity.joy_cb(self.get_joy(['L2']))
            self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
            self.zero_gravity.joy_cb(self.get_joy(['L2']))
            self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_less(self.zero_gravity.min_stiffness, stiffness_after)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_inc_dec_stiffness_left00(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'right']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_inc_dec_stiffness_left01(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'right']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'left']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.min_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_inc_stiffness_left00(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'right']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_less(self.zero_gravity.min_stiffness, stiffness_after)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_inc_stiffness_left01(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'right']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(self.zero_gravity.max_stiffness, stiffness_after)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_inc_stiffness_left02(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['right']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.min_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_inc_stiffness_left03(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        for i in range(self.zero_gravity.number_of_steps-1):
            self.zero_gravity.joy_cb(self.get_joy())
            self.zero_gravity.joy_cb(self.get_joy(['L2']))
            self.zero_gravity.joy_cb(self.get_joy(['L2', 'right']))
            self.zero_gravity.joy_cb(self.get_joy(['L2']))
            self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_less(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_inc_stiffness_left04(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        for i in range(self.zero_gravity.number_of_steps):
            self.zero_gravity.joy_cb(self.get_joy())
            self.zero_gravity.joy_cb(self.get_joy(['L2']))
            self.zero_gravity.joy_cb(self.get_joy(['L2', 'right']))
            self.zero_gravity.joy_cb(self.get_joy(['L2']))
            self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])


    def test_misc_stiffness_left00(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2','down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up','right']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(self.zero_gravity.min_stiffness, stiffness_after)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    def test_misc_stiffness_left01(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2','down']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'right','up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'up']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
        np.testing.assert_array_equal(self.zero_gravity.max_stiffness, stiffness_after)
        np.testing.assert_array_equal(self.stiffness_before_right, self.zero_gravity.stiffness['right'])

    # right arm

    def test_max_stiffness_right00(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'triangle']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['right'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_left, self.zero_gravity.stiffness['left'])

    def test_max_stiffness_right01(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'x']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'triangle']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['right'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_left, self.zero_gravity.stiffness['left'])

    def test_min_stiffness_right00(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'x']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['right'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.min_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_left, self.zero_gravity.stiffness['left'])

    def test_min_stiffness_right01(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'triangle']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'x']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['right'])
        np.testing.assert_array_equal(stiffness_after, self.zero_gravity.min_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_left, self.zero_gravity.stiffness['left'])

    def test_dec_stiffness_right00(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'triangle']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'square']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['right'])
        np.testing.assert_array_less(stiffness_after, self.zero_gravity.max_stiffness)
        np.testing.assert_array_equal(self.stiffness_before_left, self.zero_gravity.stiffness['left'])

    def test_inc_stiffness_right00(self):
        self.zero_gravity.joy_cb(self.get_joy())
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'x']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy(['L2', 'circle']))
        self.zero_gravity.joy_cb(self.get_joy(['L2']))
        self.zero_gravity.joy_cb(self.get_joy())
        stiffness_after = np.copy(self.zero_gravity.stiffness['right'])
        np.testing.assert_array_less(self.zero_gravity.min_stiffness, stiffness_after)
        np.testing.assert_array_equal(self.stiffness_before_left, self.zero_gravity.stiffness['left'])

    # misc

    def test_misc00(self):
        for btm in BUTTONS.keys():
            stiffness_before = np.copy(self.zero_gravity.stiffness['left'])
            self.zero_gravity.joy_cb(self.get_joy())
            self.zero_gravity.joy_cb(self.get_joy([btm]))
            self.zero_gravity.joy_cb(self.get_joy())
            stiffness_after = np.copy(self.zero_gravity.stiffness['left'])
            np.testing.assert_array_equal(stiffness_before, stiffness_after)

    def test_misc01(self):
        for btm in BUTTONS.keys():
            stiffness_before = np.copy(self.zero_gravity.stiffness['right'])
            self.zero_gravity.joy_cb(self.get_joy())
            self.zero_gravity.joy_cb(self.get_joy([btm]))
            self.zero_gravity.joy_cb(self.get_joy())
            stiffness_after = np.copy(self.zero_gravity.stiffness['right'])
            np.testing.assert_array_equal(stiffness_before, stiffness_after)


if __name__ == '__main__':
    rospy.init_node("test")
    unittest.main()
