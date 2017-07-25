#!/usr/bin/env python
import rospy
from iai_naive_kinematics_sim.srv._SetJointState import SetJointState, SetJointStateRequest
from iai_wsg_50_msgs.msg._PositionCmd import PositionCmd
from iai_wsg_50_msgs.msg._Status import Status
from sensor_msgs.msg._JointState import JointState
from std_msgs.msg._Bool import Bool


class WSG50SimDriver(object):
    def __init__(self, gripper_name):
        self.js = None
        self.gripper_name = gripper_name
        self.link_id = None

        self.moving_pub = rospy.Publisher('~moving', Bool, queue_size=10)
        self.state_pub = rospy.Publisher('~state', Status, queue_size=10)
        self.boxy_cmd = rospy.Publisher('boxy/commands', JointState, queue_size=10)

        self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.js_cb, queue_size=10)
        self.goal_pose_sub = rospy.Subscriber('~goal_position', PositionCmd, self.goal_pose_cb, queue_size=10)
        self.goal_speed_sub = rospy.Subscriber('~goal_speed', PositionCmd, self.goal_speed_cb, queue_size=10)

        self.boxy_joint_state = rospy.ServiceProxy('/boxy/set_joint_states', SetJointState)

    def js_cb(self, data):
        if self.link_id is None:
            self.link_id = data.name.index(self.gripper_name)
        self.js = data
        moving = Bool()
        moving.data = abs(self.js.velocity[self.link_id]) > 0e-4
        self.moving_pub.publish(moving)
        state = Status()
        state.header.stamp = rospy.get_rostime()
        state.width = self.js.position[self.link_id]
        state.speed = self.js.velocity[self.link_id] * 1000
        self.state_pub.publish(state)

    def goal_pose_cb(self, data):
        rospy.loginfo('Position command: pos={}, speed={}'.format(data.pos, data.speed))
        multiplier = 1000
        move_cmd = JointState()
        move_cmd.name = [self.gripper_name]
        move_cmd.position = [data.pos / multiplier]
        move_cmd.velocity = [abs(data.speed) / multiplier]
        if move_cmd.position[0] < self.js.position[self.link_id]:
            move_cmd.velocity = [x * -1 for x in move_cmd.velocity]
        rate = rospy.Rate(10)
        start_time = rospy.get_rostime()
        while not rospy.is_shutdown() and abs(self.js.position[self.link_id] - move_cmd.position[0]) > .007:
            self.boxy_cmd.publish(move_cmd)
            rate.sleep()
            if rospy.get_rostime() - start_time > rospy.Duration(10):
                rospy.logwarn('movement took too long, stopping.')
                break
        move_cmd.velocity = [0.0 for _ in move_cmd.velocity]
        move_cmd.effort = [0.0 for _ in move_cmd.velocity]
        srv_cmd = SetJointStateRequest()
        srv_cmd.state = move_cmd
        # print('send {}'.format(srv_cmd))
        self.boxy_joint_state.call(srv_cmd)
        # self.boxy_joint_state(srv_cmd)
        rospy.loginfo('done.')

    def goal_speed_cb(self, data):
        move_cmd = JointState()
        move_cmd.name = [self.gripper_name]
        move_cmd.velocity = [data.speed / 1000]
        self.boxy_cmd.publish(move_cmd)


if __name__ == '__main__':
    rospy.init_node("left_arm_gripper")
    gripper_joint_name = rospy.get_param('~gripper_joint_name', default='left_gripper_joint')
    node = WSG50SimDriver(gripper_joint_name)
    rospy.loginfo("kms 50 sim driver for '{}' joint running.".format(gripper_joint_name))
    rospy.spin()
