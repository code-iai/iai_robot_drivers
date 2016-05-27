#!/usr/bin/env python
import telnetlib
import timeit

import rospy
import sys
from geometry_msgs.msg._WrenchStamped import WrenchStamped
from iai_kms_40_msgs.srv._SetTare import SetTare, SetTareResponse
from multiprocessing import Lock


class KMS40Driver(object):

    def __init__(self, ip="192.168.102.70", port=1000, hz=50, frame_id="/kms40", tcp_timeout=2., topic_name="/ft", service_name="/tare"):
        self.frame_id = frame_id
        self.tcp_timeout = tcp_timeout
        self.tn = telnetlib.Telnet(ip, port)
        self.mutex = Lock()
        # set verbose level to 1 to see the error msgs
        if not self.send("VL(1)\n", "VL=1\n"):
            raise Exception("Unable to set verbose level.")
        # 5hz filter
        if not self.send("FLT(1)\n", "FLT=1\n"):
            raise Exception("Unable to set filter to 5hz.")
        if hz > 500:
            raise Exception("hz cannot be higher than 500.")
        frame_divider = 500 / hz
        if not self.send("LDIV({})\n".format(frame_divider), "LDIV={}\n".format(frame_divider)):
            raise Exception("Unable to set frame divider.")

        self.ft_pub = rospy.Publisher(topic_name, WrenchStamped, queue_size=10)
        self.tare_srv = rospy.Service(service_name, SetTare, self.tare_cb)
        rospy.sleep(.5)
        rospy.loginfo("kms driver is now running")

    def tare_cb(self, msg):
        """
        Callback for the tare service.
        :param msg: SetTareRequest
        :return: SetTareResponse
        """
        with self.mutex:
            l0_received = self.send("L0()\n", "L0\n")
            # there might be some unprocessed wrench msgs left, therefor we have to search for the L0
            max_waiting_tries = 500
            while not l0_received:
                max_waiting_tries -= 1
                if max_waiting_tries == 0:
                    raise Exception("Failed not stop wrench stream.")
                l0_received = self.recv_chunk() == "L0\n"

            result = SetTareResponse()
            new_state = 1 if msg.tare else 0
            result.success = self.send("TARE({})\n".format(new_state), "TARE={}\n".format(new_state))
            if result.success:
                rospy.loginfo("Changed tare stat to {}".format(new_state))
            else:
                rospy.logerr("Failed to change tare state.")
            self.send("L1()\n", "L1\n")
            return result

    def send(self, msg, expected):
        """
        Sends a msg to the sensor and checks for the excepted response.
        :param msg: string
        :param expected: string
        :return: bool
        """
        self.tn.write(msg)
        msg = self.recv_chunk()
        return msg == expected

    def recv_chunk(self):
        """
        Receives the next msg.
        :return: string
        """
        msg = self.tn.read_until("\n", timeout=self.tcp_timeout)
        if msg == "":
            raise Exception("timeout while receiving, you probably need to reboot the sensor.")
        return msg

    def ft_to_msg(self, ft):
        """
        Converts a string msg from the sensor into a WrenchStamped
        :param ft: string
        :return: WrenchStamped
        """
        if "F" == ft[0]:
            l = ft[3:].split(",")[:-1]
            w = WrenchStamped()
            w.header.frame_id = self.frame_id
            w.header.stamp = rospy.get_rostime()
            w.wrench.force.x = float(l[0])
            w.wrench.force.y = float(l[1])
            w.wrench.force.z = float(l[2])
            w.wrench.torque.x = float(l[3])
            w.wrench.torque.y = float(l[4])
            w.wrench.torque.z = float(l[5][:-1])
            return w
        elif ft[0] == "E":
            raise Exception("received error {}".format(msg))
        return None

    def stream_measurements(self):
        """
        Start the wrench stream and publishes the wrenches on a topic.
        """
        self.send("L1()\n", "L1\n")
        while not rospy.is_shutdown():
            with self.mutex:
                self.ft_pub.publish(self.ft_to_msg(self.recv_chunk()))

    def stop(self):
        rospy.loginfo("shutdown kms40 driver")
        self.send("L0()\n", "L0")


if __name__ == '__main__':
    rospy.init_node('kms40_driver', anonymous=True)
    kms = None
    try:
        kms = KMS40Driver(rospy.get_param("/kms40/ip"),
                          rospy.get_param("/kms40/port"),
                          rospy.get_param("/kms40/publish_rate"),
                          rospy.get_param("/kms40/frame_id"),
                          rospy.get_param("/kms40/tcp_timeout"),
                          rospy.get_param("/kms40/topic_name"),
                          rospy.get_param("/kms40/service_name"))
        # kms = KMS40Driver()
        # number = 100000
        # print(timeit.timeit(lambda : kms.ft_to_msg("F={-1.545,0.642,15.048,-0.051,-0.072,0.037},7181019"), number=number)/number)
        kms.stream_measurements()
    finally:
        if kms is not None:
            kms.stop()
