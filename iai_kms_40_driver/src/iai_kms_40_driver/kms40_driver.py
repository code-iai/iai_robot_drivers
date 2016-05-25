#!/usr/bin/env python
import telnetlib
import rospy
import sys
from geometry_msgs.msg._WrenchStamped import WrenchStamped
from iai_kms_40_msgs.srv._SetTare import SetTare, SetTareResponse
from multiprocessing import Lock


class KMS40Driver(object):
    def __init__(self, ip, port, hz=50, frame_id="/kms40", tcp_timeout=2., topic_name="/ft", service_name="/tare"):
        self.frame_id = frame_id
        self.tcp_timeout = tcp_timeout
        self.tn = telnetlib.Telnet(ip, port)
        self.mutex = Lock()
        if not self.send("VL(1)\n", "VL=1\n"):
            return
        if not self.send("FLT(1)\n", "FLT=1\n"):
            return
        frame_divider = 500 / hz
        if not self.send("LDIV({})\n".format(frame_divider), "LDIV={}\n".format(frame_divider)):
            return
        self.ft_pub = rospy.Publisher(topic_name, WrenchStamped, queue_size=10)
        self.tare_srv = rospy.Service(service_name, SetTare, self.tare_cb)
        rospy.sleep(.5)
        rospy.loginfo("kms driver is now running")

    def tare_cb(self, msg):
        with self.mutex:
            l0_received = self.send("L0()\n", "L0\n")
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
                print("Changed tare stat to {}".format(new_state))
            else:
                rospy.logerr("Failed to change tare state.")
            self.send("L1()\n", "L1\n")
            return result

    def send(self, msg, expected):
        self.tn.write(msg)
        msg = self.recv_chunk()
        return msg == expected

    def recv_chunk(self):
        msg = self.tn.read_until("\n", timeout=self.tcp_timeout)
        if msg == "":
            rospy.logerr("timeout while receiving, you probably need to reboot the sensor.")
        elif msg.startswith("ERR"):
            rospy.logerr("received error {}".format(msg))
        return msg

    def ft_to_msg(self, ft):
        start, end = ft.split("=")
        if start == "F":
            l = end.split("}")[0][1:].split(",")
            msg = self.list_to_wrench_stamped(l)
            msg.header.frame_id = self.frame_id
            msg.header.stamp = rospy.get_rostime()
            return msg
        return None

    def list_to_wrench_stamped(self, l):
        """
        :param l: list
        :return: WrenchStamped
        """
        w = WrenchStamped()
        w.wrench.force.x = float(l[0])
        w.wrench.force.y = float(l[1])
        w.wrench.force.z = float(l[2])
        w.wrench.torque.x = float(l[3])
        w.wrench.torque.y = float(l[4])
        w.wrench.torque.z = float(l[5])
        return w

    def stream_measurements(self):
        self.send("L1()\n", "L1\n")
        try:
            while not rospy.is_shutdown():
                with self.mutex:
                    msg = self.ft_to_msg(self.recv_chunk())
                    if msg is not None:
                        self.ft_pub.publish(msg)
        except:
            print(sys.exc_info()[0])
        finally:
            rospy.loginfo("shutdown kms40 driver")
            self.send("L0()\n", "L0")


if __name__ == '__main__':
    rospy.init_node('kms40_driver', anonymous=True)
    kms = KMS40Driver(rospy.get_param("/kms40/ip"),
                      rospy.get_param("/kms40/port"),
                      rospy.get_param("/kms40/publish_rate"),
                      rospy.get_param("/kms40/frame_id"),
                      rospy.get_param("/kms40/tcp_timeout"),
                      rospy.get_param("/kms40/topic_name"),
                      rospy.get_param("/kms40/service_name"))
    kms.stream_measurements()
