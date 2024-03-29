#!/usr/bin/env python
import telnetlib
import timeit

import rospy
import sys
from geometry_msgs.msg._WrenchStamped import WrenchStamped
from multiprocessing import Lock
from std_srvs.srv._SetBool import SetBool, SetBoolResponse, SetBoolRequest


class KMS40Driver:

    def __init__(self,
                 ip: str = '192.168.102.70',
                 port: int = 1000,
                 hz: int = 50,
                 frame_id: str = '/kms40',
                 tcp_timeout: float = 2.0,
                 topic_name: str = '/ft',
                 service_name: str = '~tare'):
        self.frame_id = frame_id
        self.tcp_timeout = tcp_timeout
        self.tn = telnetlib.Telnet(ip, port)
        self.mutex = Lock()
        # set verbose level to 1 to see the error msgs
        if not self.send_and_wait('VL(1)\n', 'VL=1\n'):
            raise Exception('Unable to set verbose level.')
        # 5hz filter
        if not self.send_and_wait('FLT(1)\n', 'FLT=1\n'):
            raise Exception('Unable to set filter to 5hz.')
        if hz > 500:
            raise Exception('hz cannot be higher than 500.')
        frame_divider = 500 / hz
        if not self.send_and_wait(f'LDIV({int(frame_divider)})\n', f'LDIV={int(frame_divider)}\n'):
            raise Exception('Unable to set frame divider.')

        self.ft_pub = rospy.Publisher(topic_name, WrenchStamped, queue_size=10)
        self.tare_srv = rospy.Service(service_name, SetBool, self.tare_cb)
        rospy.sleep(.5)
        rospy.loginfo('kms driver is now running')

        self.net_error_count = 0

        self.ip = ip
        self.port = port

    def tare_cb(self, msg: SetBoolRequest) -> SetBoolResponse:
        """
        Callback for the tare service.
        """
        with self.mutex:
            if not self.send_and_wait('L0()\n', 'L0\n'):
                # there might be some unprocessed wrench msgs left, therefor we have to search for the L0
                raise Exception('Failed not stop wrench stream.')

            result = SetBoolResponse()
            new_state = 1 if msg.data else 0
            result.success = self.send_and_wait('TARE({})\n'.format(new_state), 'TARE={}\n'.format(new_state))
            if result.success:
                rospy.loginfo('Changed tare stat to {}'.format(new_state))
                result.message = 'Changed tare stat to {}'.format(new_state)
            else:
                rospy.logerr('Failed to change tare state.')
            self.send('L1()\n', 'L1\n')
            return result

    def send(self, msg: str, expected: str) -> bool:
        """
        Sends a msg to the sensor and checks for the excepted response.
        """
        self.tn.write(msg.encode('utf-8'))
        return expected == self.recv_chunk()

    def send_and_wait(self, msg: str, expected: str, max_waiting_tries: int = 10) -> bool:
        """
        Sends a msg and waits for the expected response. Returns False if it has not been received within
        'max_waiting_tries' msgs.
        """
        self.tn.write(msg.encode('utf-8'))
        for i in range(max_waiting_tries):
            if self.recv_chunk() == expected.encode('utf-8'):
                return True
        return False

    def recv_chunk(self) -> bytes:
        """
        Receives the next msg.
        """

        try_to_read = True
        self.net_error_count = 0

        msg = ''
        import socket

        while try_to_read:
            if self.net_error_count > 10:
                raise Exception('timeout while receiving, you probably need to reboot the sensor.')

            try:
                msg = self.tn.read_until(b'\n', timeout=self.tcp_timeout)
            except socket.error:
                self.net_error_count += 1
                rospy.logerr(sys.exc_info()[0])
                import time
                rospy.logerr('will restart connection')
                time.sleep(5)
                self.tn = telnetlib.Telnet(self.ip, self.port)

                rospy.logwarn('restarting streaming')
                kms.stream_measurements()

                rospy.logwarn('should work again')

            if msg == '':
                self.net_error_count += 1
                break
            else:
                self.net_error_count = 0
                try_to_read = False

        return msg

    def ft_to_msg(self, ft: bytes) -> WrenchStamped:
        """
        Converts a string msg from the sensor into a WrenchStamped
        """
        ft = ft.decode('utf-8')
        if 'F' == ft[0]:
            l = ft[3:].split(',')[:-1]
            # creates a list that looks like this: ['1.23', '2.34', .., '7.89}']
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
        elif ft[0] == 'E':
            raise Exception(f'received error {ft}.')

    def stream_measurements(self):
        """
        Start the wrench stream and publishes the wrenches on a topic.
        """
        self.send('L1()\n', 'L1\n')
        while not rospy.is_shutdown():
            with self.mutex:
                recv_data = self.recv_chunk()
                if recv_data != '':
                    self.ft_pub.publish(self.ft_to_msg(recv_data))

    def stop(self):
        """
        Stops the transmission
        """
        rospy.loginfo('shutdown kms40 driver')
        self.send_and_wait('L0()\n', 'L0\n')


if __name__ == '__main__':
    rospy.init_node('kms40_driver')
    kms = None
    try:
        kms = KMS40Driver(rospy.get_param('~ip'),
                          rospy.get_param('~port'),
                          rospy.get_param('~publish_rate'),
                          rospy.get_param('~frame_id'),
                          rospy.get_param('~tcp_timeout'),
                          rospy.get_param('~topic_name'),
                          rospy.get_param('~service_name'))
        kms.stream_measurements()
    finally:
        # this hopefully always stops the data transmission, otherwise it is likely, that the sensor has to be rebooted
        if kms is not None:
            kms.stop()
