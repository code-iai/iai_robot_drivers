#!/usr/bin/env python
#
# Scanning Table Driver - Commanding an Elmo Gold DC-Whistle (G-DCWHI20/100EET) motor driver over UDP
#
# Copyright (c) 2013, 2017 Universitaet Bremen, Institute for Artificial Intelligence - Prof. Michael Beetz
# Author: Alexis Maldonado <amaldo@cs.uni-bremen.de>
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
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


# The commands used for the Elmo Communication are taken from the
# document "Command Reference for Gold Line Drives"

import socket
import time
import math
import threading


# FIXME: Lower the current to the motor when not moving actively


def change_ps_name(name):
    """Change process name as used by ps(1) and killall(1)."""
    try:
        import ctypes

        libc = ctypes.CDLL('libc.so.6')
        libc.prctl(15, name, 0, 0, 0)
    except:
        pass


# -----  Important constants ------
MAC_ADDRESS = '10:18:9E:00:37:D3'
ELMO_IP = '192.168.100.167'
ELMO_PORT = 5001



BELT_RATIO = 34.0 / 15.0
MOTOR_ENCODER_LINES = 1000  # 4 encoder ticks per encoder line
ENCODER_TO_TABLE_ANGLE = 2 * math.pi / (BELT_RATIO * MOTOR_ENCODER_LINES * 4)  # In rads/tick
#MOTOR_NOMINAL_CURRENT = 5  # Motor has 5.6A , we are using a bit less

# -----


class ElmoUdp(object):
    def __init__(self):
        """Open an UDP socket and connect to the given IP and Port.
        Also set the timeout for the blocking operations (read)."""
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.connect((ELMO_IP, ELMO_PORT))
        self.s.settimeout(0.1)
        self.last_speed = 0
        self.last_angle_goal = 0
        self.last_acceleration = 0
        self.comm_lock = threading.Lock()
        self.time_at_target = 0

    def __del__(self):
        """Turn all controllers off (no more current) when exiting"""
        self.stop_controller()

        self.s.shutdown(socket.SHUT_RDWR)

    def get_controller_state(self):
        return int(self.sendcmd("SO"))

    def stop_controller(self):
        """Stop the motor controller using the MO (Motor On) command"""
        self.sendcmd("MO=0")

    @staticmethod
    def convert_rad_to_ticks(rad):
        """Convert from rads to ticks"""
        return int(rad / ENCODER_TO_TABLE_ANGLE)

    @staticmethod
    def convert_ticks_to_rad(ticks):
        """Convert from ticks to rads"""
        return ticks * ENCODER_TO_TABLE_ANGLE
    
    def set_speed(self, speed):
        """change the maximum speed of the motion profile. In rads/s
        Meaningful values are 20000"""

        sp = self.convert_rad_to_ticks(speed)
        print("Setting speed to sp=%d" % (sp))
        self.sendcmd("sp=%d" % (sp))
        self.last_speed = speed

    def set_speed_deg(self, deg):
        """Set the maximum speed in degrees/s"""
        self.set_speed(math.radians(deg))

    def set_accel(self, accel):
        """change the acceleration of the motion profile. In rads/s^2"""
        ac = self.convert_rad_to_ticks(accel)
        print("Setting acceleration to ac=%d" % (ac))
        self.sendcmd("ac=%d" % (sp))
        self.sendcmd("dc=%d" % (sp))

        self.last_acceleration = accel

    def get_firmware_version(self):
        """Return the firmware version information"""
        return (self.sendcmd("vr"))

    def sendcmd(self, cmd):
        """Send a command to the elmo drive and receive the response.
        The termination ";" is added automatically to the command and removed from the answer.
        The function checks that the drive echos the original command.
        If the answer comes in more than one UDP packet, it reads until it has seen both ";" terminators
        """

        self.comm_lock.acquire()
        terminated_cmd = cmd + ";"

        # Write to the UDP socket
        msg_len = self.s.send(terminated_cmd)

        acc_ans = ""
        try:
            # Wait until we get two ';'s or time-out
            receiving = True

            while receiving:
                acc_ans += self.s.recv(4096)
                if acc_ans.count(";") == 2:
                    receiving = False
        except:
            print("waited for too long for an answer")

        data = acc_ans.split(";")
        if data[0] != cmd:
            print("Error, received an unexpected answer. Sent \"%s\" Received \"%s\" ") % (terminated_cmd, acc_ans)
        else:
            receiving = False
            self.comm_lock.release()
            return (data[1])

        self.comm_lock.release()
        return ("")

    def configure(self):
        """Configure the parameters of the controller.
        Currently configures only the profiler.
        The rest of the options are saved in EEPROM from the drive setup using the Elmo EAS"""

        # ac = profile acceleration (ticks/s/s)
        # dc = profile deceleration (ticks/s/s)
        # sd = stopping speed (for emergencies)
        # sp = profile speed (ticks/s)
        self.sendcmd("ac=3000")
        self.sendcmd("dc=3000")
        self.sendcmd("sd=10000")
        self.sendcmd("sp=2000")

        # Read values once to cache them
        self.get_profile_speed()
        self.get_profile_accel()

    def start_controller(self):
        """Set the driver to the right mode, and start up the controller"""
        self.sendcmd("mo=0")  # make sure motor is disabled (needed for changing UM Unit Mode)
        self.sendcmd("um=5")  # position mode
        self.sendcmd("mo=1")  # activate the motor

        time.sleep(0.5)  # some time for it to find the conmutation

        waiting = True
        counter = 20

        while waiting:
            ready = self.sendcmd("SO")
            # print "ready %s" %(ready)
            if ready == "1":
                waiting = False
            if counter == 0:
                print("Timeout while waiting for the conmutation to be found")
                waiting = False
                return (False)

            counter -= 1
            time.sleep(0.1)

        return (True)

    def set_current(self, current):
        print('Setting current to %f amps' % current)
        self.sendcmd("tc=%f" % (current))

    def get_holding_current(self):
        return float(self.sendcmd("tc"))


    def get_profile_speed(self):
        """Return the configured speed in radians/s. It is cached. """
        if self.last_speed == 0:
            speed = int(self.sendcmd("sp"))
            self.last_speed = self.convert_ticks_to_rad(speed)
        else:
            speed = self.last_speed

        return (speed)

    def get_profile_accel(self):
        """Return the configured acceleration in radians/s^2. It is cached."""
        if self.last_acceleration == 0:
            accel = int(self.sendcmd("ac"))
            self.last_acceleration = self.convert_ticks_to_rad(accel)
        else:
            accel = self.last_acceleration

        return (accel)
    
    def estimate_time_to_reach_angle(self, angle):
        """Estimates the time needed to reach an angle, with the current parameters.
        """
        # Store when it should approx get ready
        speed = self.get_profile_speed()
        accel = self.get_profile_accel()
        cur_pos = self.get_encoder_angle()
        distance = abs(cur_pos - angle)
        # print("Distance = %f. cur_pos = %f. Angle = %f" %(distance, cur_pos, angle))

        # This is a rough worst estimate. If it never reaches full speed, it over-estimates the time
        # Also assumes starts and ends with speed=0
        accel_time = speed / accel  # t = (vf-vi)/a
        travel_while_accelerating = 0.5 * accel * accel_time ** 2  # s=vi*t + 1/2 * a * t^2
        travel_while_decelerating = travel_while_accelerating
        # print("speed = %f  accel = %f travel_while_accelerating = %f" %(speed, accel, travel_while_accelerating))
        travel_time_at_full_speed = (distance - (travel_while_accelerating + travel_while_decelerating)) / speed
        if travel_time_at_full_speed < 0:
            travel_time_at_full_speed = 0

        travel_time = travel_time_at_full_speed + 2 * accel_time

        return(travel_time)
        

    def move_to_angle(self, angle):
        """Move to an absolute position in radians"""

        self.last_angle_goal = angle  # Store the goal command
        self.time_at_target = time.time() + self.estimate_time_to_reach_angle(angle)  #Store a time estimate to finish the motion

        ticks = self.convert_rad_to_ticks(angle)
        self.sendcmd("pa=%s" % ticks)   # PA = move to Absolute Position
        self.sendcmd("bg")  # BG = Begin movement        

    def move_to_deg(self, deg):
        """Call move_to_angle converting the degrees to radians first"""
        self.move_to_angle(math.radians(deg))

    def clear_cache(self):
        """Read from the UDP socket until it times out, discarding the data"""
        self.s.settimeout(0.1)

        try:
            while (True):
                tmp = self.s.recv(4096)

        except:
            print("Cache is empty")

    def reset_encoder(self):
        """Set to zero the counter of the encoder on the motor.
        Useful after starting the current controller (and before any movement)
        to bring both electrical angle ticks and optical encoder in sync (in um=3)"""
        self.sendcmd("px=0")

    def check_device(self):
        """Check that the MAC address of the ELMO controller fits"""
        mac = self.sendcmd("AA[2]")
        if mac != MAC_ADDRESS:
            print("Warning: Received MAC address %s, expected %s") % (mac, MAC_ADDRESS)
            return (False)

        return True

    def at_target(self):
        """Check that the motor has reached the target"""
        ans = self.sendcmd("MS")  # MS = Motion Status

        # MS will return in PA or PR movement modes:
        # 3 : Motor disabled
        # 2 : Position command is in motion
        # 1 : Position command has reached the target
        # 0 : Actual position has reached the target        

        # print("MS=%s" %(ans))
        if (ans == "0"):
            return (True)
        else:
            return (False)

    def wait_to_reach_target(self, extra_timeout=1.0):
        """Block until the target position is reached
           Take the time from the predicted time and add an extra_timeout"""
        waiting = True
        now = time.time()

        while (waiting):
            if self.at_target():
                waiting = False
            rem_time = time.time() - self.time_at_target
            if (rem_time > extra_timeout):
                print("Timed out while waiting for motor to reach the target position")
                waiting = False
                return (False)

            print("Pos = %06.2f  Time to go: %06.2f" % (math.degrees(self.get_encoder_angle()), -rem_time))
            time.sleep(0.1)

        return True

    def get_encoder_angle(self):
        """Read PX (Main Position in counts) and
           convert the encoder ticks to radians of the turning table
        """
        enc_ticks = int(self.sendcmd("PX"))
        return (enc_ticks * ENCODER_TO_TABLE_ANGLE)

    def get_encoder_velocity(self):
        """Read VX (Main encoder velocity in counts) and
           convert the encoder ticks to radians of the turning table
        """
        enc_ticks = int(self.sendcmd("VX"))
        return (enc_ticks * ENCODER_TO_TABLE_ANGLE)

    def get_active_current(self):
        """Read IQ (active current in amps)
        """
        return float(self.sendcmd("IQ"))




def main():
    # Small demo of the turning table

    change_ps_name('st_control.py')

    import time

    table = ElmoUdp()

    if not table.check_device():
        print("Wrong or no device found, exiting")
        return (False)

    print("Connected to ELMO driver with Firmware version: %s" % (table.get_firmware_version()))

    table.configure()
    table.reset_encoder()
    table.start_controller()


    table.set_speed_deg(30)
    table.move_to_deg(45)
    table.wait_to_reach_target()

    table.set_speed_deg(10)
    table.move_to_deg(90)
    table.wait_to_reach_target()

    table.set_speed_deg(30)
    table.move_to_deg(0)
    table.wait_to_reach_target()


if __name__ == "__main__":
    main()
