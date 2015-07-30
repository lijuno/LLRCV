#!/usr/bin/python

import pygatt
import time

# A very quick and dirty test on RCV1 test with pygatt
# This simply uses time.sleep() to blindly wait long enough for BLE callbacks to finish, 
# which is not the "canonical" way to handle BLE events which are asynchronous by nature. 

class RCV1:
    def __init__(self, mac_addr):
        self.mac_addr = mac_addr
        self.moving_duration_total = 0
        self.sqtime = 0.5   # squirrel time, unit: s
        self.handle = None
        self.rv1_mode_cmd_handle = 0x11
        self.rv1_motion_cmd_handle = 0x13

    def mac_addr_type(self):
        pass

    def connect(self):
        pygatt.util.reset_bluetooth_controller()
        self.handle = pygatt.pygatt.BluetoothLEDevice(mac_addr, mac_random=True)
        self.handle.connect()

    def disconnect(self):
        self.handle.stop()

    def motion_job_add(self, properties, moving_duration, left_speed, right_speed):
        # moving_duration: in ms, an int
        # left_speed: 0~255, an int
        # right_speed: 0~255, an int
        self.handle.char_write(self.rv1_motion_cmd_handle,
                               bytearray([0x00,
                                  (moving_duration/10) & 0x00ff,
                                  (moving_duration/10) >>8,
                                  left_speed, right_speed]))
        self.moving_duration_total += moving_duration
        time.sleep(self.sqtime)

    def move(self):
        self.handle.char_write(self.rv1_mode_cmd_handle, bytearray([0x01]))
        time.sleep(self.moving_duration_total/float(1000) + self.sqtime)

    def stop(self):
        self.handle.char_write(self.rv1_mode_cmd_handle, bytearray([0x00]))
        time.sleep(self.sqtime)


if __name__=="__main__":
    mac_addr = "EF:30:5A:80:CB:26"  # BLE peripheral MAC address

    rcv = RCV1(mac_addr)
    rcv.connect()
    rcv.motion_job_add(0, 1000, 200, 200)
    rcv.motion_job_add(0, 1000, 100, 200)
    rcv.motion_job_add(0, 1000, 200, 100)
    rcv.motion_job_add(0, 1000, 200, 200)
    rcv.move()
    rcv.stop()
    rcv.disconnect()


