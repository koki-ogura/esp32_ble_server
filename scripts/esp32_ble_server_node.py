#!/usr/bin/env python

#---------------------------------------------------------------------------
# esp32_ble_server_node.py
#---------------------------------------------------------------------------
import sys
import signal
import math
from threading import Lock
import serial
import rospy
from esp32_ble_server.msg import *
from esp32_ble_server.srv import *

#---------------------------------------------------------------------------
# BLE Controller Interface class
#---------------------------------------------------------------------------
class BleController:
    def __init__(self):
        self.serial_lock = Lock()
        self.sp = None

    def start(self):
        with self.serial_lock:
            if self.sp is None:
                self.sp = serial.Serial('/dev/blecon', baudrate=115200, timeout=0.2)
        
    def finish(self):
        with self.serial_lock:
            if self.sp is not None:
                self.sp.close()
                self.sp = None

    def do_command(self, command, max_lines=100, timeout=8.0):
        command += '\n'
        lines = []
        with self.serial_lock:
            if self.sp is not None:
                self.sp.timeout = timeout
                self.sp.write(command.encode('utf-8'))
                self.sp.flush()
                for i in range(max_lines):
                    line = self.sp.readline().decode('utf-8')
                    if (line == ''): break
                    line = line.replace('\n', '').replace('\r', '')
                    lines.append(line)
        return lines

    def print_results(self, lines):
        for line in lines:
            print line

    def get_ble_msg(self):
        lines = self.do_command('ble', max_lines=2)
        return lines[1]

    def put_ble_msg(self, msg):
        self.do_command('ble %s' % msg, max_lines=1)

#---------------------------------------------------------------------------
# esp32_ble_server_node
#---------------------------------------------------------------------------
cont = True
ble_cont = None

def sigint_handler(signal, frame):
    global cont
    cont = False

def ble_srv_handler(msg):
    global ble_cont
    ble_cont.put_ble_msg(msg.message)
    return BleSrvResponse('ok')

def esp32_ble_server_node():
    global cont
    global ble_cont
    # ROS
    rospy.init_node('esp32_ble_server')
    rospy.loginfo('%s: Initialize esp32_ble_server', rospy.get_name())
    ble_cont = BleController()
    ble_cont.start()
    ble_cont.do_command('')
    # ble_msg publisher
    ble_msg_pub = rospy.Publisher('ble_msg', BleMsg, queue_size = 10)
    ble_seq = 0
    # ble_srv server
    ble_srv_server = rospy.Service('ble_srv', BleSrv, ble_srv_handler)

    rate = rospy.Rate(40)
    while cont:
        cur_time = rospy.Time.now()
        # ble
        try:
            msg = ble_cont.get_ble_msg()
            if msg != '':
                ble_seq += 1
                ble_msg = BleMsg()
                ble_msg.header.seq = ble_seq
                ble_msg.header.stamp = cur_time
                ble_msg.header.frame_id = 'ble'
                ble_msg.message = msg
                ble_msg_pub.publish(ble_msg)
        except Exception as e:
            rospy.loginfo('get_ble_msg exception: %s', e)
            #cont = False
            break

        rate.sleep()

    ble_cont.finish()
    rospy.loginfo('finish: esp32_ble_server')
    rospy.signal_shutdown('finish')
    rospy.spin()

#---------------------------------------------------------------------------
# main
#---------------------------------------------------------------------------
if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    esp32_ble_server_node()
