#!/usr/bin/env python
import rospy
import sys
import time
import threading
from robolimb import RoboLimbCAN as RoboLimb
from robolimb import SerialCommsBus
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class Ros_node(object):
    def __init__(self):
        port = rospy.get_param('~port', '/dev/can')
        self.joint = rospy.get_param('~joint_names', [])

        self.pub = rospy.Publisher('state', JointState, queue_size=10)
        self.msg = JointState()
        self.msg.position = [0.0] * 10
        self.msg.velocity = [0.0] * 10
        self.msg.name = self.joint

        self.last_position = [0.0] * 6

        rospy.loginfo('Openning port ' + port)
        bus = SerialCommsBus(port=port)
        self.r = RoboLimb(bus)
        self.r.start()
        self.r.open_all()
        time.sleep(1.5)
        self.command_time = time.time()

    def start(self):
        rospy.loginfo('Listening for commands')
        rospy.Subscriber("command", Float64MultiArray, self.callback)
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            self.msg.header.stamp = rospy.Time.now()
            for i in range(6):
                self.msg.position[i] = self.last_position[i]
                if i >= 1 and i <= 4:
                    self.msg.position[i+5] = self.last_position[i]
            self.pub.publish(self.msg)
            rate.sleep()

    def callback(self, data):
        rospy.loginfo('Received command: ' + str(data.data))
        if len(data.data) != 6:
            raise RuntimeError('Invalid trajectory data size!')

        now = time.time()
        if now - self.command_time < 0.2:
            return

        self.command_time = now
        for i in range(len(data.data)):
            if data.data[i] < -1.0 or data.data[i] > 1.0:
                rospy.logerr('Invalid finger command!')
                return
            if data.data[i!=5] < 0 and data.data[5] < 0:
                return
        for i in range(len(data.data)):
            if data.data[i] < 0:
                self.r.close_finger(i+1, int(-data.data[i]*self.r.def_vel))
            elif data.data[i] > 0:
                self.r.open_finger(i+1, int(data.data[i]*self.r.def_vel))
            else:
                self.r.stop_finger(i+1)

        for i in range(6):
            if data.data[i] > 0.0:
                self.last_position[i] = 0.0
            elif data.data[i] < 0.0:
                self.last_position[i] = 1.4
        

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    ex = Ros_node()
    ex.start()

    # Close on exit
    ex.r.open_all()
    time.sleep(1)
    ex.r.close_finger(1)
    time.sleep(1)
    ex.r.stop()


