import rospy
import sys
import time
import threading
from .robolimb import RoboLimbCAN as RoboLimb
from .robolimb import SerialCommsBus
from six.moves import input
from std_msgs.msg import Float64MultiArray

class Ros_node(object):
    def __init__(self):
        bus = SerialCommsBus()
        self.r = RoboLimb(bus)
        self.r.start()
        self.r.open_all()
        time.sleep(1.5)

    def start(self):
        rospy.loginfo('Listening for commands')
        rospy.Subscriber("command", Float64MultiArray, self.callback)
        rospy.spin()

    def callback(self, data):
        rospy.loginfo('Received command: ' + str(data.data))
        for i in range(len(data.data)):
            if data.data[i] < -1.0 or data.data[i] > 1.0:
                rospy.logerr('Invalid finger command!')
                return
            if data.data[i!=5] < 0 and data.data[5] < 0:
                print(data.data[i])
                rospy.logerr('---')
                return
        for i in range(len(data.data)):
            if data.data[i] < 0:
                self.r.close_finger(i+1, int(-data.data[i]*self.r.def_vel))
            elif data.data[i] > 0:
                self.r.open_finger(i+1, int(data.data[i]*self.r.def_vel))
            else:
                self.r.stop_finger(i+1)

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


