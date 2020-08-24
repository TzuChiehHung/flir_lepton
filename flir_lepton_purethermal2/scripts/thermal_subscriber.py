#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flir_lepton_msgs.msg import TemperatureRaw
import numpy as np


class ThermalSubscriber(object):


    def __init__(self):
        self.sub_thermal = rospy.Subscriber('thermal/raw', TemperatureRaw, self.callback)

    def callback(self, data):
        temp = np.array(data.data).reshape(data.height, data.width)
        print(temp)


if __name__ == '__main__':
    rospy.init_node('thermal_subscriber', anonymous=True)
    ThermalSubscriber()
    rospy.spin()
