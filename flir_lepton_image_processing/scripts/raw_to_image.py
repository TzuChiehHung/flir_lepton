#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flir_lepton_msgs.msg import TemperatureRaw
from sensor_msgs.msg import Image
import numpy as np
import cv2


class RawToImage(object):


    def __init__(self):
        self.thermal_topic = rospy.get_param('subscribed_topic', 'raw')
        self.gray_image_topic = rospy.get_param('gray_image_topic', 'gray_image')
        self.display_min_max = rospy.get_param('display_min_max', False)

        self.sub_thermal = rospy.Subscriber(self.thermal_topic, TemperatureRaw, self.callback)

        self.pub_gray = rospy.Publisher(self.gray_image_topic, Image, queue_size=2)



    def callback(self, data):
        data = np.array(data.data, dtype=np.uint16).reshape(data.height, data.width)
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data)
        img = self.raw_to_8bit(data)

        if self.display_min_max:
            self.display_temperature(img, minVal, minLoc, (255, 0, 0))
            self.display_temperature(img, maxVal, maxLoc, (0, 0, 255))

        cv2.imshow('Lepton Radiometry', img)
        cv2.waitKey(1)

        # TODO: publish image topic


    @staticmethod
    def raw_to_c(val):
        return (val - 27315) / 100.0

    @staticmethod
    def raw_to_8bit(data):
        cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
        np.right_shift(data, 8, data)
        return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)

    def display_temperature(self, img, val, loc, color):
        val = self.raw_to_c(val)
        cv2.putText(img,"{0:.1f} degC".format(val), loc, cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
        x, y = loc
        cv2.line(img, (x - 2, y), (x + 2, y), color, 1)
        cv2.line(img, (x, y - 2), (x, y + 2), color, 1)


if __name__ == '__main__':
    rospy.init_node('raw_to_image', anonymous=True)
    RawToImage()
    rospy.spin()
