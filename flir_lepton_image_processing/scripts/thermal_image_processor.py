#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flir_lepton_msgs.msg import TemperatureRaw, TemperatureMsg
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ThermalImageProcessor(object):


    def __init__(self):
        self.raw_topic = rospy.get_param('raw_topic', 'raw')
        self.image_topic = rospy.get_param('image_topic', 'image')
        self.temperature_topic = rospy.get_param('temperature_topic', 'celsius')
        self.display_min_max = rospy.get_param('display_min_max', False)
        self.use_colormap = rospy.get_param('use_colormap', False)

        self.sub_raw = rospy.Subscriber(self.raw_topic, TemperatureRaw, self.callback)

        self.pub_temp = rospy.Publisher(self.temperature_topic, TemperatureMsg, queue_size=2)
        self.pub_image = rospy.Publisher(self.image_topic, Image, queue_size=2)
        self.bridge = CvBridge()

    def callback(self, data):
        self.raw_to_temperature(data)
        self.raw_to_image(data)

    def raw_to_temperature(self, data):
        temp = TemperatureMsg()
        temp.header = data.header
        temp.height = data.height
        temp.width = data.width
        temp.unit = 'celsius'
        temp.data = self.raw_to_c(np.array(data.data, dtype=np.uint16))
        self.pub_temp.publish(temp)

    def raw_to_image(self, data):
        header = data.header
        data = np.array(data.data, dtype=np.uint16).reshape(data.height, data.width)
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data)

        img = self.raw_to_8bit(data)

        if self.use_colormap:
            img = cv2.applyColorMap(img, cv2.COLORMAP_JET)

        if self.display_min_max:
            self.display_temperature(img, minVal, minLoc, (255, 0, 0))
            self.display_temperature(img, maxVal, maxLoc, (0, 0, 255))

        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            img_msg.header = header
            self.pub_image.publish(img_msg)
        except CvBridgeError() as e:
            rospy.logerr(e)

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
    rospy.init_node('thermal_image_processor', anonymous=True)
    ThermalImageProcessor()
    rospy.spin()
