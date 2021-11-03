#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from flir_lepton_msgs.msg import TemperatureRaw, TemperatureMsg
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ThermalImageProcessor(Node):


    def __init__(self):
        super().__init__('thermal_image_processor')
        self.raw_topic = self.declare_parameter('raw_topic', 'raw')
        self.image_topic = self.declare_parameter('image_topic', 'image')
        self.temperature_topic = self.declare_parameter('temperature_topic', 'celsius')
        self.display_min_max = self.declare_parameter('display_min_max', False)

        self.get_logger().info('subscribe topic: {}'.format(self.raw_topic.value))
        self.get_logger().info('publish image topic: {}'.format(self.image_topic.value))
        self.get_logger().info('publish temperature topic: {}'.format(self.temperature_topic.value))
        self.get_logger().info('display_min_max: {}'.format(self.display_min_max.value))

        self.sub = self.create_subscription(TemperatureRaw, self.raw_topic.value, self.callback, 10)
        self.pub_temp = self.create_publisher(TemperatureMsg, self.temperature_topic.value, 10)
        self.pub_image = self.create_publisher(Image, self.image_topic.value, 10)
        self.bridge = CvBridge()

    def callback(self, data):
        self.raw_to_temperature(data)
        self.raw_to_image(data)

    def raw_to_temperature(self, data):
        msg = TemperatureMsg()
        msg.header = data.header
        msg.height = data.height
        msg.width = data.width
        msg.unit = 'celsius'
        msg.data = self.raw_to_c(np.array(data.data, dtype=np.uint16)).tolist()
        self.pub_temp.publish(msg)

    def raw_to_image(self, data):
        header = data.header
        data = np.array(data.data, dtype=np.uint16).reshape(data.height, data.width)
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data)

        img = self.raw_to_8bit(data)

        if self.display_min_max.value:
            self.display_temperature(img, minVal, minLoc, (255, 0, 0))
            self.display_temperature(img, maxVal, maxLoc, (0, 0, 255))

        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            img_msg.header = header
            self.pub_image.publish(img_msg)
        except CvBridgeError() as e:
            self.get_logger().error(e)

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


def main(args=None):
    rclpy.init(args=args)
    nh = ThermalImageProcessor()

    try:
        rclpy.spin(nh)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        nh.get_logger().error(e)
    finally:
        nh.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()