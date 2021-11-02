#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from flir_lepton_msgs.msg import TemperatureRaw
import numpy as np


class SubscriberNode(Node):


    def __init__(self):
        super().__init__('purethermal2_subscriber')
        self.subscriber = self.create_subscription(TemperatureRaw, 'raw', self.callback, qos_profile_sensor_data)

    def callback(self, data):
        temp = np.array(data.data).reshape(data.height, data.width)
        print(temp)

def main(args=None):
    rclpy.init(args=args)

    nh = SubscriberNode()
    rclpy.spin(nh)

    nh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
