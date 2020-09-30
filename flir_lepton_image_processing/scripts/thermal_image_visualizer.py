#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flir_lepton_msgs.msg import TemperatureRaw, TemperatureMsg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ThermalImageVisualizer:


    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(12, 9))

        self.data = np.zeros((120, 160))
        self.img = plt.imshow(self.data, vmin=0, vmax=1)
        plt.tight_layout()

    def plot_init(self):
        self.ax.set_xlim(0, 160)
        self.ax.set_ylim(0, 120)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        return self.img

    def thermal_callback(self, msg):
        self.data = np.array(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    def update_plot(self, frame):
        self.img.set_array(self.normalize(self.data))
        return self.img

    def normalize(self, data):
        data = data.astype(float)
        return (data-data.min())/(data.max()-data.min())


if __name__ == '__main__':
    rospy.init_node('thermal_image_visualizer', anonymous=True)
    vis = ThermalImageVisualizer()
    sub = rospy.Subscriber('raw', TemperatureRaw, vis.thermal_callback)
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True)
