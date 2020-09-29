#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flir_lepton_purethermal2.uvctypes import *
from flir_lepton_msgs.msg import TemperatureRaw
import numpy as np

class PureThermal2(object):


    def __init__(self, on_thermal):

        self.on_thermal = on_thermal
        self.is_active = False

        self.ctx = POINTER(uvc_context)()
        self.dev = POINTER(uvc_device)()
        self.devh = POINTER(uvc_device_handle)()
        self.ctrl = uvc_stream_ctrl()

        self.ptr_py_frame_callback = CFUNCTYPE(
            None, POINTER(uvc_frame), c_void_p)(self.py_frame_callback)

        res = libuvc.uvc_init(byref(self.ctx), 0)
        if res < 0:
            rospy.logerr('uvc_init error')
            exit(1)
        else:
            rospy.loginfo('uvc_init success')

        res = libuvc.uvc_find_device(
            self.ctx, byref(self.dev), PT_USB_VID, PT_USB_PID, 0)
        if res < 0:
            rospy.logerr('uvc_find_device error')
            exit(1)
        else:
            rospy.loginfo('uvc_find_device success')

        res = libuvc.uvc_open(self.dev, byref(self.devh))
        if res < 0:
            rospy.logerr('uvc_open error')
            exit(1)
        else:
            rospy.loginfo('PureThermal2 device opened!')

        # print_device_info(self.devh)
        # print_device_formats(self.devh)

        frame_formats = uvc_get_frame_formats_by_guid(
            self.devh, VS_FMT_GUID_Y16)

        if len(frame_formats) == 0:
            rospy.logerr('device does not support Y16')
            exit(1)

        libuvc.uvc_get_stream_ctrl_format_size(
            self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_Y16,
            frame_formats[0].wWidth, frame_formats[0].wHeight,
            int(1e7 / frame_formats[0].dwDefaultFrameInterval)
        )

    def py_frame_callback(self, frame, userptr, copy=False):

        if copy:
            # copy
            data = np.fromiter(
            frame.contents.data, dtype=np.dtype(np.uint8), count=frame.contents.data_bytes
            ).reshape(
            frame.contents.height, frame.contents.width, 2
            ) 
        else:
            # no copy
            array_pointer = cast(frame.contents.data, POINTER(
                c_uint16 * (frame.contents.width * frame.contents.height)))
            data = np.frombuffer(
                array_pointer.contents, dtype=np.dtype(np.uint16)
            ).reshape(
                frame.contents.height, frame.contents.width
            )

        if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
            return

        self.on_thermal(data)

    def start(self):
        if not self.is_active:
            res = libuvc.uvc_start_streaming(
                self.devh, byref(self.ctrl), self.ptr_py_frame_callback, None, 0)
            if res < 0:
                rospy.logerr('uvc_start_streaming failed: {0}'.format(res))
                exit(1)
            else:
                rospy.loginfo('uvc_start_streaming success')

    def stop(self):
        if self.is_active:
            libuvc.uvc_stop_streaming(self.devh)
            rospy.loginfo('uvc_stop_streaming')

        libuvc.uvc_unref_device(self.dev)
        libuvc.uvc_exit(self.ctx)


class PureThermal2Node(object):


    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)
        self.purethermal2 = PureThermal2(self.on_thermal)

        # advertise
        self.pub_thermal = rospy.Publisher('raw', TemperatureRaw, queue_size=2)

        self.purethermal2.start()

    def on_thermal(self, data):
        temp = TemperatureRaw()
        temp.header.stamp = rospy.Time.now()
        temp.height, temp.width = data.shape
        temp.data = data.ravel()
        self.pub_thermal.publish(temp)

    def on_shutdown(self):
        try:
            self.purethermal2.stop()
        except:
            pass
        finally:
            self.purethermal2 = None


if __name__ == '__main__':
    rospy.init_node('purethermal2_node')
    n = PureThermal2Node()
    rospy.spin()
