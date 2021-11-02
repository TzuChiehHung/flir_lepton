#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from flir_lepton_msgs.msg import TemperatureRaw

from .submodules.uvctypes import *
import numpy as np

class PureThermal2(object):


    def __init__(self, on_thermal, logger=None):

        self.on_thermal = on_thermal

        self.loginfo = lambda x: logger.info(x) if logger else print(x)
        self.logwarn = lambda x: logger.warn(x) if logger else print(x)
        self.logerr = lambda x: logger.error(x) if logger else print(x)

        self.is_active = False

        self.ctx = POINTER(uvc_context)()
        self.dev = POINTER(uvc_device)()
        self.devh = POINTER(uvc_device_handle)()
        self.ctrl = uvc_stream_ctrl()

        self.ptr_py_frame_callback = CFUNCTYPE(
            None, POINTER(uvc_frame), c_void_p)(self.py_frame_callback)

        res = libuvc.uvc_init(byref(self.ctx), 0)
        if res < 0:
            self.logerr('uvc_init error')
            exit(1)
        else:
            self.loginfo('uvc_init success')

        res = libuvc.uvc_find_device(
            self.ctx, byref(self.dev), PT_USB_VID, PT_USB_PID, 0)
        if res < 0:
            self.logerr('uvc_find_device error')
            exit(1)
        else:
            self.loginfo('uvc_find_device success')

        res = libuvc.uvc_open(self.dev, byref(self.devh))
        if res < 0:
            self.logerr('uvc_open error')
            exit(1)
        else:
            self.loginfo('PureThermal2 device opened!')

        # print_device_info(self.devh)
        # print_device_formats(self.devh)

        frame_formats = uvc_get_frame_formats_by_guid(
            self.devh, VS_FMT_GUID_Y16)

        if len(frame_formats) == 0:
            self.logerr('device does not support Y16')
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
                self.logerr('uvc_start_streaming failed: {0}'.format(res))
                exit(1)
            else:
                self.is_active = True
                self.loginfo('uvc_start_streaming success')

    def stop(self):
        if self.is_active:
            libuvc.uvc_stop_streaming(self.devh)
            self.is_active = False
            self.loginfo('uvc_stop_streaming')

        libuvc.uvc_unref_device(self.dev)
        libuvc.uvc_exit(self.ctx)


class PureThermal2Node(Node):


    def __init__(self):
        super().__init__('purethermal2_node')

        self.publisher = self.create_publisher(TemperatureRaw, 'raw', qos_profile_sensor_data)

        self.purethermal2 = PureThermal2(self.on_thermal, logger=self.get_logger())
        self.purethermal2.start()

    def on_thermal(self, data):
        msg = TemperatureRaw()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = data.shape
        msg.data = data.ravel().tolist()
        self.publisher.publish(msg)

    def on_shutdown(self):
        self.get_logger().info('shutting down process')
        try:
            self.purethermal2.stop()
        except:
            pass
        finally:
            self.purethermal2 = None

def main(args=None):
    rclpy.init(args=args)
    nh = PureThermal2Node()
    
    try:
        rclpy.spin(nh)
    except KeyboardInterrupt:
        nh.get_logger().info('KeyboardInterrupt')
    except Exception as e:
        nh.get_logger().error(e)
    finally:
        nh.on_shutdown()
        nh.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()