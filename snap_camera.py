#!/usr/bin/env python3
"""Capture une image de la caméra et la sauvegarde dans ~/captures/"""
import rclpy, os, time
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node('snap')
msg = None
def cb(m): global msg; msg = m
node.create_subscription(Image, '/camera/color/image_raw', cb, 1)
while msg is None: rclpy.spin_once(node, timeout_sec=0.1)

os.makedirs(os.path.expanduser('~/captures'), exist_ok=True)
filename = os.path.expanduser(f'~/captures/snap_{time.strftime("%Y%m%d_%H%M%S")}.ppm')
with open(filename, 'wb') as f:
    f.write(f'P6\n{msg.width} {msg.height}\n255\n'.encode())
    f.write(bytes(msg.data))
node.destroy_node()
print(f'Image saved: {filename}')
