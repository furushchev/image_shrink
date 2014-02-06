#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('image_shrink_server')
import rospy as r
import cv2
import numpy as np
from sensor_msgs.msg import Image
from tempfile import TemporaryFile
from image_shrink_server.msg import StringArray
from cv_bridge import CvBridge, CvBridgeError

class ImageShrinkServer:
    def __init__(self):

        self.pub = r.Publisher('image_edge_shrinked', StringArray)
        self.bridge = CvBridge()
        self.sub = r.Subscriber('/openni/rgb/image_rect', Image, self.callback)

    def run(self):
        r.init_node('image_shrink_server')
        r.Rate(10.0)
        r.spin()

    def callback(self, img):
        try:
            ci = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        except CvBridgeError, e:
            print e

        edge = cv2.Canny(ci, 100, 200)
        tmp = TemporaryFile()
        np.savez_compressed(tmp, img=edge)
        tmp.seek(0)
        pubData = StringArray()
        pubData.data = tmp.readlines()
        self.pub.publish(pubData)

def init():
    shrink_server = ImageShrinkServer()
    shrink_server.run()

if __name__ == '__main__':
    init()

