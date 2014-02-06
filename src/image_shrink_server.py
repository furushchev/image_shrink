#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('image_shrink')
import rospy as r
import cv2
import numpy as np
from sensor_msgs.msg import Image
from tempfile import TemporaryFile
from image_shrink.msg import StringArray
from cv_bridge import CvBridge, CvBridgeError

class ImageShrinkServer:
    def __init__(self):
        r.init_node('image_shrink_server')
        self.pub = r.Publisher('~image_shrinked', StringArray)
        self.bridge = CvBridge()
        self.sub = r.Subscriber('~raw_image', Image, self.callback)
        self.rate = r.get_param('~rate')

    def run(self):

        r.Rate(self.rate)
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
        r.sleep()

def init():
    shrink_server = ImageShrinkServer()
    shrink_server.run()

if __name__ == '__main__':
    init()
