#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('image_shrink')
import rospy as r
import cv2
import numpy as np
from scipy import sparse
from sensor_msgs.msg import Image
from tempfile import TemporaryFile
from image_shrink.msg import StringArray
from cv_bridge import CvBridge, CvBridgeError

class ImageShrinkServer:
    def __init__(self):
        r.init_node('image_shrink_server')
        self.pub = r.Publisher('image_shrinked', StringArray)
        self.bridge = CvBridge()
        self.sub = r.Subscriber('raw_image', Image, self.callback)
        self.rate = r.get_param('~rate')
        self.scale = r.get_param('~scale')
        self.sparse = r.get_param('~sparse')

    def run(self):

        r.Rate(self.rate)
        r.spin()

    def callback(self, img):
        try:
            ci = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        except CvBridgeError, e:
            print e

        edge = cv2.Canny(ci, 100, 200)
        h = ci.shape[0]
        w = ci.shape[1]
        edge_resized = cv2.resize(edge, (int(h * self.scale), int(w * self.scale)))
        if self.sparse:
            img_mat = sparse.lil_matrix(edge_resized)
        else:
            img_mat = edge_resized

        tmp = TemporaryFile()
        np.savez_compressed(tmp, img=img_mat)
        tmp.seek(0)

        if self.sparse:
            sendData = ["s"].extend(tmp.readlines())
        else:
            sendData = ["n"].extend(tmp.readlines())

        pubData = StringArray(data=sendData)
        self.pub.publish(pubData)
        r.sleep(1. / self.rate)

def init():
    shrink_server = ImageShrinkServer()
    shrink_server.run()

if __name__ == '__main__':
    init()
