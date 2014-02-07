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

        self.zippub = r.Publisher('image_shrinked', StringArray)
        self.imgpub = r.Publisher('image_edge', Image)
        self.bridge = CvBridge()
        self.sub = r.Subscriber('raw_image', Image, self.callback)
        self.rate = r.get_param('~rate')
        self.scale = r.get_param('~scale')

    def run(self):
        r.Rate(self.rate)
        r.spin()

    def callback(self, img):
        try:
            ci = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        except CvBridgeError, e:
            print e

        # detect edges
        edge = cv2.Canny(ci, 100, 200)

        # resize
        h = ci.shape[0]
        w = ci.shape[1]
        edge_resized = cv2.resize(edge, (int(w * self.scale), int(h * self.scale)))

        # publish raw edged img
        self.imgpub.publish(self.bridge.cv2_to_imgmsg(edge_resized))

        # publish compressed edged img
        header = [str(edge_resized.shape[0]), str(edge_resized.shape[1])]
        body = self.generateCompressedMessage(edge_resized)
        tmp = TemporaryFile()
        np.savez_compressed(tmp, img=body)
        tmp.seek(0)
        strPubData = StringArray(data=header.extend(tmp.readlines()))
        self.zippub.publish(strPubData)
        r.sleep(1. / self.rate)

    def generateCompressedMessage(self, src):
        height = src.shape[0]
        width = src.shape[1]
        resized_src = np.resize(src, (1, src.size))[0] > 0
        imgsize = height * width
        intsize = int(imgsize / 8)  + 1
        lst = [0] * intsize
        for i in range(imgsize):
            if resized_src[i]:
                lst[i/8] |= (1 << i/8)
        ret = np.array(lst)
        return ret

def init():
    shrink_server = ImageShrinkServer()
    shrink_server.run()

if __name__ == '__main__':
    init()
