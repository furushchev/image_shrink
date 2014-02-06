#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('image_shrink')
import rospy as r
import cv2
import numpy as np
from tempfile import TemporaryFile
from sensor_msgs.msg import Image
from image_shrink.msg import StringArray
from cv_bridge import CvBridge, CvBridgeError

class ImageShrinkClient:
    def __init__(self):
        r.init_node('image_shrink_client')
        self.pub = r.Publisher('out_image', Image)
        self.sub = r.Subscriber('image_shrinked', StringArray, self.callback)
        self.bridge = CvBridge()

    def run(self):
        r.Rate(r.get_param('~rate'))
        r.spin()

    def callback(self, sarray):
        tmp = TemporaryFile()
        tmp.writelines(sarray.data)
        tmp.seek(0)
        data = np.load(tmp)
        img = data['img']
        img.shape = (img.shape[0], img.shape[1], 1)
        imgmsg = self.bridge.cv2_to_imgmsg(img)
        imgmsg.encoding = 'mono8'
        self.pub.publish(imgmsg)

def init():
    shrink_client = ImageShrinkClient()
    shrink_client.run()

if __name__ == '__main__':
    init()
