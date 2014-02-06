#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('image_shrink_server')
import rospy as r
import cv2
import numpy as np
from tempfile import TemporaryFile
from sensor_msgs.msg import Image
from image_shrink_server.msg import StringArray
from cv_bridge import CvBridge, CvBridgeError

class ImageShrinkClient:
    def __init__(self):
        self.pub = r.Publisher('image_edge', Image)
        self.sub = r.Subscriber('image_edge_shrinked', StringArray, self.callback)

    def run(self):
        r.init_node('image_shrink_client')
        r.rate(10.0)
        r.spin()

    def callback(self, sarray):
        tmp = TemporaryFile()
        tmp.writelines(sarray.data)
        tmp.seek(0)
        data = np.load(tmp)
        img = data['img']
        imgmsg = cv2_to_imgmsg(img)
        self.pub.publish(imgmsg)

def init():
    shrink_client = ImageShrinkClient()
    shrink_client.run()

if __name__ == '__main__':
    try:
        init()
    except r.ROSInterruptExeption:
        pass
