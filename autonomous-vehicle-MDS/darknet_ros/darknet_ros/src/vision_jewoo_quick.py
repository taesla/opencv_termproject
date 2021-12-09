#!/usr/bin/env python
# This code is written at BigVision LLC. It is based on the OpenCV project. It is subject to the license terms in the LICENSE file found in this distribution and at http://opencv.org/license.html

# Usage example:  python3 object_detection_yolo.py --video=run.mp4
#                 python3 object_detection_yolo.py --image=bird.jpg
import cv_bridge
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2, time
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import os.path
import time
import rospy
import rospkg
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int32MultiArray


def pub_delivery_sign(mission):
    global list_ABnum, list_sign, true, pub_sign, sign_time,delivery_time #list_ABnum -> list_ABnum delivery_time->delivery_time
    total = 2
    thresh = 2
    if(pub_sign==True):
        if ((mission == 'A1') or (mission == 'A2') or (mission == 'A3') or (mission == 'B1')or (mission == 'B2')or (mission == 'B3')):
            if mission == 'A1':
                delivery_array.data = [1,0,0,0,0,0]
                print('clearly A1')

            elif mission == 'A2':
                delivery_array.data = [0,1,0,0,0,0]
                print('clearly A2')

            elif mission == 'A3':
                delivery_array.data = [0,0,1,0,0,0]
                print('clearly A3')
            elif mission == 'B1':
                delivery_array.data = [0,0,0,1,0,0]
                print('clearly B1')
            elif mission == 'B2':
                delivery_array.data = [0,0,0,0,1,0]
                print('clearly B2')
            elif mission == 'B3':
                delivery_array.data = [0,0,0,0,0,1]
                print('clearly B3')
            #print(send_msg)
            true=[]

            pub_deliverysign.publish(delivery_array)
            print("publish deliver sign")
    else:
        if (time.time()-sign_time>=3):
            #true=[]
            list_sign=[]
        if (time.time()-delivery_time>=3):
            true=[]
            list_ABnum=[]

        #print(true, list_ABnum, list_sign)
def BoundingBoxes_callback(data):
    global label, pub_sign

    label=data.bounding_boxes[0].Class
    pub_sign=True

if __name__ == '__main__':
    list_ABnum = []
    list_sign = []
    true = []
    sign_time=0
    delivery_time=0
    pub_sign=True
    label=""

    rospy.init_node('Delivery_sign', anonymous=True)

    rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, BoundingBoxes_callback)
    pub_deliverysign = rospy.Publisher('/detect/delivery_sign', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(1)

    delivery_array=Int32MultiArray()
    delivery_array.data=[0,0,0,0,0,0]
    while (True):
        try:
            pub_delivery_sign(label)
            pub_sign=False
            if cv2.waitKey(1) == ord('q'):
                break

        except rospy.ROSInterruptException:
            pass
