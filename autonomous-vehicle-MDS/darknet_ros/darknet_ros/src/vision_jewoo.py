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
            delivery_time=time.time()
            list_ABnum.append(mission)

        else:
            sign_time=time.time()
            list_sign.append(mission)

        if len(list_ABnum) == total:
            A1 = list_ABnum.count('A1')
            A2 = list_ABnum.count('A2')
            A3 = list_ABnum.count('A3')
            B1 = list_ABnum.count('B1')
            B2 = list_ABnum.count('B2')
            B3 = list_ABnum.count('B3')
            list_ABnum = []
            #print(R,G,L,SL)
            if A1 >= thresh:
                #send_msg = 'Red'
                print('A1???')
                true.append('A1')
                #print(true)

            elif A2 >=thresh:
                #send_msg = 'Green'
                print('A2???')
                true.append('A2')

            elif A3 >=thresh:
                #send_msg = 'Left'
                print('A3???')
                true.append('A3')

            elif B1 >=thresh:
                #send_msg = 'StraightLeft'
                print('B1???')
                true.append('B1')
            elif B2 >=thresh:
                #send_msg = 'StraightLeft'
                print('B2???')
                true.append('B2')
            elif B3 >=thresh:
                #send_msg = 'StraightLeft'
                print('B3???')
                true.append('B3')
        #print(true, list_ABnum)
        if len(true)>=2:
            if true[-2]=='A1' and true[-1]=='A1':
                delivery_array.data = [1,0,0,0,0,0]
                print('clearly A1')

            elif true[-2]=='A2' and true[-1]=='A2':
                delivery_array.data = [0,1,0,0,0,0]
                print('clearly A2')

            elif true[-2]=='A3' and true[-1]=='A3':
                delivery_array.data = [0,0,1,0,0,0]
                print('clearly A3')
            elif true[-2]=='B1' and true[-1]=='B1':
                delivery_array.data = [0,0,0,1,0,0]
                print('clearly B1')
            elif true[-2]=='B2' and true[-1]=='B2':
                delivery_array.data = [0,0,0,0,1,0]
                print('clearly B2')
            elif true[-2]=='B3' and true[-1]=='B3':
                delivery_array.data = [0,0,0,0,0,1]
                print('clearly B3')
            #print(send_msg)
            true=[]

            pub_deliverysign.publish(delivery_array)
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
    rate = rospy.Rate(10)

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
