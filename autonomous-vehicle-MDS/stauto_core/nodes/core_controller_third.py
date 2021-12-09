#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This is a stauto core_controller.py
# Copyright (c) 2021, jeonjonghyun

import rospy, roslaunch
import numpy as np
import subprocess
import os
import sys
import rospkg
import math
from enum import Enum
from std_msgs.msg import UInt8, Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray, Int32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point , PoseStamped

class CoreController():
    def __init__(self):
        #subscriber
        self.sub_traffic = rospy.Subscriber('/detect/traffic_sign', Int32MultiArray, self.cbTraffic, queue_size=1)
        self.sub_parking = rospy.Subscriber('/detect/parking_sign', Bool, self.cbParking, queue_size=1)
        self.sub_stop = rospy.Subscriber('stop_line',Int32, self.cbStop, queue_size=1)
        self.sub_cruise = rospy.Subscriber('/detect/cruise',Bool, self.cbcruise, queue_size=1)
        self.sub_backup = rospy.Subscriber('current_step', PoseStamped, self.cbBackup, queue_size=1)
        self.sub_parking_end = rospy.Subscriber('/parking_end',Bool, self.cbParkingEnd, queue_size=1)
        self.sub_delivery = rospy.Subscriber('/detect/delivery_sign', Bool, self.cbDelivery, queue_size=1)
        self.sub_delivery_end = rospy.Subscriber('/delivery_end',Bool, self.cbDeliveryEnd, queue_size=1)
        #self.sub_bigavoid = rospy.Subscriber('/detect/bigavoid_sign', Bool, self.cbbigavoid, queue_size=1)
        #self.sub_bigavoid_fin = rospy.Subscriber('/big_obstacle_finish',Bool, self.cbbigavoid_fin, queue_size=1)
        self.sub_big_obstacle = rospy.Subscriber('big_obstacle_sign', Bool, self.cbBig_obstacle, queue_size=1)
        self.sub_big_obstacle_fin = rospy.Subscriber('/big_obstacle_finish',Bool, self.cbBig_obstacle_fin, queue_size=1)

        #publisher
        self.pub_state = rospy.Publisher('/state_machine',Int32MultiArray, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        self.Machine_State = Enum('Machine_State', 'cruise avoid_cruise stop traffic parking delivery backup',start=0)
        self.TrafficSign = Enum('TrafficSign','red green left straightleft',start=0)
        #self.StopSign = Enum('StopSign','traffic_stop parking_stop')

        self.StateGraph = Int32MultiArray()
        self.StateGraph.layout.dim.append(MultiArrayDimension())
        self.StateGraph.layout.dim[0].label = "state_graph"
        self.StateGraph.layout.dim[0].size = 6
        self.StateGraph.layout.dim[0].stride = 6
        self.StateGraph.layout.data_offset = 0
        self.StateGraph.data=[0]*6
        for i in range(6):
            if i == 1:
                self.StateGraph.data[i] = 1
            else:
                self.StateGraph.data[i] = 0

        self.cur_state = self.Machine_State.cruise.value
        self.backup_state = self.Machine_State.cruise.value
        self.cur_traffic = self.TrafficSign.red.value
        self.stop_flag = False
        self.stop_line = 0
        self.stop_line_count=0
        self.stop_line_timer=False
        self.step_num = 0
        self.prev_step = 0
        # self.avoid_count = 0
        # self.avoid_flag = False
        # self.avoid_timer = False
        self.traffic_timer = False
        self.traffic_count = 0
        self.parking_end = False
        self.delivery_end = False
        self.big_obstacle_finish = False

        self.delivery_A_timer = False
        self.delivery_A_count = 0

        # self.obstacle_timer = False
        # self.obstacle_count = 0

        loop_rate = rospy.Rate(10)

    def cbTraffic(self,event_msg):
        self.fnDecideMode(self.Machine_State.traffic.value,event_msg)
        if event_msg.data[self.TrafficSign.red.value] == 1:
            self.cur_traffic = self.TrafficSign.red.value
        elif event_msg.data[self.TrafficSign.green.value] == 1:
            self.cur_traffic = self.TrafficSign.green.value
        elif event_msg.data[self.TrafficSign.left.value] == 1:
            self.cur_traffic = self.TrafficSign.left.value
        elif event_msg.data[self.TrafficSign.straightleft.value] == 1:
            self.cur_traffic = self.TrafficSign.straightleft.value

    def cbStop(self,event_msg):
        #self.stop_line = event_msg.data
        if (event_msg.data == 1):
            self.stop_line_timer = True
            self.stop_line = 1
        if (self.stop_line_timer == True):
            self.stop_line = 1
        if (event_msg.data == 0) and (self.stop_line_timer == False):
            self.stop_line = 0

    # def cbAvoidance(self,event_msg):
    #     if event_msg.data == True:
    #         self.avoid_flag = 1
    #         self.fnDecideMode(self.Machine_State.avoid_cruise.value,self.avoid_count)
    #         #self.avoid_time = rospy.get_rostime()
    #     if event_msg.data == False:
    #         if self.avoid_flag == 1:
    #             self.avoid_count += 1
    #             if self.avoid_count >= 3:
    #                 self.avoid_count = 0
    #                 self.avoid_flag = 0
    #                 self.fnDecideMode(self.Machine_State.cruise.value,0)
    #         elif self.avoid_flag == 0:
    #             self.fnDecideMode(self.Machine_State.cruise.value,0)
    #         #if(rospy.get_rostime() - self.avoid_time >= 1.5):
    #         #    self.fnDecideMode(self.Machine_State.cruise.value,0)

    # def cbAvoidance(self,event_msg):
    #     self.avoid_flag = event_msg.data
    #     if (event_msg.data == True):
    #         self.avoid_timer = True
    #         self.avoid_count = 0
    #     #     self.cur_state = self.Machine_State.avoid_cruise.value
    #     # if (self.avoid_timer == True):
    #     #     self.cur_state = self.Machine_State.avoid_cruise.value
    #     # elif (self.avoid_timer == False) :
    #     #     self.cur_state = self.Machine_State.cruise.value

    def cbParking(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.parking.value,0)

    def cbParkingEnd(self,event_msg):
        self.parking_end = event_msg.data

    def cbDelivery(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.delivery.value,0)

    def cbDeliveryEnd(self,event_msg):
        self.delivery_end = event_msg.data

    def cbBig_obstacle(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.avoid_cruise.value,0)

    def cbBig_obstacle_fin(self,event_msg):
        self.big_obstacle_finish = event_msg.data

    def cbcruise(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.cruise.value,0)

    def cbBackup(self,event_msg):
        self.step_num = event_msg.pose.position.z
        print('step num : ' ,self.step_num)
    
    # def my_callback(event):
        # self.backup_state = self.Machine_State.cruise.value

        # if (15 <= self.step_num and self.step_num <=34): #parking
        #     if(self.parking_end == False):
        #         self.backup_state = self.Machine_State.parking.value
        #     elif(self.parking_end == True):
        #         self.backup_state = self.Machine_State.cruise.value

        # elif(66 <= self.step_num and self.step_num <= 70): #traffic
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 110 and self.step_num == 111):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(84 <= self.step_num and self.step_num <= 87): #traffic
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 211 and self.step_num == 212):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        if(5 <= self.step_num and self.step_num <=40):                                                  #big_obstacle
            if(self.big_obstacle_finish == False):
                self.backup_state = self.Machine_State.avoid_cruise.value
            elif(self.big_obstacle_finish == True):
                self.backup_state = self.Machine_State.cruise.value

        # elif(103 <= self.step_num and self.step_num <= 105): #traffic
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 250 and self.step_num == 251):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # if (self.step_num == 10): #delivery_A
        #     # self.backup_state = self.Machine_State.stop.value
        #     # rospy.Timer(rospy.Duration(5), my_callback)

        #    print(self.delivery_A_timer)
        #    print(self.delivery_A_count)

        #    self.delivery_A_timer = True
        #    self.delivery_A_count = self.delivery_A_count + 1
        #    if(self.delivery_A_count > 500):
        #         self.delivery_A_timer = False
        #         # self.delivery_A_count=0
        #    if(self.delivery_A_timer == True):
        #        self.backup_state = self.Machine_State.stop.value
        #    elif(self.delivery_A_timer == False):
        #        self.backup_state = self.Machine_State.cruise.value

        # elif (self.step_num == 11):
        #     self.delivery_A_count=0

        # elif (15 <= self.step_num and self.step_num <= 45): #delivery_B
        #     if(self.delivery_end == False):
        #         self.backup_state = self.Machine_State.delivery.value
        #     elif(self.delivery_end == True):
        #         self.backup_state = self.Machine_State.cruise.value
                
        # elif(171 <= self.step_num and self.step_num <= 172): #traffic
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 283 and self.step_num == 284):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(185 <= self.step_num and self.step_num <= 187): #traffic
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 283 and self.step_num == 284):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(231 <= self.step_num and self.step_num <= 236): #traffic
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 406 and self.step_num == 407):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(268 <= self.step_num and self.step_num <= 272): #traffic
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 423 and self.step_num == 424):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(295 <= self.step_num and self.step_num <= 298): #traffic
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 423 and self.step_num == 424):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(306 <= self.step_num and self.step_num <= 309): #traffic
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 423 and self.step_num == 424):
        #         self.stop_line_timer = True
        #         self.stop_line = 1
    
        else:
            self.backup_state = self.Machine_State.cruise.value

        self.prev_step=self.step_num
        self.fnDecideMode(self.Machine_State.backup.value,self.backup_state)

    def timer_callback(self,data):
        if(self.traffic_timer == True):
            self.traffic_count = self.traffic_count + 1
            if (self.traffic_count > 30):
                print("traffic mode release!!!")
                #self.cur_traffic = 1
                self.traffic_count = 0
                self.traffic_timer = False
        if(self.stop_line_timer == True):
            self.stop_line_count = self.stop_line_count + 1
            if(self.stop_line_count > 1):
                self.stop_line_timer = False
                self.stop_line_count=0
        # if(self.avoid_timer == True):
        #     self.avoid_count = self.avoid_count + 1
        #     if(self.avoid_count > 10):
        #         self.avoid_timer = False
        #         self.avoid_count=0
        # if(self.delivery_A_timer == True):
        #     self.delivery_A_count = self.delivery_A_count + 1
        #     if(self.delivery_A_count > 6):
        #         self.delivery_A_timer = False
        #         self.delivery_A_count=0

    def fnDecideMode(self,mode,sub_event):
        for i in range(6):
            self.StateGraph.data[i] = 0

        if mode == self.Machine_State.cruise.value:
            self.cur_state = self.Machine_State.cruise.value

        elif mode == self.Machine_State.traffic.value:
            self.cur_state = self.Machine_State.traffic.value

        elif mode == self.Machine_State.avoid_cruise.value: #self.Machine_State.backup.value and sub_event==self.Machine_State.avoid_cruise.value:
            self.cur_state = self.Machine_State.avoid_cruise.value

        elif mode == self.Machine_State.parking.value:
            self.cur_state = self.Machine_State.parking.value

        elif mode == self.Machine_State.delivery.value:
            self.cur_state = self.Machine_State.delivery.value

        if(self.cur_state != self.backup_state):
            self.cur_state = self.backup_state

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.parking.value:
            self.cur_state = self.Machine_State.parking.value

        if (self.stop_line == 1) and (self.cur_state == self.Machine_State.parking.value):
            self.cur_state = self.Machine_State.stop.value

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.delivery.value:
            self.cur_state = self.Machine_State.delivery.value

        if (self.stop_line == 1) and (self.cur_state == self.Machine_State.delivery.value):
            self.cur_state = self.Machine_State.stop.value

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.avoid_cruise.value:
            self.cur_state = self.Machine_State.avoid_cruise.value

        if (self.stop_line == 1) and (self.cur_state == self.Machine_State.avoid_cruise.value):
            self.cur_state = self.Machine_State.stop.value

        # if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.avoid_cruise.value:
        #     print(self.avoid_timer,self.avoid_count)
        #     if self.avoid_timer == True:
        #         self.cur_state = self.Machine_State.avoid_cruise.value
        #     elif self.avoid_timer == False:
        #         self.cur_state = self.Machine_State.cruise.value

        if self.cur_state == self.Machine_State.traffic.value or sub_event == self.Machine_State.traffic.value:
            print(self.stop_flag, self.cur_traffic, self.stop_line,self.traffic_count)

            if (self.cur_traffic == 0) and (self.stop_line == 1):
                self.stop_flag=True
                self.cur_state = self.Machine_State.stop.value
                self.traffic_timer=True
                self.traffic_count = 0

            elif (243 <= self.step_num and self.step_num <= 254) and (self.cur_traffic == 1) and (self.stop_line == 1):
                self.stop_flag=True
                self.cur_state = self.Machine_State.stop.value
                self.traffic_timer=True
                self.traffic_count = 0

            elif (263 <= self.step_num and self.step_num <= 274) and (self.cur_traffic == 1) and (self.stop_line == 1):
                self.stop_flag=True
                self.cur_state = self.Machine_State.stop.value
                self.traffic_timer=True
                self.traffic_count = 0

            elif (self.traffic_timer == True) and (self.stop_flag == True):
                self.cur_state = self.Machine_State.stop.value

            if (self.cur_traffic!=0) or (self.traffic_count > 29):
                print("Traffic Release ", self.traffic_count)

                if(243 <= self.step_num and self.step_num <= 254):
                    if(self.cur_traffic ==2 or self.cur_traffic==3):
                        self.stop_flag=False
                        self.cur_state = self.Machine_State.cruise.value
                    elif(self.cur_traffic ==1) and (self.stop_line == 1):
                        self.cur_state = self.Machine_State.stop.value

                if(243 <= self.step_num and self.step_num <= 254):
                    if(self.cur_traffic ==2 or self.cur_traffic==3):
                        self.stop_flag=False
                        self.cur_state = self.Machine_State.cruise.value
                    elif(self.cur_traffic ==1) and (self.stop_line == 1):
                        self.cur_state = self.Machine_State.stop.value

                else:
                    self.stop_flag=False
                    self.cur_state = self.Machine_State.cruise.value

        self.StateGraph.data[self.cur_state] = 1
        self.fnPublishMode()

    def fnPublishMode(self):
        if self.StateGraph.data[self.Machine_State.cruise.value] == 1:
            print('Cruise_mode')
        elif self.StateGraph.data[self.Machine_State.avoid_cruise.value] == 1:
            print('Avoidance_Cruise_mode')
        elif self.StateGraph.data[self.Machine_State.stop.value] == 1:
            print('Stop_mode')
        elif self.StateGraph.data[self.Machine_State.traffic.value] == 1:
            print('Traffic_mode')
        elif self.StateGraph.data[self.Machine_State.parking.value] == 1:
            print('Parking_mode')
        elif self.StateGraph.data[self.Machine_State.delivery.value] == 1:
            print('Delivery_mode')
        self.pub_state.publish(self.StateGraph)

    def main(self):
        rospy.spin()

if __name__ == "__main__":

    rospy.init_node('Core_Controller')
    node = CoreController()
    node.main()