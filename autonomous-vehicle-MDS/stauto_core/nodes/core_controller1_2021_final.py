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
import time
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
        self.sub_avoidance = rospy.Subscriber('/adaptive_clustering/is_bool', Bool, self.cbAvoidance, queue_size=1)
        self.sub_parking = rospy.Subscriber('/detect/parking_sign', Bool, self.cbParking, queue_size=1)
        self.sub_stop = rospy.Subscriber('stop_line',Int32, self.cbStop, queue_size=1)
        self.sub_cruise = rospy.Subscriber('/detect/cruise',Bool, self.cbcruise, queue_size=1)
        self.sub_backup = rospy.Subscriber('current_step', PoseStamped, self.cbBackup, queue_size=1)
        self.sub_parking_end = rospy.Subscriber('/parking_end',Bool, self.cbParkingEnd, queue_size=1)
        self.sub_delivery_A = rospy.Subscriber('/detect/delivery_sign_A', Bool, self.cbDeliveryA, queue_size=1)
        self.sub_delivery_end_A = rospy.Subscriber('/delivery_end_A',Bool, self.cbDeliveryEndA, queue_size=1)
        self.sub_delivery_B = rospy.Subscriber('/detect/delivery_sign_B', Bool, self.cbDeliveryB, queue_size=1)
        self.sub_delivery_end_B = rospy.Subscriber('/delivery_end_B',Bool, self.cbDeliveryEndB, queue_size=1)

        #publisher
        self.pub_state = rospy.Publisher('/state_machine',Int32MultiArray, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        self.Machine_State = Enum('Machine_State', 'cruise avoid_cruise stop traffic parking deliveryA deliveryB safety backup',start=0)
        self.TrafficSign = Enum('TrafficSign','red green left straightleft',start=0)
        #self.StopSign = Enum('StopSign','traffic_stop parking_stop')

        self.StateGraph = Int32MultiArray()
        self.StateGraph.layout.dim.append(MultiArrayDimension())
        self.StateGraph.layout.dim[0].label = "state_graph"
        self.StateGraph.layout.dim[0].size = 8
        self.StateGraph.layout.dim[0].stride = 8
        self.StateGraph.layout.data_offset = 0
        self.StateGraph.data=[0]*8
        for i in range(8):
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
        self.avoid_count = 0
        self.avoid_flag = False
        self.avoid_timer = False
        self.traffic_timer = False
        self.traffic_count = 0
        self.parking_end = False
        self.delivery_end_A = False
        self.delivery_end_B = False

        # self.delivery_A_timer = False
        # self.delivery_A_count = 0
        # self.delivery_time = time.time()
        # self.init_time=rospy.Time.now()

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

    def cbAvoidance(self,event_msg):
        self.avoid_flag = event_msg.data
        if (event_msg.data == True):
            self.avoid_timer = True
            self.avoid_count = 0
        #     self.cur_state = self.Machine_State.avoid_cruise.value
        # if (self.avoid_timer == True):
        #     self.cur_state = self.Machine_State.avoid_cruise.value
        # elif (self.avoid_timer == False) :
        #     self.cur_state = self.Machine_State.cruise.value

    def cbParking(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.parking.value,0)

    def cbParkingEnd(self,event_msg):
        self.parking_end = event_msg.data

    def cbDeliveryA(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.deliveryA.value,0)

    def cbDeliveryEndA(self,event_msg):
        self.delivery_end_A = event_msg.data

    def cbDeliveryB(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.deliveryB.value,0)

    def cbDeliveryEndB(self,event_msg):
        self.delivery_end_B = event_msg.data

    def cbcruise(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.cruise.value,0)

    def cbBackup(self,event_msg):
        self.step_num = event_msg.pose.position.z
        print('step num : ' ,self.step_num)

        # if (7 <= self.step_num and self.step_num <=26): #parking   mission(1)
        #     if(self.parking_end == False):
        #         self.backup_state = self.Machine_State.parking.value
        #     elif(self.parking_end == True):
        #         self.backup_state = self.Machine_State.cruise.value

        # elif(34 <= self.step_num and self.step_num <= 39): #traffic mission(3)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 35 and self.step_num == 36):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(50 <= self.step_num and self.step_num <= 56): #traffic mission(4)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 53 and self.step_num == 54):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        if(15<= self.step_num and self.step_num <=19):   #test test test test
            self.backup_state = self.Machine_State.safety.value

        elif(20<= self.step_num and self.step_num <= 49): #big_obstacle  mission(5) (75, 94) 20 49
           self.backup_state = self.Machine_State.avoid_cruise.value

        elif(95 <= self.step_num and self.step_num <= 98): #traffic   mission(6)
            self.backup_state = self.Machine_State.traffic.value
            if (self.prev_step == 96 and self.step_num == 97):
                self.stop_line_timer = True
                self.stop_line = 1

        elif (50 <= self.step_num and self.step_num <= 58): #delivery_A   mission(7) (101~114)  50 58
            if(self.delivery_end_A == False):
                self.backup_state = self.Machine_State.deliveryA.value
            elif(self.delivery_end_A == True):
                self.backup_state = self.Machine_State.cruise.value

        #     self.delivery_time=time.time()

        #     print(self.delivery_A_timer)
        #     print(self.delivery_A_count)

        #     self.delivery_A_timer = True
        #     self.delivery_A_count = self.delivery_A_count + 1
        #     if(self.delivery_A_count > 200):
        #             self.delivery_A_timer = False
        #             # self.delivery_A_count=0
        #     if(self.delivery_A_timer == True):
        #         self.backup_state = self.Machine_State.stop.value
        #     elif(self.delivery_A_timer == False):
        #         self.backup_state = self.Machine_State.cruise.value

        #     if(self.time.time()-self.delivery_time > 4):
        #             self.delivery_A_timer = False
        #             # self.delivery_A_count=0
        #     if(self.delivery_A_timer == True):
        #         self.backup_state = self.Machine_State.stop.value
        #     elif(self.delivery_A_timer == False):
        #         self.backup_state = self.Machine_State.cruise.value

        # elif (self.step_num == 11):
        #     self.delivery_A_count=0

        elif (59 <= self.step_num and self.step_num <= 67): #delivery_B  mission(9) (201, 208)   59 67
            if(self.delivery_end_B == False):
                self.backup_state = self.Machine_State.deliveryB.value
            elif(self.delivery_end_B == True):
                self.backup_state = self.Machine_State.cruise.value
                
        # elif(209 <= self.step_num and self.step_num <= 212): #traffic   mission(10)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 210 and self.step_num == 211):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(244 <= self.step_num and self.step_num <= 249): #traffic   mission(11)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 247 and self.step_num == 248):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(297 <= self.step_num and self.step_num <= 304): #trafficleft   mission(12)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 300 and self.step_num == 301):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(331 <= self.step_num and self.step_num <= 336): #trafficleft   mission(15)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 333 and self.step_num == 334):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(365 <= self.step_num and self.step_num <= 370): #traffic   mission(16)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 367 and self.step_num == 368):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(405 <= self.step_num and self.step_num <= 411): #traffic   mission(17)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 408 and self.step_num == 409):
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
        if(self.avoid_timer == True):
            self.avoid_count = self.avoid_count + 1
            if(self.avoid_count > 10):
                self.avoid_timer = False
                self.avoid_count=0
        # if(self.delivery_A_timer == True):
        #     self.delivery_A_count = self.delivery_A_count + 1
        #     if(self.delivery_A_count > 6):
        #         self.delivery_A_timer = False
        #         self.delivery_A_count=0

    def fnDecideMode(self,mode,sub_event):
        for i in range(8):
            self.StateGraph.data[i] = 0

        if mode == self.Machine_State.cruise.value:
            self.cur_state = self.Machine_State.cruise.value

        elif mode == self.Machine_State.traffic.value:
            self.cur_state = self.Machine_State.traffic.value

        elif mode == self.Machine_State.avoid_cruise.value: #self.Machine_State.backup.value and sub_event==self.Machine_State.avoid_cruise.value:
            self.cur_state = self.Machine_State.avoid_cruise.value

        elif mode == self.Machine_State.parking.value:
            self.cur_state = self.Machine_State.parking.value

        elif mode == self.Machine_State.deliveryA.value:
            self.cur_state = self.Machine_State.deliveryA.value

        elif mode == self.Machine_State.deliveryB.value:
            self.cur_state = self.Machine_State.deliveryB.value

        elif mode == self.Machine_State.safety.value:
            self.cur_state = self.Machine_State.safety.value

        if(self.cur_state != self.backup_state):
            self.cur_state = self.backup_state

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.parking.value:
            self.cur_state = self.Machine_State.parking.value

        if (self.stop_line == 1) and (self.cur_state == self.Machine_State.parking.value):
            self.cur_state = self.Machine_State.stop.value

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.deliveryA.value:
            self.cur_state = self.Machine_State.deliveryA.value

        # if (self.stop_line == 1) and (self.cur_state == self.Machine_State.delivery.value):
        #     self.cur_state = self.Machine_State.stop.value
        
        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.deliveryB.value:
            self.cur_state = self.Machine_State.deliveryB.value

        # if (self.stop_line == 1) and (self.cur_state == self.Machine_State.delivery.value):
        #     self.cur_state = self.Machine_State.stop.value

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.avoid_cruise.value:
            print(self.avoid_timer,self.avoid_count)
            if self.avoid_timer == True:
                self.cur_state = self.Machine_State.avoid_cruise.value
            elif self.avoid_timer == False:
                self.cur_state = self.Machine_State.cruise.value

        if self.cur_state == self.Machine_State.traffic.value or sub_event == self.Machine_State.traffic.value:
            print(self.stop_flag, self.cur_traffic, self.stop_line,self.traffic_count)

            if (self.cur_traffic == 0) and (self.stop_line == 1):
                self.stop_flag=True
                self.cur_state = self.Machine_State.stop.value
                self.traffic_timer=True
                self.traffic_count = 0

            elif(208 <= self.step_num and self.step_num <= 212):
                    if(self.cur_traffic ==1 or self.cur_traffic==3):
                        self.stop_flag=False
                        self.cur_state = self.Machine_State.cruise.value
                    elif(self.cur_traffic ==2) and (self.stop_line == 1):
                        self.cur_state = self.Machine_State.stop.value

            # elif (246 <= self.step_num and self.step_num <= 248) and (self.cur_traffic == 1) and (self.stop_line == 1):
            #     self.stop_flag=True
            #     self.cur_state = self.Machine_State.stop.value
            #     self.traffic_timer=True
            #     self.traffic_count = 0

            elif (296 <= self.step_num and self.step_num <= 304) and (self.cur_traffic == 1) and (self.stop_line == 1):
                self.stop_flag=True
                self.cur_state = self.Machine_State.stop.value
                self.traffic_timer=True
                self.traffic_count = 0

            elif (self.traffic_timer == True) and (self.stop_flag == True):
                self.cur_state = self.Machine_State.stop.value

            if (self.cur_traffic!=0) or (self.traffic_count > 29):
                print("Traffic Release ", self.traffic_count)

                if(208 <= self.step_num and self.step_num <= 212):
                    if(self.cur_traffic ==1 or self.cur_traffic==3):
                        self.stop_flag=False
                        self.cur_state = self.Machine_State.cruise.value
                    elif(self.cur_traffic ==2) and (self.stop_line == 1):
                        self.cur_state = self.Machine_State.stop.value

                # if(246 <= self.step_num and self.step_num <= 248):
                #     if(self.cur_traffic ==2 or self.cur_traffic==3):
                #         self.stop_flag=False
                #         self.cur_state = self.Machine_State.cruise.value
                #     elif(self.cur_traffic ==1) and (self.stop_line == 1):
                #         self.cur_state = self.Machine_State.stop.value

                elif(296 <= self.step_num and self.step_num <= 304):
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
        elif self.StateGraph.data[self.Machine_State.deliveryA.value] == 1:
            print('Delivery_A_mode')
        elif self.StateGraph.data[self.Machine_State.deliveryB.value] == 1:
            print('Delivery_B_mode')
        elif self.StateGraph.data[self.Machine_State.safety.value] == 1:
            print('Safety')
        self.pub_state.publish(self.StateGraph)

    def main(self):
        rospy.spin()

if __name__ == "__main__":

    rospy.init_node('Core_Controller')
    node = CoreController()
    node.main()