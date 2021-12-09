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
        self.sub_delivery = rospy.Subscriber('/detect/delivery_sign', Bool, self.cbDelivery, queue_size=1)
        self.sub_delivery_end = rospy.Subscriber('/delivery_end',Bool, self.cbDeliveryEnd, queue_size=1)

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
        self.avoid_count = 0
        self.avoid_flag = False
        self.avoid_timer = False
        self.traffic_timer = False
        self.traffic_count = 0
        self.parking_end = False
        self.delivery_end = False

        self.delivery_A_timer = False
        self.delivery_A_count = 0
        # self.delivery_time = time.time()
        # self.init_time=rospy.Time.now()

        # self.obstacle_timer = False
        self.obstacle_count = 0

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

    def cbDelivery(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.delivery.value,0)

    def cbDeliveryEnd(self,event_msg):
        self.delivery_end = event_msg.data

    def cbcruise(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.cruise.value,0)

    def cbBackup(self,event_msg):
        self.step_num = event_msg.pose.position.z
        print('step num : ' ,self.step_num)

        # if (14 <= self.step_num and self.step_num <=50): #parking   mission(1)
        #     if(self.parking_end == False):
        #         self.backup_state = self.Machine_State.parking.value
        #     elif(self.parking_end == True):
        #         self.backup_state = self.Machine_State.cruise.value

        # elif(79 <= self.step_num and self.step_num <= 84): #traffic mission(3)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 80 and self.step_num == 81):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(112 <= self.step_num and self.step_num <= 117): #traffic mission(4)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 114 and self.step_num == 115):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(163 <= self.step_num and self.step_num <= 177): #big_obstacle  mission(5)
        #    self.backup_state = self.Machine_State.avoid_cruise.value

        # elif(196 <= self.step_num and self.step_num <= 200): #traffic   mission(6)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 198 and self.step_num == 199):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        if (215<= self.step_num and self.step_num <= 235): #delivery_A   mission(7) (215~235)

            # self.delivery_time=time.time()

            print(self.delivery_A_timer)
            print(self.delivery_A_count)

            self.delivery_A_timer = True
            self.delivery_A_count = self.delivery_A_count + 1
            if(self.delivery_A_count > 200):
                    self.delivery_A_timer = False
                    # self.delivery_A_count=0
            if(self.delivery_A_timer == True):
                self.backup_state = self.Machine_State.stop.value
            elif(self.delivery_A_timer == False):
                self.backup_state = self.Machine_State.cruise.value

            # if(self.time.time()-self.delivery_time > 4):
            #         self.delivery_A_timer = False
            #         # self.delivery_A_count=0
            # if(self.delivery_A_timer == True):
            #     self.backup_state = self.Machine_State.stop.value
            # elif(self.delivery_A_timer == False):
            #     self.backup_state = self.Machine_State.cruise.value

        # elif (self.step_num == 11):
        #     self.delivery_A_count=0

        elif (395 <= self.step_num and self.step_num <= 415): #delivery_B  mission(9) (395~415)
            if(self.delivery_end == False):
                self.backup_state = self.Machine_State.delivery.value
            elif(self.delivery_end == True):
                self.backup_state = self.Machine_State.cruise.value
                
        # elif(416 <= self.step_num and self.step_num <= 420): #traffic   mission(10)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 417 and self.step_num == 418):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(483 <= self.step_num and self.step_num <= 487): #traffic   mission(11)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 485 and self.step_num == 486):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(589 <= self.step_num and self.step_num <= 594): #trafficleft   mission(12)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 592 and self.step_num == 593):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(656 <= self.step_num and self.step_num <= 660): #trafficleft   mission(15)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 657 and self.step_num == 658):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(725 <= self.step_num and self.step_num <= 729): #traffic   mission(16)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 727 and self.step_num == 728):
        #         self.stop_line_timer = True
        #         self.stop_line = 1

        # elif(808 <= self.step_num and self.step_num <= 813): #traffic   mission(17)
        #     self.backup_state = self.Machine_State.traffic.value
        #     if (self.prev_step == 810 and self.step_num == 811):
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

            elif (483 <= self.step_num and self.step_num <= 488) and (self.cur_traffic == 1) and (self.stop_line == 1):
                self.stop_flag=True
                self.cur_state = self.Machine_State.stop.value
                self.traffic_timer=True
                self.traffic_count = 0

            elif (589 <= self.step_num and self.step_num <= 595) and (self.cur_traffic == 1) and (self.stop_line == 1):
                self.stop_flag=True
                self.cur_state = self.Machine_State.stop.value
                self.traffic_timer=True
                self.traffic_count = 0

            elif (self.traffic_timer == True) and (self.stop_flag == True):
                self.cur_state = self.Machine_State.stop.value

            if (self.cur_traffic!=0) or (self.traffic_count > 29):
                print("Traffic Release ", self.traffic_count)

                if(483 <= self.step_num and self.step_num <= 488):
                    if(self.cur_traffic ==2 or self.cur_traffic==3):
                        self.stop_flag=False
                        self.cur_state = self.Machine_State.cruise.value
                    elif(self.cur_traffic ==1) and (self.stop_line == 1):
                        self.cur_state = self.Machine_State.stop.value

                if(589 <= self.step_num and self.step_num <= 595):
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