#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This is a stauto core_controller.py
# Copyright (c) 2020, choiyungsik

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
        self.sub_avoidance = rospy.Subscriber('/adaptive_clustering/is_bool', Bool, self.cbAvoidance, queue_size=1)
        self.sub_parking = rospy.Subscriber('/detect/parking_sign', Bool, self.cbParking, queue_size=1)
        #self.sub_safetyzone = rospy.Subscriber('/detect/safety_sign', Bool, self.cbSafetyZone, queue_size=1)
        #self.sub_crosswalk = rospy.Subscriber('/detect/crosswalk_sign', Bool, self.cdCrosswalk, queue_size=1)
        self.sub_stop = rospy.Subscriber('stop_line',Int32, self.cbStop,queue_size=1)                       #dynamic obstacle mission
        self.sub_cruise = rospy.Subscriber('/detect/cruise',Bool, self.cbcruise, queue_size=1)
        self.sub_backup = rospy.Subscriber('current_step', PoseStamped, self.cbBackup, queue_size=1)
        self.sub_parking_end = rospy.Subscriber('/parking_end',Bool,self.cbParkingEnd, queue_size=1)
        #self.sub_delivery = rospy.Subscriber('/detect/delivery_sign', Bool, self.cbDelivery, queue_size=1)
        self.sub_delivery_end = rospy.Subscriber('/delivery_end',Bool, self.cbDeliveryEnd, queue_size=1)
        self.sub_delivery_sign = rospy.Subscriber('/detect/delivery_sign', Int32MultiArray,self.cbDelivery_sign, queue_size=1) #/////////////////////////////////////////////////////////////////////////////////////#


        #publisher
        self.pub_state = rospy.Publisher('/state_machine',Int32MultiArray, queue_size=1)
        self.delivery_finish = rospy.Publisher('/delivery_finish',Bool,queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        self.timer2 = rospy.Timer(rospy.Duration(0.5), self.timer2_callback)  #/////////////////////////////////////////////////////////////////////////////////////#
        self.Machine_State = Enum('Machine_State', 'cruise avoid_cruise stop traffic parking delivery backup',start=0)
        self.TrafficSign = Enum('TrafficSign','red green left straightleft',start=0)
        self.DeliverySign = Enum('DeliverySign','A1 A2 A3 B1 B2 B3',start=0) #/////////////////////////////////////////////////////////////////////////////////////#
        #self.StopSign = Enum('StopSign','obstacle_stop traffic_stop parking_stop crosswalk_stop')

        self.StateGraph = Int32MultiArray()
        self.StateGraph.layout.dim.append(MultiArrayDimension())
        self.StateGraph.layout.dim[0].label = "state_graph"
        self.StateGraph.layout.dim[0].size = 7
        self.StateGraph.layout.dim[0].stride = 7
        self.StateGraph.layout.data_offset = 0
        self.StateGraph.data=[0]*6
        for i in range(6):
            if i == 1:
                self.StateGraph.data[i] = 1
            else:
                self.StateGraph.data[i] = 0

        self.cur_state = self.Machine_State.cruise.value     #######################################################################################
        self.backup_state = self.Machine_State.cruise.value  #######################################################################################
        self.cur_traffic = self.TrafficSign.red.value        #######################################################################################
        self.cur_deliverysign = -1  #self.DeliverySign.A1.value   #/////////////////////////////////////////////////////////////////////////////////////#
        self.delivery_sign_flag = False                      #/////////////////////////////////////////////////////////////////////////////////////#
        self.delivery_destination = self.DeliverySign.B1.value
        self.stop_flag = False
        self.stop_line = 0
        self.stop_line_count=0
        self.stop_line_timer=False
        self.step_num=0
        self.prev_step=0
        self.avoid_count = 0
        self.avoid_flag = False
        self.avoid_timer = False
        self.traffic_timer = False
        self.traffic_count = 0
        self.parking_end = False
        
        self.dynamic_obstacle = False
        self.obstacle_timer = False
        self.obstacle_count=0

        self.delivery_end = False

        self.delivery_A_timer = False
        self.delivery_A_count = 0

        self.delivery_B_timer = False
        self.delivery_B_count = 0

        self.delivery_timer = False    #/////////////////////////////////////////////////////////////////////////////////////#
        self.delivery_count =0          #/////////////////////////////////////////////////////////////////////////////////////#
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

    def cbcruise(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.cruise.value,0)

    #def cbDelivery(self,event_msg):
    #    if event_msg.data == True:
    #        self.fnDecideMode(self.Machine_State.delivery.value,0) 

    def cbDeliveryEnd(self,event_msg):
        self.delivery_end = event_msg.data

    def cbDelivery_sign(self,event_msg):
        self.delivery_timer == True
        self.fnDecideMode(self.Machine_State.delivery.value,event_msg)
        
        if event_msg.data[self.DeliverySign.A1.value] == 1:
            self.cur_deliverysign = self.DeliverySign.A1.value
            print('A1')
            self.delivery_finish.publish(True)
        elif event_msg.data[self.DeliverySign.A2.value] == 1:
            self.cur_deliverysign = self.DeliverySign.A2.value
            print('A2')
            self.delivery_finish.publish(True)
        elif event_msg.data[self.DeliverySign.A3.value] == 1:
            self.cur_deliverysign = self.DeliverySign.A3.value
            print('A3')
            self.delivery_finish.publish(True)
        elif event_msg.data[self.DeliverySign.B1.value] == 1:
            self.cur_deliverysign = self.DeliverySign.B1.value
            print('B1')
        elif event_msg.data[self.DeliverySign.B2.value] == 1:
            self.cur_deliverysign = self.DeliverySign.B2.value
            print('B2')
            #self.delivery_finish.publish(True)
        elif event_msg.data[self.DeliverySign.B3.value] == 1:
            self.cur_deliverysign = self.DeliverySign.B3.value
            print('B3')

            
            


    def cbBackup(self,event_msg):
        self.step_num = event_msg.pose.position.z
        print('step num : ' ,self.step_num)      
        ###################################### delivery_A ##############################################
        if (2<= self.step_num <= 4): 
            print("delivery zone A")
            self.backup_state = self.Machine_State.delivery.value
        #    self.delivery_finish.publish(False)
        elif(5<= self.step_num <= 51): 
            self.backup_state = self.Machine_State.cruise.value
            print('A finished back up cruise')
        #    self.delivery_finish.publish(True)
        ###################################### delivery_B ##############################################
        elif (52 <= self.step_num and self.step_num <= 54): 
           print("delivery zone B")
           self.backup_state = self.Machine_State.delivery.value
        #   self.delivery_finish.publish(False)
        elif(55<= self.step_num <= 65): 
            self.backup_state = self.Machine_State.cruise.value
            print('B finished back up cruise')
        #    self.delivery_finish.publish(True)
        elif (66 <= self.step_num and self.step_num <= 72): 
           print("delivery zone B")
           self.backup_state = self.Machine_State.delivery.value
        #   self.delivery_finish.publish(False)
        elif(73<= self.step_num <= 83): 
            self.backup_state = self.Machine_State.cruise.value
            print('B finished back up cruise')
        #    self.delivery_finish.publish(True)
        elif (84 <= self.step_num and self.step_num <= 88): 
           print("delivery zone B")
           self.backup_state = self.Machine_State.delivery.value
        #   self.delivery_finish.publish(False)
        elif(89<= self.step_num <= 183): 
            self.backup_state = self.Machine_State.cruise.value
            print('B finished back up cruise')
         #   self.delivery_finish.publish(True)
        else:
            self.backup_state = self.Machine_State.cruise.value
            print("back up else cruise")
            #print("back up else")

        self.prev_step=self.step_num
        self.fnDecideMode(self.Machine_State.backup.value,self.backup_state)

    def timer_callback(self,data):
        if(self.traffic_timer == True):
            self.traffic_count = self.traffic_count + 1
            if (self.traffic_count > 2000):
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
            if(self.avoid_count > 20):
                self.avoid_timer = False
                self.avoid_count=0
        if(self.dynamic_obstacle == True) and (self.avoid_flag == True):
            self.obstacle_count = self.obstacle_count + 1

    def timer2_callback(self,data):
        if(self.delivery_timer == True):
            self.delivery_count = self.delivery_count + 1
            print("delivery_count : ",delivery_count)
            self.fnDecideMode(self.Machine_State.delivery.value,event_msg)
            if (self.delivery_count > 500):
                print("delivery mode release!!!")
                self.delivery_count = 0
                self.delivery_timer = False
                

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
        ############################################################################## 

        #if(self.cur_state != self.backup_state):
        #    self.cur_state = self.backup_state
        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.cruise.value:
            self.cur_state = self.Machine_State.cruise.value
            print("mode == self.Machine_State.backup.value  cruise")

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.parking.value:
            self.cur_state = self.Machine_State.parking.value

        if (self.stop_line == 1) and (self.cur_state == self.Machine_State.parking.value):
            self.cur_state = self.Machine_State.stop.value

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.delivery.value:
            self.cur_state = self.Machine_State.delivery.value

        #if (self.stop_line == 1) and (self.cur_state == self.Machine_State.delivery.value):
        #    self.cur_state = self.Machine_State.stop.value


        #elif (self.stop_line == 1) and (self.cur_state == self.Machine_State.safety_zone.value):
        #   self.cur_state = self.Machine_State.stop.value

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.avoid_cruise.value:
            print(self.avoid_timer,self.avoid_count, self.obstacle_count)
            if(self.dynamic_obstacle == True) and (self.avoid_flag == True):
                if (self.obstacle_count <= 30):
                    self.cur_state = self.Machine_State.stop.value
            elif (self.dynamic_obstacle == False) and (self.avoid_timer == True):
                self.cur_state = self.Machine_State.avoid_cruise.value
            elif (self.dynamic_obstacle == False) and (self.avoid_timer == False):
                self.cur_state = self.Machine_State.cruise.value


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

        if self.cur_state == self.Machine_State.delivery.value or sub_event == self.Machine_State.delivery.value:  #////////////////////////////////////////////////////////#
            print("ok",self.cur_deliverysign)
            print(" self.delivery_destination  : ", self.delivery_destination )
            if (self.cur_deliverysign == self.DeliverySign.A1.value):
                delivery_sign_flag = True
                self.cur_state = self.Machine_State.delivery.value
                self.delivery_destination = self.DeliverySign.B1.value
            elif(self.cur_deliverysign == self.DeliverySign.A2.value):
                delivery_sign_flag = True
                self.cur_state = self.Machine_State.delivery.value
                self.delivery_destination = self.DeliverySign.B2.value
            elif(self.cur_deliverysign == self.DeliverySign.A3.value):
                delivery_sign_flag = True
                self.cur_state = self.Machine_State.delivery.value
                self.delivery_destination = self.DeliverySign.B3.value
            elif(self.cur_deliverysign == self.DeliverySign.B1.value):
                if(self.delivery_destination==self.DeliverySign.B1.value):
                    delivery_sign_flag = True
                    self.cur_state = self.Machine_State.delivery.value
                    self.delivery_finish.publish(True)
                    print('B1 pub')
                else:
                    delivery_sign_flag = False
                    #self.cur_state = self.Machine_State.cruise.value
                    #print("b1 cruise")
            elif(self.cur_deliverysign == self.DeliverySign.B2.value):
                if(self.delivery_destination==self.DeliverySign.B2.value):
                    delivery_sign_flag = True
                    self.cur_state = self.Machine_State.delivery.value
                    self.delivery_finish.publish(True)
                    print('B2 pub')
                else:
                    delivery_sign_flag = False                    #self.cur_state = self.Machine_State.cruise.value
                    #print("b2 cruise")
            elif(self.cur_deliverysign == self.DeliverySign.B3.value):
                if(self.delivery_destination==self.DeliverySign.B3.value):
                    delivery_sign_flag = True
                    self.cur_state = self.Machine_State.delivery.value
                    self.delivery_finish.publish(True)
                    print('B3 pub')
                else:
                    delivery_sign_flag = False                    #self.cur_state = self.Machine_State.cruise.value
                    #print("b3 cruise")
            else:
                #self.cur_state = self.Machine_State.cruise.value
                self.delivery_sign_flag=False
                self.cur_deliverysign = -1
                #print("b1,b2,b3 else: cruise")
            #print("delivery sign : ", self.cur_deliverysign)
        else:
            self.delivery_sign_flag=False
            print('no delivery sign')
        
            #self.delivery_finish.publish(False)  
            #self.cur_state = self.Machine_State.cruise.value  
            #print("self.cur_state == self.Machine_State.delivery.value else: cruise")            

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