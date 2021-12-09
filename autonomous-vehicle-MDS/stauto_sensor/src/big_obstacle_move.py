#!/usr/bin/env python

from std_msgs.msg import Int32MultiArray, Bool
from sensor_msgs.msg import NavSatFix
import rospkg, rospy, time
import numpy as np
from pyproj import Proj
from pyproj import transform
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from math import *

WGS84 = { 'proj':'latlong', 'datum':'WGS84', 'ellps':'WGS84', }
GRS80 = { 'proj':'tmerc', 'lat_0':'38', 'lon_0':'127', 'k':1, 'x_0':0,
    'y_0':0, 'ellps':'GRS80', 'units':'m' }

def grs80_to_wgs84(x, y):
   return transform( Proj(**GRS80), Proj(**WGS84), x, y )

def wgs84_to_grs80(x, y):
   return transform( Proj(**WGS84), Proj(**GRS80), y, x )

def gps_callback(data):
    global lat, lon

    lat = data.latitude
    lon = data.longitude

def big_obstacle_zone_callback(data):
    #global big_obstacle_spot_array
    global big_array
    #avoid_spot_array = data.data
    big_array = data.data

def path_converter(gps, step_gps, last_step): # convert wgs84 to grs80
    #print(step_gps)
    pathmsg.header.seq = step_gps
    pathmsg.header.stamp = rospy.Time.now()
    pathmsg.header.frame_id = "map"

    pose = PoseStamped()
    pose.header.seq = step_gps
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"

    x,y=wgs84_to_grs80(gps[0],gps[1])
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0
    #print(x,y)

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 0

    #rospy.sleep(0.5)
    pathmsg.poses.append(pose)

def find_gps_step(last_step, cur_gps): # find current gps step
    min_length=100
    cur_step=0

    for step_gps in range(last_step-4):
        gps_n = big_obstacle_data[step_gps].split(',')
        gps_n_1 = big_obstacle_data[step_gps+1].split(',')
        gps_n_2 = big_obstacle_data[step_gps+2].split(',')

        # gps_n = [float(gps_n[0]) - float(gps_origin[0]), float(gps_n[1]) - float(gps_origin[1])]
        # gps_n_1 = [float(gps_n_1[0]) - float(gps_origin[0]), float(gps_n_1[1])- float(gps_origin[1])]
        gps_n = [float(gps_n[0]), float(gps_n[1])]
        gps_n_1 = [float(gps_n_1[0]), float(gps_n_1[1])]
        gps_n_2 = [float(gps_n_2[0]), float(gps_n_2[1])]


        utm_gps_n = wgs84_to_grs80(gps_n[0],gps_n[1])
        utm_gps_n_1 = wgs84_to_grs80(gps_n_1[0],gps_n_1[1])
        utm_gps_n_2 = wgs84_to_grs80(gps_n_2[0],gps_n_2[1])

        utm_gps_cur = wgs84_to_grs80(lat,lon)

        length1 = sqrt((utm_gps_cur[0]-utm_gps_n[0])**(2)+(utm_gps_cur[1]-utm_gps_n[1])**(2))
        length2 = sqrt((utm_gps_cur[0]-utm_gps_n_1[0])**(2)+(utm_gps_cur[1]-utm_gps_n_1[1])**(2))

        length = length1+length2
        '''
        line_data_x=[utm_gps_n[0],utm_gps_n_1[0],utm_gps_n_2[0]]
        line_data_y=[utm_gps_n[1],utm_gps_n_1[1],utm_gps_n_2[1]]
        fp1 = np.polyfit(line_data_x,line_data_y,1)
        y= fp1[0]*step_gps+fp1[1]
        length=abs(fp1[0]*utm_gps_cur[0] - utm_gps_cur[1] + fp1[1])/sqrt(fp1[0]**(2)+(-1)**(2)) #find length
        '''

        if(length<min_length):
            min_length=length
            cur_step=step_gps+1
        '''
        elif (length<min_length and cur_gps-2 <= step_gps <= cur_gps+2):
            min_length=length
            cur_step=step_gps+1
        '''
    return cur_step

def pub_path(path, step): #publish going big_obstacle path based current gps_step
    big_obstacle_pathmsg=Path()
    big_obstacle_pathmsg.header.seq = step
    big_obstacle_pathmsg.header.stamp = rospy.Time.now()
    big_obstacle_pathmsg.header.frame_id = "map"

    #for i in range(len(path.poses)-step-1):
    for i in range(20):
        pose = PoseStamped()
        #print("len",len(path.poses))
        #print("step",step)
        pose.header.seq = step
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        #print(step)
        #print(len(path.poses)-(step+i+1))
        if(len(path.poses)-(step+i+1)>0):
            pose.pose.position.x = path.poses[step+i].pose.position.x
            pose.pose.position.y = path.poses[step+i].pose.position.y
            pose.pose.position.z = 0

            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
            #print(path.poses[step+i].pose.position.x)
            #rospy.sleep(0.5)
            big_obstacle_pathmsg.poses.append(pose)
        else:
            pass
    #print(len(big_obstacle_pathmsg.poses))
    big_obstacle_path.publish(big_obstacle_pathmsg)


if __name__ == '__main__':

    rospy.init_node('big_obstacle_move_node')

    rospy.Subscriber("/big_obstacle_zone",Int32MultiArray,big_obstacle_zone_callback)
    rospy.Subscriber("/gps/fix",NavSatFix,gps_callback)
    big_obstacle_path = rospy.Publisher("/big_obstacle_path",Path, queue_size=10)
    big_obstacle_fin = rospy.Publisher("/big_obstacle_finish",Bool, queue_size=10)
    #big_obstacle_spot_array = np.zeros(5)
    big_array = np.zeros(5)
    pathmsg=Path()

    rospack = rospkg.RosPack()
    rospack.list()
    arg_name = rospack.get_path('stauto_sensor') + "/src/big_obstacle_data/"        # gps path data file name
    
    lat = 0
    lon = 0
    step = 0
    step_gps = 0
    last_step = 0
    big_obstacle_data=""
    big_obstacle_sign=False
    big_obstacle_finish=False
    create_path_msg=False
    big_obstacle_zone_number=10

    rospy.sleep(1)
    init_time=rospy.Time.now()
    prev_time=time.time()
    big_obstacle_finish_time = time.time()
    '''
    f1 = open(arg_name + "big_obstacle1.txt","r")
    f1 = open(arg_name + "big_obstacle2.txt","r")
    f1 = open(arg_name + "big_obstacle3.txt","r")
    f1 = open(arg_name + "big_obstacle4.txt","r")
    f1 = open(arg_name + "big_obstacle5.txt","r")

    big_obstacle1_data = f1.readlines()
    '''
    while not rospy.is_shutdown():
        try:
            #print(big_obstacle_end)
            if big_obstacle_finish==False:
                print(big_obstacle_zone_number)
                if big_obstacle_sign==False: #find big_obstacle space & read big_obstacle path
                    print(big_obstacle_sign)
                    for i in range(5):
                        
                        #print(big_obstacle_spot_array)
                        #big_obstacle_spot_array[1]=1
                        #if(time.time()-prev_time>=20):
                        #    big_obstacle_spot_array[2]=1
                        
                        #if big_obstacle_spot_array[i]==1:
                        if big_array[i]==1:    
                            #print(i)
                            if big_obstacle_zone_number != i:
                                big_obstacle_zone_number = i
                                print(big_obstacle_zone_number)
                                big_obstacle_sign=True
                                create_path_msg=False
                                f = open(arg_name + "big_obstacle"+str(i+1)+"0.txt","r")
                                big_obstacle_data = f.readlines()
                                last_step=len(big_obstacle_data)
                                pathmsg=Path()
                                #print(last_step)
                                #print(1.1)
                                
                            else:
                                big_obstacle_sign=True
                                #print(1.2)
                        
                        if big_obstacle_sign==True:
                            break
                    #print(1)
                if (big_obstacle_sign==True) and (create_path_msg==False): #create big_obstacle path msg
                    if (step<last_step):

                        gps_n = big_obstacle_data[step].split(',')
                        #print(float(gps_n[0]), float(gps_n[1]))
                        gps_n = [float(gps_n[0]), float(gps_n[1])]

                        path_converter(gps_n, step, last_step)
                        step=step+1
                    else:
                        create_path_msg=True
                        step=0
                    #print(2)
                if (big_obstacle_sign==True) and (create_path_msg==True): #find current gps step based big_obstacle path & publish going big_obstacle path to control.py
                    step_gps=find_gps_step(last_step, step_gps)
                    big_obstacle_finish_time=time.time()
                    print(step_gps, last_step)
                    if (time.time()-big_obstacle_finish_time<1):
                        pub_path(pathmsg, step_gps)
                        big_obstacle_fin.publish(False)
                    else:
                        big_obstacle_fin.publish(True)
                        #big_obstacle_end=True
                    big_obstacle_sign=False
                        #print(3)
            else:
                pass
        except rospy.ROSInterruptException:
            pass