#!/usr/bin/env python
from std_msgs.msg import Int16MultiArray, Bool
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

def delivery_zone_callback(data):
    global D_num

    D_num=data.data

def path_converter(gps, step_gps, last_step):
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

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 0

    pathmsg.poses.append(pose)

def find_gps_step(last_step, cur_gps):
    min_length=100
    cur_step=0

    for step_gps in range(last_step-4):
        gps_n = parking_data[step_gps].split(',')
        gps_n_1 = parking_data[step_gps+1].split(',')
        gps_n_2 = parking_data[step_gps+2].split(',')


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

def pub_path(path, step): #publish going delivery path based current gps_step
    delivery_pathmsg=Path()
    delivery_pathmsg.header.seq = step
    delivery_pathmsg.header.stamp = rospy.Time.now()
    delivery_pathmsg.header.frame_id = "map"

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
            delivery_pathmsg.poses.append(pose)
        else:
            pass
    #print(len(parking_pathmsg.poses))
    delivery_path.publish(parking_pathmsg)


if __name__ == '__main__':

    rospy.init_node('parking_move_node')

    rospy.Subscriber("/delivery_zone",int32,delivery_zone_callback)
    rospy.Subscriber("/gps/fix",NavSatFix,gps_callback)
    delivery_path = rospy.Publisher("/delivery_path",Path, queue_size=10)
    delivery_finish = rospy.Publisher("/delivery_finish",Bool, queue_size=10)
    D_num = 0
    pathmsg=Path()

    rospack = rospkg.RosPack()
    rospack.list()
    arg_name = rospack.get_path('stauto_sensor') + "/src/delivery_data/"
    
    lat = 0
    lon = 0
    step = 0
    step_gps = 0
    last_step = 0
    delivery_data=""
    D_sign=False
    D_end=False
    create_path_msg=False
    D_zone_number=10

    rospy.sleep(1)
    init_time=rospy.Time.now()
    prev_time=time.time()
    
    while not rospy.is_shutdown():
        try:
            if D_end==False:
                print(D_zone_number)
                if D_sign==False: #find delivery space & read delivery path
                    for i in range(1,4):
                        if D_num==1:
                            if D_zone_number != i:
                                D_zone_number=i # D_zone_number ==1
                                D_sign=True
                                create_path_msg=False
                                f = open(arg_name + "delivery"+str(i+1)+".txt","r")
                                D_data = f.readlines() # D_data : txt 파일에서 읽은 gps 좌표
                                last_step=len(parking_data)
                                pathmsg=Path()
                               
                            
                            else:
                                D_sign=True
                        
                        if D_sign==True:
                            break #-----------------------------------------------------------> check
                if (D_sign==True) and (create_path_msg==False): #create delivery path msg
                    if (step<last_step):

                        gps_n = D_data[step].split(',')
                        gps_n = [float(gps_n[0]), float(gps_n[1])]

                        path_converter(gps_n, step, last_step)
                        step=step+1
                    else:
                        create_path_msg=True
                        step=0
                if (D_sign==True) and (create_path_msg==True): #find current gps step based delivery path & publish going delivery path to control.py
                    step_gps=find_gps_step(last_step, step_gps)
                    print(step_gps, last_step)
                    if step_gps<last_step-6:
                        pub_path(pathmsg, step_gps)
                        delivery_finish.publish(False)
                    else:
                        delivery_finish.publish(True)
                        D_end=True
                    D_sign=False
                        #print(3)
            else:
                pass
        except rospy.ROSInterruptException:
            pass
