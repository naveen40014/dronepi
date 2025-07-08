#*******************from precise landing*****************************
from os import sys, path
from re import T
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import time
import math
import argparse
import csv
from datetime import datetime
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from opencv.lib_aruco_pose import *

now= datetime.now()
dt_str=now.strftime("%H:%M___%d-%m-%y")
name="log_terminal_" + dt_str +".csv"
file=open (name,'w',newline='')
writer=csv.writer(file)
#writer.writerow(['Time','Lat','long','baro','aruco_dist','detect'] )
detect=0
gnd_speed = 4 # [m/s]
altitude= 6 # m/s

id_to_find      = 0
marker_size     = 39 #- [cm]
freq_send       = 4 #- Hz


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
    print ("dlat, dlon", dLat, dLon)
    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return(newlat, newlon)

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
    
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)

#**********************************from mission****************************************/
#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------
#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 0.8:
          print("Target altitude reached")
          break
      time.sleep(0.5)

def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.flush()

    # After clearing the mission you MUST re-download the mission from the vehicle
    # before vehicle.commands can be used again
    # (see https://github.com/dronekit/dronekit-python/issues/230)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

def download_mission(vehicle):
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    

def get_current_mission(vehicle):
    """
    Downloads the mission and returns the wp list and number of WP 
    
    Input: 
        vehicle
        
    Return:
        n_wp, wpList
    """

    print ("Downloading mission")
    download_mission(vehicle)
    missionList = []
    n_WP        = 0
    for wp in vehicle.commands:
        missionList.append(wp)
        n_WP += 1 
        
    return n_WP, missionList


def add_last_waypoint_to_mission(#--- Adds a last waypoint on the current mission file
        vehicle,            #--- vehicle object
        wp_Last_Latitude,   #--- [deg]  Target Latitude
        wp_Last_Longitude,  #--- [deg]  Target Longitude
        wp_Last_Altitude):  #--- [m]    Target Altitude
        
   
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
    
    
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # Save the vehicle commands to a list
    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)

    # Modify the mission as needed. For example, here we change the
    wpLastObject = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0, 
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    # Clear the current mission (command is sent when we call upload())
    cmds.clear()

    #Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()
    
    return (cmds.count)    

def ChangeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True
#--------------------------------------------------
#-------------- INITIALIZE  
#--------------------------------------------------      
#-- Setup the commanded flying speed
mode      = 'GROUND'

#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
print('Connecting...')
vehicle = connect("/dev/ttyUSB0",wait_ready=True,baud=57600)  
#vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)


#*************************from precise landing*************************

#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag

land_alt_cm         = 500.0
angle_descend       = 5*deg_2_rad
land_speed_cms      = 0.0

#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
camera_matrix       = np.loadtxt(calib_path+'/logitech/cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'/logitech/cameraDistortion.txt', delimiter=',')                                      
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False, 
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)
                
  
time_0 = time.time()
#************************mission***********************

#--------------------------------------------------
#-------------- MAIN FUNCTION  
#--------------------------------------------------    
while True:
    
    if mode == 'GROUND':
        #--- Wait until a valid mission has been uploaded
        n_WP, missionList = get_current_mission(vehicle)
        time.sleep(2)
        if n_WP > 0:
            print ("A valid mission has been uploaded: takeoff!")
            mode = 'TAKEOFF'
            
    elif mode == 'TAKEOFF':
       
        #-- Add a fake waypoint at the end of the mission
        #add_last_waypoint_to_mission(vehicle, vehicle.location.global_relative_frame.lat, 
                                       #vehicle.location.global_relative_frame.lon, 
                                       #vehicle.location.global_relative_frame.alt)
        #("Home waypoint added to the mission")
        time.sleep(1)
        #-- Takeoff
        arm_and_takeoff(altitude)
        uav_location        = vehicle.location.global_relative_frame
        #writer.writerow([str(time.time),str(uav_location.lat),str(uav_location.lon),str(uav_location.alt),str(0),str(detect)])
        #-- Change the UAV mode to AUTO
        print("Changing to AUTO")
        ChangeMode(vehicle,"AUTO")
        
        #-- Change mode, set the ground speed
        vehicle.groundspeed = gnd_speed
        mode = 'MISSION'
        print ("Switch mode to MISSION")
        
    elif mode == 'MISSION':
        #-- Here we just monitor the mission status. Once the mission is completed we go back
        #-- vehicle.commands.cout is the total number of waypoints
        #-- vehicle.commands.next is the waypoint the vehicle is going to
        #-- once next == cout, we just go home

#**********************precise landing*************************
        #time.sleep(2)
        i=0        
        done=0
        #writer.writerow([str(time.time),str(uav_location.lat),str(uav_location.lon),str(uav_location.alt),str(0),str(detect)])

        marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
        if marker_found and done==0:
            detect=1
            #writer.writerow([str(time.time),str(uav_location.lat),str(uav_location.lon),str(uav_location.alt),str(0),str(detect)])
            ChangeMode(vehicle,"GUIDED")
            print("changing mode to GUIDED")
            print("aruco found")
            while(i<60):
                marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
                #if vehicle.mode!="AUTO":
                 #   print("Manual over-ride, closing the connection")
                  #  vehicle.close()
                   # break
                #time.sleep(0.5)
                x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
                uav_location        = vehicle.location.global_relative_frame
        
                #-- If high altitude, use baro rather than visual
                #if uav_location.alt >= 5.0:
                z_baro = uav_location.alt*100.0
                angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)
                if time.time() >= time_0 + 1.0/freq_send:
                    time_0 = time.time()
                    print (" ")
                    print ("Altitude aruco= %.0fcm  , baro= %.0f"%(z_cm,z_baro))
                    print( "Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg))
                    north, east = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
                    print ("Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, vehicle.attitude.yaw*rad_2_deg))
                    marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01)  
                    #-- If angle is good, descend
                    #if check_angle_descend(angle_x, angle_y, angle_descend):
                    #    print ("Low error: loiter")
                    #location_marker = LocationGlobalRelative(marker_lat, marker_lon, altitude) #-(land_speed_cms*0.01/freq_send))
                    #else:
                    location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                    vehicle.simple_goto(location_marker)
                    print("UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon))
                    print ("Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon))
                    i=i+1
                    #writer.writerow([str(time.time),str(uav_location.lat),str(uav_location.lon),str(uav_location.alt),str(z_cm),str(detect)])
                time.sleep(1)
                done==1
            ChangeMode(vehicle,"RTL")
            mode = "BACK"
                    

        else:
            print("Searching for Aruco Marker")
            detect=0
           # z_cm = uav_location.alt*100.0
            #print ("Altitude baro = %.0fcm"%z_cm)
             #location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
            #continue
            
#***************************mission******************************        
        print ("Current WP: %d of %d "%(vehicle.commands.next, vehicle.commands.count))
        if vehicle.commands.next == vehicle.commands.count:
            print ("Final waypoint reached: go back home")
            #-- First we clear the flight mission
            clear_mission(vehicle)
            print ("Mission deleted")
            file.close()
            #-- We go back home
            ChangeMode(vehicle,"RTL")
            mode = "BACK"
            
    elif mode == "BACK":
        if vehicle.location.global_relative_frame.alt < 1:
            print ("Switch to GROUND mode, waiting for new missions")
            mode = 'GROUND'
    #key = cv2.waitKey(1) & 0xFF
    #if key == ord('q'):
      #  self._cap.release()
     #   cv2.destroyAllWindows()
    #    break
    time.sleep(0.5)
    file.close()
    
    
