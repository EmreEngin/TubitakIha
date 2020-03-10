import dronekit as dk
import time
import os
import math
from pymavlink import mavutil
import paramiko 
from cv2 import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar

def arm_and_takeoff(vehicle,aTargetAltitude):
    
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = dk.VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        bilgiYaz(vehicle)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        vehicle.simple_takeoff(aTargetAltitude)
        time.sleep(1)
def bilgiYaz(vehicle):
    frame = vehicle.location.global_relative_frame
    bilgi = {
        "Hiz: " : vehicle.groundspeed,
        "Irtifa: " : frame.alt,
        "Konum: " : { 
            "Latitude: " : frame.lat,
            "Longitude: " : frame.lon
        }
    }
    os.system("echo '{}'".format(bilgi))
def konumVer(nesne):
    if nesne == "sag":
        with open("sag.txt", "r") as a:
            dizi = a.readlines()
            dizi = [x.strip() for x in dizi]
            latsag = float(dizi[0])
            lonsag = float(dizi[1])
            sagdirek = (latsag, lonsag)
            return sagdirek

    elif nesne == "sol":
        with open("sol.txt", "r") as o:
            dizi =o.readlines()
            dizi = [x.strip() for x in dizi]
            latsol = float(dizi[0])
            lonsol = float(dizi[1])
            soldirek = (latsol, lonsol)
            return soldirek
    elif nesne == "sise1":
        with open("sise1.txt", "r") as bir:
            dizi =bir.readlines()
            dizi = [x.strip() for x in dizi]
            latsise1 = float(dizi[0])
            lonsise1 = float(dizi[1])
            sise1 = (latsise1, lonsise1)
            return sise1

    elif nesne == "sise2":
        with open("sise2.txt", "r") as iki:
            dizi =iki.readlines()
            dizi = [x.strip() for x in dizi]
            latsise2 = float(dizi[0])
            lonsise2 = float(dizi[1])
            sise2 = (latsise2, lonsise2)
            return sise2
    elif nesne == "yarimdolualan":
        with open("yarimdolualan.txt", "r") as y:
            dizi =y.readlines()
            dizi = [x.strip() for x in dizi]
            laty = float(dizi[0])
            lony = float(dizi[1])
            yarimdolualan = (laty, lony)
            return yarimdolualan
    elif nesne == "tamdolualan":
        with open("tamdolualan.txt", "r") as t:
            dizi =t.readlines()
            dizi = [x.strip() for x in dizi]
            latt = float(dizi[0])
            lont = float(dizi[1])
            tamdolualan = (latt, lont)
            return tamdolualan
def MeterToDd(meter):
    return (0.00001 * meter) / 1.1132
def ddToMeter(decimalDegree):
    return (1.1132 * decimalDegree) / 0.00001
def get_distance_metres(right, left):
    rlat, rlon = right
    llat, llon = left
    dlat = llat - rlat
    dlong = llon - rlon
    dlat = abs(dlat)
    dlong = abs(dlong)
    dlat = ddToMeter(dlat)
    dlong = ddToMeter(dlong)
    return math.sqrt((dlat*dlat) + (dlong*dlong))
def OradaMiyim(vehicle,HedefKonum, kontrol):
        while True: 
            mevcutKonum = (vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon)
            mesafe = get_distance_metres(mevcutKonum, HedefKonum)
            if mesafe < kontrol:
                return True
            print "Kalan mesafe : {}".format(mesafe)
            return False
def konumcevirme (location,altitude):
    a = dk.LocationGlobalRelative(location[0],location[1],altitude)
    return a 
def set_servo(vehicle, servo_number, pwm_value):
	pwm_value_int = int(pwm_value)
	msg = vehicle.message_factory.command_long_encode(
		0, 0, 
		mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
		0,
		servo_number,
		pwm_value_int,
		0,0,0,0,0
		)
	vehicle.send_mavlink(msg)
def run_download(hostname,port,username,password):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    com= "python pin37.py"
    ssh.connect(hostname,port,username,password)   
    stdin,stdout,stder= ssh.exec_command(com)
    type(stdin)
    a = stdout.readlines()
    ssh.close()
    if a == [u'0\n']:
        return 0 
    return 1
def don():
    cap = cv2.VideoCapture("rtsp://192.168.1.44:8554/unicast")
    while True:
        
        try:
            _, frame = cap.read()
            #cv2.circle(frame, (320, 240), 5, (255, 0, 0), 3)
            decodedObjects = pyzbar.decode(frame)
            obj=decodedObjects
            if len(obj)>1:
                if obj[0].data == b'sol':
                    left = obj[0]
                    right = obj[1]
                else:
                    left = obj[1]
                    right = obj[0]
                (leftx, lefty, leftw, lefth) = left.rect
                (rightx, righty, rightw,righth) = right.rect
                leftCenter = (leftx+leftw, lefty+lefth)
                rightCenter = (rightx+rightw, righty+righth)
                #cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 2), 2)
                # cv2.putText(frame, str(obj.data), (50, 50), font, 3, (255, 0, 0), 3)
                # cv2.line(frame, (int(x+(w/2)), 0), (int(x+(w/2)), y+1000), (0, 0, 255), 4)
                # cv2.line(frame, (int(x+(w/2)), 0), (int(x+(w/2)), y+1000), (0, 0, 255), 4)
                # cv2.line(frame,obj.polygon[0],obj.polygon[1],(81,54,94),3)
                # cv2.line(frame,obj.polygon[1],obj.polygon[2],(81,54,94),3)
                # cv2.line(frame,obj.polygon[2],obj.polygon[3],(81,54,94),3)
                # cv2.line(frame,obj.polygon[3],obj.polygon[0],(81,54,94),3)   
                try:
                    # 1 CW -1 CCW
                    rad=math.atan(abs((leftCenter[1]-rightCenter[1]))/abs((leftCenter[0]-rightCenter[0]))) 
                    degree=math.degrees(rad)
                    if degree>10:
                        if leftCenter[0]<rightCenter[0]:
                            if leftCenter[1]<rightCenter[0]:
                                return[1,degree]     
                            else:
                                return[-1,degree]
                        else:
                            if rightCenter[1]<leftCenter[1]:
                                return[-1,degree]
                            else:
                                return[1,degree]
                    else:
                        if leftCenter[0]<rightCenter[0]:
                            return "ok"
                        else:
                            return "180"
                except ZeroDivisionError:
                    pass
                # ---------------------------------------------------------------
                # RIGHT-LEFT FOR X AXIS ACCORDING TO ONE QR CODE BY THE AGENCY OF CENTER VERTICAL LINE
                # print(check(x,y,w,h))
                # ---------------------------------------------------------------
                # DISTANCE FROM CAMERA
                # RPI CAMERA MODULE 1.3 FOCAL LENGHT = 1.57480315
                # d=(1.57480315*599.999)/w
                # print(round(d*2.54),2)
                # ---------------------------------------------------------------
            cv2.imshow("Frame", frame)
        except:
            pass
        key = cv2.waitKey(1)
        if key == 27:
            break
def sagsol():
    cap = cv2.VideoCapture("rtsp://192.168.1.44:8554/unicast")
    while True:
        try:
            _, frame = cap.read()
            decodedObjects = pyzbar.decode(frame)
            obj=decodedObjects
            if len(obj)>1:
                if obj[0].data == b'sol':
                    left = obj[0]
                    #right = obj[1]
                else:
                    left = obj[1]
                    #right = obj[0]
                (leftx, lefty, leftw, lefth) = left.rect
                #(rightx, righty, rightw,righth) = right.rect
                leftCenter = (leftx+leftw, lefty+lefth)
                #rightCenter = (rightx+rightw, righty+righth)
                if (leftCenter[0]+(leftw/2)) < 260:
                    return -1
                elif (leftCenter[0]+(leftw/2)) > 380:
                    return 1
                else:
                    return 0
        except:
            pass
def cameraac(hostname,port,username,password):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(hostname,port,username,password)
    com1 = "cd v4l2rtspserver/ && ./v4l2rtspserver -W 640 -H 480 -F 10"
    stdin,stdout,stder= ssh.exec_command(com1)
    type(stdin)
    ssh.close()
def camerakapat(hostname,port,username,password):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(hostname,port,username,password)
    com2 = "pkill -n v4l2rtspserver"
    stdin,stdout,stder= ssh.exec_command(com2)
    type(stdin)
    ssh.close()
def fixYaw(yaw):
    if yaw < 0:
        return 360 + yaw
    else:
        return yaw
def rotateYaw(vehicle, angleInDegrees, direction):  # 1 for right, -1 for left

    currentYaw = math.degrees(vehicle.attitude.yaw)
    currentYaw = fixYaw(currentYaw)
    lastYaw = (currentYaw + (direction * angleInDegrees)) % 360
    print "current yaw: {}, last yaw: {}, ek: {}".format(currentYaw, lastYaw, 180+ currentYaw)

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        lastYaw,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        direction,          # param 3, direction -1 ccw, 1 cw
        0,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

    while True:
        yaw = fixYaw(math.degrees(vehicle.attitude.yaw))
        print "yaw: ", yaw
        if yaw < lastYaw + 2 and yaw > lastYaw - 2:
            print "Reached target yaw!"
            time.sleep(1)
            break
        time.sleep(1)
def git(vehicle, sag, on, sleep):
    current = vehicle.location.global_relative_frame
    yaw = fixYaw(math.degrees(vehicle.attitude.yaw))
    ddlat = MeterToDd(on)
    ddlon = MeterToDd(sag)
    lat = current.lat + ddlat
    lon = current.lon + ddlon
    goPoint = (lat, lon)
    x, y = rotatePoint((current.lat, current.lon), goPoint, -1 * math.radians(yaw))
    go = dk.LocationGlobalRelative(x, y, current.alt)
    vehicle.simple_goto(go)
    time.sleep(sleep)
def rotatePoint(origin, point, angle):
    ox, oy = origin
    px, py = point
    qy = oy + math.cos(angle) * (py - oy) - math.sin(angle) * (px - ox)
    qx = ox + math.sin(angle) * (py - oy) + math.cos(angle) * (px - ox)
    return qx, qy
def control (vehicle):

    while True:
        sonuc=don()
        if sonuc == "ok":
            break
        elif sonuc == "180":
            rotateYaw(vehicle,180,1)
        elif sonuc[0] == 1 :
            "Saat yonunde sonuc[1] kadar dondur "
            rotateYaw(vehicle,sonuc[1],1)
        elif sonuc[0] == -1:
            "saat yonun tersinde sonuc[1] kadar donduru"  
            rotateYaw(vehicle,sonuc[1],-1)

    r = sagsol()
    if r == 1:
        # 1 sag 
        git(vehicle,0.1,0,1)
    elif r == -1 :
        git(vehicle,-0.1,0,1)
    else: 
        git(vehicle,0,-0.3,3)
        git(vehicle,0,0.3,3)        