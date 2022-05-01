from lidar_lite import Lidar_Lite
import time
import keyboard
from gpiozero import AngularServo
from djitellopy import Tello
import math
import constant 

lidar = Lidar_Lite()
servo = AngularServo(18,min_pulse_width = 0.0006, max_pulse_width = 0.0023) 
#pin gpio 18 not pin 18 -_-

#max distance from lidar in ft
package_distance = 5

#convert to cm
package_distance_cm = package_distance * 30.48

#test if lidar is connected
connected = lidar.connect(1)
if connected < -1:
    print("not connected")
else:
    print("Connected")

#creates an array byween -90 to 90
degrees = range(-90,90,1)

#I hope this works i swear to god :) 
#Creat4es a 180 sweep
for alpha in degrees:
    servo.angle = alpha
    
    #Gets distance from lidar in cm
    distance = lidar.getDistance()
    #print("distance is: %s" % (distance))
    #print("angle is %s"  % (alpha))
    
    #delay of 0.02 s or 20 ms
    time.sleep(0.02) 

    #set the max range
    if int(distance) <= package_distance_cm:
        #conver to x and y, cos and sine are in radians
        x = distance*math.cos(alpha/180*math.pi)
        y = distance*math.sin(alpha/180*math.pi)
        print("X = %.2f" %x)
        print("Y = %2.f" %y)
    alpha = alpha + 1
