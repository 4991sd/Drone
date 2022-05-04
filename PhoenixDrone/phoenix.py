from lidar_lite import Lidar_Lite
import time
import keyboard
from gpiozero import AngularServo
from djitellopy import Tello
import math
import constant 


#Servo and Lidar Start
lidar = Lidar_Lite()
tello = Tello()
servo = AngularServo(18,min_pulse_width = 0.0006, max_pulse_width = 0.0023) 
#pin gpio 18 not pin 18 -_-

Xs = []
Ys = []

#connect to drone
tello.connect()
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

i = 0
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
        print("Y = %.2f" %y)

        if alpha >= 0 and alpha <=90:
            alpha1 = alpha + 90
            #convert to x and y, cos and sine are in radians
            x = distance*math.cos(alpha1/180*math.pi)
            y = distance*math.sin(alpha1/180*math.pi)

            #Formatting to show 2 sig fig
            #print("X = %.2f" %x)
            #print("Y = %.2f" %y)
        if alpha < 0:
            #convert again but correcting angles to be within 90 - 180
            alpha1 = alpha + 90
            x = distance*math.cos(alpha1/180*math.pi)
            y = distance*math.sin(alpha1/180*math.pi)

            #formatting
            #print("X = %.2f" %x)
            #print("Y = %2.f" %y)
        if alpha + 90 > 45 and alpha + 90 < 135:
            alpha1 = alpha + 90

            Xs.append(distance*math.cos(alpha1/180*math.pi))
            Ys.append(distance*math.sin(alpha1/180*math.pi))

            #display output
            print("X = %.2f" %Xs[i])
            print("Y = %.2f" %Ys[i])
            i = i + 1

    alpha = alpha + 1
#calculate center of package 

x_length = 0
y_length = 0
x_center = 0
y_center = 0

x_length = abs(Xs[-1]) - abs(Xs[0])
y_length = abs(Ys[-1]) - abs(Ys[0])
x_center = abs(x_length / 2)
y_center = abs(y_length / 2)
print("x length = %.2f"  %x_length)
print("y length = %.2f"  %y_length)
print("x center is %.2f" %x_center)
print("y center is %.2f" %y_center)

#Drone Start


tello.takeoff()
tello.move_forward(int(Ys[-1])
if Xs[-1] < 0:
    tello.move_left(int(abs(Xs[-1])
if Xs[-1] > 0:
    tello.move_right(int(abs(Xs[-1])
#drop done to package and lift

tello.land()
tello.takeoff()

#return to starting
tello.rotate_cw(180)
tello.move_forward(int(Ys[-1]))
if Xs[-1] < 0:
    tello.move_right(int(abs(Xs[-1])))
if Xs[-1] > 0:
    tello.move_left(int(abs(Xs[-1])))
#EOF
