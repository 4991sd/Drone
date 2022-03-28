#include <Wire.h>
#include <LIDARLite.h>
#include "Arduino.h"


class Detect
{
  //Global Varibles
  LIDARLite lidarStuff;
 
  int distance [4] = {0,0,0,0};
  int theta [4] = {0,0,0,0};
  double markerX [4] = {0,0,0,0};
  double markerY [4] = {0,0,0,0};
  
  
  
  public:
double CalculateCenter(int angle, int r)
{
   
   double  x5 = r*cos(angle*3.1416/180);
   double   y5 = r*sin(angle*3.1416/180); 
  
 //   Serial.print( "r = ") + Serial.print(r) + Serial.print(" theta = ") + Serial.println(angle);
    Serial.print("x = ") + Serial.print(x5) + Serial.print(" y = ") + Serial.println(y5);
     
   for(int i = 0; i < 4; i++)
     {  
       distance[i] = r;
       theta[i] = angle;
      }
      if (r < distance[0])
      {
        distance[0] = r;
        theta[0] = angle;
      }
      else if (r < distance[1])
      {
        distance[1] = r;
        theta[1] = angle;
      }
      else if (r < distance[2])
      {
        distance[2] = r;
        theta[2] = angle;
      }
      else if (r < distance[3])
      {
        distance[3] = r;
        theta[3] = angle;
      }
    
    for(int k = 0; k < 4; k++)
    {
      //sin and cos are radians only had to convert 
      markerX[k] = distance[k]*cos(theta[k]*3.14/180);
      markerY[k] = distance[k]*sin(theta[k]*3.14/180);  
     
      //testing output
     Serial.print(k+1) + Serial.print(" x = ") + Serial.println(markerX[k]);
     Serial.print(k+1) + Serial.print(" y = ") + Serial.println(markerY[k]);
    }
    
}


 };
