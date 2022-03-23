#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>

LIDARLite lidarStuff;
Servo panServo;

  //Hold Variables
  struct Var{
    
  // Create a servo object
  // Declaration of Pins
  
  #define buzzerPin 8      // buzzer
  #define servoPin 4      // servo motor
  #define pingPin 17     // Ultrasonic sensor
  
  // Variables 
  double meter = lidarStuff.distance(); 
  //int angle = 0;
  float marker1x,marker2x,marker3x,side1,side2,center = 0.0;
  float marker1y, marker2y, marker3y = 0.0;
  float x, y = 0;
  
  };

void setup() {
  struct Var v1;
  Serial.begin(9600);           // opens serial port, sets data rate to 9600 bps
  lidarStuff.begin(0, true);    // default I2C freq to 400 kHz
  pinMode(buzzerPin, OUTPUT);   // Set buzzer - pin 8 as an output
  panServo.attach(servoPin);    // We need to attach the servo to the used pin number
  lidarStuff.configure(0);      // default mode, balanced performance.
}


  //CONSTANT VALUES
  const int OutOfRange      = 30;   //distance lidar measures but is too far away and we can ignore
  const int perimeterSides  = 10;   //length of sides of  one of the perimeters
                                  //We can also measure these sides with lidar and calculations



  //MAIN LOOP
void loop() { 
  struct Var v;
  servo(v);
 }


/*
FUNCTIONS TO-DO LIST
----------------------------
-----------------------------

SENSORS (declare pins and create functions if needed)
------------
BUZZER (DONE)
SERVO  (SEMI-DONE)
GYRO
Ultrasonic Sensor (DONE)

ACTIONS
-----------
DroneStart
GetDistance (DONE)
PickUpPackage
Yaw
Pitch
Roll
Throttle
GetAltitude (DONE)
CalculateCenter (IN PROGRESS)
Drone2CenterDist
AltHold
AutoLevel

PWM
----------
CHAN1
CHAN2
CHAN3
CHAN4


*/




//-------The buzzer sensor is used to alert when Package is Located-----------
void buzzer(Var v) { 
  tone(buzzerPin, 1000); // Send 1KHz sound signal...
  delay(500);            // ...for 1/2 sec
  noTone(buzzerPin);     // Stop sound...
  delay(500);            // ...for 1/2 sec
}

//-----------------Using Ultrasonic sensor to get the altitude of the drone------------------

int GetAltitude(Var v) { 
    int duration, cm;
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(5);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pingPin, LOW);
    pinMode(pingPin, INPUT); //Read from same pin. The duration of the HIGH pulse tells the distance
    duration = pulseIn(pingPin, HIGH);
    cm = duration / 58; //convert time into a distance
    delay(20);
    
    return cm;
  }

 //------Uses the LiDAR Sensor to get horizontal distance of Markers or Objects----------

//trying to be consistent with units remooving CM 
double GetDistance(Var v) { 
  if (v.meter >= 100) {
    v.meter = v.meter;
    //Serial.println(" meters");
  }
  else {
    v.meter = lidarStuff.distance();
    //Serial.println(" cm");
  }
  return  v.meter;
}
//-------SERVO-------------------------
void servo(Var v) {
    for (int angle = panServo.read() ; angle < 180; angle += 2) {
        panServo.write(angle); 
        delay(100);
        CalculateCenter(angle, v);
    }
 
    for (int angle = panServo.read(); angle > 0; angle -= 2) {
        panServo.write(angle);
        delay(100);
        CalculateCenter(angle, v);
    }
}
// -----Calculates the Center
//  **Assmuning the drone starts at the same spot and has to be close to a marker.**
void CalculateCenter(int angle, Var v){ 
  
   //sin and cos are radians only had to convert 
     double r = GetDistance(v);
     double x = r*cos(angle*3.1416/180);
     double y = r*sin(angle*3.1416/180);
     
     //testing ouput
     //Serial.print( "r = ") + Serial.print(r) + Serial.print(" theta = ") + Serial.println(angle);
     //Serial.print("x = ") + Serial.print(x) + Serial.print(" y = ") + Serial.println(y);
     
     if( x != v.marker1x && y != v.marker1y)
     {
      v.marker1x = x;
      v.maker1y = y;
      
     if(v.marker2x != x && v.marker2y != y)
      {
        v.marker2x = x;
        v.marker2y = y;
      }
     }
}
//--------------PICK-UP PACKAGE------------
void PickUpPackage() {

}
