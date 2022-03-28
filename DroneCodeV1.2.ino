#include <Wire.h>
#include <LIDARLite.h>
#include "Detect.h"
#include <Servo.h>

  Detect d = Detect();
  LIDARLite lidarStuff;
  Servo panServo;

  struct Var{
    
  // Create a servo object
  // Declaration of Pins
   
  #define buzzerPin 8      // buzzer
  #define pingPin 17     // Ultrasonic sensor
  #define servoPin 4      // servo motor

  // Variables 
    double meter = lidarStuff.distance(); 
    int angle = 0;
    float x, y = 0;
    int distance [4] = {0, 0, 0, 0};
    int theta [4] = {0, 0, 0, 0};
    double markerX[4] = {0,0,0,0};
    double markerY[4] = {0,0,0,0};
   
  
  };
 
void setup() {
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

  servo();
 }




//-------The buzzer sensor is used to alert when Package is Located-----------
void buzzer() { 
  tone(buzzerPin, 1000); // Send 1KHz sound signal...
  delay(500);            // ...for 1/2 sec
  noTone(buzzerPin);     // Stop sound...
  delay(500);            // ...for 1/2 sec
}

//-----------------Using Ultrasonic sensor to get the altitude of the drone------------------

int GetAltitude() { 
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
 int GetDistance() {
    int meter = 0; 
    if (meter >= 100) {
      meter = meter * 0.01;
     //Serial.println(" meters");
                      }
    else {
    meter = lidarStuff.distance();
    //Serial.println(" cm");
         }
  return  meter;
}
//trying to be consistent with units remooving CM 

//-------SERVO-------------------------
  void servo() {
    int angle = 0;
    //angle in degrees 
    for (angle = panServo.read(); angle < 180; angle += 2) {
   
        panServo.write(angle); 
        delay(100);
        d.CalculateCenter(angle, GetDistance());
    }
 
    for (angle = panServo.read(); angle > 0; angle -= 2) {
        
        panServo.write(angle);
        delay(100);
    }
}

// -----Calculates the Center

//--------------PICK-UP PACKAGE------------
void PickUpPackage() {

}
