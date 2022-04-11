#include <Wire.h>
#include <LIDARLite.h>
#include "Detect.h"
#include <Servo.h>

  Detect d = Detect();
  LIDARLite lidarStuff;
  Servo panServo;
  
  #define buzzerPin 8      // buzzer
  #define pingPin 17     // Ultrasonic sensor
  #define servoPin 4      // servo motor

 

void setup() {
  Serial.begin(9600);           // opens serial port, sets data rate to 9600 bps
  lidarStuff.begin(0, true);    // default I2C freq to 400 kHz
  pinMode(buzzerPin, OUTPUT);   // Set buzzer - pin 8 as an output
  panServo.attach(servoPin);    // We need to attach the servo to the used pin number
  lidarStuff.configure(0);      // default mode, balanced performance.
  Serial1.begin(19200);
}


  //CONSTANT VALUES
  const int OutOfRange      = 30;   //distance lidar measures but is too far away and we can ignore
  const int perimeterSides  = 10;   //length of sides of  one of the perimeters
                                  //We can also measure these sides with lidar and calculations


  //MAIN LOOP
void loop() { 
    int Command = 0;
    int  Message = Serial1.read();
    while(Message != -1 && Message != 10)
    {
      Command = Message + Command;
    }
      if( Message !=  -1){
      Serial.print(Message); Serial.print("\n");
      //start = 115+116+97+114+116+13+10
      //stop = 115+116+1111121310
    if (Command == 571)
    {
      DataCollection();  
   }
 }
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
 //trying to be consistent with units removing CM 

/*
 void GetDistance(int r[180]) {
    int meter = lidarStuff.distance(); 

    for(int l = 0; l < 180; l++)
    { 
    r [l] = meter;
    delay(100);
    }
}
*/

  //-----Servo Motor and Lidar Sensor-----
    
int angle = 0;
void DataCollection(){
    //angle in degrees 
    for (angle = 0; angle <= 180; angle += 2) {
        panServo.write(angle); 
 //       int range = lidarStuff.distance();
        delay(100);
        
    }
        //5ft range
//        if( range < 305){
 //       d.CalculateCenter(range, angle);
//       }
    for (angle = panServo.read(); angle > 0; angle -= 2) {        
        panServo.write(angle);
        delay(100);
      } 
    
}
//--------------PICK-UP PACKAGE------------
void PickUpPackage(){

}
