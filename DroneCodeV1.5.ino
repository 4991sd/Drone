
#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
LIDARLite lidarStuff;
Servo panServo;


//Declaration of Pins
#define buzzerPin 8  // buzzer
#define servoPin 4  // servo motor
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
#define ThrotPin 11               //Throttle pin definition controlled by OCR1A = PB5
#define RollPin 6                 //Roll pin definition controlled by OCR4A = PH3
#define YawPin 5                  //Yaw pin definition controlled by OCR3A = PE3
#define PitPin 46                 //Pitch pin definition controlled by OCR5A = PL3
#define echoPin 22 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 23 //attach pin D3 Arduino to pin Trig of HC-SR04


//CONSTANT VALUES
const int OutOfRange      = 30;   //distance lidar measures but is too far away and we can ignore
const int perimeterSides  = 10;   //length of sides of  one of the perimeters
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
int Gnd;
int AltSt;
bool LandFlag;
bool LaunchFlag;

// ================================================================
// ===               MPU control/status vars               ===
// ================================================================
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
//-----------------------------------------------------------------------------------



void setup() {
  Wire.begin();
  pinMode(ThrotPin, OUTPUT);  //Throttle
  pinMode(RollPin, OUTPUT);   //Roll
  pinMode(YawPin, OUTPUT);    //Yaw
  pinMode(PitPin, OUTPUT);    //Pitch
  pinMode(A0, INPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  lidarStuff.begin(0, true); // default I2C freq to 400 kHz
  pinMode(buzzerPin, OUTPUT); // Set buzzer - pin 8 as an output
  panServo.attach(servoPin); // We need to attach the servo to the used pin number
  lidarStuff.configure(0); // default mode, balanced performance.


  //Clock 1 is being used for Yaw which is represented by the numbers in each register
  //  Timer Counter Control Register sets the modes for PWM and clk prescaler

  //1A,3A,4A,5A represent throttle,Yaw,Roll,Pitch RESPECTIVELY*/
  TCCR1A = TCCR3A = TCCR4A = TCCR5A = 0b10000010;
  /*Timer Counter Control Register sets the modes for PWM and clk prescaler
    1B,3B,4B,5B represent throttle,Yaw,Roll,Pitch RESPECTIVELY*/
  TCCR1B = TCCR3B = TCCR4B = TCCR5B = 0b00011100;
  /*Input Capture Register which controls the frequency of the signal
      1H,3H,4H,5H represent throttle,Yaw,Roll,Pitch RESPECTIVELY*/
  ICR1H = ICR3H = ICR4H = ICR5H = 0b00000100;
  /*Input Capture Register which controls the frequency of the signal
    1L,3L,4L,5L represent throttle,Yaw,Roll,Pitch RESPECTIVELY*/
  ICR1L = ICR3L = ICR4L = ICR5L = 0b01100101;
  OCR1A = 62;
  /*Output Compare Register controls the PWM duty cycle. 62 represents 5.5% in this case.
    This simulates the throttle at bottom of the remote control.
    Clock 3 is being used for Throttle which is represented by the numbers in each register
    3A, 4A, 5A represent Yaw, Roll, Pitch RESPECTIVELY
  */
  OCR3A = OCR4A =  OCR5A = 94; //PWM duty cycle of 8.33% for neutral position

  initialize_MPU();
  Gnd = GetAltitude();
  Serial.print("Gnd:  ");
  Serial.println(Gnd);
  AltSt = 80 + Gnd;
  delay(1000);
  Serial.print("throttle: ");
  Serial.println(OCR1A);
  FlightMode();
  //buzzer();
  Serial.println("---------------------Launch Started---------------------");
  LandFlag = false;
  LaunchFlag = true;
  Launch();
  LaunchFlag = false;
  Serial.println("---------------------Launch Ended---------------------");
  LandFlag = true;
  Serial.println("---------------------Land Started---------------------");
  Land(GetAltitude());
  Serial.println("---------------------Land Ended---------------------");
  //  Elevation(GetAltitude(), 80);

  //AltSt = Altitude at Start. 7ft = 213cm
}




// ================================================================
// ===               Main Loop                          ===
// ================================================================
void loop() {

  //read_gyro('s');

  //AutoLevel();

  // MotorTest();



}

void Launch() {
  LaunchFlag = true;
  Serial.println("Launching");

  for (float i = OCR1A; i < 95; i = i + .1) {
    if (GetAltitude() <= Gnd) {
      OCR1A = round(i);
      Serial.println("Launching First Loop");
      Serial.println("rising");
      Serial.print("OCR1A = ");
      Serial.println(OCR1A);
      delay(40);
    }
    else {
      break;
    }
  }
  for (float i = GetAltitude(); i <= AltSt; i = i + .1) {
    Elevation(GetAltitude(), round(i));
    //AutoLevel(0);
    Serial.println("Launching Second Loop");
    Serial.print("i = ");
    Serial.println(i);
    Serial.print("OCR1A = ");
    Serial.println(OCR1A);
    delay(40);
  }


void Land(int AltMes) {

  LandFlag = true;
  if (AltMes > Gnd) {
    for (float i = GetAltitude(); i > Gnd; i = i - .1) {
      Elevation(GetAltitude(), round(i));
      //AutoLevel(0);
      Serial.println("Landing Loop");
      Serial.print("OCR1A = ");
      Serial.println(OCR1A);
      //    if (AltMes <= Gnd) {
      //      OCR1A = 62;
      //    }
    }
  }
  else {
    OCR1A = 62;
    Serial.print("Off");
  }
}
  
/*
  AltMes = Altitude Measured by the ultrasonic sensor
  AltReq = The required; altitude needed for the task
*/

int Elevation(int AltMes, int AltReq) {

  if (OCR1A > 85 && OCR1A < 105 && AltMes > Gnd) {// && LaunchFlag == false && LandFlag == false
    if (AltMes < (AltReq - 2)) {
      OCR1A++;
      Serial.print("Distance = ");
      Serial.print(GetAltitude());
      Serial.println("OCR1A rising");
    }
    else if (AltMes > (AltReq + 2)) {
      OCR1A--;
      Serial.print("Distance = ");
      Serial.print(GetAltitude());
      Serial.println("OCR1A falling");
    }
    //    else if (AltMes <= Gnd) {
    //
    //      OCR1A = 62;
    //    }
    //    else {
    //      OCR1A = 90;
    //    }
  }
  if ((OCR1A <= 85 || OCR1A >= 105 ) && AltMes > Gnd) {
    OCR1A = 86;
  }
  else if (OCR1A <= 85 && LandFlag == false && LaunchFlag == false) {
    OCR1A++;
    Serial.println("rising");
    Serial.print("Distance = ");
    Serial.print(GetAltitude());
    Serial.print("OCR1A = ");
    Serial.println(OCR1A);
  }

  else if (LandFlag == true && AltMes <= Gnd) {
    OCR1A = 62;
    Serial.print("OCR1A = ");
    Serial.println(OCR1A);
    Serial.println("Off");
  }
}

void MotorTest() {
  for (int i = 62; i < 70; i++) {
    OCR1A = i;
    /*
      Serial.println("OCR1A,OCR3A, OCR4A, OCR5A: ");
      Serial.print(OCR1A);
      Serial.print(",  ");
      Serial.print(OCR3A);
      Serial.print(",  ");
      Serial.print(OCR4A);
      Serial.print(",  ");
      Serial.println(OCR5A);
      delay(10);
      /
      delay(10);
      }
      for (int i = 95; i > 62; i--) {
      OCR1A = i;
      /
      Serial.println("OCR1A,OCR3A, OCR4A, OCR5A: ");
      Serial.print(OCR1A);
      Serial.print(",  ");
      Serial.print(OCR3A);
      Serial.print(",  ");
      Serial.print(OCR4A);
      Serial.print(",  ");
      Serial.println(OCR5A);
      delay(10);
    */
    delay(100);

  }
}


void initialize_MPU() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

}



int FlightMode() {
  //FlightMode() shall be called in the setup phase of the main code so that it only runs once
  OCR3A = 125;
  delay(2000);
  OCR3A = 95;


}

/*
   RollMes = Value of roll measured by the gyroscope
   YawMes = Value of yaw measured by the gyroscope
   PitMes = Value of pitch measured by the gyroscope
*/

void Roll(float RollMes, int RollReq) {//both should be measured in degrees
  int OcrNum = adj(RollMes, RollReq) / 16;
  OCR4A = OcrNum;
}

void Yaw(int YawMes, int YawReq) {
  int OcrNum = adj(YawMes, YawReq) / 16;
  OCR3A = OcrNum;
}

void Pitch(float PitMes, int PitReq) {
  int OcrNum = adj(PitMes, PitReq) / 16;
  OCR5A = OcrNum;
}

/*
   MesGyro = Measured Value
   alpha = Required Value
   both should be measured in degrees
*/
int adj(int Mes, int alpha) {
  int PW_1 = alpha * 15 + 1500;    //Required PW signal to obtain alpha
  int PW_2 = Mes * 15;             //Change in PW required for maintaining alpha
  int pwAdj = PW_1 - PW_2;         //Total PW signal for alpha including changes in direction
  return pwAdj;
}
float ReadRoll() {
  return read_gyro('r');
}
float ReadPitch() {
  return read_gyro('p');
}
float ReadYaw() {
  return read_gyro('y');
}


float read_gyro(char yprangle) {
  char YPR = yprangle;
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    if (YPR == 'y') { //Read Yaw angle
      return ypr[0] * 180 / M_PI;
    }
    else if (YPR == 'p') { //Read Pitch angle
      return ypr[1] * 180 / M_PI;
    }
    else if (YPR == 'r') { //Read Roll angle
      return ypr[2] * 180 / M_PI;
    } else {
      //Serial.print("Error");

      Serial.print("Yaw:");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print(" Pitch:");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print(" Roll:");
      Serial.println(ypr[2] * 180 / M_PI);
      delay(10);
    }
#endif
  }

}


// ================================================================
// ===               AutoLevel      (IN PROGRESS)         ===
// ================================================================
void AutoLevel(int alpha) {
  Roll(ReadRoll(), alpha);
  Serial.print("Roll: ");
  Serial.println(OCR4A);
  Yaw(ReadYaw(), alpha);
  Serial.print("Yaw: ");
  Serial.println(OCR3A);
  Pitch(ReadPitch(), alpha);
  Serial.print("Pitch: ");
  Serial.println(OCR5A);

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
    //Serial.println(" cm"); //trying to be consistent with units removing CM
  }
  return  meter;
}

//-------The buzzer sensor is used to alert when Package is Located-----------
void buzzer() {
  tone(buzzerPin, 1000); // Send 1KHz sound signal...
  delay(500);        // ...for 1/2 sec
  noTone(buzzerPin);     // Stop sound...
  delay(500);        // ...for 1/2 sec
}


// ==========================================
// ===               Servo               ===
// ===========================================
void servo() {
  int angle = 0;
  //angle in degrees
  for (angle = panServo.read(); angle < 180; angle += 2) {

    panServo.write(angle);
    delay(100);
    //d.CalculateCenter(angle, GetDistance());
  }

  for (angle = panServo.read(); angle > 0; angle -= 2) {

    panServo.write(angle);
    delay(100);
  }
}

// ==========================================
// ===  Ultrasonic sensor for altitude    ===
// ===========================================
int GetAltitude() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  return distance;
}


//--------------PICK-UP PACKAGE------------
//void PickUpPackage() {
//
//}
