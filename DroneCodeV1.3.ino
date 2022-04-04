/*
  FUNCTIONS TO-DO LIST
  ----------------------------
  -----------------------------
  SENSORS (declare pins and create functions if needed)
  ------------
  BUZZER (DONE)
  SERVO  (DONE)
  GYRO (Done)
  Ultrasonic Sensor (DONE)
  LIDAR (Done)

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

#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Detect.h"


MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2

//Declaration of Pins
Detect d = Detect();
LIDARLite lidarStuff;
Servo panServo;

struct Var {

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
  double markerX[4] = {0, 0, 0, 0};
  double markerY[4] = {0, 0, 0, 0};


};


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
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  lidarStuff.begin(0, true); // default I2C freq to 400 kHz
  pinMode(buzzerPin, OUTPUT); // Set buzzer - pin 8 as an output
  panServo.attach(servoPin); // We need to attach the servo to the used pin number
  lidarStuff.configure(0); // default mode, balanced performance.
  initialize_MPU();
}

//CONSTANT VALUES
const int OutOfRange      = 30;   //distance lidar measures but is too far away and we can ignore
const int perimeterSides  = 10;   //length of sides of  one of the perimeters



// ================================================================
// ===               Main Loop                          ===
// ================================================================
void loop() {

  read_MPU();//read gyro values


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

void read_MPU() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("Yaw:");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print(" Pitch:");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print(" Roll:");
    Serial.println(ypr[2] * 180 / M_PI);
    delay(10);
#endif
  }

}

// ================================================================
// ===               AutoLevel      (IN PROGRESS)         ===
// ================================================================
void AutoLevel() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  //if pitch angles are greater or less than zero then adjust
if(ypr[1] >.5 && ypr[1] < -0.5){
  //send pwm for adjusting pitch
}
if(ypr[2] >.5 && ypr[2] < -0.5){
  //send pwm for adjusting roll
}
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
    d.CalculateCenter(angle, GetDistance());
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

//void CalculateCenter(){
//int OutOfRange = 30; //distance lidar measures but is too far away and we can ignore
//int perimeterSides= 10; //length of sides of  one of the perimeters
//We can also measure these sides with lidar and calculations
//  double marker1,marker2,marker3,ang1,ang2,ang3,side1,side2,center;
//
//  if (GetDistance() < OutOfRange){
//
//  }
//
//
//
//}



//--------------PICK-UP PACKAGE------------
void PickUpPackage() {

}
