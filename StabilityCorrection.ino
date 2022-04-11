#define ThrotPin 11               //Throttle pin definition controlled by OCR1A = PB5
#define RollPin 6                 //Roll pin definition controlled by OCR4A = PH3
#define YawPin 5                  //Yaw pin definition controlled by OCR3A = PE3
#define PitPin 46                 //Pitch pin definition controlled by OCR5A = PL3

#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
int Gnd;
int AltSt;

void setup() {
  pinMode(ThrotPin, OUTPUT);  //Throttle
  pinMode(RollPin, OUTPUT);   //Roll
  pinMode(YawPin, OUTPUT);    //Yaw
  pinMode(PitPin, OUTPUT);    //Pitch
  pinMode(A0, INPUT);
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
  /*Throttle PWM settings

    Clock 1 is being used for Yaw which is represented by the numbers in each register
    Timer Counter Control Register sets the modes for PWM and clk prescaler

    1A,3A,4A,5A represent throttle,Yaw,Roll,Pitch RESPECTIVELY*/
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
  /*
     The following code is for diagnoses purpose only:
     Serial.begin(2400);
  */
  const int Gnd = GetAltitutde();
  const int AltSt = 50 + Gnd;
  //  const int AltSt = 50 + Gnd;
  //  Serial.println("Ground initial");
  //  delay(1000);
  //  FlightMode();
  //  Serial.println("FlightMode");
  //  MotorTest();
  //  Serial.println("MotorTest");
  
  Launch();
}
void loop() {
  //This code is primarily used for diagnosing the inputs for the clocks

  //This code will provide information needed to diagnose the Duty cycle inputs by OCRnA
  //It is not needed for the stability program
  /*   Serial.println("OCR1A,OCR3A, OCR4A, OCR5A: ");
      Serial.print(OCR1A);
      Serial.print(",  ");
      Serial.print(OCR3A);
      Serial.print(",  ");
      Serial.print(OCR4A);
      Serial.print(",  ");
      Serial.println(OCR5A);
  */
  //int Gnd = GetAltitude();
  //const int AltSt = 50 + Gnd;          //AltSt = Altitude at Start. 7ft = 213cm

  Elevation(GetAltitude(), 35);
}


void MotorTest() {
  for (int i = 62; i < 95; i++) {
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
    */
    delay(10);
  }
  for (int i = 95; i > 62; i--) {
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
    */
    delay(100);

  }
}

int FlightMode() {
  //FlightMode() shall be called in the setup phase of the main code so that it only runs once
  OCR3A = 125;
  delay(2000);
  OCR3A = 95;
}



/*
  AltMes = Altitude Measured by the ultrasonic sensor
  AltReq = The required; altitude needed for the task
*/

int Elevation(int AltMes, int AltReq) {

  if  (AltMes > Gnd) {
    if (OCR1A <= 105 && (AltMes < (AltReq - 2))) {
      OCR1A++;
      Serial.println("rising");
    }
    else if (OCR1A >= 85 && (AltMes > (AltReq + 2))) {
      OCR1A--;
      Serial.println("falling");
    }
  }
  else {
    OCR1A = 62;
    Serial.println("off");

  }
}

void Launch() {
  for (int i = Gnd; i++; i < AltSt) {
    Elevation(GetAltitude(), i);
  }
}

/*
   RollMes = Value of roll measured by the gyroscope
   YawMes = Value of yaw measured by the gyroscope
   PitMes = Value of pitch measured by the gyroscope
*/

int Roll(int RollMes, int RollReq) {//both should be measured in degrees
  int OcrNum = adj(RollMes, RollReq) / 16;
  OCR4A = OcrNum;
}

int Yaw(int YawMes, int YawReq) {
  int OcrNum = adj(YawMes, YawReq) / 16;
  OCR3A = OcrNum;
}

int Pitch(int PitMes, int PitReq) {
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
  Serial.print("distance  ");
  Serial.println(distance);
}
