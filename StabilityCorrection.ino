#define ThrotPin 11               //Throttle pin definition controlled by OCR1A = PB5
#define RollPin 6                 //Roll pin definition controlled by OCR4A = PH3
#define YawPin 5                  //Yaw pin definition controlled by OCR3A = PE3
#define PitPin 46                 //Pitch pin definition controlled by OCR5A = PL3



void setup() {
  pinMode(ThrotPin, OUTPUT);  //Throttle
  pinMode(RollPin, OUTPUT);   //Roll
  pinMode(YawPin, OUTPUT);    //Yaw
  pinMode(PitPin, OUTPUT);    //Pitch
  pinMode(A0, INPUT);

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


  OCR1A = 62;/*Output Compare Register controls the PWM duty cycle. 62 represents 5.5% in this case.
               This simulates the throttle at bottom of the remote control.



     Clock 3 is being used for Throttle which is represented by the numbers in each register
     3A,4A,5A represent Yaw,Roll,Pitch RESPECTIVELY
*/
  OCR3A = OCR4A =  OCR5A = 94; //PWM duty cycle of 8.33% for neutral position
  /*
     The following code is for diagnoses purpose only:
     Serial.begin(2400);
  */

}
void loop() {
  //This code is primarily used for diagnosing the inputs for the clocks
  /*
    This code will provide information needed to diagnose the Duty cycle inputs by OCRnA
    It is not needed for the stability program

    Serial.println("OCR1A,OCR3A, OCR4A, OCR5A: ");
    Serial.print(OCR1A);
    Serial.print(",  ");
    Serial.print(OCR3A);
    Serial.print(",  ");
    Serial.print(OCR4A);
    Serial.print(",  ");
    Serial.println(OCR5A);

  */
}


int FlightMode() {                  //FlightMode() shall be called in the setup phase of the main code so that it only runs once
  delay(2000);
  OCR3A = 125;
  delay(3000);
  OCR3A = 94;
}

/*
  AltMes = Altitude Measured by the ultrasonic sensor
  AltReq = The required altitude needed for the task
*/

int Launch(int AltMes) {
  const int Gnd = 0;                //Gnd = ground level and shall be a constant that is measured when the drone is positioned on the ground
  const int AltSt = 213;            //AltSt = Altitude at Start. 7ft = 213cm
  if (AltMes < Gnd) {
    while (AltSt > AltMes) {
      if (OCR1A < 94) {
        OCR1A = OCR1A + 3;
        delay(1000);
      }
      else {
        OCR1A = OCR1A++;
        delay(1500);
      }
    }
  }
  return OCR1A;
}


int Elevation(int AltMes, int AltReq) {
  if (AltMes < AltReq) {
    OCR1A++;
    delay(250);
  }
  else {
    OCR1A--;
    delay(250);
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
   Mes = Measured Value
   alpha = Required Value
   both should be measured in degrees
*/
int adj(int Mes, int alpha) {
  int PW_1 = alpha * 15 + 1500;    //Required PW signal to obtain alpha
  int PW_2 = Mes * 15;             //Change in PW required for maintaining alpha
  int pwAdj = PW_1 - PW_2;         //Total PW signal for alpha including changes in direction
  return pwAdj;
}
