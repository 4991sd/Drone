#define Rxd 16
#define Txd 17

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
 SerialBT.begin("David's Drone"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial2.begin(9600, SERIAL_8N1, Rxd, Txd);
}

void loop() {
 
 if (SerialBT.available()) {
int  Message = SerialBT.read();
     Serial.write(Message);
     Serial2.write(Message);
     delay(20);
  }
}
