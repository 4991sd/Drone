#define Rxd 25  // pin RX on arduino
#define Txd 33  //pin tx on arduino

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
 SerialBT.begin("David's Drone"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial2.begin(19200, SERIAL_8N1, Rxd, Txd);
}

void loop() {
 
 if (SerialBT.available()) {
int  Message = SerialBT.read();
     Serial.write(Message);
     Serial2.write(Message);
     delay(20);
  }
}
