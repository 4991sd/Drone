#include "WiFi.h"

#define Tx 33 // connect to RX1 
#define Rx 25 // connect to Tx1


const char* ssid = "Tello";
const char* password = "3754857706";

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());}

void loop() {
  // put your main code here, to run repeatedly:

}
