#include <Arduino.h>
#include <XBee.h>
#include <SoftwareSerial.h>

SoftwareSerial serial1(2, 3); // RX, TX

XBee xbee=XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();

uint8_t option = 0;
uint8_t data = 0;
uint8_t rssi = 0;

void setup() {

  Serial.begin(9600);
  serial1.begin(9600);
  xbee.setSerial(serial1);

}


void loop() {
  //Serial.println("Serial Works");
  xbee.readPacket(100);
  if (xbee.getResponse().isAvailable())
  {
    //Serial.println("available");
    if(xbee.getResponse().getApiId() == RX_64_RESPONSE || xbee.getResponse().getApiId() == RX_16_RESPONSE)
    { 
      //Serial.println("16");
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE) 
      {

        //Serial.println("16");
        xbee.getResponse().getRx16Response(rx16);
        //option = rx16.getOption();
        //data = rx16.getData(0);
        uint16_t senderShortAddress = rx16.getRemoteAddress16();

        // Serial.print(" (");
        // Serial.print(senderShortAddress);
        // Serial.println(")");

        rssi = rx16.getRssi();
        //Serial.print("data: ");Serial.println(data);
        //Serial.print("option: ");Serial.println(option);
        Serial.println(rssi);
      } 
      else 
      {
        Serial.println("64");
        xbee.getResponse().getRx64Response(rx64);
        //option = rx64.getOption();
        //data = rx64.getData(0);
        rssi = rx64.getRssi();
        Serial.println(rssi);
      }
    }
  }
}