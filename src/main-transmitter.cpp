#include <Arduino.h>
#include <XBee.h>
#include <SoftwareSerial.h>

SoftwareSerial serial1(2, 3); // RX, TX

XBee xbee=XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();

uint8_t data[] = {0, 0};
Tx16Request tx = Tx16Request(0x0001, data, sizeof(data));

void setup() {

  Serial.begin(9600);
  serial1.begin(9600);
  xbee.setSerial(serial1);

}


void loop() {
  xbee.send(tx);
  delay(30);
}