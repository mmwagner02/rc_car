#include <SPI.h>
#include <RH_NRF24.h>
#include "RCCAR.h"
//#define DEBUG
//#define STATUS

const int xpin = A4;
const int ypin = A5;
const int bpin = 4;

RH_NRF24 driver;//(9,10);

boolean carStatus = false;
long carStatusTime = 0;
const int carStatusPin = 2;

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println ("boot! 1");
  #endif
  pinMode(xpin, INPUT);
  pinMode(ypin, INPUT);
  pinMode(bpin, INPUT_PULLUP);
  #ifdef STATUS
  pinMode(carStatusPin, OUTPUT);
  #endif
  if (!driver.init()) {
    #ifdef DEBUG
    Serial.println("Failed to initialize radio transmitter");
    #endif
    resetFunc(); 
  }
  driver.setChannel(RF_CHANNEL);
  driver.setRF(RF_DATA_RATE, RF_POWER);
  driver.setNetworkAddress(RF_NET_ADDR, sizeof(RF_NET_ADDR));
  #ifdef DEBUG
  Serial.println ("boot! 2");
  #endif
}
boolean buttonRead(const int pin) {
//  return (analogRead(pin) == 0);
    return !digitalRead(pin);
}
/*
 * Loop!
 */
void loop() {
  uint8_t  msg[MSG_LENGTH];
  uint16_t xread = analogRead(xpin);
  uint16_t yread = analogRead(ypin);
  boolean  bread = buttonRead(bpin);
  #ifdef DEBUG
  Serial.print(xread); Serial.print(" ");
  Serial.print(yread); Serial.print(" ");
  Serial.print(bread); Serial.println(" ");
  #endif
  msg[0] = highByte(xread);msg[1] = lowByte(xread);
  msg[2] = highByte(yread);msg[3] = lowByte(yread);
  msg[4] = (uint8_t)bread;
  driver.send(msg, (uint8_t)sizeof(msg));

  #ifdef STATUS
  recvStatus();
  #endif
  #ifdef DEBUG
  delay(1000);
  #endif
}


/*
 * Try to read the status of the car
 */
#ifdef STATUS
void recvStatus() {
  uint8_t msgSize = 1;
  uint8_t statusMsg[msgSize];
  if (driver.recv(statusMsg, &msgSize)) {
    if (msgSize == 1 && statusMsg[0] == STATUS_OK) {
      digitalWrite(carStatusPin, HIGH);
      carStatusTime = millis();
      #ifdef DEBUG
      Serial.println("We have heard from the car");
      #endif
    }
  }
  #ifdef DEBUG
  else {
    Serial.println("No status available");
  }
  #endif
  if (millis()-carStatusTime > 5000) {
    digitalWrite(carStatusPin, LOW);
    #ifdef DEBUG
    Serial.println("We have not heard from the car");
    #endif
  }
}
#endif
