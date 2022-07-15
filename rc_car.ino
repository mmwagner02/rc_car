#include <RH_NRF24.h>
#include <SPI.h>
#include "RCCAR.h"

//#define DEBUG
//#define STATUS

#define HORN_PIN 2
#define RIGHT_SPEED_PIN 3
#define RIGHT_CTRLA_PIN 4
#define RIGHT_CTRLB_PIN 5
#define LEFT_CTRLA_PIN 6
#define LEFT_CTRLB_PIN 7
#define LEFT_SPEED_PIN 9

const uint16_t mid = 512;
const uint16_t hrt = 520; // hi read threshold
const uint16_t lrt = 500; // lo read threshold

RH_NRF24 driver;
uint8_t msg[MSG_LENGTH];
uint8_t bytesCopied;

void setup () { 
  pinMode(RIGHT_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_CTRLA_PIN, OUTPUT);
  pinMode(RIGHT_CTRLB_PIN, OUTPUT);
  pinMode(LEFT_CTRLA_PIN, OUTPUT);
  pinMode(LEFT_CTRLB_PIN, OUTPUT);
  pinMode(LEFT_SPEED_PIN, OUTPUT);
  pinMode(HORN_PIN, OUTPUT);

  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  
  if (!driver.init()) {
    #ifdef DEBUG
    Serial.println("Failed to initialize radio receiver");
    #endif
    resetFunc();
  }
  driver.setChannel(RF_CHANNEL);
  driver.setRF(RF_DATA_RATE, RF_POWER);
  driver.setNetworkAddress(RF_NET_ADDR, sizeof(RF_NET_ADDR));
  #ifdef DEBUG
  Serial.println ("done booting!");
  #endif
}

long elapsedms = 0;
long delayms = 50;

void loop () {
  #ifdef STATUS
  sendStatus();
  #endif
  bytesCopied = MSG_LENGTH;
  if (! driver.recv(msg, &bytesCopied) ) {
    elapsedms+=delayms;
    #ifdef DEBUG
    if (elapsedms % 1000 == 0) Serial.println(elapsedms);
    #endif
    if (elapsedms > 5000) resetFunc();
    delay(delayms);

    return;
  }
  elapsedms = 0;
//  #ifdef DEBUG
//  printMsg(msg, bytesCopied);
//  #endif
  
  uint16_t xread = word(msg[0], msg[1]);
  uint16_t yread = word(msg[2], msg[3]);
  boolean  bread = msg[4];
  #ifdef DEBUG
  Serial.print  (xread); Serial.print(" ");
  Serial.print  (yread); Serial.print(" ");
  Serial.println (bread);
  #endif
  digitalWrite(HORN_PIN,bread);

  int16_t turnFactor = map(yread, 0, 1023, -200, 200);
  if (abs(turnFactor) < 5) turnFactor = 0;


  uint8_t motorSpeed;
  bool halt = false;
  if (xread > hrt) { //forwards
    motorSpeed = map(xread, mid, 1023, 0, 255);
    digitalWrite(LEFT_CTRLA_PIN, LOW);
    digitalWrite(LEFT_CTRLB_PIN, HIGH);
    digitalWrite(RIGHT_CTRLA_PIN, LOW);
    digitalWrite(RIGHT_CTRLB_PIN, HIGH);
  } else if (xread < lrt) {
    motorSpeed = map(xread, mid, 0, 0, 255);
    digitalWrite(LEFT_CTRLA_PIN, HIGH);
    digitalWrite(LEFT_CTRLB_PIN, LOW);
    digitalWrite(RIGHT_CTRLA_PIN, HIGH);
    digitalWrite(RIGHT_CTRLB_PIN, LOW);
  } else {
    halt = true;
    motorSpeed = 0;
    digitalWrite(LEFT_CTRLA_PIN, LOW);
    digitalWrite(LEFT_CTRLB_PIN, LOW);
    digitalWrite(RIGHT_CTRLA_PIN, LOW);
    digitalWrite(RIGHT_CTRLB_PIN, LOW);
  }
  if (!halt) {
    analogWrite(LEFT_SPEED_PIN,  constrain(motorSpeed - turnFactor, 50, 255));
    analogWrite(RIGHT_SPEED_PIN, constrain(motorSpeed + turnFactor, 50, 255));
  }
    #ifdef DEBUG
  Serial.print ("turn Factor "); Serial.println(turnFactor);
  Serial.print ("motor Speed "); Serial.println(motorSpeed);
  #endif
  #ifdef DEBUG
  delay(500);
  #endif
  return;
}

void sendStatus() {
  driver.send(&STATUS_OK,(uint8_t)1);
  #ifdef DEBUG
  Serial.println ("Sent status message");
  #endif
}
