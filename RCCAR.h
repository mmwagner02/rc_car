#ifndef RCCAR_H
#define RCCAR_H
const uint8_t STATUS_OK  = 128;
const uint8_t RF_CHANNEL = 100;
uint8_t RF_NET_ADDR[] = { 19, 87, 5, 20, 2 };

const RH_NRF24::DataRate  RF_DATA_RATE = RH_NRF24::DataRate2Mbps;
const RH_NRF24::TransmitPower  RF_POWER = RH_NRF24::TransmitPower0dBm;

const uint8_t MSG_LENGTH = 5;

void(* resetFunc) (void) = 0;
#endif
