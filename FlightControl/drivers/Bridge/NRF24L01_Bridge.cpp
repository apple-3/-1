#include "NRF24L01_Bridge.h"
#include "NRF24L01.hpp"
#include <cstdint>
#include <stdint.h>
static NRF24L01 nrf;//单例模式
extern "C" {

void NRF_Init(void) { nrf.NRF24L01_Init(); }
uint8_t NRF_Send(uint8_t *data) {
  uint8_t Status = nrf.NRF24L01_send(data);
  return Status;
}
uint8_t NRF_Receive(uint8_t *data) { return nrf.NRF24L01_receive(data); }
}