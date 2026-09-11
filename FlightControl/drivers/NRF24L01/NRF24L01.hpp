#ifndef NRF24L01_H
#define NRF24L01_H
#include "spi.h"
#include <cstdint>
#define NRF24L01_CSN_SET HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET)
#define NRF24L01_CSN_RESET HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET)
#define NRF24L01_CE_SET HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET)
#define NRF24L01_CE_RESET HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET)

#define NRF24L01_CONFIG 0x00
#define NRF24L01_EN_AA 0x01
#define NRF24L01_EN_RXADDR 0x02
#define NRF24L01_SETUP_AW 0x03
#define NRF24L01_SETUP_RETR 0x04
#define NRF24L01_RF_CH 0x05
#define NRF24L01_RF_SETUP 0x06
#define NRF24L01_STATUS 0x07
#define NRF24L01_OBSERVE_TX 0x08
#define NRF24L01_CD 0x09
#define NRF24L01_RX_ADDR_P0 0x0A
#define NRF24L01_RX_ADDR_P1 0x0B
#define NRF24L01_RX_ADDR_P2 0x0C
#define NRF24L01_RX_ADDR_P3 0x0D
#define NRF24L01_RX_ADDR_P4 0x0E
#define NRF24L01_RX_ADDR_P5 0x0F
#define NRF24L01_TX_ADDR 0x10
#define NRF24L01_RX_PW_P0 0x11
#define NRF24L01_RX_PW_P1 0x12
#define NRF24L01_RX_PW_P2 0x13
#define NRF24L01_RX_PW_P3 0x14
#define NRF24L01_RX_PW_P4 0x15
#define NRF24L01_RX_PW_P5 0x16
#define NRF24L01_FIFO_STATUS 0x17
#define NRF24L01_DYNPD 0x1C
#define NRF24L01_FEATURE 0x1D

// 操作指令代码
#define NRF24L01_R_REGISTER 0x00
#define NRF24L01_W_REGISTER 0x20
#define NRF24L01_R_RX_PAYLOAD 0x61
#define NRF24L01_W_TX_PAYLOAD 0xA0
#define NRF24L01_FLUSH_TX 0xE1
#define NRF24L01_FLUSH_RX 0xE2
#define NRF24L01_NOP 0xFF

// 状态
#define NRF24L01_RX_OK 0x40
#define NRF24L01_TX_OK 0x20
#define NRF24L01_MAX_TX 0x10

#define NRF24L01_Buf_Len 24

class NRF24L01 {
  public:
    void    NRF24L01_Init(void);
    uint8_t NRF24L01_receive(uint8_t *buf);
    uint8_t NRF24L01_send(uint8_t *buf);

  private:
    uint8_t NRF24L01_SPI_SwapByte(uint8_t Byte);
    void    NRF24L01_write_reg(uint8_t reg, uint8_t value);
    uint8_t NRF24L01_read_reg(uint8_t reg);
    void    NRF24L01_write_buf(uint8_t reg, uint8_t *buf, uint8_t len);
    void    NRF24L01_read_buf(uint8_t reg, uint8_t *buf, uint8_t len);
};

#endif // NRF24L01_H