#include "NRF24L01.hpp"
#include "FreeRTOS.h"
#include "NRF24L01_Bridge.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "spi.h"
#include "stm32_hal_legacy.h"
#include <cstdint>
#include <cstdio>
uint8_t front_password[7] = "c8suco";
uint8_t out_password[7]   = "c8suco";
static uint8_t NRF24L01_Send_ADDR[5]    = {0xF0, 0xF0, 0xF0, 0xF0, 0xF0};
static uint8_t NRF24L01_Receive_ADDR[5] = {0xF0, 0xF0, 0xF0, 0xF0, 0xF0};

// SPI 超时时间(ms)：HAL_MAX_DELAY 会让 SPI 异常时无限阻塞 Status_Task
#define NRF_SPI_TIMEOUT_MS 10U

uint8_t NRF24L01::NRF24L01_SPI_SwapByte(uint8_t Byte) {
    uint8_t receive = 0;
    // 超时/出错时返回 0，调用方读到的寄存器值即 0，
    // 不会误判 STATUS 的 RX_OK 位，最终返回 NRF24L01_ERROR
    if (HAL_SPI_TransmitReceive(&hspi1, &Byte, &receive, 1, NRF_SPI_TIMEOUT_MS) != HAL_OK) {
        return 0;
    }
    return receive;
}
void NRF24L01::NRF24L01_write_reg(uint8_t reg, uint8_t value) {
    NRF24L01_CSN_RESET;
    NRF24L01::NRF24L01_SPI_SwapByte(reg);
    NRF24L01::NRF24L01_SPI_SwapByte(value);
    NRF24L01_CSN_SET;
}
uint8_t NRF24L01::NRF24L01_read_reg(uint8_t reg) {
    uint8_t value;
    NRF24L01_CSN_RESET;
    NRF24L01::NRF24L01_SPI_SwapByte(reg);
    value = NRF24L01::NRF24L01_SPI_SwapByte(NRF24L01_NOP);
    NRF24L01_CSN_SET;
    return value;
}
void NRF24L01::NRF24L01_write_buf(uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t i;
    NRF24L01_CSN_RESET;
    NRF24L01::NRF24L01_SPI_SwapByte(reg);
    for (i = 0; i < len; i++)
        NRF24L01::NRF24L01_SPI_SwapByte(buf[i]);
    NRF24L01_CSN_SET;
}
void NRF24L01::NRF24L01_read_buf(uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t i;
    NRF24L01_CSN_RESET;
    NRF24L01::NRF24L01_SPI_SwapByte(reg);
    for (i = 0; i < len; i++)
        buf[i] = NRF24L01::NRF24L01_SPI_SwapByte(NRF24L01_NOP);
    NRF24L01_CSN_SET;
}
void NRF24L01::NRF24L01_Init(void) {
    NRF24L01_CE_RESET;
    NRF24L01::NRF24L01_write_buf(NRF24L01_W_REGISTER + NRF24L01_TX_ADDR, NRF24L01_Send_ADDR, 5);
    NRF24L01::NRF24L01_write_buf(NRF24L01_W_REGISTER + NRF24L01_RX_ADDR_P0, NRF24L01_Receive_ADDR, 5);
    NRF24L01::NRF24L01_write_reg(NRF24L01_W_REGISTER + NRF24L01_CONFIG, 0x0F);
    NRF24L01::NRF24L01_write_reg(NRF24L01_W_REGISTER + NRF24L01_EN_AA, 0x00);
    NRF24L01::NRF24L01_write_reg(NRF24L01_W_REGISTER + NRF24L01_RF_CH, 0x00);
    NRF24L01::NRF24L01_write_reg(NRF24L01_W_REGISTER + NRF24L01_RX_PW_P0, NRF24L01_Buf_Len);
    NRF24L01::NRF24L01_write_reg(NRF24L01_W_REGISTER + NRF24L01_EN_RXADDR, 0x01);
    NRF24L01::NRF24L01_write_reg(NRF24L01_W_REGISTER + NRF24L01_SETUP_RETR, 0x1A);
    NRF24L01::NRF24L01_write_reg(NRF24L01_FLUSH_RX, NRF24L01_NOP);
    NRF24L01_CE_SET;
}
uint8_t NRF24L01::NRF24L01_receive(uint8_t *buf) {
    uint8_t Status = NRF24L01_read_reg(NRF24L01_STATUS);
    if (Status & NRF24L01_RX_OK) {
        NRF24L01_read_buf(NRF24L01_R_RX_PAYLOAD, buf, NRF24L01_Buf_Len);
        NRF24L01_write_reg(NRF24L01_FLUSH_RX, NRF24L01_NOP);
        NRF24L01_write_reg(NRF24L01_W_REGISTER + NRF24L01_STATUS, NRF24L01_RX_OK);
        return NRF24L01_RX_OK;
    }
    return NRF24L01_ERROR;
}
uint8_t NRF24L01::NRF24L01_send(uint8_t *buf) {
    NRF24L01_CE_RESET;
    uint8_t Status;
    NRF24L01::NRF24L01_write_buf(NRF24L01_W_TX_PAYLOAD, buf, NRF24L01_Buf_Len);
    NRF24L01::NRF24L01_write_reg(NRF24L01_W_REGISTER + NRF24L01_CONFIG, 0x0E);
    NRF24L01_CE_SET;
    Status = NRF24L01::NRF24L01_read_reg(NRF24L01_R_REGISTER + NRF24L01_STATUS);
    if (Status & NRF24L01_TX_OK) {
        return NRF24L01_TX_OK;
    }
    return 0;
}