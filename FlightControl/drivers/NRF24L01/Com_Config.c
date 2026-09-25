#include "Com_Config.h"
#include "NRF24L01_Bridge.h"
extern uint8_t Buf[NRF24L01_Buf_Len];
void Com_Limit(float *inner, float Max, float Min) {
  if (*inner > Max) {
    *inner = Max;
  }
  if (*inner < Min) {
    *inner = Min;
  }
}
void Com_NRF_Access(Remote_data *data,uint8_t *Buf) {
  data->pit = (uint16_t)(Buf[6] | (Buf[7] << 8));
  data->rol = (uint16_t)(Buf[8] | (Buf[9] << 8));
  data->thr = (uint16_t)(Buf[10] | (Buf[11] << 8));
  data->yaw = (uint16_t)(Buf[12] | (Buf[13] << 8));
  data->shortdown = Buf[14];
  data->fix_high = Buf[15];
  data->calibrate = Buf[16];
}
