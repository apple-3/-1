#ifndef FILTER_BRIDGE_H
#define FILTER_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

float First_order_Lowpass_Filter(float input, float last);
float Kalman_Filter(float input, float last);

#ifdef __cplusplus
}

#endif

#endif // FILTER_BRIDGE_H