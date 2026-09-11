#include "Com_Filter.hpp"
Filter::Filter() : P(1.0f) {}

float Filter::First_order_Lowpass_Filter(float input, float last) {
  float alpha = 0.15f;
  return alpha * input + (1.0f - alpha) * last;
}

float Filter::Kalman_Filter(float input, float last) {
  float Q = 0.01f;
  float R = 0.1f;
  float K = P / (P + R);
  float estimate = last + K * (input - last);
  P = (1.0f - K) * P + Q;
  return estimate;
}
