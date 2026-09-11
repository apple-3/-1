#ifndef COM_FILTER_H
#define COM_FILTER_H

#include <cmath>
class Filter {
public:
  Filter();
  float First_order_Lowpass_Filter(float input, float last);
  float Kalman_Filter(float input, float last);

private:
  float Kalman_kernal[3][3];
  float P; 
};

#endif // COM_FILTER_H