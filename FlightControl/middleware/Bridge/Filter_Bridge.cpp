#include "Filter_Bridge.h"
#include "Com_Filter.hpp"
static Filter filter;
extern "C" {
float First_order_Lowpass_Filter(float input,float last){
    return filter.First_order_Lowpass_Filter(input,last);
}
float Kalman_Filter(float input,float last){
    return filter.Kalman_Filter(input,last);
}
}
