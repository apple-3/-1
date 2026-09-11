#ifndef IIMU_H
#define IIMU_H

#include <stdint.h>
struct imusample {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t dt;
};

class IImu {
public:
  virtual ~IImu() = default;
  virtual bool read(imusample &imu) = 0;
  virtual bool init() = 0;
};

#endif // IIMU_H