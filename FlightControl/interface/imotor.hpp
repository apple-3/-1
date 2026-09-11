#ifndef IMOTOR_H
#define IMOTOR_H

class imotor {
public:
  virtual ~imotor() = default;
  virtual bool init() = 0;
  virtual bool stop() = 0;
};

#endif // IMOTOR_H