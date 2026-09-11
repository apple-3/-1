#ifndef PIDSET_H
#define PIDSET_H
template <typename T>

class PID {
private:
  T Kp, Ki, Kd;
  T setpoint;
  T measure;
  T error;
  T integral;
  T last_error;
  T output;

public:
  PID(T Kp, T Ki, T Kd)
      : Kp(Kp), Ki(Ki), Kd(Kd), setpoint(0), measure(0), error(0), integral(0),
        last_error(0), output(0) {}

  void Set_Setpoint(T target) { setpoint = target; }
  void Set_Measure(T input) { measure = input; }
  T Get_Error() const { return error; }
  T Get_Output() const { return output; }

  T Update(T input, T dt) {
    measure = input;
    error = setpoint - measure;
    integral += error * dt;
    if (integral > 50.0f)
      integral = 50.0f;
    if (integral < -50.0f)
      integral = -50.0f;
    T derivative = (dt > 0) ? (error - last_error) / dt : 0;
    output = Kp * error + Ki * integral + Kd * derivative;
    last_error = error;
    return output;
  }

  T Update_Chain(PID<T> &inner, T outer_measure, T inner_measure, T dt) {
    T outer_out = Update(outer_measure, dt); // 外环计算
    inner.Set_Setpoint(outer_out);           // 级联给内环
    return inner.Update(inner_measure, dt);  // 内环输出
  }

  void Reset() {
    integral = 0;
    last_error = 0;
    output = 0;
  }
};

#endif // PIDSET_H