#include <math.h>
class PID
{
private:
  float min_val_, max_val_;
  float KP, KI, KD;
  float kp, kpT,
      ki, kiT, kd, kdT;
  float encPrev;
  float angular_vel_Prev;
  float error_integral;
  float error_previous;
  float pwm_max = 255;

  float total_gear_ratio = 19.2;
  float enc_ppr = 200;

  float angular_vel_Filt = 0;
  float angular_vel = 0;
  float eIntegral = 0;
  float prevError = 0;
  float lowpass_filt = 0;
  float lowpass_prev = 0;
  float radian = 0;
  float eProportional;

public:

  PID(float MIN_VAL, float MAX_VAL, float kp_, float ki_, float kd_) : min_val_(MIN_VAL),
                                                                       max_val_(MAX_VAL),
                                                                       KP(kp_),
                                                                       KI(ki_),
                                                                       KD(kd_)
  {
    // encPrev = 0.0;
    // angular_vel_Prev = 0.0;
    // angular_vel_Filt = 0.0;
    // error_integral = 0.0;
    // error_previous = 0.0;
  }
  void paramater(float kp_, float ki_, float kd_){
    kp = kp_;
    ki = ki_;
    kd = kd_;
  }
  void paramaterT(float kp_, float ki_, float kd_){
    kpT = kp_;
    kiT = ki_;
    kdT = kd_;
  }

  float control_angle(float target, float enc, float deltaT)
  {
    float error = target - enc;
    float error_trans = error;
    if (error_trans > 180)
    {
      error_trans -= 360;
    }
    else if (error_trans < -180)
    {
      error += 360;
    }
    eIntegral += error_trans * deltaT;
    float eDerivative = (error_trans - prevError) / deltaT;
    prevError = error_trans;
    float u = KP * error_trans + KI * eIntegral + KD * eDerivative;
    return fmax(min_val_, fmin(u, max_val_));
  }

  float control_base(float error, float speed, int condition, float deltaT)
  {
    if (condition)
    {
      eProportional = error;
      if (eProportional > 180)
      {
        eProportional -= 360;
      }
      else if (eProportional < -180)
      {
        eProportional += 360;
      }
    }
    else
    {
      eProportional = error;
    }
    eIntegral += eProportional * deltaT;
    float eDerivative = (eProportional - prevError) / deltaT;
    prevError = eProportional;
    float u = kp * eProportional + ki * eIntegral + kd * eDerivative;
    float uT = kpT * eProportional + kiT * eIntegral + kdT * eDerivative;
    return condition ? uT : u;
  }
  float control_speed(float target, float enc, float deltaT)
  {
    /*convert nilai*/
    radian = (enc - encPrev) / deltaT; // encoder count menjadid encoder count/second
    encPrev = enc;
    angular_vel = radian / (total_gear_ratio * enc_ppr); // convert encoder count/second jadi radian/second

    // masukkan ke low pass filter untuk memperbagus hasil
    angular_vel_Filt = 0.854 * angular_vel_Filt + 0.0728 * angular_vel + 0.0728 * angular_vel_Prev;
    angular_vel_Prev = angular_vel;

    /*hitung pid*/
    // ubah angular_vel_Filt menjadi angular_vel jika ingin menghitung tanpa filter
    float error = target - angular_vel_Filt;

    error_integral += error * deltaT;

    float error_derivative = (error - error_previous) / deltaT;
    error_previous = error;

    float u = KP * error + KI * error_integral + KD * error_derivative;
    return fmax(min_val_, fmin(u, max_val_));
  }
  float control_default(float target, float curr, float deltaT)
  {
    float error = target - curr;

    error_integral += error * deltaT;

    float error_derivative = (error - error_previous) / deltaT;
    error_previous = error;

    float u = KP * error + KI * error_integral + KD * error_derivative;
    return fmax(min_val_, fmin(u, max_val_));
  }
  float get_lowPass(float lowpass_input)
  {
    lowpass_filt = 0.854 * lowpass_filt + 0.0728 * lowpass_input + 0.0728 * lowpass_prev;
    lowpass_prev = lowpass_input;

    return lowpass_filt;
  }
  float get_filt_vel() const
  {
    return angular_vel_Filt;
  }
  float get_vel() const
  {
    return angular_vel;
  }
  float get_enc_vel() const
  {
    return radian;
  }
};