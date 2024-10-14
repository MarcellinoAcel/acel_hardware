#include "speed.h"
#include "math.h"

Speed::Speed(int one_revolution, float wheel_diameter)
    : one_full_rev(one_revolution),
      wheel_diameter(wheel_diameter),
      count_prev(0)
{
}

Speed::~Speed()
{
}

float Speed::calculate_angular_speed(float count, float deltaT)
{

    float count_diff = (count - count_prev) / deltaT;
    float angular_vel = count_diff / one_full_rev; 
    count_prev = count;
    return angular_vel;
}
float Speed::calculate_linear_speed(float count, float deltaT)
{

    float count_diff = count - count_prev;
    count_prev = count;
    float angular_vel = (count_diff / one_full_rev) * (2 * M_PI);
    float linear_speed = angular_vel * wheel_diameter;
    return linear_speed;
}

float Speed::calc_speed_lowPass(float count, float deltaT)
{
    float radian = (count - count_prev) / deltaT;
    count_prev = count;

    float angular_vel = radian / one_full_rev;

    angular_vel_Filt = 0.854 * angular_vel_Filt + 0.0728 * angular_vel + 0.0728 * angular_vel_Prev;
    angular_vel_Prev = angular_vel;

    return angular_vel_Filt;
}

float Speed::calc_speed_highPass(float count, float deltaT)
{

    float radian = (count - count_prev) / deltaT;
    count_prev = count;

    float angular_vel = radian / one_full_rev;

    EMA_s = (EMA_a * angular_vel) + ((1 - EMA_a) * EMA_s);
    highpass = angular_vel - EMA_s;
    return highpass;
}

float Speed::calc_speed_bandPass(float count, float deltaT)
{
    float radian = (count - count_prev) / deltaT;
    count_prev = count;

    float angular_vel = radian / one_full_rev;
    EMA_S_low = (EMA_a_low * angular_vel) + ((1 - EMA_a_low) * EMA_S_low);
    EMA_S_high = (EMA_a_high * angular_vel) + ((1 - EMA_a_high) * EMA_S_high);

    highpass = angular_vel - EMA_S_low;
    bandpass = EMA_S_high - EMA_S_low;

    return bandpass;
}

float Speed::bandPass(float value)
{
    EMA_S_low = (EMA_a_low * value) + ((1 - EMA_a_low) * EMA_S_low);
    EMA_S_high = (EMA_a_high * value) + ((1 - EMA_a_high) * EMA_S_high);

    highpass = value - EMA_S_low;
    bandpass = EMA_S_high - EMA_S_low;

    return bandpass;
}

float Speed::highPass(float value)
{
    EMA_s = (EMA_a * value) + ((1 - EMA_a) * EMA_s);
    highpass = value - EMA_s;
    return highpass;
}

float Speed::lowPass(float value)
{

    filtered_value = 0.854 * filtered_value + 0.0728 * value + 0.0728 * value_Prev;
    value_Prev = value;

    return filtered_value;
}