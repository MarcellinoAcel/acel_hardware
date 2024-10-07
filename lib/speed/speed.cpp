#include "speed.h"


Speed::Speed() {
    // Constructor implementation
}

Speed::~Speed() {
    // Destructor implementation
}

void Speed::parameter(int one_revolution, int gear_total)
{
    total_gear_count = gear_total;
    one_full_rev = one_revolution;
}

float Speed::calculate_speed(float count, float deltaT)
{

    float radian = (count - count_prev) / deltaT; // encoder count menjadid encoder count/second
    count_prev = count;
    float angular_vel = radian / one_full_rev; // convert encoder count/second jadi radian/second
    return angular_vel;
}

float Speed::calc_speed_lowPass(float count, float deltaT)
{
    float radian = (count - count_prev) / deltaT;
    count_prev = count;

    float angular_vel = radian / one_full_rev;

    float angular_vel_Filt = 0.854 * angular_vel_Filt + 0.0728 * angular_vel + 0.0728 * angular_vel_Prev;
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

float Speed::bandPass(float value){
    EMA_S_low = (EMA_a_low * value) + ((1 - EMA_a_low) * EMA_S_low);
    EMA_S_high = (EMA_a_high * value) + ((1 - EMA_a_high) * EMA_S_high);

    highpass = value - EMA_S_low;
    bandpass = EMA_S_high - EMA_S_low;

    return bandpass;
}

float Speed::highPass(float value){
    EMA_s = (EMA_a * value) + ((1 - EMA_a) * EMA_s);
    highpass = value - EMA_s;
    return highpass;
}

float Speed::lowPass(float value){
 
    float filtered_value = 0.854 * filtered_value + 0.0728 * value + 0.0728 * value_Prev;
    value_Prev = value;

    return filtered_value;   
}