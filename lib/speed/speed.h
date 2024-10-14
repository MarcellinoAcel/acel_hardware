class Speed
{
private:
    float lowpass_filt = 0;
    float lowpass_prev = 0;
    float radian = 0;
    float encPrev = 0;
    float angular_vel_Prev = 0;
    float total_gear_count = 0;
    float one_full_rev = 0;
    float wheel_diameter = 0;
    float count_prev = 0;

    float EMA_a = 0.3;
    float EMA_s = 0;
    float highpass = 0;

    float bandpass = 0;
    float EMA_a_low = 0.3;
    float EMA_a_high = 0.5;

    int EMA_S_low = 0;
    int EMA_S_high = 0;

    float value_Prev = 0;
    float angular_vel_Filt = 0;
    float filtered_value = 0;
public:
    Speed(int one_revolution, float wheel_diameter);
    ~Speed();
    void parameter(int one_revolution, int gear_total);
    float calculate_angular_speed(float count, float deltaT);
    float calculate_linear_speed(float count, float deltaT);
    float calc_speed_lowPass(float count, float deltaT);
    float calc_speed_highPass(float count, float deltaT);
    float calc_speed_bandPass(float count, float deltaT);

    float highPass(float value);
    float bandPass(float value);
    float lowPass(float value);
};
