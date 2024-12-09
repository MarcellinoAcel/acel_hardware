#include <Arduino.h>
#include <stdio.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "odometry.h"
#include "config.h"
#include "kinematics.h"
#include "pid.h"
#include "imu.h"
#include "Servo.h"

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ------------------------------------------------- //
// anomali
rcl_publisher_t checking_input_motor;
std_msgs__msg__Float32MultiArray checking_input_msg;
// ------------------------------------------------- //

rcl_subscription_t button_sub;
rcl_subscription_t twist_subscriber;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int32 button_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

void setMotor(int cwPin, int ccwPin, float pwmVal);
void moveBase();
void publishData();
void syncTime();
void error_loop();
void subscription_callback(const void *msgin);
void twistCallback(const void *msgin);
struct timespec getTime();
bool createEntities();
bool destroyEntities();
void flashLED(int n_times);
template <int j>
void readEncoder();

void dribble_call(float target_angle, float pwm);
void upperRobot();

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long prevT = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

const int enca[5] = {MOTOR1_ENCODER_A, MOTOR2_ENCODER_A, MOTOR3_ENCODER_A, MOTOR4_ENCODER_A, dribble_enc_a};
const int encb[5] = {MOTOR1_ENCODER_B, MOTOR2_ENCODER_B, MOTOR3_ENCODER_B, MOTOR4_ENCODER_B, dribble_enc_b};

volatile long pos[5];

PID wheel1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel4(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID dribble(PWM_MIN, PWM_MAX, drib_kp, drib_ki, drib_kd);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPS,
    MAX_RPS_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    ROBOT_DIAMETER);

Odometry odometry;

IMU imu_sensor;

Servo ball_holder;

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    while (!imu_sensor.init())
    {
        flashLED(3);
    }

    ball_holder.attach(servo);

    for (int i = 0; i < 5; i++)
    {
        pinMode(cw[i], OUTPUT);
        pinMode(ccw[i], OUTPUT);

        pinMode(dribble_cw, OUTPUT);
        pinMode(dribble_ccw, OUTPUT);

        analogWriteFrequency(cw[i], PWM_FREQUENCY);
        analogWriteFrequency(ccw[i], PWM_FREQUENCY);

        analogWriteFrequency(dribble_cw, PWM_FREQUENCY);
        analogWriteFrequency(dribble_ccw, PWM_FREQUENCY);

        analogWriteResolution(PWM_BITS);
        analogWrite(cw[i], 0);
        analogWrite(ccw[i], 0);

        pinMode(enca[i], INPUT);
        pinMode(encb[i], INPUT);
    }

    attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[4]), readEncoder<4>, RISING);
    pinMode(LED_PIN, OUTPUT);

    ball_holder.write(60);
    delay(200);
}
float toCount(float count)
{
    return 3830 * (count / 360);
}
float toDeg(float count)
{
    return 360 * (count / 3840);
}
void loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
            publishData();
            moveBase();
            upperRobot();

            checking_input_msg.data.data[0] = dribble.get_deg2Targt();    // 1
            checking_input_msg.data.data[1] = toDeg(pos[4]);              // 2
            checking_input_msg.data.data[2] = 0;                          // 3
            checking_input_msg.data.data[3] = toDeg(dribble.get_error()); // 4

            RCSOFTCHECK(rcl_publish(&checking_input_motor, &checking_input_msg, NULL));
        }
        break;
    case AGENT_DISCONNECTED:
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

unsigned long dribble_prevT = 0;
int cmd_to_dribble = 0;
void dribble_call_once(float target, float pwm)
{
    while (true)
    {

        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
        publishData();
        moveBase();

        checking_input_msg.data.data[0] = dribble.get_deg2Targt();    // 1
        checking_input_msg.data.data[1] = toDeg(pos[4]);              // 2
        checking_input_msg.data.data[2] = 0;                          // 3
        checking_input_msg.data.data[3] = toDeg(dribble.get_error()); // 4

        RCSOFTCHECK(rcl_publish(&checking_input_motor, &checking_input_msg, NULL));
        unsigned long dribble_currT = micros();
        float deltaT = ((float)(dribble_currT - dribble_prevT)) / 1.0e6;
        setMotor(dribble_cw, dribble_ccw, dribble.control_angle(target, pos[4], pwm, deltaT));
        dribble_prevT = dribble_currT;
        if (fabs(toDeg(dribble.get_error())) <5)
        {
            break;
        }
    }
}
void dribble_call(float target, float pwm)
{
    unsigned long dribble_currT = micros();
    float deltaT = ((float)(dribble_currT - dribble_prevT)) / 1.0e6;
    setMotor(dribble_cw, dribble_ccw, dribble.control_angle(target, pos[4], pwm, deltaT));
    dribble_prevT = dribble_currT;
}
bool buttonPressed = false;
void upperRobot()
{
    if (button_msg.data == 1 && buttonPressed == false)
    {
        cmd_to_dribble = 1;
        buttonPressed = true;
    }
    else if (button_msg.data == 0)
    {
        buttonPressed = false;
    }
    if (cmd_to_dribble == 1)
    {

        ball_holder.write(60);
        delay(100);
        dribble_call_once(0, 150);
        // delay(1000);
        dribble_call_once(90, 150);
        ball_holder.write(120);
        delay(100);
        ball_holder.write(60);
        delay(100);
        dribble_call_once(45, 150);
        cmd_to_dribble = 0;
    }
    else
    {
        dribble_call(10, 100);
    }
}

void moveBase()
{
    unsigned long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;

    if (((millis() - prev_cmd_time) >= 200))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }

    Kinematics::rps req_rps;
    req_rps = kinematics.getRPS(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z);

    float controlled_motor1 = wheel1.control_speed(req_rps.motor1, pos[0], deltaT);
    float controlled_motor2 = wheel2.control_speed(req_rps.motor2, pos[1], deltaT);
    float controlled_motor3 = wheel3.control_speed(req_rps.motor3, pos[2], deltaT);
    float controlled_motor4 = wheel4.control_speed(req_rps.motor4, pos[3], deltaT);

    float current_rps1 = wheel1.get_filt_vel();
    float current_rps2 = wheel2.get_filt_vel();
    float current_rps3 = wheel3.get_filt_vel();
    float current_rps4 = wheel4.get_filt_vel();

    if (fabs(req_rps.motor1) < 0.02)
    {
        controlled_motor1 = 0.0;
    }
    if (fabs(req_rps.motor2) < 0.02)
    {
        controlled_motor2 = 0.0;
    }
    if (fabs(req_rps.motor3) < 0.02)
    {
        controlled_motor3 = 0.0;
    }
    if (fabs(req_rps.motor4) < 0.02)
    {
        controlled_motor4 = 0.0;
    }

    setMotor(cw[0], ccw[0], controlled_motor4);
    setMotor(cw[1], ccw[1], controlled_motor2);
    setMotor(cw[2], ccw[2], controlled_motor3);
    setMotor(cw[3], ccw[3], controlled_motor1);

    Kinematics::velocities vel = kinematics.getVelocities(
        current_rps1,
        current_rps2,
        current_rps3,
        current_rps4);

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        vel.linear_x,
        vel.linear_y,
        vel.angular_z);

    prevT = currT;

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
}

void publishData()
{

    odom_msg = odometry.getData();
    imu_msg = imu_sensor.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

bool createEntities()
{

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    // create subscriber
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "omni_cont/cmd_vel"));

    RCCHECK(rclc_subscription_init_default(
        &button_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "button_micros"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &button_sub,
        &button_msg,
        &subscription_callback,
        ON_NEW_DATA));

    // publisher
    RCCHECK(rclc_publisher_init_default(
        &checking_input_motor,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "checking_input"));
    checking_input_msg.data.data = (float *)malloc(4 * sizeof(float)); // Sesuaikan jumlah elemen
    checking_input_msg.data.size = 4;
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"));
    syncTime();
    digitalWrite(LED_PIN, HIGH);
    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // rcl_publisher_fini(&odom_publisher, &node);
    // rcl_publisher_fini(&imu_publisher, &node);
    // rcl_subscription_fini(&twist_subscriber, &node);
    // rcl_node_fini(&node);
    // rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

void error_loop()
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
}

void subscription_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    button_msg = *msg;
}

void twistCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    twist_msg = *msg;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    prev_cmd_time = millis();
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}

void setMotor(int cwPin, int ccwPin, float pwmVal)
{
    if (pwmVal > 0)
    {
        analogWrite(cwPin, fabs(pwmVal));
        analogWrite(ccwPin, 0);
    }
    else if (pwmVal < 0)
    {
        analogWrite(cwPin, 0);
        analogWrite(ccwPin, fabs(pwmVal));
    }
    else
    {
        analogWrite(cwPin, 0);
        analogWrite(ccwPin, 0);
    }
}
template <int j>
void readEncoder()
{
    int b = digitalRead(encb[j]);
    if (b > 0)
    {
        pos[j]++;
    }
    else
    {
        pos[j]--;
    }
}