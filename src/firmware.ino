#include <Arduino.h>
#include <stdio.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "odometry.h"
#include "config.h"
#include "kinematics.h"
#include "pid.h"
#include "speed.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int motorPWM = 6;
const int motorCW = 7;
const int motorCCW = 8;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

Speed enc_x, enc_y;

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
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

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;
rcl_publisher_t checking_output_motor;
rcl_publisher_t checking_input_motor;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32MultiArray checking_output_msg;
std_msgs__msg__Float32MultiArray checking_input_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

const int enca[6] = {MOTOR1_ENCODER_A, MOTOR2_ENCODER_A, MOTOR3_ENCODER_A, MOTOR4_ENCODER_A, EXTERNAL_X_ENCODER_A, EXTERNAL_Y_ENCODER_A};
const int encb[6] = {MOTOR1_ENCODER_B, MOTOR2_ENCODER_B, MOTOR3_ENCODER_B, MOTOR4_ENCODER_B, EXTERNAL_X_ENCODER_B, EXTERNAL_Y_ENCODER_B};

volatile long pos[6] = {0, 0, 0, 0, 0, 0};

PID wheel1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel4(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPS,
    MAX_RPS_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    ROBOT_DIAMETER);

Odometry odometry;

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    // trouble shooting
    RCCHECK(rclc_publisher_init_default(
        &checking_output_motor,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "checking_output"));
    checking_output_msg.data.data = (float *)malloc(4 * sizeof(float)); // Sesuaikan jumlah elemen
    checking_output_msg.data.size = 4;
    RCCHECK(rclc_publisher_init_default(
        &checking_input_motor,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "checking_input"));
    checking_input_msg.data.data = (float *)malloc(4 * sizeof(float)); // Sesuaikan jumlah elemen
    checking_input_msg.data.size = 4;
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
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

float heading = 0;
imu::Vector<3> initialEulerAngles;
void setup()
{
    pinMode(LED_PIN, OUTPUT);

    enc_x.parameter(1024, 0);
    enc_y.parameter(1024, 0);

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    while (!bno.begin())
    {
        flashLED(3);
    }
    for (int i = 0; i < 4; i++)
    {
        pinMode(cw[i], OUTPUT);
        pinMode(ccw[i], OUTPUT);
        analogWriteFrequency(cw[i], PWM_FREQUENCY);
        analogWriteFrequency(ccw[i], PWM_FREQUENCY);
        analogWriteResolution(PWM_BITS);
        analogWrite(cw[i], 0);
        analogWrite(ccw[i], 0);
    }
    for (int j = 0; j < 6; j++)
    {

        pinMode(enca[j], INPUT);
        pinMode(encb[j], INPUT);
    }
    attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[4]), readEncoder<4>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[5]), readEncoder<5>, RISING);
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
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            publishData();
            moveBase();
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

float prevT = 0;
float deltaT = 0;
float x_pos = 0;
float y_pos = 0;
float heading_ = 0;

// keliling roda encoder 4.635 cm
// 1024 ppr
void moveBase()
{
    sensors_event_t angVelocityData, linearVelocityData, orientationData;
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearVelocityData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    float currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    prevT = currT;
    // brake if there's no command received, or when it's only the first command sent
    if (((millis() - prev_cmd_time) >= 200))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }
    // get the required rps for each motor based on required velocities, and base used
    Kinematics::rps req_rps;
    req_rps = kinematics.getRPS(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z);

    // get the current speed of each motor
    float current_rps1 = wheel1.get_vel();
    float current_rps2 = wheel2.get_vel();
    float current_rps3 = wheel3.get_vel();
    float current_rps4 = wheel4.get_vel();

    // Kinematics::velocities current_vel = kinematics.getVelocities(
    //     current_rps1,
    //     current_rps2,
    //     current_rps3,
    //     current_rps4);

    // the required rps is capped at -/+ MAX_rps to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required rps vs measured rps
    float controlled_motor1 = wheel1.control_speed(req_rps.motor1, pos[0], deltaT);
    float controlled_motor2 = wheel2.control_speed(req_rps.motor2, pos[1], deltaT);
    float controlled_motor3 = wheel3.control_speed(req_rps.motor3, pos[2], deltaT);
    float controlled_motor4 = wheel4.control_speed(req_rps.motor4, pos[3], deltaT);

    if (fabs(req_rps.motor1) < 0.02)
    {
        controlled_motor1 = 0;
    }
    if (fabs(req_rps.motor2) < 0.02)
    {
        controlled_motor2 = 0;
    }
    if (fabs(req_rps.motor3) < 0.02)
    {
        controlled_motor3 = 0;
    }
    if (fabs(req_rps.motor4) < 0.02)
    {
        controlled_motor4 = 0;
    }

    setMotor(cw[0], ccw[0], controlled_motor4);
    setMotor(cw[1], ccw[1], controlled_motor2);
    setMotor(cw[2], ccw[2], controlled_motor3);
    setMotor(cw[3], ccw[3], controlled_motor1);

    Kinematics::position current_pos = kinematics.getPosition(
        pos[4],
        pos[5]);

    Kinematics::velocities current_vel = kinematics.getVelExternal(
        enc_x.calculate_speed(pos[4], deltaT),
        enc_y.calculate_speed(pos[5], deltaT));

    checking_input_msg.data.data[0] = req_rps.motor1; // 1
    checking_input_msg.data.data[1] = req_rps.motor2; // 2
    checking_input_msg.data.data[2] = req_rps.motor3; // 3
    checking_input_msg.data.data[3] = req_rps.motor4; // 4
    RCSOFTCHECK(rcl_publish(&checking_output_motor, &checking_output_msg, NULL));

    checking_output_msg.data.data[0] = current_pos.linear_x;          // 1
    checking_output_msg.data.data[1] = heading_;                      // 2
    checking_output_msg.data.data[2] = orientationData.orientation.x; // 3
    checking_output_msg.data.data[3] = current_pos.angular_z;         // 4
    RCSOFTCHECK(rcl_publish(&checking_input_motor, &checking_input_msg, NULL));
    odometry.update(deltaT,current_vel.linear_x,current_vel.linear_y,current_pos.angular_z);
}

void publishData()
{
    sensors_event_t angVelocityData, linearVelocityData, orientationData;
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearVelocityData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    odom_msg = odometry.getData();
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = angVelocityData.gyro.z;

    imu_msg.linear_acceleration.x = linearVelocityData.acceleration.x;
    imu_msg.linear_acceleration.y = linearVelocityData.acceleration.y;
    imu_msg.linear_acceleration.z = 0.0;

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void twistCallback(const void *msgin)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    prev_cmd_time = millis();
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    setMotor(cw[0], ccw[0], 0);
    setMotor(cw[1], ccw[1], 0);
    setMotor(cw[2], ccw[2], 0);
    setMotor(cw[3], ccw[3], 0);
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

void rclErrorLoop()
{
    flashLED(2);
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