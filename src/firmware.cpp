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

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32_multi_array.h>

#include "odometry.h"
#include "config.h"
#include "kinematics.h"
#include "pid.h"
#include "speed.h"
#include "imu.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "Servo.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

Speed enc1(COUNTS_PER_REV1, WHEEL_DIAMETER);
Speed enc2(COUNTS_PER_REV2, WHEEL_DIAMETER);
Speed enc3(COUNTS_PER_REV3, WHEEL_DIAMETER);
Speed enc4(COUNTS_PER_REV4, WHEEL_DIAMETER);

void rclErrorLoop();
void flashLED(int n_times);
bool destroyEntities();
void fullStop();
void syncTime();
struct timespec getTime();
void setMotor(int cwPin, int ccwPin, float pwmVal);
void moveBase();
void publishData();
void upperRobot();
void twistCallback(const void *msgin);
void dribble_call(float target_angle, float pwm);

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
rcl_subscription_t button_sub;
rcl_publisher_t checking_output_motor;
rcl_publisher_t checking_input_motor;
rcl_publisher_t sending_;
rcl_publisher_t debug_pub;

rcl_subscription_t button_subs;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int32MultiArray button_msg;
std_msgs__msg__Float32MultiArray checking_output_msg;
std_msgs__msg__Float32MultiArray checking_input_msg;
std_msgs__msg__Float32MultiArray sending_msg;
std_msgs__msg__String debug_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

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

void buttonCallback(const void *msg_in)
{
    // const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msg_in;
    // button_msg = *msg;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    prev_cmd_time = millis();
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msg_in;
    for (size_t i = 0; i < msg->data.size; i++)
    {
        switch (i)
        {
        case 0:
            button.A = msg->data.data[0];
            break;
        case 1:
            button.B = msg->data.data[1];
            break;
        case 3:
            button.X = msg->data.data[3];
            break;
        case 4:
            button.Y = msg->data.data[4];
            break;
        case 10:
            button.RT = msg->data.data[9];
            break;
        case 9:
            button.LT = msg->data.data[8];
            break;
        case 7:
            button.LB = msg->data.data[6];
            break;
        case 8:
            button.RB = msg->data.data[7];
            break;
        default:
            break;
        }
    }
}

void twistCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    twist_msg = *msg;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    prev_cmd_time = millis();
}
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
        "omni_cont/cmd_vel"));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA));

    RCCHECK(rclc_subscription_init_default(
        &button_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "button"));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &button_sub,
        &button_msg,
        &buttonCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

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
    // tuning pid
    RCCHECK(rclc_publisher_init_default(
        &sending_,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "data_for_regres"));
    sending_msg.data.data = (float *)malloc(4 * sizeof(float)); // Sesuaikan jumlah elemen
    sending_msg.data.size = 4;

    RCCHECK(rclc_publisher_init_default(
        &debug_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "debug_checking"));

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
Servo ball_holder;

void setup()
{
    pinMode(LED_PIN, OUTPUT);

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
            upperRobot();
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

unsigned long dribble_prevT = 0;
enum DribbleState
{
    DRIBBLE_START,
    DRIBBLE_CONTROL,
    DRIBBLE_FINISHED
};

DribbleState dribble_state = DRIBBLE_START;
float dribble_controlled = 0.0;
void dribble_call(float target_angle, float pwm)
{
    unsigned long dribble_currT = micros();
    float deltaT = ((float)(dribble_currT - dribble_prevT)) / 1.0e6;
    switch (dribble_state)
    {
    case DRIBBLE_START:
        dribble_prevT = dribble_currT;
        dribble_state = DRIBBLE_CONTROL;
        break;

    case DRIBBLE_CONTROL:
        dribble_controlled = dribble.control_angle(target_angle, pos[4], pwm, deltaT);
        if (dribble.get_error() < 10)
        {
            break;
        }
        setMotor(dribble_cw, dribble_ccw, dribble_controlled);
        dribble_prevT = dribble_currT;
        publishData();
        moveBase();

        break;

    case DRIBBLE_FINISHED:
        setMotor(0, 0, 0);
        break;
    }
}

int cmd_to_dribble = 0;
bool rt_prev_state = false;
void upperRobot()
{
    if (twist_msg.linear.x > 0)
    {
        debug_msg.data.data = (uint8_t *)"moveit";
        debug_msg.data.size = strlen((char *)debug_msg.data.data);
        RCSOFTCHECK(rcl_publish(&debug_pub, &debug_msg, NULL));
        dribble_call(0, 150);
        ball_holder.write(55);
        delay(500);
        ball_holder.write(120);
        delay(100);
        dribble_call(100, 200);
        cmd_to_dribble = 0;
    }
    else
    {

        debug_msg.data.data = (uint8_t *)"stuckit";
        debug_msg.data.size = strlen((char *)debug_msg.data.data);
        RCSOFTCHECK(rcl_publish(&debug_pub, &debug_msg, NULL));
        ball_holder.write(55);
        dribble_call(30, 150);
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

    checking_input_msg.data.data[0] = fabs(req_rps.motor1); // 1
    checking_input_msg.data.data[1] = fabs(req_rps.motor2); // 2
    checking_input_msg.data.data[2] = fabs(req_rps.motor3); // 3
    checking_input_msg.data.data[3] = fabs(req_rps.motor4); // 4

    RCSOFTCHECK(rcl_publish(&checking_input_motor, &checking_input_msg, NULL));
    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        vel.linear_x,
        vel.linear_y,
        vel.angular_z);

    checking_output_msg.data.data[0] = fabs(wheel1.get_filt_vel()); // 1
    checking_output_msg.data.data[1] = fabs(wheel2.get_filt_vel()); // 2
    checking_output_msg.data.data[2] = fabs(wheel3.get_filt_vel()); // 3
    checking_output_msg.data.data[3] = fabs(wheel4.get_filt_vel()); // 4

    RCSOFTCHECK(rcl_publish(&checking_output_motor, &checking_output_msg, NULL));

    sending_msg.data.data[0] = wheel1.get_error();     // 1
    sending_msg.data.data[1] = wheel1.get_error_int(); // 2
    sending_msg.data.data[2] = wheel1.get_error_der(); // 3
    sending_msg.data.data[3] = wheel1.get_pid_out();   // 4

    RCSOFTCHECK(rcl_publish(&sending_, &sending_msg, NULL));

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
void rclErrorLoop()
{
    flashLED(2);
}
