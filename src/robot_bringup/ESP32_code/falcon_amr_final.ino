#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <math.h>

/* ================= 1. PIN DEFINITIONS (BTS7960) ================= */
#define L_EN1      6
#define L_EN2      7
#define L_RPWM     4
#define L_LPWM     5
#define ENC_L_PIN  2

#define R_EN1      35
#define R_EN2      48
#define R_RPWM     36
#define R_LPWM     37
#define ENC_R_PIN  1

/* ================= 2. ROBOT GEOMETRY & PWM CONFIG ================= */
#define WHEEL_RADIUS   0.09
#define WHEEL_BASE     0.44
#define TICKS_PER_REV  20.0
#define PWM_FREQ       20000
#define PWM_RES        8
#define MAX_SPEED      0.5  // m/s linear speed

/* ================= 3. VARIABLES ================= */
volatile long countL = 0, countR = 0;
volatile int8_t dirSignL = 0, dirSignR = 0;
long odom_prevL = 0, odom_prevR = 0; 
double x = 0.0, y = 0.0, theta = 0.0;

static char odom_frame[] = "odom";
static char base_frame[] = "base_footprint";

rcl_publisher_t odom_pub;
rcl_publisher_t encL_pub;
rcl_publisher_t encR_pub;
rcl_subscription_t cmd_sub;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Int32 encL_msg;
std_msgs__msg__Int32 encR_msg;
geometry_msgs__msg__Twist twist_msg;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

/* ================= 4. INTERRUPTS ================= */
void IRAM_ATTR ISR_L() { countL += dirSignL; }
void IRAM_ATTR ISR_R() { countR += dirSignR; }

/* ================= 5. MOTOR CONTROL ================= */
const int TRIM_L_F = 0;
const int TRIM_R_F = -10;
const int TRIM_L_B = -10;
const int TRIM_R_B = 0;

uint8_t targetSpeedL = 0;
uint8_t targetSpeedR = 0;
uint8_t actualSpeedL = 0;
uint8_t actualSpeedR = 0;

char actualDir  = 'X';  // X = stop, F = forward, B = backward, R = turn right, T = turn left

const int CH_L_FWD = 0;
const int CH_L_BWD = 1;
const int CH_R_FWD = 2;
const int CH_R_BWD = 3;

int motorSign(char dir, int motor) {
    switch (dir) {
        case 'F': return  1;
        case 'B': return -1;
        case 'R': return (motor == 0 ?  1 : -1);
        case 'T': return (motor == 0 ? -1 :  1);
        default:  return  0;
    }
}

void applyMotors(char dir, uint8_t spdL, uint8_t spdR) {
    int sL = motorSign(dir, 0);
    int sR = -motorSign(dir, 1);

    int cmdL = spdL;
    int cmdR = spdR;

    if (dir == 'F') { cmdL += TRIM_L_F; cmdR += TRIM_R_F; }
    else if (dir == 'B') { cmdL += TRIM_L_B; cmdR += TRIM_R_B; }

    cmdL = constrain(cmdL, 0, 255);
    cmdR = constrain(cmdR, 0, 255);

    ledcWrite(CH_L_FWD, sL > 0 ? cmdL : 0);
    ledcWrite(CH_L_BWD, sL < 0 ? cmdL : 0);
    ledcWrite(CH_R_FWD, sR > 0 ? cmdR : 0);
    ledcWrite(CH_R_BWD, sR < 0 ? cmdR : 0);
}

/* ================= 5b. Exponential Stop ================= */
float expFactor = 0.2; // smaller = slower stop
void updateRamp() {
    // exponential-like ramping
    actualSpeedL = (uint8_t)((float)actualSpeedL + (float)(targetSpeedL - actualSpeedL) * expFactor);
    actualSpeedR = (uint8_t)((float)actualSpeedR + (float)(targetSpeedR - actualSpeedR) * expFactor);

    applyMotors(actualDir, actualSpeedL, actualSpeedR);

    dirSignL = motorSign(actualDir, 0);
    dirSignR = motorSign(actualDir, 1);
}

/* ================= 6. ODOMETRY ================= */
void publishOdometry() {
    static uint32_t last_time = 0;
    uint32_t now = millis();
    float dt = (now - last_time) / 1000.0;
    if (dt <= 0 || dt > 0.5) { last_time = now; return; }
    last_time = now;

    long cL, cR;
    noInterrupts(); cL = countL; cR = countR; interrupts();
    long dL = cL - odom_prevL;
    long dR = cR - odom_prevR;
    odom_prevL = cL; odom_prevR = cR;

    double dS = (2.0 * M_PI * WHEEL_RADIUS * (dL + dR)) / (2.0 * TICKS_PER_REV);
    double dTh = (2.0 * M_PI * WHEEL_RADIUS * (dR - dL)) / (WHEEL_BASE * TICKS_PER_REV);

    x += dS * cos(theta + dTh / 2.0);
    y += dS * sin(theta + dTh / 2.0);
    theta += dTh;

    int64_t ros_now = rmw_uros_epoch_nanos();
    odom_msg.header.stamp.sec = ros_now / 1000000000ULL;
    odom_msg.header.stamp.nanosec = ros_now % 1000000000ULL;

    for(int i=0; i<36; i++) odom_msg.pose.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;

    odom_msg.header.frame_id.data = odom_frame;
    odom_msg.header.frame_id.size = strlen(odom_frame);
    odom_msg.header.frame_id.capacity = strlen(odom_frame)+1;
    odom_msg.child_frame_id.data = base_frame;
    odom_msg.child_frame_id.size = strlen(base_frame);
    odom_msg.child_frame_id.capacity = strlen(base_frame)+1;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation.z = sin(theta/2.0);
    odom_msg.pose.pose.orientation.w = cos(theta/2.0);

    rcl_publish(&odom_pub, &odom_msg, NULL);

    encL_msg.data = cL;
    encR_msg.data = cR;
    rcl_publish(&encL_pub, &encL_msg, NULL);
    rcl_publish(&encR_pub, &encR_msg, NULL);
}

/* ================= 7. CMD_VEL CALLBACK ================= */
void cmd_vel_cb(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    float linear  = msg->linear.x;
    float angular = msg->angular.z;

    // differential drive
    float left_vel  = linear - (angular * WHEEL_BASE / 2.0);
    float right_vel = linear + (angular * WHEEL_BASE / 2.0);

    // map velocity to PWM
    targetSpeedL = constrain((uint8_t)(fabs(left_vel) * 255.0 / MAX_SPEED), 0, 255);
    targetSpeedR = constrain((uint8_t)(fabs(right_vel) * 255.0 / MAX_SPEED), 0, 255);

    // set direction based on actual wheel velocity
    if (left_vel > 0 && right_vel > 0) actualDir = 'F';
    else if (left_vel < 0 && right_vel < 0) actualDir = 'B';
    else if (left_vel < right_vel) actualDir = 'R';
    else if (left_vel > right_vel) actualDir = 'T';
    else actualDir = 'X';
}

/* ================= 8. SETUP ================= */
void setup() {
    Serial.begin(115200);
    set_microros_transports();

    pinMode(L_EN1, OUTPUT); digitalWrite(L_EN1, HIGH);
    pinMode(L_EN2, OUTPUT); digitalWrite(L_EN2, HIGH);
    pinMode(R_EN1, OUTPUT); digitalWrite(R_EN1, HIGH);
    pinMode(R_EN2, OUTPUT); digitalWrite(R_EN2, HIGH);

    pinMode(ENC_L_PIN, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), ISR_L, CHANGE);
    pinMode(ENC_R_PIN, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), ISR_R, CHANGE);

    ledcSetup(CH_L_FWD, PWM_FREQ, PWM_RES);
    ledcSetup(CH_L_BWD, PWM_FREQ, PWM_RES);
    ledcSetup(CH_R_FWD, PWM_FREQ, PWM_RES);
    ledcSetup(CH_R_BWD, PWM_FREQ, PWM_RES);

    ledcAttachPin(L_RPWM, CH_L_FWD);
    ledcAttachPin(L_LPWM, CH_L_BWD);
    ledcAttachPin(R_RPWM, CH_R_FWD);
    ledcAttachPin(R_LPWM, CH_R_BWD);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "falcon_bot_node", "", &support);

    rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/wheel");
    rclc_publisher_init_default(&encL_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder/left");
    rclc_publisher_init_default(&encR_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder/right");

    rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

    while (rmw_uros_sync_session(1000) != RMW_RET_OK) { delay(100); }

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_sub, &twist_msg, &cmd_vel_cb, ON_NEW_DATA);
}

/* ================= 9. LOOP ================= */
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    updateRamp();

    static uint32_t last_odom = 0;
    if (millis() - last_odom > 50) { 
        publishOdometry();
        last_odom = millis();
    }
}
