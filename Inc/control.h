#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
typedef struct PID
{
    float kp;
    float ki;
    float kd;
} PID;


#define turnLeft  1
#define turnRight 2
#define straight  3
extern uint8_t ser_flag;
extern uint8_t cross_flag;
extern uint8_t turn_flag;
extern uint8_t time_flag;
extern uint8_t cross_num;
extern uint8_t cross_enable;
extern uint8_t stop_num;
extern uint8_t rx[3];
extern float servo_pwm;
extern float dir_bias;
extern float motor_target;
extern float motor_pid_sum;
extern float motor_pwm;
extern float motor_encoder;
extern float bal_pid_sum;
extern float bal_pwm;
extern float bal_encoder;
void servo(void);
void servoControl(void);
int encoder_motor(void);
int encoder_balance(void);
void motorControl(float motor_pwm);
float motor_pid(float encoder);
float balance_pid(float encoder);
float pitch_pid(float angle,float angle_zero,float gyro);
void balanceControl(float balance_pwm);

void PWM_bal(int duty);
void PWM_motor(int duty);
void PWM_ser(int duty);
#endif