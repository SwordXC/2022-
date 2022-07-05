#include "control.h"
#include "tim.h"
#include "balance.h"
#include  "math.h"
#include "usart.h"
#include "stdio.h"
PID ser_pid = {.kp = 1.1f, .ki = 0, .kd = 0};
PID vel_pid = {.kp = 4	, .ki = 0, .kd =0};
PID bal_pid = {.kp = 69 , .ki= 0.5, .kd = 0 };
PID pit_pid = {.kp=1900,.ki=3.2,.kd=60};
float servo_pwm;
float dir_bias;
float motor_target=50;
float motor_pid_sum;
float motor_pwm;
float motor_encoder;
float bal_pid_sum;
float bal_pwm;
float bal_encoder;
float bal_pid_last;


void PWM_bal(int duty)
{
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,duty);
}
void PWM_motor(int duty)
{
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,duty);
}
void PWM_ser(int duty)
{
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty);
}

uint8_t ser_flag=0;
uint8_t cross_flag=0;
uint8_t turn_flag=0;
uint8_t time_flag=0;
uint8_t cross_num=0;
uint8_t cross_enable=1;
uint8_t stop_num=3;
uint8_t rx[3];
void servo()//left:350 mid:320  right:290  RESET:white,SET:black    C7 right  C9 left
{
	if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)==GPIO_PIN_SET) && (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)==GPIO_PIN_SET)
	&&(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)==GPIO_PIN_SET) && (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)==GPIO_PIN_SET))
	{
		if((cross_enable==1) && (cross_num!=stop_num) &&(motor_stop==0))
		{
			cross_num++;
			HAL_UART_Transmit(&huart4,"3",1,50);
			cross_enable=0;
		}
	}
//	else
//	{
//		cross_enable=1;
//	}
	if(cross_num==1)
	{
		//  nturn_flag=turnRight;
		turn_flag=rx[0];
	}
	else if(cross_num==2)
	{
		turn_flag=0;
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);
	}
	else if(cross_num==4)
	{
		turn_flag=rx[1];
	}
	else if(cross_num==5)
	{
		turn_flag=0;
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);
	}
	//正常循迹部分
	if(turn_flag==0)
	{
		servoControl();
	}
	//左转
	if(turn_flag==turnLeft)
	{
//		if(time3<=(servo_time/5))PWM_ser(333);
//		else if(time3>(servo_time/5) && time3<=(2*servo_time/5))PWM_ser(344);
//		else if(time3>(2*servo_time/5) && time3<=(3*servo_time/5))PWM_ser(355);
//		else if(time3>(3*servo_time/5) && time3<=(4*servo_time/5))PWM_ser(366);
//		else if(time3>(4*servo_time/5))
		PWM_ser(377);
	}
	//右转
	if(turn_flag==turnRight)
	{
//		if(time3<=20)PWM_ser(309);
//		else if(time3>(servo_time/5) && time3<=(2*servo_time/5))PWM_ser(298);
//		else if(time3>(2*servo_time/5) && time3<=(3*servo_time/5))PWM_ser(287);
//		else if(time3>(3*servo_time/5) && time3<=(4*servo_time/5))PWM_ser(276);
//		else if(time3>(4*servo_time/5))
		PWM_ser(285);
	}
	//直行
	if(turn_flag==straight)
	{
		PWM_ser(320);
		turn_flag=0;
	}
	//转向标志位计数判断

}

void servoControl()
{   

			if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==GPIO_PIN_SET) && (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)==SET))
			{
				PWM_ser(320);
			}
			else if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==GPIO_PIN_SET) && (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)==GPIO_PIN_RESET))
			{
				PWM_ser(335);
				ser_flag=turnLeft;
			}
			else if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)==GPIO_PIN_SET))
			{
				PWM_ser(305);
				ser_flag=turnRight;
			}
			else if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)==GPIO_PIN_RESET))
			{
				
				if(ser_flag==turnLeft)
				{
					PWM_ser(335);
				}
				else if(ser_flag==turnRight)
				{
					PWM_ser(305);
				}
				else
				{
					PWM_ser(320);
				}

			}
}

void motorControl(float motor_pwm)
{
	    
    if (motor_pwm > 0)
    {
        if (motor_pwm > 999)
        {
            motor_pwm = 999;
        }
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
    }
    else
    {
        motor_pwm = -motor_pwm;
        if (motor_pwm > 999)
        {
            motor_pwm = 999;
        }
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
    }
		PWM_motor(motor_pwm);
}

void balanceControl(float balance_pwm)
{
	    if (balance_pwm > 0)
    {
        if (balance_pwm > 799)
        {
            balance_pwm = 799;
        }
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
    }
    else
    {
        balance_pwm = -balance_pwm;
        if (balance_pwm > 799)
        {
            balance_pwm = 799;
        }
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
    }
		PWM_bal(balance_pwm);
}

int encoder_motor(void)
{
	int encoder;
	encoder=motor_encoder;
	return encoder;
}

int encoder_balance(void)
{
	int encoder;
	encoder=bal_encoder;
	return encoder;
}

float motor_pid(float encoder)
{
	float Velocity;
    if (fabs(encoder) > 10)
    {
        motor_pid_sum += motor_target - encoder;
    }
    else
    {
        motor_pid_sum += 0;
    }
    if (motor_pid_sum > 2000)
    {
        motor_pid_sum = 2000;
    }
    if (motor_pid_sum < -2000)
    {
        motor_pid_sum = -2000;
    }
    Velocity=(motor_target-encoder)*vel_pid.kp+motor_pid_sum*vel_pid.ki;
		return Velocity;
}
float balance_pid(float encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;
    Encoder_Least = encoder;
    Encoder *= 0.7;                                                           //一阶低通滤波器
    Encoder += Encoder_Least*0.3;
    Encoder_Integral += Encoder;

    if (bal_pid_sum > 2000)
    {
        bal_pid_sum = 2000;
    }
    if (bal_pid_sum < -2000)
    {
        bal_pid_sum = -2000;
    }
    Velocity = Encoder * bal_pid.kp + Encoder_Integral * bal_pid.ki/100;
    return Velocity;
}
float pitch_pid(float angle,float angle_zero,float gyro)
{
    float PWM,Bias;
    static float error;
    Bias=angle-angle_zero;                                            //获取偏差
    error+=Bias;                                                      //偏差累积
    if(error>+30) error=+30;                                          //积分限幅
    if(error<-30) error=-30;                                          //积分限幅
    PWM=pit_pid.kp*Bias + pit_pid.ki*error + gyro*pit_pid.kd;   //获取最终数值
    return  PWM;
}