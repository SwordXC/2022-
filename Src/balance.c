#include "balance.h"
#include "math.h"
#include "SEEKFREE_MPU6050.h"
#include "control.h"
float balance_K=0.008;
float dt=0.00155;	
float gy,pitch,angle_ax;	
float pitch_zero=88.8;
uint8_t mpu_flag=0;
void Pitch_init(void)
{

	
}
void Pitch_compute(void )
{
	get_accdata( );
	get_gyro( );
	angle_ax=(float)atan2(mpu_acc_x,mpu_acc_y)*57.3;
	if(mpu_flag==0)
	{
		pitch=angle_ax;
		mpu_flag=1;
	}
	else
	{
		gy=(float)(mpu_gyro_z-14);
	pitch=balance_K*angle_ax+(1-balance_K)*(pitch+gy*dt);
	}
	
}