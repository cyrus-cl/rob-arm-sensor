#include "MPU6050.h"
#include "main.h"
#include <math.h>


/*传感器读取的值*/
int16_t AX,AY,AZ,GX,GY,GZ;
int16_t AX0,AY0,AZ0,GX0,GY0,GZ0;

/*偏移量*/
float i;                               // 计算偏移量的循环次数
float ax_offset,ay_offset;             // x,y轴的加速度偏移量
float gx_offset,gy_offset;             // x,y轴的角速度偏移量

/*参数*/
float rad2deg = 57.29578;              // 弧度换算到角度的换算单位
float roll_v = 0, pitch_v = 0;         // 换算到x,y轴的角速度

/*微分时间*/
float now = 0 , lasttime = 0 , dt = 0; // 定义微分时间

/*三个状态*/
float gyro_roll = 0,gyro_pitch = 0;    // 陀螺仪计算出的角度，先验状态
float acc_roll = 0,acc_pitch = 0;      // 加速度计算出的角度，观测状态
//float k_roll = 0,k_pitch = 0;          // 卡尔曼滤波之后估计最优角度

/*协方差矩阵*/
float e_P[2][2] = {{1,0},{0,1}};       // e_P既是先验估计的矩阵，也是最后更新的P

/*卡尔曼增益*/
float k_k[2][2] = {{0,0},{0,0}};       //卡尔曼增益矩阵是一个2X2矩阵

void setup(void)
{
	MPU6050_Init();
	MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);
	
	/*计算偏移量*/
	for(i = 0;i < 2000;i++)
	{
		ax_offset += AX;
		ay_offset += AY;
		gx_offset += GX;
		gy_offset += GY;
	}
	ax_offset = ax_offset / 2000;
	ay_offset = ay_offset / 2000;
	gx_offset = gx_offset / 2000;
	gy_offset = gy_offset / 2000;
	
}

void loop(float *k_roll,float *k_pitch)
{
	dt = 0.001;
	MPU6050_Init();
	MPU6050_GetData(&AX0,&AY0,&AZ0,&GX0,&GY0,&GZ0);
	
	roll_v = (GX0-gx_offset) + ((sin(*k_pitch)*sin(*k_roll))/cos(*k_pitch))*(GY0-gy_offset)+((sin(*k_pitch)*cos(*k_roll))/cos(*k_pitch))*GZ0;
	pitch_v = cos(*k_roll)*(GY0-gy_offset)-sin(*k_roll)*GZ0;
	gyro_roll = *k_roll + dt*roll_v;
	gyro_pitch = *k_pitch + dt*pitch_v;
	
	/*计算协方差矩阵*/
	e_P[0][0] = e_P[0][0] + 0.0025;
	e_P[0][1] = 0;
	e_P[1][0] = 0;
	e_P[1][1] = e_P[1][1] + 0.0025;
	
	/*更新卡尔曼增益*/
	k_k[0][0] = e_P[0][0]/(e_P[0][0]+0.3);
	k_k[0][1] = 0;
	k_k[1][0] = 0;
	k_k[1][1] = e_P[1][1]/(e_P[1][1]+0.3);
	
	acc_roll = atan((AY0 - ay_offset)/(AZ0))*rad2deg;
	acc_pitch = -1*atan((AX0 - ax_offset)/sqrt(pow(AY0 - ay_offset,2)+pow(AZ0,2)))*rad2deg;
	
	*k_roll = gyro_roll + k_k[0][0]*(acc_roll-gyro_roll);
	*k_pitch = gyro_pitch + k_k[1][1]*(acc_pitch-gyro_pitch);
	
	e_P[0][0] = (1 - k_k[0][0])*e_P[0][0];
	e_P[0][1] = 0;
	e_P[1][0] = 0;
	e_P[1][1] = (1 - k_k[1][1])*e_P[1][1];
	
}
