/** 
* @brief    姿态解算文件
* @details  姿态解算和零点矫正等函数都在这个文件中
* @author   Evan-GH
* @date      2019.11
* @version  1.0
* @par Copyright (c):  RM2020电控
* @par 日志
*					根据代码规范修改了RM2019的陀螺仪库
*					自检无法通过请根据读出的原始数据修改 Zero_Threshold[3] 数组的值，如果一开始就读出来一万多的奇葩数据，建议更换陀螺仪，它多半凉了
*					调用初始化函数和姿态解算的时候最好关掉中断或者进入临界区使用
*					姿态解算的间隔时间自己按着调节效果来决定，1MS一次，5MS一次其实应该都能用，看实际调起来的使用效果了
*					原本的imu变量现在被替换为app_imu_Data
*					查看陀螺仪工作是否正常就看offset 原始六轴数据就看original 低通滤波之后的六轴数据就看LPF 一般姿态角就看soft 纯陀螺仪不融合加速度计的姿态
*					角就看integral
*/

#include "app_imu.h"
#include "app_math.h"
// 功能性函数宏定义
#define micros() 1000*HAL_GetTick()  //计时，单位us,除1000000是秒
#define USE_OFFSET      // 初始化使用零点校正   只要有动静态校正，陀螺仪本身还是不太漂的，这个用不用看效果吧   ！！！！！！这个注释掉的话，三个轴的效果是之前的，但是soft不受影响！！！！！！！！！！！！！！
#define DYNAMIC_OFFSET    // 使用动态校正零点，这个可以使用，app_imu_Data还是主要受上面的影响    
//#define USE_KALMAN      // 卡尔曼滤波和低通滤波只能存在一个，同时开启的话默认为低通滤波
#define USE_LPF           // 使用低通滤波器 
// 变量宏定义 
#define Sample_Frequency 1000     //采样频率
#define Gyro_Cut_Off_Frequency 80    //截止频率，这里要根据情况和需求更改  !!!!!!!!!!!!!!往届有把截止频率改的特别小的（10左右），底盘实测收敛很慢，会造成超调，故改大。过大的话过滤高频噪声的能力差
#define Acce_Cut_Off_Frequency 5
#define SELF_TEST_T  5           //自检时间，单位秒
#define g 9.80665f                           //< 重力加速度
#define TORADIAN   0.0174533f                //< 转换为弧度制，四元数的姿态解算需要使用 π/180
#define TOANGLE    57.2957795f               //< 最后解算出来的弧度制转换为角度
#define ACC_RESOLUTION  (2.0f*g/32768.0f)    //< 加速度计分辨率 m/s^2/LSb
#define GYRO_RESOLUTION (2000.0f/32768.0f)   //< 陀螺仪分辨率   dps/LSb  
// 需调整的全局变量 
float so3_comp_params_Kp = 2.0f ;            //< 四元数校正PI参数
float so3_comp_params_Ki = 0.03f; 
uint16_t Zero_Threshold[3] = {300,100,100};  //< 用于零点校正，判断数据是否为静止数据------------------------> 100足够大了，最好看一下原始数据，看看够不够（其实有点大）<--------------------建议更改，提升自检要求
float  Dynamic_Zero_Thre = 4.0f;             //< 动态校正的零点阈值
float Offset_Coeff[3] = {1.0f,1.0f,1.0f};    //< 对量程进行校正，角速度校正系数           ！！！！！！在此标准化量程，即转一个直角，显示90度的效果！！！！！！！！！！！！！！
float manualOffsetGyro[3] = {0,0,0};         //< 手动添加校正值，解决零点误差在+-1之间的问题，因为原始数据是int16_t类型，这个参数能解决有规律的漂移,追求完美的可以试试
// 无需调整的全局变量
int16_t Flash_Val[4];                        //< [0]记录Flash写入的次数，[1-3]为陀螺仪零点值点）
float startOffsetVal[3];
// 结构体 
MPU_HMC app_imu_Data;
kalman_filter AccFilter[3];
kalman_filter GyroFilter[3];
LPF2 Acc_LPF[3];
LPF2 Gyro_LPF[3];

/** 
* @brief   零点值计算
* @remarks 用于零点校正，采样均值,只对陀螺仪进行
*/
uint8_t app_imu_Init(void)
{
	static uint16_t unstable_num;

	#ifdef USE_LPF	
		for(uint8_t i=0;i<3;i++)
		{
			app_math_LPF2pSetCutoffFreq(&Acc_LPF[i],Sample_Frequency,Acce_Cut_Off_Frequency);
			app_math_LPF2pSetCutoffFreq(&Gyro_LPF[i],Sample_Frequency,Gyro_Cut_Off_Frequency);
		}
	#endif
	static uint8_t flag = 1;  ///<采样标志位
	static float tick;        ///<用于超时计数
	while(flag)
	{
		tick = micros();
		for(uint16_t num=0;num<Zero_Sample_Num;num++)
		{   /*零点采样*/
			app_imu_Data.original.Gyro[0] = bsp_mpu9250_Readreg(MPU6500_GYRO_XOUT_H)<<8|bsp_mpu9250_Readreg(MPU6500_GYRO_XOUT_L);
			app_imu_Data.original.Gyro[1] = bsp_mpu9250_Readreg(MPU6500_GYRO_YOUT_H)<<8|bsp_mpu9250_Readreg(MPU6500_GYRO_YOUT_L);
			app_imu_Data.original.Gyro[2] = bsp_mpu9250_Readreg(MPU6500_GYRO_ZOUT_H)<<8|bsp_mpu9250_Readreg(MPU6500_GYRO_ZOUT_L);		
			while(APP_MATH_ABS(app_imu_Data.original.Gyro[0])>Zero_Threshold[0] || APP_MATH_ABS(app_imu_Data.original.Gyro[1])>Zero_Threshold[1] || APP_MATH_ABS(app_imu_Data.original.Gyro[2])>Zero_Threshold[2])  
			{
				app_imu_Data.original.Gyro[0] = bsp_mpu9250_Readreg(MPU6500_GYRO_XOUT_H)<<8|bsp_mpu9250_Readreg(MPU6500_GYRO_XOUT_L);
				app_imu_Data.original.Gyro[1] = bsp_mpu9250_Readreg(MPU6500_GYRO_YOUT_H)<<8|bsp_mpu9250_Readreg(MPU6500_GYRO_YOUT_L);
				app_imu_Data.original.Gyro[2] = bsp_mpu9250_Readreg(MPU6500_GYRO_ZOUT_H)<<8|bsp_mpu9250_Readreg(MPU6500_GYRO_ZOUT_L);
				unstable_num++;
	      if((micros() - tick)/1000000.0f > SELF_TEST_T) 
				{
					for(uint8_t j=0;j<3;j++)
					{
					  app_imu_Data.offset.Gyro[j] = Flash_Val[j+1]/300.0f;
					  app_imu_Data.isThisTimeInvalid[j] = 1;
						app_imu_Data.offset.Cnt[j] = 0;      // 清除计数
					}
					return 0;   //5s内初始化不成功就使用Flash内存的历史零点值
				}
			}
			for(uint8_t k=0;k<3;k++)
			{
				app_imu_Data.offset.Data[k][app_imu_Data.offset.Cnt[k]] = app_imu_Data.original.Gyro[k]; //<零点采样值
				app_imu_Data.offset.Sum[k] += app_imu_Data.offset.Data[k][app_imu_Data.offset.Cnt[k]];   //<零点采样和
				app_imu_Data.offset.Cnt[k]++;                                          //<采样计数
			}	
		}	
		if (unstable_num > 300)
		{   /*采样数据无效*/  
			unstable_num = 0;
			for(uint8_t i=0;i<3;i++)
			{
				app_imu_Data.offset.Sum[i] = 0;
				app_imu_Data.offset.Cnt[i] = 0;
			}					 
		}
		else
		{
			for(uint8_t i=0;i<3;i++)
			{  /*采样数据有效*/
				startOffsetVal[i] = app_imu_Data.offset.Gyro[i] = app_imu_Data.offset.Sum[i]/Zero_Sample_Num;
				Flash_Val[i+1] = (int16_t)app_imu_Data.offset.Gyro[i]*300;  //更新flsh的值，×300的目的是让数值更准一点，毕竟将float转成了int16_t存起来的			
				app_imu_Data.offset.Cnt[i] = 0;
			}
			Flash_Val[0]++;		
			flag = 0;
		}
	} 
  return 1;	
}

/** 
* @brief   读取原始数据和单位换算
* @remarks 
*/
#ifdef USE_MAG
int16_t Mag_max[2];  //平面校准法，磁力计只校正x,和y
int16_t Mag_min[2];
#endif
static void MPU_Read_Raw(void)
{
	static uint8_t dynamicFlag[3];
#ifdef USE_MAG	
	static uint8_t akm_data[6];	
#endif	
	static uint8_t mpu_data_buf[14];
	
	/* 读取加速度计&陀螺仪 */
	bsp_mpu9250_Readregs(MPU6500_ACCEL_XOUT_H,mpu_data_buf,14);      
	app_imu_Data.original.Accel[0] = (mpu_data_buf[0]<<8 | mpu_data_buf[1]); 
	app_imu_Data.original.Accel[1] = (mpu_data_buf[2]<<8 | mpu_data_buf[3]);
	app_imu_Data.original.Accel[2] = (mpu_data_buf[4]<<8 | mpu_data_buf[5]);
	app_imu_Data.original.MPU_Temp = (mpu_data_buf[6]<<8 | mpu_data_buf[7]);
	app_imu_Data.unitized.MPU_Temp = app_imu_Data.original.MPU_Temp/333.87f + 21;
	app_imu_Data.original.Gyro[0] = (mpu_data_buf[8]<<8  | mpu_data_buf[9]);  
	app_imu_Data.original.Gyro[1] = (mpu_data_buf[10]<<8 | mpu_data_buf[11]);
	app_imu_Data.original.Gyro[2] = (mpu_data_buf[12]<<8 | mpu_data_buf[13]);
#ifdef USE_MAG	
	/* 读取磁力计 */
	Mag_Read(akm_data);
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;	调节校准的公式
	for(uint8_t i=0;i<3;i++)
		app_imu_Data.original.Mag[i] = (akm_data[i*2+1]<<8 | akm_data[i*2]);	
#ifdef MAG_OFFSET  
	for(uint8_t i=0;i<2;i++){  //水平校正磁力计
		Mag_max[i] = app_imu_Data.original.Mag[i]>Mag_max[i]?app_imu_Data.original.Mag[i]:Mag_max[i];
		Mag_min[i] = app_imu_Data.original.Mag[i]<Mag_min[i]?app_imu_Data.original.Mag[i]:Mag_min[i];
		app_imu_Data.offset.Mag[i] = (float)(Mag_max[i] + Mag_min[i])/2;
	}
#endif
#endif	
	for(uint8_t i=0;i<3;i++){	
		/* 进行卡尔曼滤波 */
#ifdef USE_KALMAN		
		app_imu_Data.kalman.Accel[i] = Kalman(&AccFilter[i],(float)app_imu_Data.original.Accel[i]);
    app_imu_Data.kalman.Gyro[i] = Kalman(&GyroFilter[i],(float)(app_imu_Data.original.Gyro[i]));	
    		/* 取角速度 */
		app_imu_Data.Angle_Rate[i] = (float)(app_imu_Data.kalman.Gyro[i]  - app_imu_Data.offset.Gyro[i])*Offset_Coeff[i];  //dps		
		/* 单位化 */
#ifdef USE_OFFSET
		app_imu_Data.unitized.Gyro[i] = app_imu_Data.Angle_Rate[i]*GYRO_RESOLUTION*TORADIAN;              //rad/s 		
#else
		app_imu_Data.unitized.Gyro[i] = app_imu_Data.kalman.Gyro[i]*Offset_Coeff[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#endif		
		app_imu_Data.unitized.Accel[i] = app_imu_Data.kalman.Accel[i]*ACC_RESOLUTION;     //m/s^2  

#ifdef USE_MAG		
		app_imu_Data.kalman.Mag[i] = Kalman(&MagFilter[i],app_imu_Data.original.Mag[i]);		
		app_imu_Data.unitized.Mag[i] = (float)app_imu_Data.kalman.Mag[i] * ((AK8963_ASA[i]-128)/256.0f+1.0f); //uT	
#endif		
#endif

#ifdef USE_LPF
		app_imu_Data.LPF.Accel[i] = app_math_LPF2pApply(&Acc_LPF[i],(float)app_imu_Data.original.Accel[i]);

    app_imu_Data.LPF.Gyro[i] = app_math_LPF2pApply(&Gyro_LPF[i],(float)app_imu_Data.original.Gyro[i]);
		/* 取角速度 */
    app_imu_Data.Angle_Rate[i] = (float)(app_imu_Data.LPF.Gyro[i] - app_imu_Data.offset.Gyro[i] + manualOffsetGyro[i])*Offset_Coeff[i];  //16位量程，只对原始数据进行补偿和滤波处理,需要根据9250改		
    /* 单位化 */		
#ifdef 	USE_OFFSET
    app_imu_Data.unitized.Gyro[i] = app_imu_Data.Angle_Rate[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#else
    app_imu_Data.unitized.Gyro[i] = (float)app_imu_Data.LPF.Gyro[i]*Offset_Coeff[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#endif	
		app_imu_Data.unitized.Accel[i] = (float)app_imu_Data.LPF.Accel[i]*ACC_RESOLUTION;     //m/s^2  	
#endif	
#ifdef DYNAMIC_OFFSET		
		/*静止时，更新零点*/
		if (APP_MATH_ABS(app_imu_Data.Angle_Rate[i]) < Dynamic_Zero_Thre && APP_MATH_ABS(app_imu_Data.offset.Gyro[i] - app_imu_Data.original.lastGyro[i]) < 3){
			dynamicFlag[i]++;
			if(dynamicFlag[i] >= 200)  dynamicFlag[i] = 200; //限位
	  }
		else
			dynamicFlag[i] = 0;
    if(dynamicFlag[i] >= 50){  //大于50个周期开始更新零点
			app_imu_Data.offset.Sum[i] -= app_imu_Data.offset.Data[i][app_imu_Data.offset.Cnt[i]];    //< 清除旧数据
			app_imu_Data.offset.Data[i][app_imu_Data.offset.Cnt[i]] = app_imu_Data.original.Gyro[i];  //< 更新数据
			app_imu_Data.offset.Sum[i] += app_imu_Data.offset.Data[i][app_imu_Data.offset.Cnt[i]];    //< 加入新数据
			if(app_imu_Data.isThisTimeInvalid[i] == 0) /* 启动时采样成功 */
			  app_imu_Data.offset.Gyro[i] = app_math_Limit(app_imu_Data.offset.Sum[i] / Zero_Sample_Num,startOffsetVal[i]+0.5f,startOffsetVal[i]-0.5f);    //< 更新零点
			app_imu_Data.offset.Cnt[i]++;
			if(app_imu_Data.offset.Cnt[i] == Zero_Sample_Num){
				app_imu_Data.offset.Cnt[i] = 0;
				app_imu_Data.isThisTimeInvalid[i] = 0;  /* 若启动时没有初始化成功，等待动态采样成功，启用动态校正值 */
			}
      app_imu_Data.Angle_Rate[i] = 0;	
		}			
#endif 
    app_imu_Data.original.lastGyro[i] = app_imu_Data.original.Gyro[i];			
	}
}

/***************************四元数解算部分*********************************/
/* 无需调整的全局变量 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f,q3 = 0.0f;  /** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f,dq3 = 0.0f;  /** quaternion of sensor frame relative to auxiliary frame */
static float q0q0, q0q1, q0q2, q0q3,q1q1, q1q2, q1q3,q2q2, q2q3,q3q3;
static float gyro_bias[3];  
static uint8_t bFilterInit;
//! Using accelerometer, sense the gravity vector.
//! Using magnetometer, sense yaw.
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx,
                                 float my, float mz)        //其实这个函数是没什么用的，他本身就是利用加速度计和磁力计算出偏航角，再算出四元数，但是一般不用磁力计，懒得注释进宏定义了
{
	float initialRoll, initialPitch;
	float cosRoll, sinRoll, cosPitch, sinPitch;
	float magX, magY;
	float initialHdg, cosHeading, sinHeading;	
	
	initialRoll = 3.1415f + atan2(-ay, -az);
	initialPitch = - atan2(ax, -az);

	cosRoll = cosf(initialRoll);
	sinRoll = sinf(initialRoll);
	cosPitch = cosf(initialPitch);
	sinPitch = sinf(initialPitch);
	if(mx == 0 || mz == 0){
		mx=1;  
    mz=1;
	}
	magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
	magY = my * cosRoll - mz * sinRoll;

	initialHdg = atan2f(-magY, magX);

	cosRoll = cosf(initialRoll * 0.5f);
	sinRoll = sinf(initialRoll * 0.5f);

	cosPitch = cosf(initialPitch * 0.5f);
	sinPitch = sinf(initialPitch * 0.5f);

	cosHeading = cosf(initialHdg * 0.5f);
	sinHeading = sinf(initialHdg * 0.5f);

	q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

	// auxillary variables to reduce number of repeated operations, for 1st pass
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
}
/** 
* @brief   Mahony算法
* @remarks 
*/
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax,
                                   float ay, float az, float mx, float my, float mz, float twoKp, float twoKi,
                                   float dt)
{
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float recipNorm;
    // Make filter converge to initial solution faster
    if (bFilterInit == 0){
			NonlinearSO3AHRSinit(ax, ay, az, mx, my, mz);
			bFilterInit = 1;
    }
    if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)))
    {
			float hx, hy, hz, bx, bz;
			float halfwx, halfwy, halfwz;
			
			recipNorm = app_math_invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;
			// Reference direction of Earth's magnetic field
			hx = 2.0f*(mx*(0.5f-q2q2-q3q3)+my*(q1q2-q0q3)+mz*(q1q3+q0q2));
			hy = 2.0f*(mx*(q1q2+q0q3)+my*(0.5f-q1q1-q3q3)+mz*(q2q3 - q0q1));
			hz = 2.0f*mx*(q1q3-q0q2)+2.0f*my*(q2q3+q0q1)+2.0f*mz*(0.5f-q1q1-q2q2);
			bx = sqrt(hx*hx+hy*hy);
			bz = hz;
			// Estimated direction of magnetic field
			halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
			halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
			halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
			// Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex += (my * halfwz - mz * halfwy);
			halfey += (mz * halfwx - mx * halfwz);
			halfez += (mx * halfwy - my * halfwx);
    }
	
    //增加一个条件：  加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){
			float halfvx, halfvy, halfvz;
			
			recipNorm = app_math_invSqrt(ax * ax + ay * ay + az * az);
			if((1/(0.75f*9.8f))>recipNorm>(1/(1.25f*9.8f))){
				ax *= recipNorm;
				ay *= recipNorm;
				az *= recipNorm;
				// Estimated direction of gravity and magnetic field
				halfvx = q1q3 - q0q2;
				halfvy = q0q1 + q2q3;
				halfvz = q0q0 - 0.5f + q3q3;
				// Error is sum of cross product between estimated direction and measured direction of field vectors
				halfex += ay * halfvz - az * halfvy;
				halfey += az * halfvx - ax * halfvz;
				halfez += ax * halfvy - ay * halfvx;
			}
    }
    // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
    if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f){
        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f){
            gyro_bias[0] += twoKi * halfex * dt;    // integral error scaled by Ki
            gyro_bias[1] += twoKi * halfey * dt;
            gyro_bias[2] += twoKi * halfez * dt;
            // apply integral feedback
            gx += gyro_bias[0];
            gy += gyro_bias[1];
            gz += gyro_bias[2];
        }
        else{
            gyro_bias[0] = 0.0f;    // prevent integral windup
            gyro_bias[1] = 0.0f;
            gyro_bias[2] = 0.0f;
        }
        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    // Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
    //! q_k = q_{k-1} + dt*\dot{q}
    //! \dot{q} = 0.5*q \otimes P(\omega)
    dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    q0 += dt * dq0;
    q1 += dt * dq1;
    q2 += dt * dq2;
    q3 += dt * dq3;

    // Normalise quaternion
    recipNorm = app_math_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

/** 
* @brief    SoftYaw
* @remarks  没有边界的Yaw值
* @par 日志
*/
static float Soft_Angle(float angle,uint8_t whichAngle)
{
	static int16_t angleCircle[3];	
	static float LastAngle[3];
	if((angle - LastAngle[whichAngle]) > 180) 
		angleCircle[whichAngle]--;
	else if((angle - LastAngle[whichAngle]) < -180)
		angleCircle[whichAngle]++;
	LastAngle[whichAngle] = angle;
	return  angleCircle[whichAngle]*360 + angle;
}
/** 
* @brief   姿态解算
* @remarks 
*/
uint32_t tPrev,tNow; 
void app_imu_Calculate(void)
{   
    float euler[3] = {0,0,0};            //rad  
    float Rot_matrix[9] = {1.0f,  0.0f,  0.0f, 0.0f,  1.0f,  0.0f, 0.0f,  0.0f,  1.0f };       //< init: identity matrix 
    // 计算两次解算时间间隔 
    tNow = micros();
    float dt = (tPrev > 0) ? (tNow - tPrev) / 1000000.0f : 0;
    tPrev = tNow;
//    if(dt == 0)  return;    // 第一次是0也没关系，反正是积分
    // 读取数据(已经滤波、校正、单位化) 
    MPU_Read_Raw();
    // 四元数姿态融合 
    #ifdef USE_LPF
    NonlinearSO3AHRSupdate(app_imu_Data.unitized.Gyro[0],app_imu_Data.unitized.Gyro[1],app_imu_Data.unitized.Gyro[2],
													 app_imu_Data.unitized.Accel[0],app_imu_Data.unitized.Accel[1],app_imu_Data.unitized.Accel[2],
													 0,0,0,so3_comp_params_Kp,so3_comp_params_Ki,dt);		
		#endif

    /* 转换成方向余弦矩阵 */
    // Convert q->R, This R converts inertial frame to body frame.
    Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
    Rot_matrix[1] = 2.f * (q1 * q2 + q0 * q3); // 12
    Rot_matrix[2] = 2.f * (q1 * q3 - q0 * q2); // 13
//    Rot_matrix[3] = 2.f * (q1 * q2 - q0 * q3); // 21
//    Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
    Rot_matrix[5] = 2.f * (q2 * q3 + q0 * q1); // 23
//    Rot_matrix[6] = 2.f * (q1 * q3 + q0 * q2); // 31
//    Rot_matrix[7] = 2.f * (q2 * q3 - q0 * q1); // 32
    Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33 
    /* 转换成欧拉角 */
    euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);    //! Roll
    euler[1] = -asinf(Rot_matrix[2]);                   //! Pitch
    euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);    //！Yaw
    /* 得出姿态角 */
    app_imu_Data.Roll = euler[0] * TOANGLE;            // 绝对角
    app_imu_Data.Pitch = euler[1] * TOANGLE;	
    app_imu_Data.Yaw = -euler[2] * TOANGLE;
    app_imu_Data.soft.Roll = Soft_Angle(app_imu_Data.Roll,0);   // 绝对路程角
    app_imu_Data.soft.Pitch = Soft_Angle(app_imu_Data.Pitch,1);
    app_imu_Data.soft.Yaw = Soft_Angle(app_imu_Data.Yaw,2);
		app_imu_Data.integral.Roll += app_imu_Data.Angle_Rate[0]*GYRO_RESOLUTION*dt;  // 相对积分角
		app_imu_Data.integral.Pitch += app_imu_Data.Angle_Rate[1]*GYRO_RESOLUTION*dt;
		app_imu_Data.integral.Yaw += app_imu_Data.Angle_Rate[2]*GYRO_RESOLUTION*dt;
    if(APP_MATH_ABS(app_imu_Data.original.Gyro[0]) <= 1 && APP_MATH_ABS(app_imu_Data.original.Gyro[1]) <= 1 && APP_MATH_ABS(app_imu_Data.original.Gyro[2]) <= 1 &&
			 APP_MATH_ABS(app_imu_Data.original.Accel[0] <= 1) && APP_MATH_ABS(app_imu_Data.original.Accel[1] <= 1) && APP_MATH_ABS(app_imu_Data.original.Accel[2] <= 1))
			app_imu_Data.ready = 0;
		else  app_imu_Data.ready = 1;
}
