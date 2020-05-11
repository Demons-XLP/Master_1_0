/** 
* @file         bsp_motor.hpp 
* @author      WMD,Onion rain
*								Evan-GH
* @version  7.5b
* @par Copyright (c):  
*       WMD,Onion rain
*				Evan-GH
* @par 日志见cpp
*/ 
#ifndef __BSP_MOTOR_HPP
#define __BSP_MOTOR_HPP
#include "bsp_car_config.hpp"
using namespace std;
#define	ABS(x)   ((x)>0?(x):-(x))


typedef struct str_Motor_t
{
	str_Motor_t(){};//避免创建空对象不成功
	str_Motor_t(uint16_t a,uint8_t b):max_mechanical_position(a),Reduction_ratio(b){};
	uint16_t max_mechanical_position = 8192;//!<机械角最大值,默认0x2000
	uint8_t Reduction_ratio = 19;        //!<减速比,默认19:1
}Motor_t;//!<电机(电调)参数结构体，默认值为3508电机及其配套C620电调

//PID可以算pid输出 可以配置pid
class pid
{
	public:
		float* Custom_Diff=NULL;//!<自定义的外界浮点型微分数值 常用于路程环的微分环节(即速度值)
		uint16_t I_Time;//!<pid时间参数 以ms为单位 plus专属,积分时间
		uint16_t D_Time;//!<微分时间，ms为单位
		uint16_t I_Limited; //!<当误差小于I_Limited时才进行I输出 plus专属
		float ap = 0, bp = 0, cp;//ap==0 && bp==0说明不是非线性pid
		float ai = 0, ci;
		float ad = 0, bd = 0, cd, dd;
		float P;
		float I;
		float D;
		float IMax;
		float PIDMax;

		//公共函数
		pid(float P, float I, float D, float IMax, float PIDMax, uint16_t I_Time=1, uint16_t D_Time=1,uint16_t I_Limited=9999);//传统pid构造函数
		pid(float ap, float bp, float cp,
				float ai,           float ci,
				float ad, float bd, float cd, float dd,
				float IMax, float PIDMax, uint16_t I_Time=1, uint16_t D_Time=1, uint16_t I_Limited=9999);//非线性pid构造函数
		float pid_run(float err);
		float nonlinear_pid_run(float err);
		float sech(float in);
	private:
		//运算储存区
		float Pout;
		float Iout;
		float Dout;
		float Dout_Accumulative;//因为有微分时间重定义,因此加入一个变量真正用于输出D,源Dout用作累积误差
		float PIDout;
		float CurrentError;
		float LastError;
		uint32_t I_start_time;//!<积分开始时间戳，用于带时间参数的pid   plus专属
		uint32_t D_start_time;//!<微分开始时间戳，用于带时间参数的pid 
};

/** 
	* @brief 堵转检测类 
	* @details 要求在初始化的时候给定需要检测堵转的电流引用和位置引用 \n
	*		  一般不会单独使用该类，电机如果支持堵转检测功能的话会在其内部有一个堵转检测的指针 \n
	*		  想要启用电机的堵转检测功能的话使用对应电机类内提供的初始化函数 \n
	*		  具体检测还需要提供堵转认定电流，堵转认定时间，堵转时间内允许的电机角度偏移量 \n
	*		  可以通过 @see block_type::IsBlock 变量来确认电机是否堵转 \n 
	*		  可以通过 @see block_type::Clear_BlockFlag() 来清除堵转标志
*/
class block_type
{
	public:
		void Block_Init(uint16_t Limit, uint16_t time, float err_num);//!<进行堵转参数初始化
		block_type(int16_t& _Current,float& _RealAngle):Current(_Current),RealAngle(_RealAngle){}//!构造函数
		void Block_Check(void);     //!<检查堵转的函数
		void Clear_BlockFlag(void); //!<去除堵转标志
		uint8_t IsBlock;            //!<是否堵转的标志，1为已经堵转，需要人为使用Clear_BlockFlag消堵转标志
	private:
		int16_t& Current;
		float& RealAngle;
		float block_Angle;            //!<堵转时的位置
		uint16_t block_Current_Limit; //!<堵转检测最低门限电流
		uint32_t block_time;          //!<堵转开始时的时间戳
		uint32_t block_Time_Limit;    //!<堵转检测时允许的堵转时间
		float    block_err_num;       //!<堵转时检测允许的容忍误差
		uint8_t  block_flag=0;        //!<处于检测堵转的标志，不堵转自动恢复为0
};

class manager//电机管家抽象类
{
	public:
		//!< 兼容性改动，在线列表的数据类型修改到int16_t
		//!<CAN1在线的ID列表兼容性更新，适配GM6020的ID会超出0x208，现在[0:10]分别为0x201到0x20B电机在线状态
		static int16_t CAN1_OnlineID;//!<CAN1在线的ID列表，[0:7]分别为0x201到0x207的电机在线状态(此处是旧的说明)
		//!<CAN2在线的ID列表兼容性更新，适配GM6020的ID会超出0x208，现在[0:10]分别为0x201到0x20B电机在线状态
		static int16_t CAN2_OnlineID;//!<CAN2在线的ID列表，[0:7]分别为0x201到0x207的电机在线状态(此处是旧的说明)
		RunState_t RunState=Stop;   //!<电机的运行状态
		//!电机是否是协同工作的，如果属于某机构的一部分，则该值可以置1，此时Handle()函数不会由CANSend执行，需要自己实现，从而完成电机组合逻辑
		uint8_t cooperative;          
		uint8_t Is_Offline(void);//!<判断当前电机是否处于离线状态
		//!全局的CAN收发函数,必须必须必须先调用CANSelect()且必须在CAN接收打开前调用本函数确认接收句柄 Update函数放在CAN接收回调里面,Send保证周期执行,
		static void CANSelect(CAN_HandleTypeDef* canhandle1,CAN_HandleTypeDef* canhandle2);
		static void CANUpdate(CAN_HandleTypeDef* _hcan,CAN_RxHeaderTypeDef* RxHead,uint8_t* Data);//!<全局的接收处理函数,统一管配所有电机
		static void UserProcess(void);          //!<在PID跑完 发送前进行的数据处理函数，建议以重写的形式进行新的处理
		static uint8_t CANSend(void);          //!<全局的发送函数，统一管配所有电机
		void Speed_F_Set(float f);             //!<设定前馈量 输入电机的新的值
		virtual void Safe_Set(void) = 0;       //!<子类必须实现的纯虚函数
	protected:
		uint8_t can_code;                           //!<CAN码,以十进制存储,百位数显示是can几，后面两位数表示id号-1，范围0~10
		static CAN_HandleTypeDef* CanHandle1;       //!<CAN设备1的指针
		static CAN_HandleTypeDef* CanHandle2;       //!<CAN设备2的指针
		static manager* CAN1MotorList[11];           //!<CAN1电机地址列表，由8改动到11
		static int16_t CAN1CurrentList[11];          //!<CAN1电机待发送电流列表，由8改动到11
		static manager* CAN2MotorList[11];           //!<CAN2电机地址列表，由8改动到11
		static int16_t CAN2CurrentList[11];          //!<CAN2电流列表，由8改动到11
		uint32_t LastUpdateTime;                    //!<上次更新的时间
		float Speed_LPF;                            //!<速度前馈低通滤波器
		float Speed_F;                              //!<速度前馈参数
		
		virtual void update(uint8_t Data[]) = 0;    //!<子类必须实现的纯虚函数
		virtual void Handle(void) = 0;              //!<子类必须实现的发送管理函数
};
class motor:public manager//!普通电机类型
{
	public:
		int16_t RealCurrent;			//编码器反馈转矩电流
		int16_t LastRealCurrent;	//编码器反馈转矩电流
		float RealAngle;          //!<根据机械角计算出的真实角度
		int16_t TargetCurrent;    //!<目标电流值
		int16_t RealPosition;	    //!<真实位置(编码器)
		int16_t TargetPosition;	  //!<目标位置
		int16_t RealSpeed;		    //!<实际速度(编码器)
		int16_t TargetSpeed;	    //!<目标速度
		Motor_t *MotorType;       //!<电机(电调)参数
		block_type *block=NULL;   //!<堵转对象指针，在使用堵转检测时会生成对象并储存指针在这里
		motor(void){};            //!<仅用于默认构造函数
		motor(uint8_t can_num,
					uint16_t _can_id,
					Motor_t *motor_type,
					pid* _PID_In,
					pid* _PID_Out=NULL);//!<构造方式之一，只提供速度环pid
		void Speed_Set(int16_t);    //!<设定速度，其实可以直接设定TargetSpeed
		void Angle_Set(float);   //!<设定位置，其实可以直接设定TargetPosition
		virtual int8_t Enable_Block(uint16_t Limit,uint16_t time,uint16_t err_num);//!<到时候写完 注意负数 被坑了
		virtual void Safe_Set(void);//!<设定电机进入安全模式，即发送电流值0
	protected:
		class pid* PID_In;          //!<PID内环
		class pid* PID_Out;         //!<PID外环
		int16_t LastPosition;       //!<上次位置
		int16_t LastSpeed;          //!<上次速度

		virtual void update(uint8_t Data[]); //!<直接Data数组的update函数
		virtual void Handle(void);           //!<数据处理函数，用于判断状态，运行PID
		virtual void Position_Run(void);     //!<使用位置环确定速度 为PID运算环节
		virtual void Speed_Run(void);        //!<使用速度环确定电流 为PID运算环节
		virtual void InsertCurrent(void);    //!<将运算好的电流按列表储存进发送缓存区内
};
/** 
    * @brief 软路程电机 \n
		* 重要特性:把电机面对自己 逆时针转动的时候 路程计数增加 
    */
class	softmotor:public motor
{
	friend class chassis;//声明底盘类型为电机的友元类，让底盘能够访问电机的私有成员
	public:
		int32_t Soft_RealPosition=0;//!<软真实路程，这里实际意义是轮子转过的圈数
		int32_t Soft_TargetPosition;//!<软目标路程，实际意义为轮子要转的圈数
		float SoftAngle; //!<软角度，根据转过圈数来进行计算 RealAngle现在是单圈内角度了

		softmotor():motor(){}//避免在构造空对象时不成功
		softmotor(uint8_t can_num,
							uint16_t _can_id,
							Motor_t *motor_type,
							pid* PID_In,
							pid* PID_Out=NULL)
							:motor(can_num, _can_id, motor_type, PID_In, PID_Out){}//!<构造函数
		void Limit(float _max,float _min);//!<设置软件限位
		void Angle_Set(float);//!<设置路程目标角度
		virtual int8_t Enable_Block(uint16_t Limit,uint16_t time,uint16_t err_num);//!<到时候写完 注意负数 被坑了
	protected:
		virtual void update(uint8_t Data[]);
		virtual void Position_Run(void);
		float max=99999999999;//!<角度最大值
		float min=-99999999999;//!<角度最小值
	private:
		uint8_t running_flag=0;//!<用来保证第一次的软路程不突变 因为开机时LastPosition肯定是0可能会导致一圈外环
};
class cloud : public manager//!扩展:云台类(6623)
{
	public:
		Motor_t *MotorType;        //!<电机(电调)参数
		float RealAngle;           //!<根据机械角计算出的真实角度
		//电流环
		int16_t TargetCurrent;     //!<发送给电机的电流值
		int16_t RealCurrent;		   //!<实际转矩电流(编码器)
		//速度环
		float RealSpeed;           //!<实际速度(陀螺仪/编码器)
		float TargetSpeed;	       //!<目标速度
		float *Gyro_RealSpeed;     //!<指向陀螺仪反馈速度的指针
		float Gyro_TargetSpeed;    //!<陀螺仪设置目标转速
		//位置环
		int16_t RealPosition;	     //!<真实位置(编码器)
		int16_t OriginalPosition;  //!<原始位置(编码器)，没有经过校正
		int16_t TargetPosition;	   //!<目标位置
		float *Gyro_RealAngle;     //!<指向陀螺仪反馈角度的指针
		float Gyro_TargetAngle;    //!<陀螺仪设置目标角度
	
		pid *PID_In;          //!<机械角PID速度环
		pid *PID_Out;         //!<机械角PID位置环
		pid *Gyro_PID_In;     //!<陀螺仪PID速度环
		pid *Gyro_PID_Out;    //!<陀螺仪PID位置环
	
		void Pid_Select(pid *PID_In_Select, pid *PID_Out_Select);//!<机械角pid选择
		void Gyro_Pid_Select(pid *Gyro_PID_In_Select, pid *Gyro_PID_Out_Select);//!<陀螺仪pid选择
		void Speed_Set(float);     //!<设定速度，通过机械角调节
		void Angle_Set(float);     //!<设定角度，通过机械角调节
		void Gyro_Speed_Set(float TargetSpeed);    //!<设定速度，通过陀螺仪调节
		void Gyro_Angle_Set(float TargetPosition); //!<设定角度，通过陀螺仪调节
		void Limit(float _max,float _min);//!<设置软件限位
		void Gyro_Speed_Run(void);   //!PID运算函数，manager类调用
		void Gyro_Position_Run(void);//!PID运算函数，manager类调用
		cloud();//!<默认构造函数
		cloud(uint8_t can_num,
						uint16_t _can_id,
						int16_t _CLOUD_STD,
						Motor_t *motor_type,
						pid *PID_I,
						pid *PID_O,
						pid *G_In,
						pid *G_Out,
						float *SpeedSource=NULL,
						float *PositionSource=NULL); //!<构造函数，指针指向位置或速度源
		virtual void Safe_Set(void);         //!<设定电机进入安全模式，即发送电流值0
						
	protected:
		int16_t CLOUD_STD;          //!<该云台在指向原点时的编码器的值
	
		int16_t LastTorque;     //!<上次转矩
		float LastSpeed;        //!<上次速度(机械角)
		int16_t LastPosition;   //!<上次位置(机械角)
		float Gyro_LastSpeed;   //!<上次速度(陀螺仪)
		float Gyro_LastPosition;//!<上次位置(陀螺仪)
		float max=99999999999;//!<角度最大值
		float min=-99999999999;//!<角度最小值
		virtual void update(uint8_t Data[]); //!<直接Data数组的update函数
		virtual void Handle(void);           //!<数据处理函数，用于判断状态，运行PID
		virtual void Speed_Run(void);        //!<使用速度环确定电流 为PID运算环节
		virtual void Position_Run(void);     //!<使用位置环确定电流 为PID运算环节
	
		virtual void InsertCurrent(void);    //!<将运算好的电流按列表储存进发送缓存区内
};
class softcloud : public cloud///软路程云台类 for 6020
{
	public:
		float TargetAngle;         //!<根据机械角计算出的真实角度
		int32_t Soft_RealPosition=0;//!<软真实路程，这里实际意义是云台转过的圈数
		int32_t Soft_TargetPosition;//!<软目标路程，实际意义为轮子要转的圈数
	
		softcloud():cloud(){}//避免在构造空对象时不成功
		softcloud(uint8_t can_num,
						uint16_t _can_id,
						int16_t _CLOUD_STD,
						Motor_t *motor_type,
						pid *PID_I,
						pid *PID_O,
						pid *G_In,
						pid *G_Out,
						float *SpeedSource=NULL,
						float *PositionSource=NULL)
				:cloud(can_num, _can_id, _CLOUD_STD, motor_type, PID_I, PID_O, G_In, G_Out, SpeedSource, PositionSource),SOFTCLOUD_STD(_CLOUD_STD){}//!<构造函数，指针指向位置或速度源
		void Angle_Set(float);//!<设置路程目标角度
	private:
		virtual void Handle(void);           //!<数据处理函数，用于判断状态，运行PID
		virtual void update(uint8_t Data[]);//!<直接Data数组的update函数
		virtual void Position_Run(void);
		int16_t SOFTCLOUD_STD;          //!<该云台在指向原点时的编码器的值
		uint8_t running_flag=0;//!<用来保证第一次的软路程不突变 因为开机时LastPosition肯定是0可能会导致一圈外环
//		float max=99999999999;//!<角度最大值
//		float min=-99999999999;//!<角度最小值
};
/** 
* @brief  底盘的控制类型，带功率控制
* @par 日志 
*   2019年3月9日16:43:30 WMD 因为需要在速度环和电流环之间插入功率限制调整函数，经过考虑重新修改了一下框架，将电流环嵌入底盘类型，而普通电机类型不涉及电流环
*/
class chassis
{
	public:
		static chassis* point;//!<指向当前已声明的底盘，一个工程只允许一个底盘的存在  该指针用于日后托管处理
		pid *Turn_PID;  //!<转弯PID
		void Run(void); //!<缺省参数 以上次的模式控制底盘
		void Run(float Vx, float Vy, float Omega);//!<以速度控制底盘
		void Safe(void);     //!停止底盘
		softmotor *Motor[4]; //!<四个轮子的电机对象
		chassis(uint8_t can_num, 
							uint16_t First_can_id, 
							Motor_t *motor_type, 
							pid *speed_pid, 
							pid* turnpid=NULL, 
							pid *current_pid=NULL, 
							int16_t *CurrentSource=NULL);//!<直接控制底盘的构造函数
		void Handle(void);//!<交由CANSend托管的底盘处理函数
	private:
		RunState_t RunState;//!<当前底盘状态
		pid *Pid_spe[4];//!<指向四个电机速度环pid的指针
		pid *Pid_current[4];//!<指向四个电机电流环pid的指针
		int16_t *CurrentSource[4];//!<指向电流真实数据源
		float Last_Vx=0, Last_Vy=0, Last_Omega=0;//!<之前的值，用于缺省参数时的使用.
}; 
class chassiscontrol//底盘控制类for云台
{
	public:
		CAN_HandleTypeDef* Canhandle;
		uint16_t Chassis_ID;
		chassiscontrol(CAN_HandleTypeDef* canhandle, uint16_t chassis_id);//构造函数
		void Run(void); //!<缺省参数 以上次的模式控制底盘
		void Run(int16_t Vx, int16_t Vy, int16_t Omega, uint8_t Mode, 
			uint8_t All_flags = 0);
		void Safe(void);     //!底盘安全模式
	private:
		int16_t Last_Vx=0, Last_Vy=0, Last_Omega=0;//之前的值，用于缺省参数时的使用
		uint8_t Last_Mode=22;
};

#endif
