
#ifndef _SIXDOF_H_
#define _SIXDOF_H_

#include <Windows.h> 

#include "../config/inihelper.h"
#include "../TYPE_DEF.H"

//平台运动最大角度
#define VISION_MAX_DEG 9.5
#define MAX_DEG 10.5
#define DEG_SCALE 0.01
//平台运动最大位移
#define VISION_MAX_XYZ 95.0
#define MAX_XYZ 105
#define XYZ_SCALE 0.1
// 平台运动最大频率
#define MAX_HZ 20
#define MAX_PHASE 360

#define DLL_NAME "Sixdofdll2010.dll"
#define FUNC_NAME "Control"
#define FUNC_NAGIN  6
#define FUNC_NAGOUT 6

//缸的数量
#define AXES_COUNT 6

//电机到丝杠的导程 一圈大概是6mm, 缸最大伸长量为700mm
#define MAX_POLE_LENGTH 700 / 2.0
#define PULSE_PER_R 8388608
#define MM_PER_R 20.00
#define LENGTH_TO_PULSE_SCALE 419430.4

#define FREEDOM_X_INDEX     0
#define FREEDOM_Y_INDEX     1
#define FREEDOM_Z_INDEX     2
#define FREEDOM_ROLL_INDEX  3
#define FREEDOM_PITCH_INDEX 4
#define FREEDOM_YAW_INDEX   5

//限位函数
#define RANGE(x,min,max)   (((x)<(min) ? (min) : ( (x)>(max) ? (max):(x) )))

typedef enum
{
	SIXDOF_STATUS_BOTTOM,
	SIXDOF_STATUS_READY,
	SIXDOF_STATUS_MIDDLE,
	SIXDOF_STATUS_RUN,
	SIXDOF_STATUS_ISRISING,
	SIXDOF_STATUS_ISFALLING,
}SixDofPlatformStatus;

extern char * SixDofStatusText[];

typedef struct 
{
	//单位1mm
	double X;
	//单位1mm
	double Y;
	//单位1mm
	double Z;
	//单位1度
	double Yaw;
	//单位1度
	double Roll;
	//单位1度
	double Pitch;
}SixdofPackage;

class SixDof
{
public:
	SixDof();
	~SixDof();
	bool IsRegister;
	void GetSixServoPulseNum(I32 * pulse_arr, double * data);
	void GetSixServoPulseNum(double * pulse_arr, double * data);
	void GetSixServoPulseNum(double * pulse_arr, SixdofPackage * data);
	double X;
	double Y;
	double Z;
	double Roll;
	double Yaw;
	double Pitch;
	double PoleLength[AXES_COUNT];
private:
	void DataInit();
	void DllInit();
	double* DuGuControl(double x, double y, double z, double roll, double pitch, double yaw);
};



#endif

