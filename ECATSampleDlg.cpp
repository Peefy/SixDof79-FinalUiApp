
#include "StdAfx.h"
#include "ECATSample.h"
#include "ECATSampleDlg.h"

#include <math.h>
#include <fstream>
#include <deque>

#include "DialogRegister.h"

#include "EtherCAT_DLL.h"
#include "EtherCAT_DLL_Err.h"

#include "chart.h"

#include "Sixdofdll2010.h"

#include "communication/sixdof.h"
#include "communication/communication.h"
#include "communication/delta.h"
#include "communication/SerialPort.h"

#include "control/sensor.h"
#include "control/pid.h"
#include "control/kalman_filter.h"
#include "control/landvision.h"

#include "ui/uiconfig.h"

#include "util/model.h"

#include "register\register.h"

#include "Sixdofdll2010.h"

#include "glut.h"
#include "opengl/sixdofopenglhelper.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

using namespace std;

#define TIMER_MS 10

#define SIXDOF_CONTROL_DELEY 50
#define SCENE_THREAD_DELAY 1000
#define SENSOR_THREAD_DELAY 1000
#define DATA_BUFFER_THREAD_DELAY 1000

#define DDA_UP_COUNT 400
#define DDA_ONCE_COUNT 100

#define CHIRP_TIME 5
#define SHOCK_VAL 5
#define SHOCK_HZ 5

bool enableShock = true;
bool enableChirp = true;
bool stopSCurve = false;

void SixdofControl();
void SensorRead();

U8 InitialFlag = 0;

bool closeDataThread = true;
bool stopAndMiddle = true;

volatile HANDLE DataThread;
volatile HANDLE SensorThread;
volatile HANDLE SceneThread;
volatile HANDLE DataBufferThread;

I32 pulse_test[AXES_COUNT];
I32 lastStartPulse[AXES_COUNT];
double pulse_cal[AXES_COUNT];

SixDof sixdof;
MotionControl delta;
DataPackage data;
DataPackage scenedata = {0};
LandVision vision;

SixDofPlatformStatus status = SIXDOF_STATUS_BOTTOM;
SixDofPlatformStatus lastStartStatus = SIXDOF_STATUS_BOTTOM;

Sensor sensor[SENSOR_COUNT] = 
{
	Sensor(SENSOR_PORT1),
	Sensor(SENSOR_PORT2),
	Sensor(SENSOR_PORT3),
};

SensorInfo_t sensorInfo[SENSOR_COUNT];
SensorInfo_t avrSensorInfo;
bool isSensorReady;

kalman2_state angle_filter[SENSOR_AXES_NUM];

double controlOut[FREEDOM_NUM];

CChartCtrl m_ChartCtrl1; 
CChartLineSerie *pLineSerie1;
CChartLineSerie *pLineSerie2;
CChartLineSerie *pLineSerie3;
CChartLineSerie *pLineSerie4;
CChartLineSerie *pLineSerie5;
CChartLineSerie *pLineSerie6;

#if PID_CONTROL_ENABLE 

double Angle_P = 1;
double Angle_I = 0.05;
double Angle_D = 0.1;
double XYZ_P = 1;
double XYZ_I = 0.1;
double XYZ_D = 0.1;

PID_Type sensorPidControler[FREEDOM_NUM] = 
{
	{ XYZ_P, XYZ_I, XYZ_D, -MAX_XYZ / XYZ_SCALE, MAX_XYZ / XYZ_SCALE },
	{ XYZ_P, XYZ_I, XYZ_D, -MAX_XYZ / XYZ_SCALE, MAX_XYZ / XYZ_SCALE },
	{ XYZ_P, XYZ_I, XYZ_D, -MAX_XYZ / XYZ_SCALE, MAX_XYZ / XYZ_SCALE },
	{ Angle_P, Angle_I, Angle_D, -MAX_DEG / DEG_SCALE, MAX_DEG / DEG_SCALE },
	{ Angle_P, Angle_I, Angle_D, -MAX_DEG / DEG_SCALE, MAX_DEG / DEG_SCALE },
	{ Angle_P, Angle_I, Angle_D, -MAX_DEG / DEG_SCALE, MAX_DEG / DEG_SCALE }
};

#define PID_INNER_OUT_MAX 999999999

double Gyro_P = 1;
double Gyro_I = 0.1;
double Gyro_D = 0.1;

PID_Type sensorGyroPidControler[ANGLE_COUNT] = 
{
	{ Gyro_P, Gyro_I, Gyro_D, -PID_INNER_OUT_MAX, PID_INNER_OUT_MAX },
	{ Gyro_P, Gyro_I, Gyro_D, -PID_INNER_OUT_MAX, PID_INNER_OUT_MAX },
	{ Gyro_P, Gyro_I, Gyro_D, -PID_INNER_OUT_MAX, PID_INNER_OUT_MAX }
};

#endif

double sin_time_pulse = 0;
double sin_time_pulse_delta = 0.01;

bool isStart = false;
bool isAutoInit = true;
bool isTest = true;

double testVal[FREEDOM_NUM];
double testHz[FREEDOM_NUM];

double chartBottomAxisPoint[CHART_POINT_NUM] = { 0 };

double chartXValPoint[CHART_POINT_NUM] = { 0 };
double chartYValPoint[CHART_POINT_NUM] = { 0 };
double chartZValPoint[CHART_POINT_NUM] = { 0 };
double chartRollValPoint[CHART_POINT_NUM] = { 0 };
double chartPitchValPoint[CHART_POINT_NUM] = { 0 };
double chartYawValPoint[CHART_POINT_NUM] = { 0 };

double runTime = 0;
double chartTime = 0;
double controltime = 0;
double dataChartTime = 0;

double t = 0;

DWORD WINAPI DataTransThread(LPVOID pParam)
{
	while (true)
	{
		SixdofControl();
	}
	return 0;
}

DWORD WINAPI SensorInfoThread(LPVOID pParam)
{
	while (true)
	{
		SensorRead();
	}
	return 0;
}

DWORD WINAPI SceneInfoThread(LPVOID pParam)
{
	while (true)
	{
		Sleep(SCENE_THREAD_DELAY);
	}
	return 0;
}

DWORD WINAPI DataBufferInfoThread(LPVOID pParam)
{
	while (true)
	{
		Sleep(DATA_BUFFER_THREAD_DELAY);
	}
	return 0;
}

void CloseThread()
{
	try
	{
		if (DataThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(DataThread);
			DataThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (SensorThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(SensorThread);
			SensorThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (SceneThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(SceneThread);
			SceneThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (DataBufferThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(DataBufferThread);
			DataBufferThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
}

void OpenThread()
{
	DataThread = (HANDLE)CreateThread(NULL, 0, DataTransThread, NULL, 0, NULL);
	SensorThread = (HANDLE)CreateThread(NULL, 0, SensorInfoThread, NULL, 0, NULL);
	SceneThread = (HANDLE)CreateThread(NULL, 0, SceneInfoThread, NULL, 0, NULL);
	DataBufferThread = (HANDLE)CreateThread(NULL, 0, DataBufferInfoThread, NULL, 0, NULL);
}

void SensorRead()
{
	for (int i = 0; i < SENSOR_COUNT; ++i)
	{
		sensorInfo[i] = sensor[i].ProvideSensorInfo();
	}
	isSensorReady = true;
	avrSensorInfo = GetAverageSenrorValue(sensorInfo, SENSOR_COUNT, &isSensorReady);
	Sleep(SENSOR_THREAD_DELAY);
}

bool isCsp = false;
U16 Counter = 0;

void SixdofControl()
{
	vision.RenewVisionData();
	if(closeDataThread == false)
	{	
		U16 upCount = DDA_UP_COUNT;
		DWORD start_time = 0;
		start_time = GetTickCount();
		auto delay = SIXDOF_CONTROL_DELEY;
		if (isCsp == false)
		{
			delay = SIXDOF_CONTROL_DELEY;
			Counter = delta.GetDDACount();
			if (Counter < upCount)
			{
				I32 dis[AXES_COUNT] = {DIS_PER_R, DIS_PER_R, DIS_PER_R, DIS_PER_R, DIS_PER_R, DIS_PER_R};
				if (true)
				{
					for (auto i = 0; i < DDA_ONCE_COUNT; ++i)
					{
						// 正弦测试运动
						if (isTest == true)
						{
							double nowHz[AXES_COUNT] = {testHz[0], testHz[1], testHz[2], testHz[3], testHz[4], testHz[5]};
							double chirpTime = CHIRP_TIME;
							double pi = 3.1415926;
							double nowt = t;
							double x = 0;
							double y = 0;
							double z = 0;
							double roll = 0;
							double pitch = 0;
							double yaw = 0;
							if (t <= chirpTime && enableChirp == true)
							{
								for (auto i = 0; i < AXES_COUNT; ++i)
								{
									nowHz[i] = t * testHz[i] / chirpTime / chirpTime; 
								}
								x = sin(2 * pi * nowHz[0] * nowt) * testVal[0];
								y = sin(2 * pi * nowHz[1] * nowt) * testVal[1];
								z = sin(2 * pi * nowHz[2] * nowt) * testVal[2];
								roll = sin(2 * pi * nowHz[3] * nowt) * testVal[3];
								pitch = sin(2 * pi * nowHz[4] * nowt) * testVal[4];
								yaw = sin(2 * pi * nowHz[5] * nowt) * testVal[5];
							}		
							else if (enableChirp == true)
							{
								nowt = t - chirpTime;
								x = sin(2 * pi * nowHz[0] * nowt + 2 * pi * testHz[0]) * testVal[0];
								y = sin(2 * pi * nowHz[1] * nowt + 2 * pi * testHz[1]) * testVal[1];
								z = sin(2 * pi * nowHz[2] * nowt + 2 * pi * testHz[2]) * testVal[2];
								roll = sin(2 * pi * nowHz[3] * nowt + 2 * pi * testHz[3]) * testVal[3];
								pitch = sin(2 * pi * nowHz[4] * nowt + 2 * pi * testHz[4]) * testVal[4];
								yaw = sin(2 * pi * nowHz[5] * nowt + 2 * pi * testHz[5]) * testVal[5];
							}
							else
							{
								x = sin(2 * pi * nowHz[0] * nowt) * testVal[0];
								y = sin(2 * pi * nowHz[1] * nowt) * testVal[1];
								z = sin(2 * pi * nowHz[2] * nowt) * testVal[2];
								roll = sin(2 * pi * nowHz[3] * nowt) * testVal[3];
								pitch = sin(2 * pi * nowHz[4] * nowt) * testVal[4];
								yaw = sin(2 * pi * nowHz[5] * nowt) * testVal[5];
							}
							data.X = (int16_t)(x * 10);
							data.Y = (int16_t)(y * 10);
							data.Z = (int16_t)(z * 10);
							data.Roll = (int16_t)(roll * 100);
							data.Pitch = (int16_t)(pitch * 100);
							data.Yaw = (int16_t)(yaw * 100);
							double* pulse_dugu = Control(x, y, z, roll, yaw, pitch);
							for (auto ii = 0; ii < AXES_COUNT; ++ii)
							{
								pulse_cal[ii] = pulse_dugu[ii];
								pulse_cal[ii] *= LENGTH_TO_PULSE_SCALE;
								auto pulse = pulse_cal[ii];
								dis[ii] = (int)pulse;
							}
							t += 0.00095;
							if (stopSCurve == true)
							{
								if ((testHz[0] - x) <= 0.001 && (testHz[1] - y) <= 0.001 && (testHz[2] - z) <= 0.001 && 
									(testHz[3] - roll) <= 0.001 && (testHz[4] - pitch) <= 0.001 && (testHz[5] - yaw) <= 0.001)
									t -= 0.00095;
							}
							delta.SetDDAData(dis);
						}
						// 视景姿态模拟运动
						else
						{
							double pi = 3.1415926;
							double shockVal = SHOCK_VAL;
							double shockHz = SHOCK_HZ;
							data.X = 0;
							data.Y = 0;
							data.Z = 0;
							data.Roll = vision.Roll / DEG_SCALE;
							data.Yaw = vision.Yaw / DEG_SCALE;
							data.Pitch = vision.Pitch / DEG_SCALE;
							auto shockz = sin(2 * pi * shockHz * t) * shockVal;
							auto x = RANGE(data.X * XYZ_SCALE, -MAX_XYZ, MAX_XYZ);
							auto y = RANGE(data.Y * XYZ_SCALE, -MAX_XYZ, MAX_XYZ);
							auto z = RANGE(data.Z * XYZ_SCALE, -MAX_XYZ, MAX_XYZ);
							auto roll = RANGE(data.Roll * DEG_SCALE, -MAX_DEG, MAX_DEG);
							auto pitch = RANGE(data.Pitch * DEG_SCALE, -MAX_DEG, MAX_DEG);
							auto yaw = RANGE(data.Yaw * DEG_SCALE, -MAX_DEG, MAX_DEG);
							z += (enableShock == true ? shockz : 0);
							double* pulse_dugu = Control(x, y, z + shockz, roll, yaw, pitch);
							for (auto ii = 0; ii < AXES_COUNT; ++ii)
							{
								pulse_cal[ii] = pulse_dugu[ii];
								pulse_cal[ii] *= LENGTH_TO_PULSE_SCALE;
								auto pulse = pulse_cal[ii];
								dis[ii] = (int)pulse;
							}
							t += 0.00095;
							delta.SetDDAData(dis);
						}					
					}
				}
			}
		}
		Sleep(delay);
		DWORD end_time = GetTickCount();
		runTime = end_time - start_time;
	}
}

void MoveValPoint()
{
	chartTime += 0.067;
	for (auto i = 0; i < CHART_POINT_NUM; ++i)
	{
		chartBottomAxisPoint[i] = chartBottomAxisPoint[i + 1];

		chartXValPoint[i] = chartXValPoint[i + 1];
		chartYValPoint[i] = chartYValPoint[i + 1];
		chartZValPoint[i] = chartZValPoint[i + 1];
		chartRollValPoint[i] = chartRollValPoint[i + 1];
		chartPitchValPoint[i] = chartPitchValPoint[i + 1];
		chartYawValPoint[i] = chartYawValPoint[i + 1];
	}

	chartBottomAxisPoint[CHART_POINT_NUM - 1] = chartTime;
	chartXValPoint[CHART_POINT_NUM - 1] = data.X * XYZ_SCALE;
	chartYValPoint[CHART_POINT_NUM - 1] = data.Y * XYZ_SCALE; 
	chartZValPoint[CHART_POINT_NUM - 1] = data.Z * XYZ_SCALE;
	chartRollValPoint[CHART_POINT_NUM - 1] = data.Roll * DEG_SCALE;
	chartPitchValPoint[CHART_POINT_NUM - 1] = data.Pitch * DEG_SCALE;
	chartYawValPoint[CHART_POINT_NUM - 1] = data.Yaw * DEG_SCALE;

	m_ChartCtrl1.EnableRefresh(false);

	pLineSerie1->ClearSerie();
	pLineSerie2->ClearSerie();
	pLineSerie3->ClearSerie();
	pLineSerie4->ClearSerie();
	pLineSerie5->ClearSerie();
	pLineSerie6->ClearSerie();

	pLineSerie1->AddPoints(chartBottomAxisPoint, chartXValPoint, CHART_POINT_NUM);
	pLineSerie2->AddPoints(chartBottomAxisPoint, chartYValPoint, CHART_POINT_NUM);
	pLineSerie3->AddPoints(chartBottomAxisPoint, chartZValPoint, CHART_POINT_NUM);
	pLineSerie4->AddPoints(chartBottomAxisPoint, chartRollValPoint, CHART_POINT_NUM);
	pLineSerie5->AddPoints(chartBottomAxisPoint, chartPitchValPoint, CHART_POINT_NUM);
	pLineSerie6->AddPoints(chartBottomAxisPoint, chartYawValPoint, CHART_POINT_NUM);

	m_ChartCtrl1.EnableRefresh(true);

}

CECATSampleDlg::CECATSampleDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CECATSampleDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDI_ICON1);
}

void CECATSampleDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CHART_CTRL, m_ChartCtrl1);
}

BEGIN_MESSAGE_MAP(CECATSampleDlg, CDialog)
	//{{AFX_MSG_MAP(CECATSampleDlg)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_InitialCard, OnBTNInitialCard)
	ON_BN_CLICKED(IDC_BTN_FindSlave, OnBTNFindSlave)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_CHK_SVON, OnChkSvon)
	ON_BN_CLICKED(IDC_CHK_ABS, OnChkAbs)
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BTN_Rise, &CECATSampleDlg::OnBnClickedBtnRise)
	ON_BN_CLICKED(IDC_BTN_Middle, &CECATSampleDlg::OnBnClickedBtnMiddle)
	ON_BN_CLICKED(IDC_BTN_Start, &CECATSampleDlg::OnBnClickedBtnStart)
	ON_BN_CLICKED(IDC_BTN_StopMe, &CECATSampleDlg::OnBnClickedBtnStopme)
	ON_BN_CLICKED(IDC_BTN_Down, &CECATSampleDlg::OnBnClickedBtnDown)
	ON_BN_CLICKED(IDOK, &CECATSampleDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BTN_CONNECT, &CECATSampleDlg::OnBnClickedBtnConnect)
	ON_BN_CLICKED(IDC_BTN_Resetme, &CECATSampleDlg::OnBnClickedBtnResetme)
	ON_BN_CLICKED(IDC_BTN_DISCONNECT, &CECATSampleDlg::OnBnClickedBtnDisconnect)
	ON_WM_SHOWWINDOW()
	ON_BN_CLICKED(IDC_BTN_SINGLE_UP, &CECATSampleDlg::OnBnClickedBtnSingleUp)
	ON_BN_CLICKED(IDC_BTN_SINGLE_DOWN, &CECATSampleDlg::OnBnClickedBtnSingleDown)
	ON_WM_KEYDOWN()
	ON_WM_CHAR()
	ON_BN_CLICKED(IDC_BUTTON_TEST, &CECATSampleDlg::OnBnClickedButtonTest)
	ON_BN_CLICKED(IDC_BUTTON_EXIT, &CECATSampleDlg::OnBnClickedButtonExit)
	ON_BN_CLICKED(IDC_BUTTON_TEST3, &CECATSampleDlg::OnBnClickedButtonTest3)
	ON_BN_CLICKED(IDC_BUTTON_STOP_TEST, &CECATSampleDlg::OnBnClickedButtonStopTest)
END_MESSAGE_MAP()

void CECATSampleDlg::ChartInit()
{
	CRect rect, rectChart;
	GetDlgItem(IDC_CHART_CTRL)->GetWindowRect(&rect);
	ScreenToClient(rect);
	rectChart = rect;
	rectChart.top = rect.bottom + 3;
	rectChart.bottom = rectChart.top + rect.Height();

	CChartAxis *pAxis = NULL;
	pAxis = m_ChartCtrl1.CreateStandardAxis(CChartCtrl::BottomAxis);
	pAxis->SetAutomatic(true);
	pAxis = m_ChartCtrl1.CreateStandardAxis(CChartCtrl::LeftAxis);
	pAxis->SetAutomatic(true);

	m_ChartCtrl1.GetTitle()->AddString(_T(CHART_TITLE));

	m_ChartCtrl1.
		GetLeftAxis()->
		GetLabel()->
		SetText(_T(CHART_LEFT_AXIS_TITLE));

	m_ChartCtrl1.EnableRefresh(false);

	m_ChartCtrl1.RemoveAllSeries();
	m_ChartCtrl1.GetLegend()->SetVisible(true);

	pLineSerie1 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie1->SetSeriesOrdering(poNoOrdering);
	pLineSerie1->SetWidth(2);
	pLineSerie1->SetName(_T(IDC_STATIC_X_VAL_SHOW_TEXT));
	
	pLineSerie2 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie2->SetSeriesOrdering(poNoOrdering);
	
	pLineSerie2->SetWidth(2);
	pLineSerie2->SetName(_T(IDC_STATIC_Y_VAL_SHOW_TEXT)); 

	pLineSerie3 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie3->SetSeriesOrdering(poNoOrdering);
	pLineSerie3->SetWidth(2);
	pLineSerie3->SetName(_T(IDC_STATIC_Z_VAL_SHOW_TEXT));

	pLineSerie4 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie4->SetSeriesOrdering(poNoOrdering);
	pLineSerie4->SetWidth(2);
	pLineSerie4->SetName(_T(IDC_STATIC_ROLL_VAL_SHOW_TEXT));

	pLineSerie5 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie5->SetSeriesOrdering(poNoOrdering); 
	pLineSerie5->SetWidth(2);
	pLineSerie5->SetName(_T(IDC_STATIC_PITCH_VAL_SHOW_TEXT));

	pLineSerie6 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie6->SetSeriesOrdering(poNoOrdering);  
	pLineSerie6->SetWidth(2);
	pLineSerie6->SetName(_T(IDC_STATIC_YAW_VAL_SHOW_TEXT));

	m_ChartCtrl1.EnableRefresh(true);

}

void CECATSampleDlg::AppInit()
{
	int statusTemp = 0;
	auto isRegister = TestAppIsRegister();
	if (isRegister == false)
	{
		DialogRegister * dlg = new DialogRegister(this);
		dlg->DoModal();
		dlg->Create(IDD_DIALOG_REGISTER, this);
		dlg->ShowWindow(SW_SHOW);
		delete dlg;
		if (DialogRegister::IsRegister == false)
		{
			OnBnClickedOk();
		}
		else
		{
			MessageBox(_T("注册成功！"));
		}
	}
	config::ReadStatusAndPulse(statusTemp, lastStartPulse);
	stopAndMiddle = config::ReadIsAutoStopAndMiddle();
	lastStartStatus = (SixDofPlatformStatus)statusTemp;

	SetDlgItemText(IDC_EDIT_X_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_Y_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_Z_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_ROLL_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_PITCH_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_YAW_VAL, _T("0"));

	SetDlgItemText(IDC_EDIT_X_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_Y_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_Z_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_ROLL_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_PITCH_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_YAW_HZ, _T("0"));

	CDialog::SetWindowTextW(_T(WINDOW_TITLE));
	GetDlgItem(IDC_BTN_Start)->SetWindowTextW(_T(IDC_BTN_START_SHOW_TEXT));
	GetDlgItem(IDC_BTN_SINGLE_UP)->SetWindowTextW(_T(IDC_BTN_SINGLE_UP_SHOW_TEXT));
	GetDlgItem(IDC_BTN_SINGLE_DOWN)->SetWindowTextW(_T(IDC_BTN_SINGLE_DOWN_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_POSE)->SetWindowTextW(_T(IDC_STATIC_POSE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_LENGTH)->SetWindowTextW(_T(IDC_STATIC_LENGTH_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_SENSOR)->SetWindowTextW(_T(IDC_STATIC_SENSOR_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_X_VAL)->SetWindowTextW(_T(IDC_STATIC_X_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Y_VAL)->SetWindowTextW(_T(IDC_STATIC_Y_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Z_VAL)->SetWindowTextW(_T(IDC_STATIC_Z_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_ROLL_VAL)->SetWindowTextW(_T(IDC_STATIC_ROLL_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_PITCH_VAL)->SetWindowTextW(_T(IDC_STATIC_PITCH_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_YAW_VAL)->SetWindowTextW(_T(IDC_STATIC_YAW_VAL_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_X_HZ)->SetWindowTextW(_T(IDC_STATIC_X_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Y_HZ)->SetWindowTextW(_T(IDC_STATIC_Y_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Z_HZ)->SetWindowTextW(_T(IDC_STATIC_Z_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_ROLL_HZ)->SetWindowTextW(_T(IDC_STATIC_ROLL_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_PITCH_HZ)->SetWindowTextW(_T(IDC_STATIC_PITCH_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_YAW_HZ)->SetWindowTextW(_T(IDC_STATIC_YAW_HZ_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_TEST)->SetWindowTextW(_T(IDC_STATIC_TEST_SHOW_TEXT));
	GetDlgItem(IDC_BUTTON_TEST)->SetWindowTextW(_T(IDC_BUTTON_TEST_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_APP_STATUS)->SetWindowTextW(_T(CORPORATION_NAME));
	GetDlgItem(IDC_STATIC_APP_TITLE)->SetWindowTextW(_T(APP_TITLE));
	CFont* font = new CFont();
	font->CreatePointFont(APP_TITLE_FONT_SIZE, _T("Times New Roman"));
	GetDlgItem(IDC_STATIC_APP_TITLE)->SetFont(font);
	ChartInit();
	//com.Start();
	for (auto i = 1; i <= AXES_COUNT; ++i)
	{
		CString xx;
		xx.Format(_T("%d"), i);
		((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->AddString(xx);
	}
	((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->SetCurSel(0);
	OpenThread();
	vision.Open(VISION_PORT, VISION_BAUDRATE);
}

double CECATSampleDlg::GetCEditNumber(int cEditId)
{
	CString str;
	GetDlgItemText(cEditId, str);
	auto val = _tstof(str);
	return val;
}

void CECATSampleDlg::OnShowWindow(BOOL bShow, UINT nStatus)
{
	CDialog::OnShowWindow(bShow, nStatus);
}

BOOL CECATSampleDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	SetIcon(m_hIcon, TRUE);			
	SetIcon(m_hIcon, FALSE);	
	
	AppInit();
	InitOpenGlControl();
	SetTimer(0, TIMER_MS, NULL);
	return TRUE;  
}

int isShowSingleUpDown = 0;
BOOL CECATSampleDlg::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN)
	{
		if (pMsg->wParam == VK_SHIFT)
		{
			isShowSingleUpDown = !isShowSingleUpDown;
			((CButton*)GetDlgItem(IDC_BTN_SINGLE_UP))->ShowWindow(isShowSingleUpDown);
			((CButton*)GetDlgItem(IDC_BTN_SINGLE_DOWN))->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_FindSlave)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_EDIT_SlaveNum)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_CBO_SingleNo)->ShowWindow(isShowSingleUpDown);

			GetDlgItem(IDC_CHK_SVON)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_Resetme)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_CONNECT)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_DISCONNECT)->ShowWindow(isShowSingleUpDown);

			GetDlgItem(IDC_BTN_InitialCard)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_STATIC)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_CBO_CardNo)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_STATIC_SHOW_INIT)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_EDIT_InitialStatus)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_SHOW)->ShowWindow(!isShowSingleUpDown);
		}
	}
	else if (pMsg->message == WM_KEYUP)
	{

	}
	return CDialog::PreTranslateMessage(pMsg);
}

BOOL CECATSampleDlg::PreCreateWindow(CREATESTRUCT& cs)
{
	cs.style |= WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
	return CECATSampleDlg::PreCreateWindow(cs);
}

void CECATSampleDlg::InitOpenGlControl()
{
	CWnd *pWnd = GetDlgItem(IDC_SHOW);
	hrenderDC = ::GetDC(pWnd->GetSafeHwnd());
	if(SetWindowPixelFormat(hrenderDC)==FALSE) 
		return; 
	if(CreateViewGLContext(hrenderDC)==FALSE) 
		return; 
	OpenGlLightInit();
}

BOOL CECATSampleDlg::SetWindowPixelFormat(HDC hDC) 
{ 
	PIXELFORMATDESCRIPTOR pixelDesc; 
	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR); 
	pixelDesc.nVersion = 1; 
	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER | PFD_STEREO_DONTCARE; 
	pixelDesc.iPixelType = PFD_TYPE_RGBA; 
	pixelDesc.cColorBits = 32; 
	pixelDesc.cRedBits = 8; 
	pixelDesc.cRedShift = 16; 
	pixelDesc.cGreenBits = 8; 
	pixelDesc.cGreenShift = 8; 
	pixelDesc.cBlueBits = 8; 
	pixelDesc.cBlueShift = 0; 
	pixelDesc.cAlphaBits = 0; 
	pixelDesc.cAlphaShift = 0; 
	pixelDesc.cAccumBits = 64; 
	pixelDesc.cAccumRedBits = 16; 
	pixelDesc.cAccumGreenBits = 16; 
	pixelDesc.cAccumBlueBits = 16; 
	pixelDesc.cAccumAlphaBits = 0; 
	pixelDesc.cDepthBits = 32; 
	pixelDesc.cStencilBits = 8; 
	pixelDesc.cAuxBuffers = 0; 
	pixelDesc.iLayerType = PFD_MAIN_PLANE; 
	pixelDesc.bReserved = 0; 
	pixelDesc.dwLayerMask = 0; 
	pixelDesc.dwVisibleMask = 0; 
	pixelDesc.dwDamageMask = 0; 
	PixelFormat = ChoosePixelFormat(hDC,&pixelDesc); 
	if(PixelFormat==0) // Choose default 
	{ 
		PixelFormat = 1; 
		if(DescribePixelFormat(hDC,PixelFormat, 
			sizeof(PIXELFORMATDESCRIPTOR),&pixelDesc)==0) 
		{ 
			return FALSE; 
		} 
	} 
	if(SetPixelFormat(hDC,PixelFormat,&pixelDesc)==FALSE) 
	{ 
		return FALSE; 
	} 
	return TRUE; 
} 

BOOL CECATSampleDlg::CreateViewGLContext(HDC hDC) 
{ 
	hrenderRC = wglCreateContext(hDC); 
	if(hrenderRC==NULL) 
		return FALSE; 
	if(wglMakeCurrent(hDC,hrenderRC)==FALSE) 
		return FALSE; 
	return TRUE; 
} 

void CECATSampleDlg::RenderScene()   
{ 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	OpenGL_SetData(data.X * XYZ_SCALE, data.Y * XYZ_SCALE, data.Z * XYZ_SCALE, 
		data.Roll * DEG_SCALE, data.Yaw * DEG_SCALE, data.Pitch * DEG_SCALE);
	RenderSixdofImage();
	SwapBuffers(hrenderDC); 
} 

void CECATSampleDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); 

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		dc.DrawIcon(x, y, m_hIcon);		
	}
	else
	{
		CDialog::OnPaint();
	}
	RenderScene();
}

HCURSOR CECATSampleDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

void CECATSampleDlg::OnBTNInitialCard() 
{
	CString xx;
	InitialFlag = delta.InitCard();
	for (int i = 0; i < SENSOR_COUNT; ++i)
	{
		sensor[i].UpdateOffset();
	}
	if (InitialFlag)
	{
		xx.Format(_T("%d"), delta.CardNo);
		((CComboBox*)GetDlgItem(IDC_CBO_CardNo))->AddString(xx);
		((CComboBox*)GetDlgItem(IDC_CBO_CardNo))->SetCurSel(0);
		((CButton *)GetDlgItem(IDC_BTN_FindSlave))->EnableWindow(TRUE);
	}	
	else
		MessageBox(_T(INIT_FAIL_MESSAGE));
}

void CECATSampleDlg::OnBTNFindSlave() 
{
	if(delta.FindSlave() == true)
		SetDlgItemInt(IDC_EDIT_SlaveNum, delta.SlaveNum);
	else
		MessageBox(_T(NOT_FIND_MOTION_MESSAGE));
}

void CECATSampleDlg::EanbleButton(int isenable)
{	
	((CButton*) GetDlgItem(IDC_CHK_SVON))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Rise))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Middle))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Start))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_StopMe))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Down))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Resetme))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BUTTON_TEST))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BUTTON_STOP_TEST))->EnableWindow(isenable);
}

void CECATSampleDlg::OnTimer(UINT nIDEvent) 
{
	I16 rt;
	U16 InitialDone;
	CString statusStr;

	if (isAutoInit == true)
	{
		isAutoInit = false;
		OnBTNInitialCard();	
	}
	MoveValPoint();
	RenderScene();
	statusStr.Format(_T("x:%d y:%d z:%d y:%d a:%d b:%d time:%.2f count:%d"), data.X, data.Y, data.Z,
		data.Yaw, data.Pitch, data.Roll, runTime, Counter);
	SetDlgItemText(IDC_EDIT_Pose, statusStr);

	statusStr.Format(_T("1:%.2f 2:%.2f 3:%.2f 4:%.2f 5:%.2f 6:%.2f"), 
		sixdof.PoleLength[0], sixdof.PoleLength[1], sixdof.PoleLength[2],
		sixdof.PoleLength[3], sixdof.PoleLength[4], sixdof.PoleLength[5]);
	SetDlgItemText(IDC_EDIT_Pulse, statusStr);

	statusStr.Format(_T("1:%.1f 2:%.1f 3:%.1f 4:%d 5:%d 6:%d"),
		avrSensorInfo.Roll, avrSensorInfo.Pitch, avrSensorInfo.Yaw,
		scenedata.Roll, scenedata.Pitch, scenedata.Yaw);
	SetDlgItemText(IDC_EDIT_Sensor, statusStr);
	
	delta.CheckStatus(status);
	if(InitialFlag == 0)
	{
		EanbleButton(0);
	}
	if(InitialFlag != 2)
	{
		rt = _ECAT_Master_Check_Initial_Done(delta.CardNo, &InitialDone);
		if (rt == 0)
		{
			if (InitialDone == 0 && InitialFlag != 2)
			{
				SetDlgItemText(IDC_EDIT_InitialStatus, _T(STATUS_INIT_FINISH));
				EanbleButton(1);
				InitialFlag = 2;
				OnChkSvon();
				Sleep(2000);
				if (lastStartStatus == 0){
					delta.ReadAllSwitchStatus();
					OnBnClickedBtnRise();
				}		
				else if(lastStartStatus == SIXDOF_STATUS_READY)
				{
					status = SIXDOF_STATUS_READY;
				}
				else
					delta.PowerOnSelfTest(lastStartStatus, lastStartPulse);
			}		
			else if (InitialDone == 1)
			{
				SetDlgItemText(IDC_EDIT_InitialStatus, _T(STATUS_INIT_ING));
				EanbleButton(0);
			}	
			else if (InitialDone == 99)
			{
				EanbleButton(0);
				SetDlgItemText(IDC_EDIT_InitialStatus, _T(STATUS_INIT_FAIL));
			}
			else
			{

			}			
		}
	}	
	CDialog::OnTimer(nIDEvent);
}

void CECATSampleDlg::OnChkSvon() 
{
	delta.ServoAllOnOff(true);
}

void CECATSampleDlg::OnOK() 
{
	delta.ServoStop();
	delta.Close();

	CDialog::OnOK();
}

void CECATSampleDlg::OnChkAbs() 
{
	CButton *lpBtnPtr = (CButton *)GetDlgItem(IDC_CHK_ABS);
}

void CECATSampleDlg::OnBnClickedBtnRise()
{	
	if (status != SIXDOF_STATUS_BOTTOM)
	{
		MessageBox(_T(SIXDOF_NOT_BOTTOM_MESSAGE));
		return;
	}
	delta.DownUsingHomeMode();
	Sleep(100);
	delta.ReadAllSwitchStatus();
	if (delta.IsAllAtBottom() == false)
	{
		MessageBox(_T(SIXDOF_NOT_BOTTOM_AND_RISE_MESSAGE));
		return;
	}	
	status = SIXDOF_STATUS_ISRISING;	
	delta.ResetAlarm();
	Sleep(100);
	delta.ResetStatus();
	Sleep(100);
	delta.Rise();
}

void CECATSampleDlg::OnBnClickedBtnMiddle()
{
	if (stopAndMiddle == true)
	{
		if (status != SIXDOF_STATUS_READY)
		{
			MessageBox(_T(SIXDOF_NOT_MIDDLE_MESSAGE));
			return;
		}
	}
	else
	{
		if (status != SIXDOF_STATUS_MIDDLE)
		{
			MessageBox(_T(SIXDOF_NOT_MIDDLE_MESSAGE));
			return;
		}
	}
	status = SIXDOF_STATUS_READY;
	delta.MoveToZeroPulseNumber();
}

void CECATSampleDlg::OnBnClickedBtnStart()
{
	if (status != SIXDOF_STATUS_READY)
	{
	    MessageBox(_T(SIXDOF_NOT_BEGIN_MESSAGE));
		return;
	}
	status = SIXDOF_STATUS_RUN;
	for (int i = 0; i < SENSOR_COUNT; ++i)
	{
		sensor[i].UpdateOffset();
	}
	delta.EnableDDA();
	delta.ServoStop();
	delta.RenewNowPulse();
	delta.ResetStatus();
	delta.GetMotionAveragePulse();
	isTest = false;
	sin_time_pulse = 0;
	t = 0;
	dataChartTime = 0;
	closeDataThread = false;
	isStart = true;	
}

void CECATSampleDlg::OnBnClickedBtnStopme()
{
	stopSCurve = true;
	Sleep(1000); //等待DDA数据运动完毕
	closeDataThread = true;
	delta.ServoStop();
	delta.DisableDDA();
	if (status == SIXDOF_STATUS_RUN)
	{
		if (stopAndMiddle == true)
		{
			delta.MoveToZeroPulseNumber();
			status = SIXDOF_STATUS_READY;
		}
		else
		{
			status = SIXDOF_STATUS_MIDDLE;
		}
	}
	ResetDefaultData(&data);
}

void CECATSampleDlg::OnBnClickedBtnDown()
{	
	status = SIXDOF_STATUS_ISFALLING;
	delta.DownUsingHomeMode();
}

void CECATSampleDlg::OnBnClickedOk()
{
	vision.Close();
	CloseThread();
	delta.ServoStop();
	delta.ServoAllOnOff(false);
	delta.LockServo();
	Sleep(10);
	delta.Close();
	try
	{
		for (int i = 0;i < SENSOR_COUNT;++i)
			sensor[i].closePort();
	}
	catch (CException * e)
	{
		e->ReportError();
	}
	CDialog::OnOK();
}

void CECATSampleDlg::OnBnClickedBtnConnect()
{

}

void CECATSampleDlg::OnBnClickedBtnResetme()
{
	delta.ResetStatus();
}

void CECATSampleDlg::OnBnClickedBtnDisconnect()
{

}

void CECATSampleDlg::OnBnClickedBtnSingleUp()
{
	auto index = ((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->GetCurSel();
	delta.ResetAlarmSingle(index);
	delta.ServoSingleMove(index, DIS_PER_R, 0);
}

void CECATSampleDlg::OnBnClickedBtnSingleDown()
{
	auto index = ((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->GetCurSel();
	delta.ResetAlarmSingle(index);
	delta.ServoSingleMove(index, -DIS_PER_R, 0);
}

void CECATSampleDlg::OnBnClickedButtonTest()
{
	//位移单位mm 角度单位 度
	auto xval = RANGE(GetCEditNumber(IDC_EDIT_X_VAL), -MAX_XYZ, MAX_XYZ); 
	auto yval = RANGE(GetCEditNumber(IDC_EDIT_Y_VAL), -MAX_XYZ, MAX_XYZ);
	auto zval = RANGE(GetCEditNumber(IDC_EDIT_Z_VAL), -MAX_XYZ, MAX_XYZ);
	auto rollval = RANGE(GetCEditNumber(IDC_EDIT_ROLL_VAL), -MAX_DEG, MAX_DEG);
	auto pitchval = RANGE(GetCEditNumber(IDC_EDIT_PITCH_VAL), -MAX_DEG, MAX_DEG);
	auto yawval = RANGE(GetCEditNumber(IDC_EDIT_YAW_VAL), -MAX_DEG, MAX_DEG);
	//频率单位1hz
	auto xhz = RANGE(GetCEditNumber(IDC_EDIT_X_HZ), 0, MAX_HZ);
	auto yhz = RANGE(GetCEditNumber(IDC_EDIT_Y_HZ), 0, MAX_HZ);
	auto zhz = RANGE(GetCEditNumber(IDC_EDIT_Z_HZ), 0, MAX_HZ);
	auto rollhz = RANGE(GetCEditNumber(IDC_EDIT_ROLL_HZ), 0, MAX_HZ);
	auto pitchhz = RANGE(GetCEditNumber(IDC_EDIT_PITCH_HZ), 0, MAX_HZ);
	auto yawhz = RANGE(GetCEditNumber(IDC_EDIT_YAW_HZ), 0, MAX_HZ);

	testVal[0] = xval;
	testVal[1] = yval;
	testVal[2] = zval;
	testVal[3] = rollval;
	testVal[4] = pitchval;
	testVal[5] = yawval;

	testHz[0] = xhz;
	testHz[1] = yhz;
	testHz[2] = zhz;
	testHz[3] = rollhz;
	testHz[4] = pitchhz;
	testHz[5] = yawhz;
	stopSCurve = false;
	isCsp = false;
	
	if (status != SIXDOF_STATUS_READY)
	{
		MessageBox(_T(SIXDOF_NOT_BEGIN_MESSAGE));
		return;
	}
	status = SIXDOF_STATUS_RUN;
	for (int i = 0; i < SENSOR_COUNT; ++i)
	{
		sensor[i].UpdateOffset();
	}
	if (isCsp == false)
	{
		delta.EnableDDA();
	}
	delta.ServoStop();
	delta.RenewNowPulse();
	delta.ResetStatus();
	delta.GetMotionAveragePulse();
	isTest = true;
	t = 0;
	dataChartTime = 0;
	closeDataThread = false;
	isStart = true;
}

void CECATSampleDlg::OnBnClickedButtonExit()
{
	OnBnClickedOk();
}

void CECATSampleDlg::OnBnClickedButtonTest3()
{
	//位移单位mm 角度单位 度
	auto xval = RANGE(GetCEditNumber(IDC_EDIT_X_VAL), -MAX_XYZ, MAX_XYZ);
	auto yval = RANGE(GetCEditNumber(IDC_EDIT_Y_VAL), -MAX_XYZ, MAX_XYZ);
	auto zval = RANGE(GetCEditNumber(IDC_EDIT_Z_VAL), -MAX_XYZ, MAX_XYZ);
	auto rollval = RANGE(GetCEditNumber(IDC_EDIT_ROLL_VAL), -MAX_DEG, MAX_DEG);
	auto pitchval = RANGE(GetCEditNumber(IDC_EDIT_PITCH_VAL), -MAX_DEG, MAX_DEG);
	auto yawval = RANGE(GetCEditNumber(IDC_EDIT_YAW_VAL), -MAX_DEG, MAX_DEG);
	//频率单位1hz
	auto xhz = RANGE(GetCEditNumber(IDC_EDIT_X_HZ), 0, MAX_HZ);
	auto yhz = RANGE(GetCEditNumber(IDC_EDIT_Y_HZ), 0, MAX_HZ);
	auto zhz = RANGE(GetCEditNumber(IDC_EDIT_Z_HZ), 0, MAX_HZ);
	auto rollhz = RANGE(GetCEditNumber(IDC_EDIT_ROLL_HZ), 0, MAX_HZ);
	auto pitchhz = RANGE(GetCEditNumber(IDC_EDIT_PITCH_HZ), 0, MAX_HZ);
	auto yawhz = RANGE(GetCEditNumber(IDC_EDIT_YAW_HZ), 0, MAX_HZ);

	testVal[0] = xval;
	testVal[1] = yval;
	testVal[2] = zval;
	testVal[3] = rollval;
	testVal[4] = pitchval;
	testVal[5] = yawval;

	testHz[0] = xhz;
	testHz[1] = yhz;
	testHz[2] = zhz;
	testHz[3] = rollhz;
	testHz[4] = pitchhz;
	testHz[5] = yawhz;

	stopSCurve = false;
	isCsp = false;

	status = SIXDOF_STATUS_RUN;
	for (int i = 0; i < SENSOR_COUNT; ++i)
	{
		sensor[i].UpdateOffset();
	}
	if (isCsp == false)
	{
		delta.EnableDDA();
	}
	delta.ServoStop();
	delta.RenewNowPulse();
	delta.ResetStatus();
	delta.GetMotionAveragePulse();
	isTest = true;
	t = 0;
	dataChartTime = 0;
	closeDataThread = false;
	isStart = true;
}


void CECATSampleDlg::OnBnClickedButtonStopTest()
{
	OnBnClickedBtnStopme();
}
