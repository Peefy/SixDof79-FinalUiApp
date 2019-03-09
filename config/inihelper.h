#pragma once

#ifndef __INI_HELPER_H_
#define __INI_HELPER_H_

#include <Windows.h> 
#include <fstream>

#include "../TYPE_DEF.H"
#include "../communication/sixdof.h"

#define JSON_PLATFORM_FILE_NAME           "platformconfig.json"

#define JSON_PlaneAboveHingeLength_KEY        "upToUpHingeLength"    //��ƽ�浽�Ͻ�֧������
#define JSON_PlaneAboveBottomLength_KEY       "upToDownHingeLength"  //��ƽ�浽�½�֧������
#define JSON_CircleTopRadius_KEY              "upCircleD"            //СԲֱ��
#define JSON_CircleBottomRadius_KEY           "downCircleD"          //��Բֱ��
#define JSON_DistanceBetweenHingeTop_KEY      "upHingesDis"          //�����Ͻ�֧�����ľ���
#define JSON_DistanceBetweenHingeBottom_KEY   "downHingesDis"        //�����½�֧�����ľ���

namespace config {
    void GenerateDefaultConfigFile();
    void ReadAll(bool& result, int& baud, int& portnum);
	int ReadScale();
	void RecordStatusAndPulse(char* status, int statusInt, I32* pulse);
	void RecordStatusAndPulse(char* status, int statusInt, double* pulse);
	void ReadStatusAndPulse(int& statusInt, I32* pulse);
	void ReadStatusAndPulse(int& statusInt, double* pulse);
	bool ReadIsAutoStopAndMiddle();
}

#endif // !__INI_HELPER_H_

