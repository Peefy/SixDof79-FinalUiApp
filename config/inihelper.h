#pragma once

#ifndef __INI_HELPER_H_
#define __INI_HELPER_H_

#include <Windows.h> 
#include <fstream>

#include "../TYPE_DEF.H"
#include "../communication/sixdof.h"

#define JSON_PLATFORM_FILE_NAME           "platformconfig.json"

#define JSON_PlaneAboveHingeLength_KEY        "upToUpHingeLength"    //上平面到上铰支座距离
#define JSON_PlaneAboveBottomLength_KEY       "upToDownHingeLength"  //上平面到下铰支座距离
#define JSON_CircleTopRadius_KEY              "upCircleD"            //小圆直径
#define JSON_CircleBottomRadius_KEY           "downCircleD"          //大圆直径
#define JSON_DistanceBetweenHingeTop_KEY      "upHingesDis"          //两两上铰支座中心距离
#define JSON_DistanceBetweenHingeBottom_KEY   "downHingesDis"        //两两下铰支座中心距离

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

