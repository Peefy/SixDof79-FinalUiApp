
#include "stdafx.h"

#include "memory.h"

#include "landvision.h"
#include "Com.h"
#include "../util/crc.h"

LandVision::LandVision() : BaseCom(VISION_RECIEVE_HAEDER, VISION_RECIEVE_HAEDER)
{
	RecievePackageLength = sizeof(LandVisionPackage);
	SendPackageLength = sizeof(LandVisionSendPackage);
	DataInit();
}

LandVision::~LandVision()
{

}

void LandVision::DataInit()
{
	X = 0;
	Y = 0;
	Z = 0;
	XAcc = 0;
	YAcc = 0;
	ZAcc = 0;
	Roll = 0;
	Yaw = 0;
	Pitch = 0;	
}

void LandVision::RenewVisionData()
{
	auto data = this->GetDataFromCom();
	if (this->IsRecievedData == false)
		return;
	Crc = 0;
	Crc += (uint8_t)data.X;
	Crc += (uint8_t)(data.X >> 8);
	Crc += (uint8_t)data.Y;
	Crc += (uint8_t)(data.Y >> 8);
	Crc += (uint8_t)data.Z;
	Crc += (uint8_t)(data.Z >> 8);
	Crc += (uint8_t)data.XAcc;
	Crc += (uint8_t)(data.XAcc >> 8);
	Crc += (uint8_t)data.YAcc;
	Crc += (uint8_t)(data.YAcc >> 8);
	Crc += (uint8_t)data.RoadTypeTemp;
	Crc += (uint8_t)data.RoadType;
	Crc += (uint8_t)data.Pitch;
	Crc += (uint8_t)(data.Pitch >> 8);
	Crc += (uint8_t)data.Roll;
	Crc += (uint8_t)(data.Roll >> 8);
	Crc += (uint8_t)data.ControlByte;
	Crc += (uint8_t)data.NoneByte;
	Crc += (uint8_t)data.CmdPcDataAcksByte;
	Crc += (uint8_t)data.FunctionsByte;
	auto recieveCrc = (uint16_t)((((uint8_t)data.Crc) << 8) + (uint8_t)(data.Crc >> 8));
	if (data.Header == VISION_RECIEVE_HAEDER && data.Tail == VISION_RECIEVE_TAIL && Crc == recieveCrc)
	{
		Y = -(int16_t)((((uint8_t)data.X) << 8) + (uint8_t)(data.X >> 8)) * VISION_XYZ_SCALE;
		X = (int16_t)((((uint8_t)data.Y) << 8) + (uint8_t)(data.Y >> 8)) * VISION_XYZ_SCALE;
		Z = (int16_t)((((uint8_t)data.Z) << 8) + (uint8_t)(data.Z >> 8)) * VISION_XYZ_SCALE;
		YAcc = -(int16_t)((((uint8_t)data.XAcc) << 8) + (uint8_t)(data.XAcc >> 8)) * VISION_ACC_SCALE;
		XAcc = (int16_t)((((uint8_t)data.YAcc) << 8) + (uint8_t)(data.YAcc >> 8)) * VISION_ACC_SCALE;
		ZAcc = 0;
		Roll = (int16_t)((((uint8_t)data.Pitch) << 8) + (uint8_t)(data.Pitch >> 8)) * VISION_ANGLE_SCALE;
		Pitch = -(int16_t)((((uint8_t)data.Roll) << 8) + (uint8_t)(data.Roll >> 8)) * VISION_ANGLE_SCALE;
		Yaw = 0;
		NowRoadType = (RoadType)data.RoadType;
		RecieveState.IsConsoleInitial = VISION_BIT_GET(data.ControlByte, 0);
		RecieveState.IsConsoleZero = VISION_BIT_GET(data.ControlByte, 1);
		RecieveState.IsServoAck = VISION_BIT_GET(data.ControlByte, 2);
		RecieveState.IsAlaAck = VISION_BIT_GET(data.ControlByte, 3);
		RecieveState.IsPwrOn = VISION_BIT_GET(data.ControlByte, 6);
		RecieveState.IsPwrOff = VISION_BIT_GET(data.ControlByte, 7);
		RecieveState.CmdPcDataAcksByte = data.CmdPcDataAcksByte;
		RecieveState.FunctionsByte = data.FunctionsByte;
	}
}

void LandVision::SendVisionData()
{
	auto sendBytes = new char[SendPackageLength];

	LandVisionSendPackage sendPackage;
	sendPackage.Header = VISION_SEND_HAEDER;
	sendPackage.Tail = VISION_SEND_TAIL;
	sendPackage.PulseCount1 = SendState.PoleLength[0];
	sendPackage.PulseCount2 = SendState.PoleLength[1];
	sendPackage.PulseCount3 = SendState.PoleLength[2];
	sendPackage.PulseCount4 = SendState.PoleLength[3];
	sendPackage.PulseCount5 = SendState.PoleLength[4];
	sendPackage.PulseCount6 = SendState.PoleLength[5];

	VISION_BIT_SET_VAL(sendPackage.StateByteOne, 0, SendState.IsServoAlarm[0]);
	VISION_BIT_SET_VAL(sendPackage.StateByteOne, 1, SendState.IsServoAlarm[1]);
	VISION_BIT_SET_VAL(sendPackage.StateByteOne, 2, SendState.IsServoAlarm[2]);
	VISION_BIT_SET_VAL(sendPackage.StateByteOne, 3, SendState.IsServoAlarm[3]);
	VISION_BIT_SET_VAL(sendPackage.StateByteOne, 4, SendState.IsServoAlarm[4]);
	VISION_BIT_SET_VAL(sendPackage.StateByteOne, 5, SendState.IsServoAlarm[5]);
	VISION_BIT_SET_VAL(sendPackage.StateByteOne, 6, SendState.IsServoFitRange[0]);
	VISION_BIT_SET_VAL(sendPackage.StateByteOne, 7, SendState.IsServoFitRange[1]);

	VISION_BIT_SET_VAL(sendPackage.StateByteTwo, 0, SendState.IsServoFitRange[2]);
	VISION_BIT_SET_VAL(sendPackage.StateByteTwo, 1, SendState.IsServoFitRange[3]);
	VISION_BIT_SET_VAL(sendPackage.StateByteTwo, 2, SendState.IsServoFitRange[4]);
	VISION_BIT_SET_VAL(sendPackage.StateByteTwo, 3, SendState.IsServoFitRange[5]);
	VISION_BIT_SET_VAL(sendPackage.StateByteTwo, 4, SendState.IsDeclination[0]);
	VISION_BIT_SET_VAL(sendPackage.StateByteTwo, 5, SendState.IsDeclination[1]);
	VISION_BIT_SET_VAL(sendPackage.StateByteTwo, 6, SendState.IsDeclination[2]);
	VISION_BIT_SET_VAL(sendPackage.StateByteTwo, 7, SendState.IsDeclination[3]);

	VISION_BIT_SET_VAL(sendPackage.StateByteThree, 0, SendState.IsDeclination[4]);
	VISION_BIT_SET_VAL(sendPackage.StateByteThree, 1, SendState.IsDeclination[5]);
	VISION_BIT_SET_VAL(sendPackage.StateByteThree, 2, SendState.IsServoInit[0]);
	VISION_BIT_SET_VAL(sendPackage.StateByteThree, 3, SendState.IsServoInit[1]);
	VISION_BIT_SET_VAL(sendPackage.StateByteThree, 4, SendState.IsServoInit[2]);
	VISION_BIT_SET_VAL(sendPackage.StateByteThree, 5, SendState.IsServoInit[3]);
	VISION_BIT_SET_VAL(sendPackage.StateByteThree, 6, SendState.IsServoInit[4]);
	VISION_BIT_SET_VAL(sendPackage.StateByteThree, 7, SendState.IsServoInit[5]);

	VISION_BIT_SET_VAL(sendPackage.StateByteFour, 0, SendState.IsLengthAlarm);
	VISION_BIT_SET_VAL(sendPackage.StateByteFour, 1, SendState.IsTotalAlarm);
	VISION_BIT_SET_VAL(sendPackage.StateByteFour, 2, SendState.IsServoShrinkAlarm);
	VISION_BIT_SET_VAL(sendPackage.StateByteFour, 3, SendState.IsEMAlarm);
	VISION_BIT_SET_VAL(sendPackage.StateByteFour, 4, SendState.IsAlarmNormal);
	VISION_BIT_SET_VAL(sendPackage.StateByteFour, 5, SendState.IsAlarmHeavy);
	VISION_BIT_SET_VAL(sendPackage.StateByteFour, 6, SendState.IsLockAlarm);
	VISION_BIT_SET_VAL(sendPackage.StateByteFour, 7, SendState.IsServoPowerAlarm);

	memcpy(sendBytes, &sendPackage, SendPackageLength);
	sendPackage.Crc = usMBCRC16(&sendBytes[1], SendPackageLength - 4); 
	memcpy(sendBytes, &sendPackage, SendPackageLength);

	SendUARTMessageLength(VISION_PORT, sendBytes, SendPackageLength);
	delete[] sendBytes;
}

void LandVision::DoConsoleInit()
{

}

void LandVision::DoConsoleZero()
{

}
