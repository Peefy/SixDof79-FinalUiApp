
#include "stdafx.h"
#include "illusion.h"

IllusionDataAdapter::IllusionDataAdapter()
{
}

IllusionDataAdapter::~IllusionDataAdapter()
{
}

void IllusionDataAdapter::RenewData()
{
	static char recieveBuffer[ILLUSION_RECIEVE_BUFFER_MAX];
	static int length = sizeof(IllusionPackage);
	static unsigned short usRxLength = 0;
	int nowlength = udpClient.RecieveFrom(recieveBuffer);
	usRxLength += nowlength;
	while (usRxLength >= length)
	{
		auto lastData = Data;
		memcpy(&Data, &recieveBuffer[0], length);
		if (Data.flagstart != ILLUSION_FLAG_START_INT32 || 
			Data.flagend != ILLUSION_FLAG_END_INT32)
		{
			Data = lastData;
		}
		else
		{
			RenewInnerData();
		}
		usRxLength -= length; 
		IsRecievedData = true;
	}
}

void IllusionDataAdapter::SendData()
{

}

void IllusionDataAdapter::RenewInnerData()
{
	Yaw = 0;
	Pitch = Data.cqj;
	Roll = Data.cqjas;
	X = 0;
	Y = 0;
	Z = 0;
}
