#ifndef __UART_NET_H
#define __UART_NET_H

#include "stdint.h"
#include "../config/inihelper.h"
#include "../communication/SerialPort.h"

signed char	SendUARTMessageLength(const unsigned long ulChannelNo, const char chrMessage[],const unsigned short usLen);
unsigned short CollectUARTData(const unsigned long ulChannelNo, char chrUARTBufferOutput[]);
signed char OpenCOMDevice(const unsigned long ulPortNo);
signed char SetBaundrate(const unsigned long ulPortNo,const unsigned long ulBaundrate);
signed char OpenCOMDevice(const unsigned long ulPortNo,const unsigned long ulBaundrate);
void CloseCOMDevice(void);

#define COM_BASE_BUFFER_MAX   4096

template <typename T>
class BaseCom
{
public:
	BaseCom(uint8_t headerOne, uint8_t headerTwo);
	~BaseCom();
	uint8_t DefaultPackageHeaderOne;
	uint8_t DefaultPackageHeaderTwo;
	int PortNumber;
	int BaudRate;
	bool IsRecievedData;
	bool IsNotConnect;
	bool Open(int portNumber, int baudRate);
	bool Close();
	T GetDataFromCom();
	T GatherDataFromCom();
	T Data;
private:
	BaseCom();
	CSerialPort serialPort;
	int NotConnectCount;
	int NotConnectUpCount;
protected:
	unsigned int uiReceived;
};

template <typename T>
BaseCom<T>::BaseCom() : uiReceived(0), NotConnectCount(0)
{
	NotConnectUpCount = 5;
	DefaultPackageHeaderOne = 0x55;
	DefaultPackageHeaderTwo = 0xAA;
}

template <typename T>
BaseCom<T>::BaseCom(uint8_t headerOne, uint8_t headerTwo)
{
	DefaultPackageHeaderOne = headerOne;
	DefaultPackageHeaderTwo = headerTwo;
	NotConnectCount = 0;
	IsRecievedData = false;
}

template <typename T>
BaseCom<T>::~BaseCom()
{
	
}

template <typename T>
bool BaseCom<T>::Open(int portNumber, int baudRate)
{
	bool result;
	int baud;
	int portnum;
	config::ReadAll(result, baud, portnum);
	PortNumber = portnum;
	BaudRate = baud;
	return serialPort.InitPort(portnum, baud) == true;
	//return OpenCOMDevice(portnum, baud) == 1;
}

template <typename T>
bool BaseCom<T>::Close()
{
	//serialPort.ClosePort();
	//CloseCOMDevice();
	return true;
}

template <typename T>
T BaseCom<T>::GetDataFromCom()
{
	static char uchrTemp[COM_BASE_BUFFER_MAX];
	static unsigned char chrTemp[COM_BASE_BUFFER_MAX];
	static unsigned char ucRxCnt = 0;	
	static unsigned short usRxLength = 0;
	static int length = sizeof(T);
	auto nowlength = serialPort.GetBytesInCOM();
	unsigned char cRecved;
	for (int i = 0; i < nowlength; ++i)
	{
		serialPort.ReadChar(cRecved);
		uchrTemp[i] = cRecved;
	}
	//auto nowlength = CollectUARTData(PortNumber, uchrTemp);
	memcpy(chrTemp, uchrTemp, sizeof(unsigned char) * COM_BASE_BUFFER_MAX);
	usRxLength += nowlength;
	while (usRxLength >= length)
	{
		if (chrTemp[0] != DefaultPackageHeaderOne)
		{
			usRxLength--;
			memcpy(&chrTemp[0], &chrTemp[1], usRxLength);                        
			continue;
		}
		memcpy(&Data, &chrTemp[0], length);
		usRxLength -= length;
		memcpy(&chrTemp[0], &chrTemp[length], usRxLength);    
		IsRecievedData = true;
	}
	return Data;
}

template <typename T>
T BaseCom<T>::GatherDataFromCom()
{
	// 数据帧处理相关
	static int uiRemainLength = 0;
	static unsigned long ulFrameNum = 0;
	static unsigned long ulFrameErr = 0;
	static int length = sizeof(T);
	static UCHAR chData[COM_BASE_BUFFER_MAX * 11] = {0};
	static UCHAR *pch = chData;
	int i;
	UCHAR chReadData[COM_BASE_BUFFER_MAX] = {0};
	uiReceived = serialPort.GetBytesInCOM();
	unsigned char cRecved = 0;
	for (int i = 0; i < uiReceived; ++i)
	{
		serialPort.ReadChar(cRecved);
		chReadData[i] = cRecved;
	}
	if(uiReceived == 0)
	{
		if(++NotConnectCount > NotConnectUpCount)
		{
			NotConnectCount = NotConnectUpCount;
			IsRecievedData = false;
		}
		return Data;		
	}
	memcpy(pch, chReadData, uiReceived);    //将数据置于chData[]中
	i = 0;
	int j = uiRemainLength + uiReceived - length;
	while(i <= j)
	{
		UCHAR *pData = &chData[i];
		if((pData[0] == DefaultPackageHeaderOne))
		{       	
			ulFrameNum++;
			memcpy(&Data, &pData[0], length);
			IsRecievedData = true;
			if (--NotConnectCount < 0)
			{
				NotConnectCount = 0;
			}
			i += length;		
			continue;
		}
		else
		{
			i += 1;
		}
	}
	uiRemainLength += uiReceived - i;
	if(uiRemainLength != 0)
	{
		memcpy(chReadData, &chData[i], uiRemainLength);
		memcpy(chData, chReadData, uiRemainLength);
	}
	pch = &chData[uiRemainLength];
	return Data;
}

#endif
