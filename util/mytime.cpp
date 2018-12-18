
#include "stdafx.h"
#include "mytime.h"

#include <windows.h>

namespace util
{
	string GetMyTimeString()
	{
		SYSTEMTIME start; 
		GetLocalTime(&start);
		return to_string((long long)start.wYear) + 
			to_string((long long)start.wMonth) + 
			to_string((long long)start.wDay) + 
			to_string((long long)start.wHour) + 
			to_string((long long)start.wMinute) + 
			to_string((long long)start.wSecond);
	}

}

