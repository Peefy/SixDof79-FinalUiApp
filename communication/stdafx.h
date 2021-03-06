// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <fstream>
#include <process.h>
#include <string>
#include <Windows.h>  

#include "Communication/SerialPort.h"
#include "Communication/communication.h"
#include "Communication/nexecm.h"
#include "Communication/sixdof.h"
#include "Communication/delta.h"

#include "config/inihelper.h"

#include "util/model.h"

#include "TYPE_DEF.H"
#include "EtherCAT_DLL.h"
#include "EtherCAT_DLL_Err.h"



// TODO: 在此处引用程序需要的其他头文件
