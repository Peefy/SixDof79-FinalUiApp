
#ifndef _ILLUSION_H_ 
#define _ILLUSION_H_ 

#include "stdint.h"
#include "udp.h"

#define UDP_PORT        9000
#define UDP_IP_STRING   "192.168.0.104"

#define ILLUSION_RECIEVE_BUFFER_MAX   1024
#define ILLUSION_FLAG_START_INT32     999
#define ILLUSION_FLAG_END_INT32       888

#define ILLUSION_CTL_CMD_START_INT32       111
#define ILLUSION_CTL_CMD_STOP_INT32        777
#define ILLUSION_CTL_CMD_CONNECT_INT32     222
#define ILLUSION_CTL_CMD_DISCONNECT_INT32  333
#define ILLUSION_CTL_CMD_PAUSE_INT32       555
#define ILLUSION_CTL_CMD_RECOVER_INT32     666

#pragma pack (1)
typedef struct
{
	int flagstart;    // 数据包标头(999)
	int zxs;          //1. 飞机在机体轴系Xb方向的线加速度   (+/-3.3f 米/秒2)
	int zxus;         //2. 飞机在机体轴系Xb方向的线加速度   (+/-3.3f 米/秒2)
	int hxs;          //3. 飞机在机体轴系Zb方向的线加速度   (+/-3.3f 米/秒2)
	int hxas;         //4. 飞机横滚角加速度                (+/-3.3f 度/秒2)
	int cxs;          //5. 飞机俯仰角加速度                (+/-3.3f 度/秒2)
	int cxas;         //6. 飞机偏航角加速度                (+/-3.3f 度/秒2)
	int fyjs;         //7. 飞机速度在机体轴系Xb方向的分量   (+/-3.3f 米/秒)
	int fyjas;        //8. 飞机速度在机体轴系Yb方向的分量   (+/-3.3f 米/秒)
	int cqjs;         //9. 飞机速度在机体轴系Zb方向的分量   (+/-3.3f 米/秒)
	int cqjas;        //10. 飞机横滚角速度   (+/-3.3f 度/秒)
	int hxjs;         //11. 飞机俯仰角速度   (+/-3.3f 度/秒)
	int hxjas;        //12. 飞机偏航角速度   (+/-3.3f 度/秒)
	int fyj;          //13. 飞机俯仰角       (+/-3.3f 度)
	int cqj;          //14. 飞机横滚角       (+/-3.3f 度)
	int phj;          //15. 飞机重心离地高度   (+/-3.3f 米)
	int scl;          //16. 飞机前起落架接地点的支反力(垂向)   (+/-3.3f 千kgf)
	int txzt;         //17. 飞机左起落架接地点的支反力(垂向) (+/-3.3f 千kgf)
	int mh;           //18. 飞机右起落架接地点的支反力(垂向) (+/-3.3f 千kgf)
	int percent;      //19. 飞机前起落架接地点的接地速度(垂向) (+/-3.3f 米/秒)
	int hxfl;         //20. 飞机左起落架接地点的接地速度(垂向) (+/-3.3f 米/秒)
	int acxfl;        //21. 飞机右起落架接地点的接地速度(垂向) (+/-3.3f 米/秒)
	int zxrs;         //22. 飞机接地垂向速度 (+/-3.3f 米/秒)
	int hxrs;         //23. 左刹车力        (+/-3.3f 千kgf)
	int cxrs;         //24. 右刹车力        (+/-3.3f 千kgf)
	int by1;          //25. 紊流在机体轴系Xb方向的线扰速度 (+/-3.3f 米/秒)
	int by2;          //26. 紊流在机体轴系Yb方向的线扰速度 (+/-3.3f 米/秒)
	int by3;          //27. 紊流在机体轴系Zb方向的线扰速度 (+/-3.3f 米/秒)
	int by4;          //28. 紊流在机体轴系Xb方向的线扰加速度 (+/-3.3f 度/秒2)
	int by5;          //29. 紊流在机体轴系Yb方向的线扰加速度 (+/-3.3f 度/秒2)
	int by6;          //30. 紊流在机体轴系Zb方向的线扰加速度 (+/-3.3f 度/秒2)
	int by7;          //31. 前起落架支柱压缩量         (+/-3.3f 米)
	int by8;          //32. 左起落架支柱压缩量         (+/-3.3f 米)
	int by9;          //33. 右起落架支柱压缩量         (+/-3.3f 米)
	int by10;         //34. 襟翼位置                  (+/-3.3f 度)
	int by11;         //35. 左发动机转速百分数         (+/-3.3f 0.0-1.0)
	int by12;         //36. 右发动机转速百分数         (+/-3.3f 0.0-1.0)
	int by13;         //37. 前起落架位置               (+/-3.3f 0.0度-90.0度)
	int by14;         //38. 左起落架位置               (+/-3.3f 0.0度-90.0度)
	int by15;         //39. 右起落架位置               (+/-3.3f 0.0度-90.0度)
	int by16;         //40. 飞行系统冻结标志           (+/-3.3f 1.0:冻结;0.0:解冻)
	int by17;         //41. 总冻结标志                 (+/-3.3f 1.0:冻结;0.0:解冻)
	int by18;         //42. 飞机在地面标志             (+/-3.3f 1.0:地面;0.0:空中)
	int by19;         //43. 前起落架收上标志           (+/-3.3f 1.0:收上;0.0:放下)
	int by20;         //44. 左起落架收上标志           (+/-3.3f 1.0:收上;0.0:放下)
	int by21;         //45. 右起落架收上标志           (+/-3.3f 1.0:收上;0.0:放下)
	int by22;         //46. 左发动机点火标志           (+/-3.3f 1.0:点火;0.0:熄火)
	int by23;         //47. 右发动机点火标志           (+/-3.3f 1.0:点火;0.0:熄火)
	int by24;         //48. 运动重置标志               (+/-3.3f 1.0:是;0.0:非)
	int by25;         //49. 请求平飞标志               (+/-3.3f 1.0:是;0.0:非)
	int by26;         //50. 气流冲击                  (冲击为1.0平台启动,其他为0.0)
	int by27;         //51. 扰流板位置
	int by28;         //52. 校正空速
	int by29;         //53. 马赫数
	int by30;         //54. 飞机迎角(度)
	int by31;         //55. 升力系数
	int by32;         //56. 飞机重心位置
	int by33;         //57. 平均缝翼位置(度)
	int by34;         //58. 跑道表面粗糙度
	int by35;         //59. 999.0是裸控数据
	int by36;         //60. 标志(启动111,停止777,联机222,脱机333,暂停555,恢复666)
	int flagend;      //61. (888)
}IllusionPackage;
#pragma pack () 

class IllusionDataAdapter
{
public:
	IllusionDataAdapter();
	~IllusionDataAdapter();
	void RenewData();
	void SendData();
	bool IsRecievedData;
	double Roll;
	double Yaw;
	double Pitch;
	double X;
	double Y;
	double Z;
private:
	IllusionPackage Data;
	void RenewInnerData();
};



#endif // !

