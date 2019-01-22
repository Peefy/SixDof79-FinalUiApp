
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
	int flagstart;    // ���ݰ���ͷ(999)
	int zxs;          //1. �ɻ��ڻ�����ϵXb������߼��ٶ�   (+/-3.3f ��/��2)
	int zxus;         //2. �ɻ��ڻ�����ϵXb������߼��ٶ�   (+/-3.3f ��/��2)
	int hxs;          //3. �ɻ��ڻ�����ϵZb������߼��ٶ�   (+/-3.3f ��/��2)
	int hxas;         //4. �ɻ�����Ǽ��ٶ�                (+/-3.3f ��/��2)
	int cxs;          //5. �ɻ������Ǽ��ٶ�                (+/-3.3f ��/��2)
	int cxas;         //6. �ɻ�ƫ���Ǽ��ٶ�                (+/-3.3f ��/��2)
	int fyjs;         //7. �ɻ��ٶ��ڻ�����ϵXb����ķ���   (+/-3.3f ��/��)
	int fyjas;        //8. �ɻ��ٶ��ڻ�����ϵYb����ķ���   (+/-3.3f ��/��)
	int cqjs;         //9. �ɻ��ٶ��ڻ�����ϵZb����ķ���   (+/-3.3f ��/��)
	int cqjas;        //10. �ɻ�������ٶ�   (+/-3.3f ��/��)
	int hxjs;         //11. �ɻ��������ٶ�   (+/-3.3f ��/��)
	int hxjas;        //12. �ɻ�ƫ�����ٶ�   (+/-3.3f ��/��)
	int fyj;          //13. �ɻ�������       (+/-3.3f ��)
	int cqj;          //14. �ɻ������       (+/-3.3f ��)
	int phj;          //15. �ɻ�������ظ߶�   (+/-3.3f ��)
	int scl;          //16. �ɻ�ǰ����ܽӵص��֧����(����)   (+/-3.3f ǧkgf)
	int txzt;         //17. �ɻ�������ܽӵص��֧����(����) (+/-3.3f ǧkgf)
	int mh;           //18. �ɻ�������ܽӵص��֧����(����) (+/-3.3f ǧkgf)
	int percent;      //19. �ɻ�ǰ����ܽӵص�Ľӵ��ٶ�(����) (+/-3.3f ��/��)
	int hxfl;         //20. �ɻ�������ܽӵص�Ľӵ��ٶ�(����) (+/-3.3f ��/��)
	int acxfl;        //21. �ɻ�������ܽӵص�Ľӵ��ٶ�(����) (+/-3.3f ��/��)
	int zxrs;         //22. �ɻ��ӵش����ٶ� (+/-3.3f ��/��)
	int hxrs;         //23. ��ɲ����        (+/-3.3f ǧkgf)
	int cxrs;         //24. ��ɲ����        (+/-3.3f ǧkgf)
	int by1;          //25. �����ڻ�����ϵXb����������ٶ� (+/-3.3f ��/��)
	int by2;          //26. �����ڻ�����ϵYb����������ٶ� (+/-3.3f ��/��)
	int by3;          //27. �����ڻ�����ϵZb����������ٶ� (+/-3.3f ��/��)
	int by4;          //28. �����ڻ�����ϵXb��������ż��ٶ� (+/-3.3f ��/��2)
	int by5;          //29. �����ڻ�����ϵYb��������ż��ٶ� (+/-3.3f ��/��2)
	int by6;          //30. �����ڻ�����ϵZb��������ż��ٶ� (+/-3.3f ��/��2)
	int by7;          //31. ǰ�����֧��ѹ����         (+/-3.3f ��)
	int by8;          //32. �������֧��ѹ����         (+/-3.3f ��)
	int by9;          //33. �������֧��ѹ����         (+/-3.3f ��)
	int by10;         //34. ����λ��                  (+/-3.3f ��)
	int by11;         //35. �󷢶���ת�ٰٷ���         (+/-3.3f 0.0-1.0)
	int by12;         //36. �ҷ�����ת�ٰٷ���         (+/-3.3f 0.0-1.0)
	int by13;         //37. ǰ�����λ��               (+/-3.3f 0.0��-90.0��)
	int by14;         //38. �������λ��               (+/-3.3f 0.0��-90.0��)
	int by15;         //39. �������λ��               (+/-3.3f 0.0��-90.0��)
	int by16;         //40. ����ϵͳ�����־           (+/-3.3f 1.0:����;0.0:�ⶳ)
	int by17;         //41. �ܶ����־                 (+/-3.3f 1.0:����;0.0:�ⶳ)
	int by18;         //42. �ɻ��ڵ����־             (+/-3.3f 1.0:����;0.0:����)
	int by19;         //43. ǰ��������ϱ�־           (+/-3.3f 1.0:����;0.0:����)
	int by20;         //44. ����������ϱ�־           (+/-3.3f 1.0:����;0.0:����)
	int by21;         //45. ����������ϱ�־           (+/-3.3f 1.0:����;0.0:����)
	int by22;         //46. �󷢶�������־           (+/-3.3f 1.0:���;0.0:Ϩ��)
	int by23;         //47. �ҷ���������־           (+/-3.3f 1.0:���;0.0:Ϩ��)
	int by24;         //48. �˶����ñ�־               (+/-3.3f 1.0:��;0.0:��)
	int by25;         //49. ����ƽ�ɱ�־               (+/-3.3f 1.0:��;0.0:��)
	int by26;         //50. �������                  (���Ϊ1.0ƽ̨����,����Ϊ0.0)
	int by27;         //51. ������λ��
	int by28;         //52. У������
	int by29;         //53. �����
	int by30;         //54. �ɻ�ӭ��(��)
	int by31;         //55. ����ϵ��
	int by32;         //56. �ɻ�����λ��
	int by33;         //57. ƽ������λ��(��)
	int by34;         //58. �ܵ�����ֲڶ�
	int by35;         //59. 999.0���������
	int by36;         //60. ��־(����111,ֹͣ777,����222,�ѻ�333,��ͣ555,�ָ�666)
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

