#ifndef _PEEFY_SIXDOF_H_
#define _PEEFY_SIXDOF_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#ifdef SIXDOFDLL2010_EXPORTS
#define SIXDOFDLL2010_API __declspec(dllexport)
#else
#define SIXDOFDLL2010_API __declspec(dllimport)
#endif

/*
����̬�ǻ�ȡ�׵��쳤�г̣�λ�Ƶ�λΪ����(mm)���Ƕȵ�λΪ�Ƕ�(deg),������3-3�ṹ��Gough-Stewart�����ɶ�ƽ̨
@para
x : X��λ�� mm
y : y��λ�� mm
z : z��λ�� mm
roll : ����� deg
yaw : ƫ���� deg
pitch : ������ deg
@return
double[0] ��1���쳤��
double[1] ��2���쳤��
double[2] ��3���쳤��
double[3] ��4���쳤��
double[4] ��5���쳤��
double[5] ��6���쳤��
*/
SIXDOFDLL2010_API double* Control(double x, double y, double z, double roll, double yaw, double pitch);
/*
��ȡ ���������Ŀռ����꣬����ԭ��Ϊƽ̨���������λ����ƽ̨���ģ���λΪ����(mm)
@return
double[0] xλ�� mm
double[1] yλ�� mm
double[2] zλ�� mm
*/
SIXDOFDLL2010_API double* GetTopPosition(int index);
/*
��ȡ �ײ������Ŀռ����꣬����ԭ��Ϊƽ̨���������λ����ƽ̨���ģ���λΪ����(mm)
@para
index : �˵�������
@return
double[0] xλ�� mm
double[1] yλ�� mm
double[2] zλ�� mm
*/
SIXDOFDLL2010_API double* GetBottomPosition(int index);
/*
���� ���������Ŀռ����꣬����ԭ��Ϊƽ̨���������λ����ƽ̨���ģ���λΪ����(mm)
@para
index : �˵�������
x : x����
y : y����
z : z����
@return
bool �Ƿ����óɹ�
*/
SIXDOFDLL2010_API bool SetTopPosition(int index, double x, double y, double z);
/*
���� �ײ������Ŀռ����꣬����ԭ��Ϊƽ̨���������λ����ƽ̨���ģ���λΪ����(mm)
@para
index : �˵�������
x : x����
y : y����
z : z����
@return
bool �Ƿ����óɹ�
*/
SIXDOFDLL2010_API bool SetBottomPosition(int index, double x, double y, double z);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !_PEEFY_SIXDOF_H_
