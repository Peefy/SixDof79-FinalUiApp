
#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include "stdint.h"

typedef struct {
	float x;  /* state */
	float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
	float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
	float q;  /* process(predict) noise convariance */
	float r;  /* measure noise convariance */
	float p;  /* estimated error convariance */
	float gain;
} kalman1_state;

/* 2 Dimension */
typedef struct {
	float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
	float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
	float H[2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
	float q[2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
	float r;        /* measure noise convariance */
	float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
	float gain[2];  /* 2x1 */
} kalman2_state;


float KalmanFilterMPU(float angle_m, float gyro_m);
extern void kalman1_init(kalman1_state *state, float init_x, float init_p);
extern float kalman1_filter(kalman1_state *state, float z_measure);
extern void kalman2_init(kalman2_state *state, float *init_x, float(*init_p)[2]);
extern float kalman2_filter(kalman2_state *state, float z_measure);

#define FILTER_DATA_NUM 256
struct MovingAverageFilterType
{
	//public:
	unsigned char FilterLevel;	
	//private:					
	double DataHistory[FILTER_DATA_NUM];		
	double DataTotal;			
	double Now;
	unsigned char Rear;
}MovingAverageFilter_Type;

double MyMAFilter(MovingAverageFilterType* This, double InputData);

#endif
