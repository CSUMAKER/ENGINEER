#ifndef _MAK_FILTER_H
#define _MAK_FILTER_H

#include "makos_includes.h"
#include "arm_math.h"

/* 
 * NOTES: n Dimension means the state is n dimension, 
 * measurement always 1 dimension 
 */

/* 1 Dimension */
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

extern void kalman1_init(kalman1_state *state, float init_x, float init_p);
extern float kalman1_filter(kalman1_state *state, float z_measure);
extern void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2]);
extern float kalman2_filter(kalman2_state *state, float z_measure);


/*********************************************************************************************/

#define mat         arm_matrix_instance_f32
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

typedef struct
{
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
	float filtered_value[2];				//滤波后的值(初始赋零)
	float xhat_data[2], xhatminus_data[2];	//当前状态、下一刻状态（初始赋零）
	float z_data[2];						//测量值（实时传递）
	float P_data[4],Pminus_data[4];			//预测误差、下一次的预测误差（可以手调找最佳初值，迭代更新，初值越好收敛至稳定的时间越快）
	float K_data[4];						//卡尔曼增益（初始置零，迭代更新）
	float A_data[4],AT_data[4];				//预测矩阵（根据物理学方程推算）
	float H_data[4],HT_data[4];				//测量矩阵（根据测量量确定）
	float Q_data[4];						//处理噪声协方差（手调）
	float R_data[4];						//测量噪声协方差（手调）
} kalman_filter_init_t;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
	

#endif
