#include "mak_filter.h"
/*********************周敬阳推荐的CSDN的卡尔曼一阶与二阶*********************************/
/*
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = 1;
 *     H = 1; 
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 *   state - Klaman filter structure
 *   init_x - initial x state value   
 *   init_p - initial estimated error convariance
 * @outputs 
 * @retval  
 */
void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->H = 1;
    state->q = 2e2;//10e-6;  /* predict noise convariance */
    state->r = 5e2;//10e-5;  /* measure error convariance */
}

/*
 * @brief   
 *   1 Dimension Kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 * @retval  
 *   Estimated result
 */
float kalman1_filter(kalman1_state *state, float z_measure)
{
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}

/*
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0}; 
 *   and @q,@r are valued after prior tests. 
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 * @outputs 
 * @retval  
 */
void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2])
{
    state->x[0]    = init_x[0];
    state->x[1]    = init_x[1];
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];
    //state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = 0.1;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    //state->H       = {1,0};
    state->H[0]    = 1;
    state->H[1]    = 0;
    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0]    = 10e-7;
    state->q[1]    = 10e-7;
    state->r       = 10e-7;  /* estimated error convariance */
}

/*
 * @brief   
 *   2 Dimension kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 *   state->x[0] - Updated state value, Such as angle,velocity
 *   state->x[1] - Updated state value, Such as diffrence angle, acceleration
 *   state->p    - Updated estimated error convatiance matrix
 * @retval  
 *   Return value is equals to state->x[0], so maybe angle or velocity.
 */
float kalman2_filter(kalman2_state *state, float z_measure)
{
    float temp0 = 0.0f;
    float temp1 = 0.0f;
    float temp = 0.0f;

    /* Step1: Predict */
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
    state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
    state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1] + state->q[1];

    /* Step2: Measurement */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
    temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x[0];
}


/*******************知乎上原大疆创新工程师分享的二阶卡尔曼********************/
/*
	匀速预测模型,使用stm32dsp库
	原网址https://zhuanlan.zhihu.com/p/38745950
*/

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
	I->xhat_data[0]=0;
	I->xhat_data[1]=0;
	I->xhatminus_data[0]=0;
	I->xhatminus_data[1]=0;
	I->z_data[0]=0;
	I->z_data[1]=0;
	I->Pminus_data[0]=0;
	I->Pminus_data[1]=0;
	I->Pminus_data[2]=0;
	I->Pminus_data[3]=0;
	I->K_data[0]=0;
	I->K_data[1]=0;
	I->K_data[2]=0;
	I->K_data[3]=0;
	
	mat_init(&F->xhat,2,1,(float *)I->xhat_data);
	//自写代码（不确定正确性，网络上此处省略了）
	mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
	mat_init(&F->z,2,1,(float *)I->z_data);
	mat_init(&F->A,2,2,(float *)I->A_data);
	mat_init(&F->H,2,2,(float *)I->H_data);
	mat_init(&F->Q,2,2,(float *)I->Q_data);
	mat_init(&F->R,2,2,(float *)I->R_data);
	mat_init(&F->P,2,2,(float *)I->P_data);
	mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
	mat_init(&F->K,2,2,(float *)I->K_data);
	mat_init(&F->HT,2,2,(float *)I->K_data);
	mat_init(&F->AT,2,2,(float *)I->K_data);
	F->filtered_value[0]=0;
	F->filtered_value[1]=0;
	//结束
	mat_trans(&F->H, &F->HT);
	mat_trans(&F->A, &F->AT);
}
/**
函数功能：匀速模型的卡尔曼滤波初始化
*/
void uniform_velocitykalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I,float* Q_param,float* R_param)
{
	//预测矩阵
	I->A_data[0]=1;
	I->A_data[1]=1;
	I->A_data[2]=0;
	I->A_data[3]=1;
	//测量矩阵
	I->H_data[0]=1;
	I->H_data[1]=0;
	I->H_data[2]=0;
	I->H_data[3]=1;
	//预测误差
	I->P_data[0]=0.5;
	I->P_data[1]=0;
	I->P_data[2]=0;
	I->P_data[3]=0.998;
	//处理噪声协方差
	I->Q_data[0]=Q_param[0];
	I->Q_data[1]=Q_param[1];
	I->Q_data[2]=Q_param[2];
	I->Q_data[3]=Q_param[3];
	//测量噪声协方差
	I->R_data[0]=R_param[0];
	I->R_data[1]=R_param[1];
	I->R_data[2]=R_param[2];
	I->R_data[3]=R_param[3];
	kalman_filter_init(F,I);
}
/*
函数功能：卡尔曼滤波计算
函数形参：
kalman_filter_t *F 卡尔曼滤波结构体指针，包含了滤波参数
signal1：测量量1,可以是x方向角度
signal2：测量量2，可以是x方向角度的微分
*/
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
	float TEMP_data[4] = {0, 0, 0, 0};
	float TEMP_data21[2] = {0, 0};
	mat TEMP,TEMP21;

	mat_init(&TEMP,2,2,(float *)TEMP_data);
	mat_init(&TEMP21,2,1,(float *)TEMP_data21);

	F->z.pData[0] = signal1;
	F->z.pData[1] = signal2;

	//1. xhat'(k)= A xhat(k-1)
	mat_mult(&F->A, &F->xhat, &F->xhatminus);

	//2. P'(k) = A P(k-1) AT + Q
	mat_mult(&F->A, &F->P, &F->Pminus);
	mat_mult(&F->Pminus, &F->AT, &TEMP);
	mat_add(&TEMP, &F->Q, &F->Pminus);

	//3. K(k) = P'(k) HT / (H P'(k) HT + R)
	mat_mult(&F->H, &F->Pminus, &F->K);
	mat_mult(&F->K, &F->HT, &TEMP);
	mat_add(&TEMP, &F->R, &F->K);

	mat_inv(&F->K, &F->P);
	mat_mult(&F->Pminus, &F->HT, &TEMP);
	mat_mult(&TEMP, &F->P, &F->K);

	//4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	mat_mult(&F->H, &F->xhatminus, &TEMP21);
	mat_sub(&F->z, &TEMP21, &F->xhat);
	mat_mult(&F->K, &F->xhat, &TEMP21);
	mat_add(&F->xhatminus, &TEMP21, &F->xhat);

	//5. P(k) = (1-K(k)H)P'(k)
	mat_mult(&F->K, &F->H, &F->P);
	mat_sub(&F->Q, &F->P, &TEMP);
	mat_mult(&TEMP, &F->Pminus, &F->P);

	F->filtered_value[0] = F->xhat.pData[0];
	F->filtered_value[1] = F->xhat.pData[1];

	return F->filtered_value;
}

