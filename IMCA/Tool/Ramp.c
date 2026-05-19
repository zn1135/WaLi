#include "Ramp.h"
#include "user_lib.h"

// 卡尔曼滤波器初始化结构体实例
one_kalman_filter_init_t kalman_pitch;
one_kalman_filter_init_t kalman_velocity;
one_kalman_filter_init_t kalman_left_fn;
one_kalman_filter_init_t kalman_right_fn;
one_kalman_filter_init_t kalman_position;
/*˲*/


first_order_filter_type_t mouse_x;
first_order_filter_type_t mouse_y;
first_order_filter_type_t xChassisCmdSlowSetVx;
first_order_filter_type_t xChassisCmdSlowSetVy;
first_order_filter_type_t xChassisCmdSlowSetVw;
// 矩阵值变量
float matrix_value1;
float matrix_value2;

/*˲ر*/
// 滑动窗口滤波器参数
#define window_size 3
float window[window_size];
int window_index = 0;

/*************************************************************************************************************
  * @brief          滑动平均值滤波器
  * @author         RM
  * @param[in]      newvalue 新输入值
  * @retval         滤波后的输出值
*************************************************************************************************************/
float MeanFilter(float newvalue)
{
  // 将新值放入窗口数组
  window[window_index % window_size] = newvalue;
  
  // 初始化累加和
  float sum = 0;

  // 计算窗口内所有值的总和
  for(int i=0;i<window_size;++i)
  {
    sum += window[(window_index - i + window_size) % window_size];
  }

  // 更新窗口索引
  window_index = (window_index +1) % window_size;

  // 计算移动平均值
  float output = sum / window_size;
  
  return output;
}

/*************************************************************************************************************
  * @brief          一阶滤波器初始化
  * @author         RM
  * @param[in]      first_order_filter_type 滤波器结构体指针
  * @param[in]      frame_period 滤波时间常数，单位 s
  * @param[in]      num 滤波参数数组
  * @retval         无
*************************************************************************************************************/
void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/*************************************************************************************************************
  * @brief          一阶滤波器计算
  * @author         RM
  * @param[in]      first_order_filter_type 滤波器结构体指针
  * @param[in]      input 输入值
  * @retval         无
*************************************************************************************************************/
void FirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

/**
  * @brief          斜坡函数初始化
  * @author         RM
  * @param[in]      ramp_source_type 斜坡函数结构体指针
  * @param[in]      frame_period 时间常数，单位 s
  * @param[in]      max 最大值
  * @param[in]      min 最小值
  * @retval         无
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out =0.0f;
}

/**
  * @brief          斜坡函数计算，根据输入值和时间常数计算斜坡变化值，单位为每秒变化量
  * @author         RM
  * @param[in]      ramp_source_type 斜坡函数结构体指针
  * @param[in]      input 输入值
  * @retval         无
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }

}

/****************************************************************************
* 函数名称 : SlopeSlow
* 功能描述 : 斜坡缓慢变化
* 说明     : 每1ms调用一次，slow_Inc为变化量
* 参数说明 :
* 返回值说明 :
*****************************************************************************/
void SlopeSlow(float *rec , float target , float slow_Inc)
{
	if(*rec < target)
	{
		*rec += slow_Inc;
		if(*rec > target)
		{
			*rec = target;
		}
	}
	else if(*rec > target)
	{
		*rec -= slow_Inc;
		if(*rec < target)
		{
			*rec = target;
		}
	}
}

/**
	* @brief  卡尔曼滤波器
  * @param  data 卡尔曼滤波器结构体指针
  * @param  ResrcData 原始数据
  * @param  ProcessNoise_Q 过程噪声Q
  * @param  MeasureNoise_R 测量噪声R
  * @retval 滤波后的数据
  * @other 	Q为过程噪声协方差矩阵，反映系统模型的不准确程度
						R为观测噪声协方差矩阵，反映测量数据的统计特性
  * @note   2023.5.14
  */
float  KalmanFilter(one_kalman_filter_init_t * data,const float ResrcData,float ProcessNoise_Q,float MeasureNoise_R)
{
   float R = MeasureNoise_R;
   float Q = ProcessNoise_Q;
   float x_mid;
   float x_now;
   float p_mid;
   float p_now;
   float kg;
   x_mid=data->x_last;
   p_mid=data->p_last+Q;
   kg=p_mid/(p_mid+R);
   x_now=x_mid+kg*(ResrcData-x_mid);

   p_now=(1-kg)*p_mid;
   data->p_last = p_now;
   data->x_last = x_now;
   return x_now;
}

// 卡尔曼滤波器参数初始化
kalman_filter_t kalman_param_imu;
kalman_filter_init_t kalman_param_imu_init =
{
  .P_data = {1, 0,
						 0, 1},
  .A_data = {1, 0.001,
						 0, 1},
  .H_data = {1, 0,
						 0, 1},
  .Q_data = {10*10*0.001*0.001, 10*10*0.001,
						 10*10*0.001, 10*10},
  .R_data = {20*20, 0,
						 0, 20*20}
};

/**
  * @brief 卡尔曼滤波器初始化
  * @param F 卡尔曼滤波器结构体指针
  * @param I 卡尔曼滤波器初始化参数结构体指针
  * @retval 无
  */
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{

  mat_init(&F->xhat,2,1,(float *)I->xhat_data);//状态向量
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
  mat_init(&F->z,2,1,(float *)I->z_data);//观测向量
  mat_init(&F->A,2,2,(float *)I->A_data);//状态转移矩阵
  mat_init(&F->H,2,2,(float *)I->H_data);//观测矩阵
  mat_init(&F->Q,2,2,(float *)I->Q_data);//过程噪声协方差矩阵
  mat_init(&F->R,2,2,(float *)I->R_data);//观测噪声协方差矩阵
  mat_init(&F->P,2,2,(float *)I->P_data);//估计协方差矩阵
  mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);//先验估计协方差矩阵
  mat_init(&F->K,2,2,(float *)I->K_data);//卡尔曼增益矩阵
  mat_init(&F->AT,2,2,(float *)I->AT_data);//状态转移矩阵转置
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT,2,2,(float *)I->HT_data);//观测矩阵转置
  mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}

/**
  * @brief 卡尔曼滤波器计算
  * @param F 卡尔曼滤波器结构体指针
  * @param signal1 信号1
  * @param signal2 信号2
  * @retval 滤波后的值数组指针
  */
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);//
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);//

  F->z.pData[0] = signal1;//z(k)
  F->z.pData[1] = signal2;//z(k)

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P);//
  mat_mult(&F->Pminus, &F->HT, &TEMP);//
  mat_mult(&TEMP, &F->P, &F->K);//

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&F->Q, &F->P, &TEMP);//
  mat_mult(&TEMP, &F->Pminus, &F->P);

  matrix_value1 = F->xhat.pData[0];
  matrix_value2 = F->xhat.pData[1];
	
  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  return F->filtered_value;
}



void MouseRampInit(void)
{
	const static float mouse_x_order_filter[1] = {MOUSE_X_NUM};
	const static float mouse_y_order_filter[1] = {MOUSE_Y_NUM};

	//用一阶滤波代替斜坡函数生成
	FirstOrderFilterInit(&mouse_x, GIMBAL_CONTROL_TIME, mouse_x_order_filter);
	FirstOrderFilterInit(&mouse_y, GIMBAL_CONTROL_TIME, mouse_y_order_filter);
}