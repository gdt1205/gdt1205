//
// Created by 45441 on 2023/7/6.
//


#include "task_gyro.h"
#include "bsp_dwt.h"
#include "QuaternionEKF.h"
#include "kalman_filter_whx.h"
//extern "C"

INS_t INS;
IMU_Param_t IMU_Param;


const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

void INS_Init(void)
{
    IMU_Param.scale[GYX] = 1;
    IMU_Param.scale[GYY] = 1;
    IMU_Param.scale[GYZ] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    // imu heat init


    INS.AccelLPF = 0.0085;
}



void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{

}
float q0=1, q1=0, q2=0, q3=0; // 四元数
float beta = 0.1f;             // Madgwick 滤波器参数

float m_yaw=0,m_yaw_deg=0;

float INS_Task(int16_t acc_x,int16_t acc_y,int16_t acc_z,int16_t gy_x,int16_t gy_y,int16_t gy_z)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        
			const float ACCEL_LSB_PER_G = 8192.0f;
        const float G_TO_MS2 = 9.80665f;
        const float GYRO_LSB_PER_DEG = 65.5f;
        const float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;

			
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // 加速度
		float ax = (acc_x) / 8192.0f;          // g
		float ay = (acc_y) / 8192.0f;
		float az = (acc_z) / 8192.0f;

		float gx = (gy_x / 65.5f) * (3.14/180.0f);  // rad/s
		float gy = (gy_y / 65.5f) * (3.14/180.0f);
		float gz = (gy_z / 65.5f) * (3.14/180.0f);
		
    recipNorm = sqrtf(ax * ax + ay * ay + az * az);
    if(recipNorm == 0.0f) return NAN; // 防止除零
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 梯度下降
    float f1 = 2*(q1*q3 - q0*q2) - ax;
    float f2 = 2*(q0*q1 + q2*q3) - ay;
    float f3 = 2*(0.5 - q1*q1 - q2*q2) - az;
    s0 = -_2q2*f1 + _2q1*f2;
    s1 = _2q3*f1 + _2q0*f2 - _4q1*f3;
    s2 = -_2q0*f1 + _2q3*f2 - _4q2*f3;
    s3 = _2q1*f1 + _2q2*f2;

 
    recipNorm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if(recipNorm == 0.0f) return NAN;
    recipNorm = 1.0f / recipNorm;
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // 四元数导数
    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz) - beta * s0;
    qDot2 = 0.5f * (q0*gx + q2*gz - q3*gy) - beta * s1;
    qDot3 = 0.5f * (q0*gy - q1*gz + q3*gx) - beta * s2;
    qDot4 = 0.5f * (q0*gz + q1*gy - q2*gx) - beta * s3;

    // 四元数
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 正规化
    recipNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    recipNorm = 1.0f / recipNorm;
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;			
			
			m_yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
			m_yaw_deg = (m_yaw *180.f/3.14f);
			    if (m_yaw_deg < 0)
        m_yaw_deg += 360.0f;

			
			
			
			
			
			
//			
//			// 核心函数,EKF更新四元数
//        IMU_QuaternionEKF_Update(INS.Gyro[GYX], INS.Gyro[GYY], INS.Gyro[GYZ], INS.Accel[GYX], INS.Accel[GYY], INS.Accel[GYZ], dt);

//        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

//        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
//        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
//        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
//        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

//        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
//        float gravity_b[3];
//        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
//        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
//        {
//            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
//        }
//        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n


//        /***--------------------------获取最终数据------------------------------***/
//        INS.Yaw = QEKF_INS.Yaw;
//        INS.Pitch = QEKF_INS.Pitch;
//        INS.Roll = QEKF_INS.Roll;
//        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
//        /***--------------------------获取最终数据------------------------------***/


        //LostCounterFeed(GYRO_LOST_OF_SIGNAL);
//        yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, GimbalGyroDataPacket.yaw,
//                                           GimbalGyroDataPacket.gx);//欧拉角和角速度/// 为啥Yaw用的是gx（不应该是gz吗） 是因为yaw轴沿着z轴旋转，所以角速度是x和y方向上的吗
//        GimbalGyroScope.Yaw.KalmanFilterAngle = yaw_kf_result[0];
//        GimbalGyroScope.Yaw.KalmanFilterSpeed = yaw_kf_result[1];
//
//        pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, GimbalGyroDataPacket.pitch,
//                                             GimbalGyroDataPacket.gy);/// 那这里pitch沿着y轴旋转为啥用的y轴的角速度？
//        GimbalGyroScope.Pitch.KalmanFilterAngle = pitch_kf_result[0];
//        GimbalGyroScope.Pitch.KalmanFilterSpeed = pitch_kf_result[1];
//
//        GyroDataConvert(GimbalGyroDataPacket, &GimbalGyroScope);//速度和角度归一化，并得到视觉需要的角度（视觉需要的单位是弧度）
//        LostOfSignalFeed(GYRO_LOST_OF_SIGNAL);//收到数据把标志位清零
//        LostOfSignalFeed(GYRO_LOST_OF_SIGNAL);//收到数据就把标志位清零
//        gyroFlag=ENABLE;//代表收到了陀螺仪数据
    }



    count++;
		return m_yaw_deg;
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[GYX] = c_11 * gyro_temp[GYX] +
              c_12 * gyro_temp[GYY] +
              c_13 * gyro_temp[GYZ];
    gyro[GYY] = c_21 * gyro_temp[GYX] +
              c_22 * gyro_temp[GYY] +
              c_23 * gyro_temp[GYZ];
    gyro[GYZ] = c_31 * gyro_temp[GYX] +
              c_32 * gyro_temp[GYY] +
              c_33 * gyro_temp[GYZ];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[GYX] = c_11 * accel_temp[GYX] +
               c_12 * accel_temp[GYY] +
               c_13 * accel_temp[GYZ];
    accel[GYY] = c_21 * accel_temp[GYX] +
               c_22 * accel_temp[GYY] +
               c_23 * accel_temp[GYZ];
    accel[GYZ] = c_31 * accel_temp[GYX] +
               c_32 * accel_temp[GYY] +
               c_33 * accel_temp[GYZ];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

/** @brief yaw轴陀螺仪数据的卡尔曼滤波
  * @note  X_k = A * X_k-1 + W
  *        Z-k = H * X_k + V
  *        P(W) ~ N(0 , Q)    P(V) ~ N(0 , R)
  *        [angle_k  = [1 Delta_T  * [angle_k-1  + [w1_k-1
  *         speed_k]    0   1    ]    speed_k-1]    w2_k-1]
  *        [z1_k  = [1 0  * [angle_k  + [v1_k
  *         z2_k]    0 1]    speed_k]    v2_k]
  *        Delta_T是数据更新的时间，陀螺仪通讯的频率是400hz，周期是2.5ms
  *        Q是系统协方差矩阵，Q[1,1]是angle估算的方差，Q[2,2]是speed估算的方差，这个值越小，超前量越大，可以用于预测
  *        R是测量协方差矩阵，R[1,1]是angle测量的方差，R[2,2]是speed测量的方差，这个值越小，约贴近测量值
  *        如果angle单位为°，speed单位为°/s，认为其误差量级大概为0.1，则其方差误差量级为0.01
  */
//kalman_filter_init_t yaw_kalman_filter_para = {
//        .xhat_data = { 0, 0 },
//        .P_data = { 1, 0, 0, 1 },
//        .A_data = { 1, 0.025f, 0, 1 },
//        .H_data = { 1, 0, 0, 1 },
//        .Q_data = { 0.09f, 0, 0, 0.13f },
//        .R_data = { 0.02f, 0, 0, 0.51f }//设置yaw轴卡尔曼滤波的初始参数
//};

/** @brief pitch轴陀螺仪数据的卡尔曼滤波
  * @note  X_k = A * X_k-1 + W
  *        Z-k = H * X_k + V
  *        P(W) ~ N(0 , Q)    P(V) ~ N(0 , R)
  *        [angle_k  = [1 Delta_T  * [angle_k-1  + [w1_k-1
  *         speed_k]    0   1    ]    speed_k-1]    w2_k-1]
  *        [z1_k  = [1 0  * [angle_k  + [v1_k
  *         z2_k]    0 1]    speed_k]    v2_k]
  *        Delta_T是数据更新的时间，陀螺仪通讯的频率是400hz，周期是2.5ms
  *        Q是系统协方差矩阵，Q[1,1]是angle估算的方差，Q[2,2]是speed估算的方差
  *        R是测量协方差矩阵，R[1,1]是angle测量的方差，R[2,2]是speed测量的方差
  *        如果angle单位为°，speed单位为°/s，认为其误差量级大概为0.1，则其方差误差量级为0.01
  */
//kalman_filter_init_t pitch_kalman_filter_para = {//设置pitch轴卡尔曼滤波的初始参数
//        .xhat_data = { 0, 0 },
//        .P_data = { 1, 0, 0, 1 },
//        .A_data = { 1, 0.025f, 0, 1 },
//        .H_data = { 1, 0, 0, 1 },
//        .Q_data = { 0.09f, 0, 0, 0.06f },
//        .R_data = { 0.02f, 0, 0, 0.2f }
//};
//
//kalman_filter_t yaw_kalman_filter = { 0 };
//kalman_filter_t pitch_kalman_filter = { 0 };

void GyroTask(void *argument)
{
//    /* USER CODE BEGIN Gyro_task */
//    INS_Init();
////    kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);//yaw的卡尔曼滤波初始化
////    kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);//pitch卡尔曼滤波初始化
//    /* Infinite loop */
//    for(;;)
//    {
//        INS_Task();
//        //osDelay(1);
//    }
//    /* USER CODE END Gyro_task */
}