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

void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        //BMI088_Read(&BMI088);

        INS.Accel[GYX] = 0;//BMI088.Accel[GYX];
        INS.Accel[GYY] = 0;//BMI088.Accel[GYY];
        INS.Accel[GYZ] = 0;//BMI088.Accel[GYZ];
        INS.Gyro[GYX] =  0;//BMI088.Gyro[GYX];
        INS.Gyro[GYY] =  0;//BMI088.Gyro[GYY];
        INS.Gyro[GYZ] =  0;//BMI088.Gyro[GYZ];

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        INS.atanxz = -atan2f(INS.Accel[GYX], INS.Accel[GYZ]) * 180 / PI;
        INS.atanyz = atan2f(INS.Accel[GYY], INS.Accel[GYZ]) * 180 / PI;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[GYX], INS.Gyro[GYY], INS.Gyro[GYZ], INS.Accel[GYX], INS.Accel[GYY], INS.Accel[GYZ], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n


        /***--------------------------获取最终数据------------------------------***/
        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
        /***--------------------------获取最终数据------------------------------***/


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
    /* USER CODE BEGIN Gyro_task */
    INS_Init();
//    kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);//yaw的卡尔曼滤波初始化
//    kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);//pitch卡尔曼滤波初始化
    /* Infinite loop */
    for(;;)
    {
        INS_Task();
        osDelay(1);
    }
    /* USER CODE END Gyro_task */
}