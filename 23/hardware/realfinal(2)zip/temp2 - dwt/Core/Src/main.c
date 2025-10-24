/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ?????? - ???????????PID??RPLIDAR??MPU6500????????????????
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms in LICENSE file or provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "task_gyro.h"
#include "bsp_dwt.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    float Kp, Ki, Kd;           // PID参数
    float integral;             // 积分项
    float prev_error;           // 上次误差
    float prev_measurement;     // 上次测量值（用于微分滤波）
    float integral_limit;       // 积分限幅
    float output_limit;         // 输出限幅
    float output;               // 输出值
    float dt;                   // 采样时间
    float alpha;                // 微分滤波系数
    uint32_t last_time;         // 上次更新时间
} Improved_PID_Controller;
typedef struct {
    float Kp, Ki, Kd;           // 航向PID参数
    float integral;             // 积分项
    float prev_error;           // 上次误差
    float output_limit;         // 输出限幅（最大转速修正量）
    float integral_limit;       // 积分限幅
    float dt;                   // 采样时间
    uint32_t last_time;         // 上次更新时间
} Heading_PID_Controller;

#pragma pack(push, 1)
typedef struct {
    uint8_t sync_quality;  // ???λ+????λ
    uint16_t angle_q6;     // ???? (Q6???)
    uint16_t distance_q2;  // ????? (Q2???)
} LidarDataPacket;
#pragma pack(pop)

/* Private define ------------------------------------------------------------*/
#define MAX_SPEED 420
#define MAX_SPEED_RIGHT 416
#define SPEED_UPDATE_INTERVAL 10
#define TURN_DURATION 800
#define TURN_SPEED 300
#define FORWARD_SPEED    400
#define DIR_STOP 0
#define DIR_FORWARD 1  
#define DIR_LEFT_90 2        // 左转90度
#define DIR_RIGHT_90 3       // 右转90度
#define DIR_TURN_AROUND 4    // 掉头180度

#define PPR 360
#define SAMPLE_TIME_MS 100
#define M_PI 3.14159265358979323846
#define DMA_BUFFER_SIZE 256
#define LIDAR_PACKET_SIZE 5
#define ANGLE_FILTER_THRESHOLD 1.0f
#define MIN_VALID_DISTANCE 50.0f
#define MAX_VALID_DISTANCE 12000.0f
#define LIDAR_TIMEOUT_THRESHOLD 500

// PID
#define PID_LEFT_KP   2.00f
#define PID_LEFT_KI   0.15f
#define PID_LEFT_KD   0.1f

#define PID_RIGHT_KP  1.80f
#define PID_RIGHT_KI  0.15f
#define PID_RIGHT_KD  0.1f

#define PID_INTEGRAL_LIMIT   120.0f
#define PID_OUTPUT_LIMIT     420.0f   // 
#define PID_SAMPLE_TIME_MS   10
#define PID_DERIVATIVE_ALPHA 0.2f     // 

// 航向PID参数
#define HEADING_PID_KP             4.26f    // 比例系数 - 中等强度
#define HEADING_PID_KI             0.25f   // 积分系数 - 消除稳态误差
#define HEADING_PID_KD             0.8f    // 微分系数 - 抑制振荡
#define HEADING_PID_OUTPUT_LIMIT   60.0f   // 修正量限制
#define HEADING_PID_INTEGRAL_LIMIT 25.0f   // 积分限幅
#define HEADING_PID_SAMPLE_TIME_MS 20

// 编码器相关
#define ENCODER_PPR         360     // 编码器每转脉冲数
#define ENCODER_SAMPLE_MS   10      // 编码器采样周期+

// MPU6500
#define MPU6500_ADDR 0xD0
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75
#define CALIBRATION_SAMPLES 2000
#define ACCEL_FS_SEL_4G 0x08
#define GYRO_FS_SEL_500DPS 0x08

// 
#define TURN_ANGLE_THRESHOLD 3.0f      // ????????? ??3??
#define AUTO_TURN_SPEED 300            // ?????????
#define MIN_TURN_SPEED 120             // ??С??????
#define SLOWDOWN_ANGLE 45.0f           // ??????????
#define OVERSHOOT_THRESHOLD 30.0f      // ?????????

// 
#define GYRO_BIAS_UPDATE_INTERVAL 10   // ?????????(ms)
#define COMPLEMENTARY_FILTER_GAIN 0.02f // ?????????????
#define STATIONARY_THRESHOLD 0.1f      // ?????????(m/s2)
#define STATIONARY_DURATION 1000       // ??????????(ms)

//里程计
// ===== ODOMETRY CONSTS (ADD) =====
#define WHEEL_RADIUS_M        0.035f    // 5 cm 半径 -> 0.05 m
#define WHEEL_BASE_M          0.170f    // 30 cm 轮距 -> 0.30 m
#define ODOM_SAMPLE_MS        10        // 里程计更新周期(与速度环对齐)
#define DEG2RAD               0.017453292519943295f
#define RAD2DEG               57.29577951308232f
#define ODOM_ALPHA            0.98f     // 航向互补滤波: 编码器主(α)、IMU辅(1-α)

// ★ 每圈计数（只影响里程计，不动你现有 RPM 计算的 ENCODER_PPR ）
// 请按你的编码器/倍频改成“实测一圈的计数”：常见为 390*4=1560（X4）
#define ENCODER_TICKS_PER_REV 360

// 编码器极性，让“前进时 Δtick > 0”
#define ENC_LEFT_DIR   (+1)
#define ENC_RIGHT_DIR  (+1)

#define STEP_MOVE_DISTANCE_M   1.31f    // 每次前进 70 cm

#define LIDAR_QUALITY_THRESHOLD 60    // 质量阈值，低于此值的数据不发送

#define SCAN_DURATION 10000  // 雷达扫描持续时间(ms)

/* Private variables ---------------------------------------------------------*/
uint8_t bluetooth_rx_data = 0;
uint8_t target_direction = DIR_STOP;
uint8_t current_direction = DIR_STOP;
uint32_t current_speed = 0;
uint32_t target_speed = 0;
uint32_t last_speed_update_time = 0;
uint32_t last_encoder_time = 0;  // 编码器上次处理时间戳

uint8_t bluetooth_connected = 0;
uint8_t connection_announced = 0;
uint8_t connection_msg[] = "Connected\r\n";

int32_t lastEncoderA = 0;
int32_t lastEncoderB = 0;

char uart_buf[256];

uint8_t lidar_dma_buffer[DMA_BUFFER_SIZE];
uint8_t lidar_dataBuffer[LIDAR_PACKET_SIZE];
uint8_t lidar_dataIndex = 0;
uint32_t lidar_rxIndex = 0;
volatile uint8_t lidar_process_packet = 0;
float lastLidarAngle = 0.0f;
uint32_t lastLidarRxTime = 0;

int16_t accel_offset_x, accel_offset_y, accel_offset_z;
int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temp;
float roll, pitch;
float yaw = 0.0f;  // ?????
uint32_t last_imu_time = 0;  // ????????????

Improved_PID_Controller pid_left, pid_right;
float target_left_rpm = 0.0f, target_right_rpm = 0.0f;
float measured_left_rpm = 0.0f, measured_right_rpm = 0.0f;
float target_rpm = 0.0f;
float pwm_left = 0, pwm_right = 0;
Heading_PID_Controller heading_pid;
float target_heading = 0.0f;            // 目标航向角
uint8_t heading_control_enabled = 0;    // 航向控制使能标志
float base_target_rpm = 0.0f;           // 基础目标转速

uint8_t auto_turn_mode = 0;      
float start_yaw = 0.0f;            
float target_yaw = 0.0f;           
float prev_angle_diff = 0.0f;      
uint8_t overshoot_count = 0;       

float gyro_z_bias = 0.0f;          
uint8_t is_stationary = 0;         
uint32_t stationary_start_time = 0;

// ===== ODOMETRY STATE (ADD) =====
volatile float odom_x = 0.0f, odom_y = 0.0f, odom_theta = 0.0f; // m, m, rad
volatile float odom_v = 0.0f, odom_omega = 0.0f;                // m/s, rad/s
static int32_t prev_cnt_left = 0, prev_cnt_right = 0;
static uint32_t last_odom_ms = 0;

volatile uint8_t step_move_active = 0;
float step_start_x = 0.0f, step_start_y = 0.0f;
float step_target_distance = STEP_MOVE_DISTANCE_M;

// ===== ONCE-PER-STOP SCAN STATE =====
volatile uint8_t scan_active = 0;         // 当前是否在做一次性的雷达扫描
volatile uint8_t sent_stop_bundle = 0;    // 本次停止是否已经发送过 ODOM+LIDAR
float scan_prev_angle = -1.0f;            // 扫描中的上一帧角度（用于判断是否绕回 360->0）

volatile uint32_t scan_start_time = 0;     // 扫描开始时间戳
volatile uint8_t scan_timer_active = 0;    // 扫描计时器激活标志

uint8_t turn_to_angle_mode = 0;    // 0: 未转弯, 1: 左转中, 2: 右转中
float turn_target_angle = 0.0f;    // 目标转弯角度（90度或180度）
float turn_start_yaw = 0.0f;       // 转弯开始时的偏航角

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void ControlMotor(uint8_t direction, uint32_t speed);
void UpdateSpeedRamp(void);
void SendConnectionNotification(void);
void ProcessLidarData(uint8_t* data);
void SendLidarToBluetooth(float angle, float distance, uint8_t quality);

void MPU6500_Init(I2C_HandleTypeDef* hi2c);
void MPU6500_Calibrate(I2C_HandleTypeDef* hi2c);
void MPU6500_Read_All(I2C_HandleTypeDef* hi2c);
void Calculate_Angles(void);
//void Send_IMU_Data_Bluetooth(void);
float CalculateAngleDifference(float current, float target);
void HandleAutoTurn(void);

void PID_Controller_Init(Improved_PID_Controller* pid, float kp, float ki, float kd, 
                        float integral_limit, float output_limit, float dt, float alpha);
float PID_Controller_Update(Improved_PID_Controller* pid, float setpoint, float measurement);
void Calculate_Motor_RPM(void);
void Update_Motor_PID_Control(void);
void Motor_Velocity_Control(uint8_t direction, float speed_rpm);

void StepMove_Start(float distance_m);
void StepMove_Check(void);


// ===== ODOMETRY PROTOTYPES (ADD) =====
void Odom_Init(void);
void Odom_Update(void);
void Send_Odom_Bluetooth(uint32_t now_ms);

void Send_Odom_Snapshot(void);


//重置运动状态
void Reset_All_Controllers(void);
void Start_Move_With_Fresh_Control(float distance_m);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  ??????????????360???磩
  * @param  current: ??????
  * @param  target: ?????
  * @retval ?????
  */
float CalculateAngleDifference(float current, float target) {
    float diff = target - current;

    // ????360???????
    if (diff > 180.0f) {
        diff -= 360.0f;
    }
    else if (diff < -180.0f) {
        diff += 360.0f;
    }

    return diff;
}

/**
  * @brief  PID控制器初始化
  * @param  pid: PID控制器结构体指针
  * @param  kp, ki, kd: PID参数
  * @param  integral_limit: 积分限幅
  * @param  output_limit: 输出限幅
  * @param  dt: 采样时间(秒)
  * @param  alpha: 微分滤波系数(0-1)
  * @retval None
  */
void PID_Controller_Init(Improved_PID_Controller* pid, float kp, float ki, float kd, 
                        float integral_limit, float output_limit, float dt, float alpha) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->dt = dt;
    pid->alpha = alpha;
    pid->output = 0.0f;
    pid->last_time = HAL_GetTick();
}

/**
  * @brief  PID控制器更新（带微分滤波和抗积分饱和）
  * @param  pid: PID控制器结构体指针
  * @param  setpoint: 设定值
  * @param  measurement: 测量值
  * @retval PID输出值
  */
float PID_Controller_Update(Improved_PID_Controller* pid, float setpoint, float measurement) {
    uint32_t current_time = HAL_GetTick();
    float actual_dt = (current_time - pid->last_time) / 1000.0f;
    
    // 确保采样时间有效
    if (actual_dt <= 0 || actual_dt > 0.1f) {
        actual_dt = pid->dt;
    }
    pid->last_time = current_time;

    // 计算误差
    float error = setpoint - measurement;

    // 比例项
    float proportional = pid->Kp * error;

    // 积分项（带抗饱和）
    pid->integral += error * actual_dt;
    
    // 积分限幅
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    float integral_term = pid->Ki * pid->integral;

    // 微分项（带滤波）
    float derivative = (measurement - pid->prev_measurement) / actual_dt;
    float filtered_derivative = pid->alpha * derivative + (1 - pid->alpha) * pid->prev_error;
    float derivative_term = pid->Kd * filtered_derivative;

    // 计算输出
    pid->output = proportional + integral_term - derivative_term; // 注意：负号用于反馈控制

    // 输出限幅
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    // 更新历史值
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return pid->output;
}
/**
  * @brief  航向PID控制器初始化
  * @retval None
  */
void Heading_PID_Controller_Init(Heading_PID_Controller* pid, float kp, float ki, float kd, 
                                float integral_limit, float output_limit, float dt) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->dt = dt;
    pid->last_time = HAL_GetTick();
}

/**
  * @brief  航向PID控制器更新
  * @retval 航向修正量 (RPM)
  */
float Heading_PID_Controller_Update(Heading_PID_Controller* pid, float setpoint, float measurement) {
    uint32_t current_time = HAL_GetTick();
    float actual_dt = (current_time - pid->last_time) / 1000.0f;
    
    // 确保采样时间有效
    if (actual_dt <= 0 || actual_dt > 0.1f) {
        actual_dt = pid->dt;
    }
    pid->last_time = current_time;

    // 计算航向误差（考虑360度环绕）
    float error = setpoint - measurement;
    
    // 将误差规范化到[-180, 180]范围
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 比例项
    float proportional = pid->Kp * error;

    // 积分项（带抗饱和）
    pid->integral += error * actual_dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float integral_term = pid->Ki * pid->integral;

    // 微分项
    float derivative = (error - pid->prev_error) / actual_dt;
    float derivative_term = pid->Kd * derivative;

    // 计算输出
    float output = proportional + integral_term + derivative_term;

    // 输出限幅
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    pid->prev_error = error;
    return output;
}
/**
  * @brief  计算电机RPM
  * @retval None
  */
void Calculate_Motor_RPM(void) {
    static uint32_t last_encoder_time = 0;
    static int32_t last_encoder_left = 0, last_encoder_right = 0;
    
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_encoder_time >= ENCODER_SAMPLE_MS) {
        // 读取当前编码器值
        int32_t current_left = __HAL_TIM_GET_COUNTER(&htim2);
        int32_t current_right = __HAL_TIM_GET_COUNTER(&htim4);
        
        // 计算脉冲差值（处理溢出）
        int32_t delta_left = (current_left - last_encoder_left);
        int32_t delta_right = (current_right - last_encoder_right);
        
        // 处理32位计数器溢出
        if (delta_left > 0x7FFFFFFF) delta_left -= 0xFFFFFFFF;
        else if (delta_left < -0x7FFFFFFF) delta_left += 0xFFFFFFFF;
        
        if (delta_right > 0x7FFFFFFF) delta_right -= 0xFFFFFFFF;
        else if (delta_right < -0x7FFFFFFF) delta_right += 0xFFFFFFFF;
        
        // 计算RPM：脉冲数/每转脉冲数 * (60000ms/采样时间ms)
        float time_factor = 60000.0f / ENCODER_SAMPLE_MS;
        measured_left_rpm = (float)delta_left / ENCODER_PPR * time_factor;
        measured_right_rpm = (float)delta_right / ENCODER_PPR * time_factor;
        
        // 更新历史值
        last_encoder_left = current_left;
        last_encoder_right = current_right;
        last_encoder_time = current_time;
    }
}

/**
  * @brief  更新电机PID控制
  * @retval None
  */
void Update_Motor_PID_Control(void) {
    static uint32_t last_pid_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_pid_time >= PID_SAMPLE_TIME_MS) {
        // 只在需要速度控制时运行PID
        if (current_direction == DIR_FORWARD) {
            // 计算PID输出
            float pid_left_output = PID_Controller_Update(&pid_left, target_left_rpm, measured_left_rpm);
            float pid_right_output = PID_Controller_Update(&pid_right, target_right_rpm, measured_right_rpm);
            
            // 转换为PWM值（确保为正）
            pwm_left = fmaxf(0, fminf(pid_left_output, MAX_SPEED));
            pwm_right = fmaxf(0, fminf(pid_right_output, MAX_SPEED_RIGHT));
            
        }
        last_pid_time = current_time;
				ControlMotor(current_direction, 0);  // === 新增：把最新 PWM 写入 TIM 比较寄存器 ===

    }
}

/**
  * @brief  带航向补偿的电机速度控制
  * @param  direction: 方向
  * @param  speed_rpm: 目标速度(RPM)
  * @retval None
  */
void Motor_Velocity_Control_With_Heading(uint8_t direction, float speed_rpm) {
    static uint32_t last_heading_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    switch (direction) {
        case DIR_FORWARD:
						
            // 直线运动时启用航向控制
            if (!heading_control_enabled) {
                target_heading = yaw;  // 记录当前航向为目标
                heading_control_enabled = 1;
                base_target_rpm = speed_rpm;
                
                // 重置航向PID
                heading_pid.integral = 0;
                heading_pid.prev_error = 0;
							
								 // === 新增：首次进入直行立即给出无航向修正的目标转速 ===
									target_left_rpm  = speed_rpm;
									target_right_rpm = speed_rpm;					

									// === 新增：强制下一拍必进更新分支，避免 20ms 门限卡死 ===
									last_heading_update = 0; 
                
               
            }
            
            // 更新航向PID
            if (current_time - last_heading_update >= HEADING_PID_SAMPLE_TIME_MS) {
                float heading_correction = Heading_PID_Controller_Update(&heading_pid, target_heading, yaw);
                
                // 应用航向修正：左加右减或左减右加
                if (direction == DIR_FORWARD) {
                    target_left_rpm = base_target_rpm + heading_correction;
                    target_right_rpm = base_target_rpm - heading_correction;
                } else { // DIR_BACKWARD
                    target_left_rpm = -base_target_rpm + heading_correction;
                    target_right_rpm = -base_target_rpm - heading_correction;
                }
                
                
                last_heading_update = current_time;
            }
            break;
            
        case DIR_STOP:
            target_left_rpm = 0;
            target_right_rpm = 0;
            heading_control_enabled = 0;
            // 重置所有PID控制器
            pid_left.integral = 0;
            pid_right.integral = 0;
            pid_left.prev_error = 0;
            pid_right.prev_error = 0;
            heading_pid.integral = 0;
            heading_pid.prev_error = 0;
						Reset_All_Controllers();
            break;
            
        default:
            // 转向时不使用航向控制和速度PID
            target_left_rpm = 0;
            target_right_rpm = 0;
            heading_control_enabled = 0;
            break;
    }
	
}
void Reset_All_Controllers(void) {
    // 重置速度PID控制器
    pid_left.integral = 0.0f;
    pid_left.prev_error = 0.0f;
    pid_left.prev_measurement = 0.0f;
    pid_left.output = 0.0f;
    
    pid_right.integral = 0.0f;
    pid_right.prev_error = 0.0f;
    pid_right.prev_measurement = 0.0f;
    pid_right.output = 0.0f;
    
    // 重置航向PID控制器
    heading_pid.integral = 0.0f;
    heading_pid.prev_error = 0.0f;
    
    // 重置航向控制状态
    heading_control_enabled = 0;
    target_heading = yaw;  // 重置为当前航向
    
    // 重置目标转速
    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;
    
    // 重置PWM输出
    pwm_left = 0;
    pwm_right = 0;
}
/* 起步控制函数 */
void Start_Move_With_Fresh_Control(float distance_m) {
    // 1. 首先重置所有控制器状态
    Reset_All_Controllers();
    
    // 2. 设置新的目标航向（当前航向）
    target_heading = yaw;
    
    // 3. 启用航向控制
    heading_control_enabled = 1;
    
    // 4. 设置基础转速
    base_target_rpm = 80.0f;  // 你的目标RPM值
    
    // 5. 开始步进移动
    StepMove_Start(distance_m);
    
    // 6. 设置运动方向
    target_direction = DIR_FORWARD;
    current_direction = DIR_FORWARD;
    target_speed = MAX_SPEED;
    current_speed = MAX_SPEED;
}


/**
  * @brief  在main函数中初始化PID控制器
  */
void Initialize_PID_System(void) {
    // 初始化内环速度PID
    PID_Controller_Init(&pid_left, 
                       PID_LEFT_KP, PID_LEFT_KI, PID_LEFT_KD,
                       PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT,
                       PID_SAMPLE_TIME_MS / 1000.0f, PID_DERIVATIVE_ALPHA);
    
    PID_Controller_Init(&pid_right,
                       PID_RIGHT_KP, PID_RIGHT_KI, PID_RIGHT_KD,
                       PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT,
                       PID_SAMPLE_TIME_MS / 1000.0f, PID_DERIVATIVE_ALPHA);
    
    // 初始化外环航向PID
    Heading_PID_Controller_Init(&heading_pid,
                               HEADING_PID_KP, HEADING_PID_KI, HEADING_PID_KD,
                               HEADING_PID_INTEGRAL_LIMIT, HEADING_PID_OUTPUT_LIMIT,
                               HEADING_PID_SAMPLE_TIME_MS / 1000.0f);
}
/**
  * @brief  UART?????????
  * @param  huart: UART???
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart1) {
        if (!bluetooth_connected) bluetooth_connected = 1;
        
        if (bluetooth_rx_data >= '0' && bluetooth_rx_data <= '4') {
            target_direction = bluetooth_rx_data - '0';
            
            // 处理自动转弯
            if (target_direction == DIR_LEFT_90 || target_direction == DIR_RIGHT_90 || 
                target_direction == DIR_TURN_AROUND) {
                
                // 记录起始偏航角
                turn_start_yaw = yaw;
                turn_to_angle_mode = 0;
                
                // 设置目标转弯角度和方向
                switch (target_direction) {
                    case DIR_LEFT_90:
                        turn_target_angle = 90.0f;
                        turn_to_angle_mode = 1; // 左转模式
                        // 开始左转
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TURN_SPEED);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                        break;
                        
                    case DIR_RIGHT_90:
                        turn_target_angle = 90.0f;
                        turn_to_angle_mode = 2; // 右转模式
                        // 开始右转
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TURN_SPEED);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, TURN_SPEED);
                        break;
                        
                    case DIR_TURN_AROUND:
                        turn_target_angle = 180.0f;
                        turn_to_angle_mode = 1; // 使用左转模式转180度
                        // 开始左转
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TURN_SPEED);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                        break;
                }
                
                current_direction = target_direction;
            }
            else {
                auto_turn_mode = 0;

                switch (target_direction) {
                    case DIR_STOP:
                        target_speed = 0;
                        current_speed = 0;
                        // 使用带航向控制的速度控制停止
                        Motor_Velocity_Control_With_Heading(DIR_STOP, 0);
												// 重置扫描状态，为下一次停止做准备
												sent_stop_bundle = 0;
												scan_active = 0;
												break;
                    case DIR_FORWARD:
												target_speed = MAX_SPEED;
												current_speed = MAX_SPEED;
												current_direction = DIR_FORWARD; // === 新增：立刻切换到直行状态 ===
												float target_rpm_value = 80.0f; // 设置目标RPM
												Motor_Velocity_Control_With_Heading(DIR_FORWARD, target_rpm_value);
												// === NEW: 现场做一次“首拍”轮速PID，得到非零PWM ===
												// 1) 目标转速此时已由上面函数设好：target_left_rpm/right_rpm
												float pid_left_output  = PID_Controller_Update(&pid_left,  target_left_rpm,  measured_left_rpm);
												float pid_right_output = PID_Controller_Update(&pid_right, target_right_rpm, measured_right_rpm);

												pwm_left  = fmaxf(0, fminf(pid_left_output,  MAX_SPEED));
												pwm_right = fmaxf(0, fminf(pid_right_output, MAX_SPEED_RIGHT));

												// 2) 立刻把 PWM 写入 TIM（避免被“0”覆盖导致只响一下）
												ControlMotor(DIR_FORWARD, 0);

												// 3) 为防止航向环卡在 20ms 门限，强制下一拍必更新
												//    （任选其一，不会影响其它功能）
												StepMove_Start(STEP_MOVE_DISTANCE_M); 
												break;
                }
            }
        }
        
        HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);
    }
		else if (huart == &huart6) {
			lidar_process_packet = 1;
			HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);
	}

}


/**
  * @brief  ??????????
  * @param  direction: ????
  * @param  speed: ???
  * @retval None
  */
void ControlMotor(uint8_t direction, uint32_t speed) {
    switch (direction) {
    case DIR_STOP:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
				Reset_All_Controllers();
        break;
    case DIR_FORWARD:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)pwm_left);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)pwm_right);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        break;

    }
}

/**
  * @brief  ???б?????
  * @retval None
  */
void UpdateSpeedRamp(void) {
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_speed_update_time >= SPEED_UPDATE_INTERVAL) {
        last_speed_update_time = current_time;

        // 如果正在自动转弯，不处理速度斜坡
        if (auto_turn_mode != 0) return;

        if (target_direction == DIR_STOP && current_direction != DIR_STOP) {
            current_speed = 0;
            current_direction = DIR_STOP;
            ControlMotor(DIR_STOP, 0);
            return;
        }

        if (target_direction != current_direction) {
            current_direction = target_direction;
            if (current_direction == DIR_FORWARD) {
                current_speed = target_speed = MAX_SPEED;
                ControlMotor(current_direction, current_speed);
            }
        }
    }
}

/**
  * @brief  ??????????
  * @retval None
  */
void SendConnectionNotification(void) {
    if (bluetooth_connected && !connection_announced) {
        HAL_UART_Transmit(&huart1, connection_msg, sizeof(connection_msg) - 1, 1000);
        connection_announced = 1;
    }
}



// 新的转弯处理函数
void HandleTurnToAngle(void) {
    if (turn_to_angle_mode == 0) {
        return;
    }
    
    // 计算从开始转弯到现在，偏航角已经变化了多少
    float current_yaw = yaw;
    float start_yaw = turn_start_yaw;
    
    // 计算角度差（考虑360度环绕）
    float angle_diff = current_yaw - start_yaw;
    
    // 处理360度环绕情况
    if (turn_to_angle_mode == 1) { // 左转
        if (angle_diff < 0) angle_diff += 360.0f;
    } else { // 右转
        if (angle_diff > 0) angle_diff -= 360.0f;
    }
    
    float abs_angle_diff = fabsf(angle_diff);
    
    // 检查是否已经转够了目标角度
    if (abs_angle_diff >= turn_target_angle -15) {
        // 立即停止
        ControlMotor(DIR_STOP, 0);
        turn_to_angle_mode = 0;
        current_direction = DIR_STOP;
        
        // 完成转弯后的零偏校正（温和一些）
        float current_gyro = gyro_z / 65.5f;
        gyro_z_bias = 0.95f * gyro_z_bias + 0.05f * current_gyro;
        
    }
}


/**
  * @brief  处理激光雷达数据 (带角度有效性检查)
  * @param  data: 数据指针
  * @retval None
  */
void ProcessLidarData(uint8_t* data) {
  //只有在小车停止时才处理雷达数据用于发送
	if (current_direction != DIR_STOP) {
        return; // 运动时不处理雷达数据发送
    }  
	
	LidarDataPacket* pkt = (LidarDataPacket*)data;

    // 检查同步位和起始位
    if ((pkt->sync_quality & 0x80) && (pkt->angle_q6 & 0x01)) {
        uint8_t quality = pkt->sync_quality & 0x7F;
        
        // 计算原始角度（可能超出0-360范围）
        float raw_angle = (pkt->angle_q6 >> 1) / 64.0f;
        float distance = pkt->distance_q2 / 4.0f;

        
        // 检查角度是否在有效范围内 (0-360度)
        if (raw_angle >= 0.0f && raw_angle <= 360.0f) {
            // 规范化角度到0-360度范围（处理浮点误差）
            float angle = fmodf(raw_angle, 360.0f);
            if (angle < 0) angle += 360.0f;

            // 检查距离和质量是否有效
            if (distance >= MIN_VALID_DISTANCE && 
                distance <= MAX_VALID_DISTANCE && 
                quality > 0) {
                
                // 角度变化检查
                if (fabsf(angle - lastLidarAngle) >= ANGLE_FILTER_THRESHOLD || 
                    angle < lastLidarAngle) {
                    
                    SendLidarToBluetooth(angle, distance, quality);
                    lastLidarAngle = angle;
                }
            }
        }
        else {
            // 可选：记录无效角度数据（调试用）
            static uint32_t invalid_angle_count = 0;
            invalid_angle_count++;
            
        }
    }
}

/**
  * @brief  发送激光雷达数据到蓝牙 (修改版：仅在停止时发送，且有时间限制)
  * @param  angle: 角度
  * @param  distance: 距离
  * @param  quality: 质量
  * @retval None
  */
void SendLidarToBluetooth(float angle, float distance, uint8_t quality) {
    static uint32_t last_send_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 只有在小车停止 且 正在一次性扫描期 且 扫描计时器激活时才发送
    if (current_direction != DIR_STOP) {
        return;
    }
    if (!scan_active) {
        return;
    }
    if (!scan_timer_active) {
        return; // 扫描时间已到，停止发送
    }

    // 检查扫描时间是否超时
    if (current_time - scan_start_time > SCAN_DURATION) {
        scan_timer_active = 0; // 停止扫描计时
        HAL_UART_Transmit(&huart1, (uint8_t*)"END_SCAN\r\n", 10, 100);
        scan_active = 0; // 结束扫描状态
        return;
    }

    // 质量过滤
    if (quality < LIDAR_QUALITY_THRESHOLD) {
        return;
    }

    // 频率限制
    if (current_time - last_send_time < 2) {
        return;
    }
    last_send_time = current_time;

    // 发送一条雷达数据（角度用度，距离转成厘米）
    float distance_cm = distance / 10.0f;  // 毫米->厘米
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer),
        "Lidar: A=%.2f, D=%.2f, Q=%d\r\n", angle, distance_cm, quality);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);

    scan_prev_angle = angle;
}


/**
  * @brief  MPU6500?????
  * @param  hi2c: I2C???
  * @retval None
  */
void MPU6500_Init(I2C_HandleTypeDef* hi2c) {
    uint8_t check = 0, data = 0;

    // 读 WHO_AM_I（MPU6500 通常为 0x70）
    HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, WHO_AM_I, 1, &check, 1, 100);
    if (check != 0x70) {
        char msg[48];
        snprintf(msg, sizeof(msg), "MPU6500 Not Found! ID:0x%02X\r\n", check);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
//        Error_Handler();
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6500 Found!\r\n", 16, 100);
    }

    // 退出睡眠
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, PWR_MGMT_1, 1, &data, 1, 100);
    HAL_Delay(100);

    // Gyro: ±500 dps（与现有宏一致）
    data = GYRO_FS_SEL_500DPS;         // 0x08
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, GYRO_CONFIG, 1, &data, 1, 100);

    // Accel: ±4g（与现有宏一致）
    data = ACCEL_FS_SEL_4G;            // 0x08
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, ACCEL_CONFIG, 1, &data, 1, 100);

    // DLPF：配置较低延迟且抑噪合适（建议 gyro/accel 都走 DLPF=3 ≈ 44Hz(gyro)/44Hz(accel) 档）
    // MPU6500 CONFIG寄存器[2:0]为 DLPF_CFG；3 => ~41~44Hz
    data = 0x06;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, CONFIG, 1, &data, 1, 100);

    // 采样分频：Gyro 内部 1kHz，SMPLRT_DIV=9 => 100Hz 输出（与主循环10ms对齐）
    data = 9;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, SMPLRT_DIV, 1, &data, 1, 100);

    // 可选：温度/工况稳定等待
    HAL_Delay(50);
}


/**
  * @brief  MPU6500У?
  * @param  hi2c: I2C???
  * @retval None
  */
void MPU6500_Calibrate(I2C_HandleTypeDef* hi2c) {
    const int N = CALIBRATION_SAMPLES;     // 500
    int64_t ax = 0, ay = 0, az = 0;
    int64_t gx = 0, gy = 0, gz = 0;

    HAL_UART_Transmit(&huart1, (uint8_t*)"Calibrating MPU6500...\r\n", 24, 100);

    // 要求静止 1s 左右
    HAL_Delay(100);

    for (int i = 0; i < N; i++) {
        uint8_t buf[14];
        if (HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100) != HAL_OK) {
            i--; continue; // 读失败重试，不影响总样本
        }

        ax += (int16_t)((buf[0] << 8) | buf[1]);
        ay += (int16_t)((buf[2] << 8) | buf[3]);
        az += (int16_t)((buf[4] << 8) | buf[5]);
        gx += (int16_t)((buf[8] << 8) | buf[9]);
        gy += (int16_t)((buf[10] << 8) | buf[11]);
        gz += (int16_t)((buf[12] << 8) | buf[13]);

        HAL_Delay(2);  // 约 500*2ms = 1s
    }

    accel_offset_x = (int16_t)(ax / N);
    accel_offset_y = (int16_t)(ay / N);
    // Z 轴以 1g 为基准（±4g => 8192 LSB/g）
    accel_offset_z = (int16_t)(az / N) - 16384;

    gyro_offset_x  = (int16_t)(gx / N);
    gyro_offset_y  = (int16_t)(gy / N);
    gyro_offset_z  = (int16_t)(gz / N);

    // 同步一次“运行时偏置学习”的起点
    gyro_z_bias = 0.0f;
    is_stationary = 0;
    stationary_start_time = HAL_GetTick();
}

/**
  * @brief  ???MPU6500????????
  * @param  hi2c: I2C???
  * @retval None
  */
void MPU6500_Read_All(I2C_HandleTypeDef* hi2c) {
    uint8_t buf[14];

    // ???14??????? (?????+???+??????)
    HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);

    // ???У??????
    accel_x = (int16_t)((buf[0] << 8) | buf[1]) - accel_offset_x;
    accel_y = (int16_t)((buf[2] << 8) | buf[3]) - accel_offset_y;
    accel_z = (int16_t)((buf[4] << 8) | buf[5]) - accel_offset_z;

    temp = (int16_t)((buf[6] << 8) | buf[7]);

    gyro_x = (int16_t)((buf[8] << 8) | buf[9]) - gyro_offset_x;
    gyro_y = (int16_t)((buf[10] << 8) | buf[11]) - gyro_offset_y;
    gyro_z = (int16_t)((buf[12] << 8) | buf[13]) - gyro_offset_z;
}

/**
  * @brief  ??????????????? - ?????
  * @retval None
  */
void Calculate_Angles(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 计算时间差(秒)
    float delta_time = (current_time - last_time) / 1000.0f;
    if (delta_time > 0.2f || delta_time <= 0) delta_time = 0.01f;
    last_time = current_time;
    
    // 转换到g单位 (4g scale, 8192 LSB/g)
    float ax = accel_x / 8192.0f;
    float ay = accel_y / 8192.0f;
    float az = accel_z / 8192.0f;
    
    // 计算加速度幅值(用于静止检测)
    float accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);
    
    // 检查是否静止
    if (fabsf(accel_magnitude - 1.0f) < STATIONARY_THRESHOLD) {
        if (is_stationary == 0) {
            is_stationary = 1;
            stationary_start_time = current_time;
        }
        // 如果静止时间够长，更新陀螺仪零偏
        else if (current_time - stationary_start_time > STATIONARY_DURATION) {
            // 低通滤波更新零偏
            float current_bias = gyro_z / 65.5f;
            gyro_z_bias = 0.98f * gyro_z_bias + 0.02f * current_bias;
        }
    }
    else {
        is_stationary = 0;
    }
    
    // 计算roll和pitch角度
    roll = atan2f(ay, az) * 180.0f / M_PI;
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    
    // 计算yaw角度(航向) 
    float gyro_z_dps = (gyro_z / 65.5f) - gyro_z_bias;
    yaw += gyro_z_dps * delta_time;
    
    // 规范化yaw角度到0-360度
    yaw = fmodf(yaw, 360.0f);
    if (yaw < 0.0f) yaw += 360.0f;
}



/**
  * @brief  ??ó??????
  * @retval int
  */
	
// ===== ODOMETRY IMPL (ADD) =====
static inline float _wrap_pi(float a){
    while (a >  M_PI) a -= 2.0f*M_PI;
    while (a < -M_PI) a += 2.0f*M_PI;
    return a;
}

void Odom_Init(void){
    prev_cnt_left  = __HAL_TIM_GET_COUNTER(&htim2);
    prev_cnt_right = __HAL_TIM_GET_COUNTER(&htim4);
    last_odom_ms   = HAL_GetTick();
    odom_x = odom_y = 0.0f;
    odom_theta = yaw * DEG2RAD;   // 用当前 IMU 航向作为里程计初始朝向
    odom_v = odom_omega = 0.0f;
}

void Odom_Update(void){
    uint32_t now = HAL_GetTick();
    if (now - last_odom_ms < ODOM_SAMPLE_MS) return;
    float dt = (now - last_odom_ms) / 1000.0f;
    if (dt <= 0.0f) return;
    last_odom_ms = now;

    // 1) 读取编码器 & Δtick（含32位溢出 + 极性）
    int32_t cntL = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t cntR = __HAL_TIM_GET_COUNTER(&htim4);
    int32_t dL = (cntL - prev_cnt_left)  * ENC_LEFT_DIR;
    int32_t dR = (cntR - prev_cnt_right) * ENC_RIGHT_DIR;
    prev_cnt_left  = cntL;
    prev_cnt_right = cntR;

    if (dL >  0x7FFFFFFF) dL -= 0xFFFFFFFF;
    if (dL < -0x7FFFFFFF) dL += 0xFFFFFFFF;
    if (dR >  0x7FFFFFFF) dR -= 0xFFFFFFFF;
    if (dR < -0x7FFFFFFF) dR += 0xFFFFFFFF;

    // 2) tick -> 弧长
    float meters_per_rev = 2.0f * M_PI * WHEEL_RADIUS_M;
    float dsL = meters_per_rev * ((float)dL / ENCODER_TICKS_PER_REV);
    float dsR = meters_per_rev * ((float)dR / ENCODER_TICKS_PER_REV);

    // 3) 里程/角度（编码器推算）
    float ds      = 0.5f * (dsR + dsL);
    float dth_enc = (dsR - dsL) / WHEEL_BASE_M;

    // 直线小差异抑制（保留你的逻辑）
    if (fabsf(dsR - dsL) < 0.002f) {
        dth_enc *= 0.2f;
    }

    // —— 计算编码器航向（度），作为卡尔曼量测 —— 
    float theta_enc = _wrap_pi(odom_theta + dth_enc);     // rad
    float theta_enc_deg = theta_enc * RAD2DEG;            // deg

		// 关键改进：动态调整量测噪声，考虑转弯状态
		float turn_mag = fabsf(dth_enc);
		float R_deg2;

		// 更合理的噪声分配策略
		if (auto_turn_mode != 0) {
				// 正在执行自动转弯：主要信任陀螺仪，编码器可能有滑差
				R_deg2 = 100.0f * 100.0f;  // 编码器噪声很大
		} else if (turn_mag > 0.01f) {
				// 微小转动或曲线行驶：适度信任
				R_deg2 = 15.0f * 15.0f;
		} else {
				// 直行：较信任编码器
				R_deg2 = 5.0f * 5.0f;
		}




		// 4) 航向融合（动态调整融合权重）
		// 直行时更信任编码器,转弯时更信任IMU
		float alpha_adaptive = ODOM_ALPHA;  // 默认0.98
		if (auto_turn_mode == 0 && turn_mag < 0.01f) {
				// 直行状态：降低陀螺仪权重，更信任编码器
				alpha_adaptive = 0.90f;
		}

		float theta_pred = odom_theta + dth_enc;              // 仅编码器预测
		float theta_imu  = yaw * DEG2RAD;                     // 直接用全局yaw(度)转弧度
		float err        = _wrap_pi(theta_imu - theta_pred);
		float theta_fused= _wrap_pi(theta_pred + (1.0f - alpha_adaptive) * err);


    // 5) 位姿更新（以中点角更新 x,y）
    float dtheta    = _wrap_pi(theta_fused - odom_theta);
    float theta_mid = _wrap_pi(odom_theta + 0.5f * dtheta);
    odom_x     += ds * cosf(theta_mid);
    odom_y     += ds * sinf(theta_mid);
    odom_theta  = theta_fused;

    // 6) 速度
    odom_v     = ds / dt;
    odom_omega = dtheta / dt;

	
}


void Send_Odom_Bluetooth(uint32_t now_ms){
    static uint32_t last_tx = 0;
    if (now_ms - last_tx < 50) return; // 20Hz，避免占用太多带宽
    last_tx = now_ms;

    // 仅保留 x, y, yaw（度）
    char buf[96];
    int len = snprintf(buf, sizeof(buf),
        "odom:{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.1f}\r\n",
        odom_x/2.2607, odom_y, yaw);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
}

// 一次性里程计快照（带起止标志），不受 Send_Odom_Bluetooth 的20Hz限流影响
void Send_Odom_Snapshot(void){
    HAL_UART_Transmit(&huart1, (uint8_t*)"START_ODOM\r\n", 12, 100);

    char buf[96];
    int len = snprintf(buf, sizeof(buf),
        "odom:{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.1f}\r\n",
        odom_x/2.2607, odom_y, yaw);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);

    HAL_UART_Transmit(&huart1, (uint8_t*)"END_ODOM\r\n", 10, 100);
}


// ===== STEP-MOVE IMPL (ADD) =====
void StepMove_Start(float distance_m){
    step_target_distance = fabsf(distance_m);
    step_start_x = odom_x;
    step_start_y = odom_y;
    step_move_active = 1;


}

void StepMove_Check(void){
    if(!step_move_active) return;

    float dx = odom_x - step_start_x;
    float dy = odom_y - step_start_y;
    float d  = sqrtf(dx*dx + dy*dy);

    if(d >= step_target_distance){
        // 达到目标：刹停并复位相关状态（不影响其它模块）
        step_move_active = 0;

        target_direction = DIR_STOP;
        current_direction = DIR_STOP;

        target_left_rpm = 0.0f;
        target_right_rpm = 0.0f;
        heading_control_enabled = 0;

        // 复位 PID 的积分/误差，避免残留
        pid_left.integral = 0;   pid_left.prev_error = 0;
        pid_right.integral = 0;  pid_right.prev_error = 0;
        heading_pid.integral = 0; heading_pid.prev_error = 0;

        ControlMotor(DIR_STOP, 0);

    }
}


int main(void) {
    HAL_Init();
    SystemClock_Config();
    // 添加中断优先级设置:
		HAL_NVIC_SetPriority(USART6_IRQn, 3, 0);    // 降低雷达串口中断优先级
		HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 0); // 降低雷达DMA优先级
		HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 0);   // 提高I2C事件中断优先级
		HAL_NVIC_SetPriority(I2C1_ER_IRQn, 1, 0);   // 提高I2C错误中断优先级

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_I2C1_Init();

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);

    // MPU6500
    MPU6500_Init(&hi2c1);
    MPU6500_Calibrate(&hi2c1);
		INS_Init();
		DWT_Init(180);

    gyro_z_bias = 0.0f;
    is_stationary = 0;
    stationary_start_time = 0;

    HAL_Delay(50);
    uint8_t startCmd[] = { 0xA5, 0x20 }; // RPLIDAR
    HAL_UART_Transmit(&huart6, startCmd, sizeof(startCmd), 100);
    uint8_t setScanMode[] = {0xA5, 0x60, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00}; // 降低扫描速率
		HAL_UART_Transmit(&huart6, setScanMode, sizeof(setScanMode), 100);
		HAL_Delay(50); // 等待设置生效
    HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);

    lastEncoderA = __HAL_TIM_GET_COUNTER(&htim2);
    lastEncoderB = __HAL_TIM_GET_COUNTER(&htim4);

    Odom_Init();

    uint32_t last_imu_time = HAL_GetTick();
    uint32_t last_debug_time = HAL_GetTick();
    

    while (1) {
    uint32_t current_time = HAL_GetTick(); // 获取当前系统时间(ms)
    
    // ============= 1. 高优先级任务：必须实时处理 =============
    // (1) 蓝牙连接状态检查
    SendConnectionNotification(); 
    
    // (2) 电机速度斜坡控制（每10ms执行一次）
    if (current_time - last_speed_update_time >= SPEED_UPDATE_INTERVAL) {
        UpdateSpeedRamp();
        last_speed_update_time = current_time;
    }
    
    // (3) 自动转向控制（如果处于自动转向模式）
    if (turn_to_angle_mode != 0) {
    HandleTurnToAngle();
}

    // ============= 2. 中优先级任务：传感器数据采集 =============
    // (1) IMU数据处理（每100ms固定周期）
		// (1) IMU数据处理（每10ms）
		if (current_time - last_imu_time >= 10) {
    // 先搬一帧原始数据（I2C 连读 14B，你的 Read_All 已减去静态 offset）
    MPU6500_Read_All(&hi2c1);
    // 再做角度/偏航预测（内部调用 YawKF_Predict）
    //Calculate_Angles();
		yaw = INS_Task(accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z);
    last_imu_time = current_time;
			// === 新增：直行时让航向环按采样周期持续更新 ===
		if (current_direction == DIR_FORWARD) {
				Motor_Velocity_Control_With_Heading(DIR_FORWARD, base_target_rpm);
		}

		}


			// 编码器速度计算（每10ms）
				Calculate_Motor_RPM();
				
				// PID控制更新（每10ms）
				Update_Motor_PID_Control();
				
				 Odom_Update();
				 StepMove_Check();
        Send_Odom_Bluetooth(current_time);
				

				


				

    // ============= 3. 低优先级任务：雷达数据处理 =============
   // ===== 停止状态检测和扫描控制 =====
	if (current_direction == DIR_STOP) {
			// 如果刚进入停止状态且还没开启扫描
			if (!scan_active && !sent_stop_bundle) {
					// 先发送里程计快照
					Send_Odom_Snapshot();
					
					// 然后开始雷达扫描（带时间限制）
					HAL_UART_Transmit(&huart1, (uint8_t*)"START_SCAN\r\n", 12, 100);
					scan_active = 1;
					scan_timer_active = 1;        // 激活扫描计时器
					scan_start_time = HAL_GetTick(); // 记录扫描开始时间
					scan_prev_angle = -1.0f;
					sent_stop_bundle = 1;         // 标记本次停止已经处理过扫描
			} else {
					// 如果已经在扫描中，检查是否超时
					if (scan_timer_active && (HAL_GetTick() - scan_start_time > SCAN_DURATION)) {
							scan_timer_active = 0;
							HAL_UART_Transmit(&huart1, (uint8_t*)"END_SCAN\r\n", 10, 100);
							scan_active = 0;
							// 注意：这里不重置 sent_stop_bundle，保持为1，防止重复扫描
					}
			}
	} else {
			// 如果开始运动，立即停止扫描
			if (scan_active) {
					if (scan_timer_active) {
							HAL_UART_Transmit(&huart1, (uint8_t*)"END_SCAN\r\n", 10, 100);
					}
					scan_active = 0;
					scan_timer_active = 0;
			}
			// 重置状态，为下一次停止做准备
			sent_stop_bundle = 0;
	}
		
		//  限制雷达数据处理频率（每次最多处理5个数据包）
    if (lidar_process_packet) {
        lidar_process_packet = 0;
        uint16_t data_length = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        uint8_t processed_packets = 0;
        
        for (uint16_t i = 0; i <= data_length - sizeof(LidarDataPacket) && processed_packets < 10; i++) {
            if (lidar_dma_buffer[i] & 0x80) { // 检查同步位
                LidarDataPacket packet;
                memcpy(&packet, &lidar_dma_buffer[i], sizeof(LidarDataPacket));
                ProcessLidarData((uint8_t*)&packet);
                i += sizeof(LidarDataPacket) - 1; // 跳过已处理的数据包
                processed_packets++;
            }
        }
        
        // 重置DMA接收
        __HAL_DMA_DISABLE(huart6.hdmarx);
        huart6.hdmarx->Instance->NDTR = DMA_BUFFER_SIZE;
        __HAL_DMA_ENABLE(huart6.hdmarx);
    }

    // ============= 4. 空闲时延时 =============
    HAL_Delay(5); // 降低CPU占用率
}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
