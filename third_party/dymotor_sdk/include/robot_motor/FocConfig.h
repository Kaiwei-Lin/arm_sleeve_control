

#ifndef _FOC_CONFIG_
#define _FOC_CONFIG_

#ifdef __cplusplus
extern "C" {
#endif

#include "BaseType.h"

/*foc控制模式*/
typedef enum {
    e_VoltageMode = 0,   // 电压模式
    e_CurrentMode = 1,   // 电流模式
    e_VelocityMode = 2,  // 速度模式
    e_PostionMode = 3,   // 位置模式
    e_PDMode = 4,        // PD模式
    e_BrakeMode = 5,
    e_FOCCtrlModeAll,
} FOCCtrlMode_e;

/*角度获取模式*/
typedef enum {
    e_ForceTheta = 0,    // 强制角度标志
    e_SensorLess = 1,    // 无感运行标志
    e_Sensor = 2,        // 有感运行标志
    e_SensorFusion = 3,  // 传感器融合角度
    e_SensorFusionFlux = 4,  // 传感器融合角度
    e_SensorFusionSMO = 5,  // 传感器融合角度
    e_ThetaSorceAll,
} ThetaSorce_e;

/*电机参数*/
typedef struct
{
    uint32_t NPP;        // 极对数
    float Rs;            // 相电阻Ω
    float Ld;            // D轴电感H
    float Lq;            // Q轴电感H
    float FluxWb;        // 磁链系数Wb
    float RotorInertia;  // 转子惯量
} MotorParam_t;

typedef enum
{
    GD_Independent = 0,//独立驱动+模拟电流反馈
    GD_Drv8353 = 1,// drv8353驱动+模拟电流反馈
    GD_Drv8323 = 2,// drv8353驱动+模拟电流反馈
    GD_IND_DIG_CUR = 3,//独立驱动+数字电流反馈
} GateDriverType_e;

/*硬件参数*/
typedef struct
{
    uint32_t ADCFullVal;              // ADC最大值
    float TimerFreq_MHz;              // 定时器频率Mhz
    uint32_t IsCenterAlign;           // 是否为中央对齐模式
    float PWMFreq_KHz;                // PWM频率Khz
    float VBusRange;                  // 母线电压范围V
    float PhaseCurRange;              // 相电流范围A(正极限-负极限)
    float PhaseCurOffset;             // 相电流偏置
    float PhaseVolRange;              // 相电压范围V
    float SampingRsNum;               // 采样电阻数量
    float DeadTime_nS;                // 死区时间纳秒
    float CurAmpGain;                 // 电流放大增益
    GateDriverType_e GateDriverType;  // 预驱类型
    uint32_t Drv8353_IDRIVE;          // Drv8353栅驱电流
    float OverVbus;
    float UnderVbus;
    
} HardwareParam_t;

// /*基础控制参数*/
// typedef struct
// {
//     float IsrDivision;        // 中断分频系数
//     float OverVbus;           // 过压保护V
//     float UnderVbus;          // 低压保护V
//     float MaxCurrent;         // 最大电流A
//     float MaxSpeedRadS;       // 最大速度Rad/S
//     float MaxAccRadSS;        // 最大加速度 Rad/S^2
//     float MaxPosRad;          // 最大位置Rad
//     float MinPosRad;          // 最小位置Rad
// } CtrlBsaeParam_t;

/*功能配置*/
// typedef struct
//{
//     FOCCtrlMode_e CtrlMode;      // 控制模式
//     ThetaSorce_e ThetaSorce;     // 电角度模式
//     float FusionHz;              // 开始融合的速度
//     float SensorCompensateGain;  // 传感器角度补偿增益
//     float CurReGenMinTime_uS;    // 电流重构最小时间us(死区+导通延迟)
// } FuncParam_t;

/*强拖控制参数*/
typedef struct
{
    float Acc_RadSS;        // 强拖加速度
    float ElecVelocity_Hz;  // 强拖速度
    float Theta_Rad;        // 强拖角度
    float Current;          // 强拖电流
    float Voltage;          // 强拖电流
} ForceParam_t;

/*电流环控制参数*/
typedef struct
{
    float Division;     // Foc相对于PWM的分频系数
    float BandWith;     // 电流环带宽Hz
    float LowFreqGain;  // 电流环低频增益
    float MaxCurrent;   // 最大电流A

    float FfdVal;  // 前馈量
    float Ts;      // 运行周期
} CurrentlParam_t;

/*速度环控制参数*/
typedef struct
{
    float Division;      // 相对电流环分频系数
    float Kp;            // KP
    float Ki;            // KI
    float LpfWc_Hz;      // 低通滤波截止频率Hz
    float MaxSpeedRadS;  // 最大速度Rad/S
    float MaxAccRadSS;   // 最大加速度 Rad/S^2

    float FfdVal;  // 前馈增益
    float Ts;      // 运行周期
} SpeedParam_t;

/*位置环控制参数*/
typedef struct
{
    float Division;   // 相对速度环分频系数
    float Kp;         //
    float Kd;         //
    float MaxPosRad;  // 最大位置Rad
    float MinPosRad;  // 最小位置Rad

    float FfdVal;  // 前馈量
    float Ts;      // 运行周期
} PositionParam_t;

/*无感控制参数*/
typedef struct
{
    float PLLWc_Hz;          // 锁相环带宽
    float PllLpfWc_Hz;       // 锁相环速度滤波
    float CompVel_Hz;        // 补偿速度
    float CompIdA;           // 补偿电流
    float CompGain;          // 补偿增益
    float DeadVel_Hz;        // 无传感器模式死区速度
    float FusionMinVel_RadS; // 无感融合最小速度
    float FusionErrMax_Deg;  // 融合最大误差角度
} FulxObsParam_t;

/*PD环控制参数*/
typedef struct
{
    float Division;  // 相对速度环分频系数
    float Kp;        //
    float Kd;        //

    float FfdVal;  // 前馈量
    float OutMax;  //
    float Ts;      // 运行周期
} PDParam_t;

/*电流力矩转换三次多项式系数*/
typedef struct
{
    float TorqueToCurrentA;  // 转矩->电流多项式系数A
    float TorqueToCurrentB;  // 转矩->电流多项式系数B
    float TorqueToCurrentC;  // 转矩->电流多项式系数C
    float CurrentToTorqueX;  // 电流->转矩多项式系数X
    float CurrentToTorqueY;  // 电流->转矩多项式系数Y
    float CurrentToTorqueZ;  // 电流->转矩多项式系数Z
    float TorqueMax;         // 力矩限幅
} TorqueCalibParam_t;

/*摄氏度电流系数*/
typedef struct
{
    float CurToTemp5;
    float CurToTemp4;
    float CurToTemp3;
    float CurToTemp2;
    float CurToTemp1;
    float CurToTemp0;
    float TempMax;
    float NormalCur;
    float MaxIqLimit;
} TempCurParam_t;

typedef struct
{
    float Fc;           // 库仑摩擦力
    float Fs;           // 最大静摩擦力
    float B;            // 粘滞摩擦系数
    float DeadVel;      // 死区速度
    uint32_t CaliFlag;  // 摩擦补偿校准成功标志
    float Percent;      // 摩擦补偿百分比
} FrictionParam_t;

#ifdef __cplusplus
}
#endif
#endif
