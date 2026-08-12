#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H
#include "UserParametersList.h"  
#include "Common.h" 
#ifdef _WIN32
#  define EXPORT_API  __declspec(dllexport)
#else
#  define EXPORT_API  __attribute__((visibility("default")))
#endif
#ifdef __cplusplus
extern "C" {
#endif

    
     /**
     * @brief 主板对象（机器人上下文结构体前向声明）
     */
    EXPORT_API typedef struct RobotCtx RobotCtx;

    /**
     * @brief 电机对象（电机结构体前向声明）
     */
    EXPORT_API typedef struct RobotMotor RobotMotor;

    /**
     * @brief 电机信息列表句柄
     * @details 以 void* 类型封装的电机信息列表抽象标识，需通过专属API操作
     * @note 不可直接解引用，仅作为列表操作的句柄使用
     */
    EXPORT_API typedef void* RobotMotorListHandle;

    /**
     * @brief 电机对象列表句柄
     * @details 以 void* 类型封装的电机对象列表抽象标识，用于批量管理电机对象
     * @note 不可直接解引用，仅作为列表操作的句柄使用
     */
    EXPORT_API typedef void* RobotMotorObjectListHandle;

    /**
     * @brief 主板对象列表句柄
     * @details 以 void* 类型封装的主板对象列表抽象标识，用于管理主板版本/配置等信息
     * @note 不可直接解引用，仅作为列表操作的句柄使用
     */
    EXPORT_API typedef void* RobotMainEditionInfoList;


    /**
    * @brief 电机对象数组结构体
    * @details 用于批量管理最多16个电机对象的容器，包含电机对象列表及已存储的电机数量
    * @note 数组最大支持16个电机对象，count值不应超过16
    */
    typedef struct {
        RobotMotor* robotmotors[16]; //电机对象列表
        int count;         // 已存储的元素数量
    } MotorArray;


    /**
    * @brief 一键固件升级进度回调函数
    * @param num 进度值
    * @param error 错误码 1为成功，-1为握手失败，-2 发送开始传输标志失败，-3发送传输文件失败，-4停止传输失败
    * @return void
    */
    typedef void (*progress_callback)(int num, int error);





     /**
    * @brief 获取主板信息列表
    * @param deviceinfoArray  主板信息数组 建议数组创建3个以上
    * @param num 返回的个数
    * @return 1 为成功，0为失败
    */
    EXPORT_API  void get_robot_mainEditioninfos(DeviceInfo* deviceinfoArray,int* num);

    /**
    * @brief 初始化主板对象
    * @param dev_id  主板ID
    * @return 主板对象，失败返回nullptr
    */
    EXPORT_API  RobotCtx* robot_create(unsigned short dev_id);
    
    /**
    * @brief 释放主板对象
    * @param ctx  主板对象
    * @return void
    */
    EXPORT_API void  robot_destroy(RobotCtx* ctx);

    /**
    * @brief 连接主板
    * @param ctx  主板对象
    * @param local_ip  本地ip
    * @param local_port 本地端口 
    * @param remote_port  主板端口
    * @param remote_ip  主板ip
    * @return 连接是否成功，1 为成功，0为失败 ,-1 主板对象为nullptr
    */
    EXPORT_API int robot_config_net(RobotCtx* ctx,
        const char* local_ip,
        int         local_port,
        int         remote_port,
        const char* remote_ip);


   
    
  /**
   * @brief 设置MIT状态机
   * @param state   参数
   */
    EXPORT_API void robot_StateMachine(RobotCtx* ctx, unsigned short state);

   
    /**
    * @brief 创建单电机对象
    * @param ctx           主板对象
    * @param canId         canId
    * @param motor         返回一个电机对象
    * @param canLineId     can线Id
    * @return RobotMotor 类型指针
    */

    EXPORT_API RobotMotor* robot_create_motor(RobotCtx* ctx, unsigned short canId, unsigned short canLineId);

      /**
    * @brief 创建电机对象列表
    * @param ctx     主板对象
    * @param handle  电机列表
    * @param motorArray** 电机对象列表
    */

    EXPORT_API void robot_create_motorObjectList(RobotCtx* ctx, RobotMotorListHandle handle,MotorArray* motorArray);



     /**
    * @brief 创建电机列表
    * @return 返回电机列表指针，失败返回NULL
    */
    EXPORT_API  RobotMotorListHandle motorlist_create();
    
    /**
    * @brief 释放电机列表
    * @return void
    */

    EXPORT_API  void motorlist_destroy(RobotMotorListHandle handle);


     /**
    * @brief 释放电机对象列表
    * @param ctx        主板对象
    * @param motorArray 电机对象数组 
    * @return void
    */

    EXPORT_API  void motorObjectlist_destroy(RobotCtx* ctx,MotorArray* motorArray);


     /**
    * @brief 获取电机信息列表
    * @param ctx        主板对象
    * @param handle     电机列表 
    * @return 1 设置成功 0 设置失败
    */

    EXPORT_API int get_robot_motorlist(RobotCtx* ctx, RobotMotorListHandle handle);

    
    
      /**
   * @brief 设置主机快速模式参数
   * @param ctx  主机对象
   * @param period_ms  快速模式上报频率
   * @return  1 设置成功 0 设置失败 ,-1 主板对象为nullptr
   */
    EXPORT_API  int robot_set_fast_mode(RobotCtx* ctx, int period_ms);

    /**
    * @brief 设置电机控制模式
    * @param motor 电机对象
    * @param idx 控制模式索引值 详见Types.h 文件 枚举 MotorCtrlMode_e
    * @return 1 设置成功 0 设置失败
    */
    EXPORT_API int robot_motor_set_control_mode(RobotMotor* motor, unsigned int idx);


    /**
    * @brief 设置电机控制字
    * @param motor 电机对象
    * @param idx 模式索引值 详见Types.h 文件 枚举 ControlWord_e
    * @return 1 设置成功 0 设置失败
    */
    EXPORT_API int robot_motor_set_control_world(RobotMotor* motor, unsigned int idx);

    /**
    * @brief 设置电机偏移角度
    * @param motor 电机对象
    * @param idOffset_anglex 偏移角度 
    * @return 1 设置成功 0 设置失败
    */
    EXPORT_API int robot_motor_off_set_angle(RobotMotor* motor, float idOffset_anglex);


    /**
     * @brief mit模式设置位置、速度和电流控制参数
     * @param motor 电机对象
     * @param Pos 位置设定值 单位:rad
     * @param Vel 速度设定值 单位:rad/s
     * @param Cur 电流设定值 单位:A
     * @return void
     */

    EXPORT_API void robot_motor_set_pos(RobotMotor* motor, float Pos, float Vel, float Cur);


    /**
    * @brief 位置模式设置位置控制参数
    * @param motor 电机对象
    * @param Pos 位置设定值 单位:rad
    * @return void
    */
    EXPORT_API void  robot_motor_set_position(RobotMotor* motor, float Pos);

    /**
     * @brief  根据电机列表组大包发送
     * @param ctx 主板对象
     * @return void
     */

    EXPORT_API void robot_motor_set_big_pose(RobotCtx* ctx );




     


     /**
     * @brief 快速获取位置、速度和估计转矩 此函数用于高速采样场景
     * 
     * @param Pos 存储位置值的引用  单位:rad
     * @param Vel 存储速度值的引用  单位:rad/s
     * @param Tor_e 存储估计转矩值的引用  单位: N.m
     * @param FastStateMechine;  //stateMechine
     * @param FastMosTemperature; //mos 温度
     * @param FastWindingTemperature; //绕组温度
     * @param FastBusVoltage;   //母线电压
     * @param FastErrorCode;   //错误码
     * @return void
     */

     EXPORT_API void robot_motor_get_PVCTFast(RobotMotor* motor,float *Pos, float *Vel, float *Tor_e,unsigned int* FastStateMechine,
      unsigned int* FastMosTemperature, unsigned int* FastWindingTemperature,
      unsigned int* FastBusVoltage, unsigned int* FastErrorCode);


   

    /**
     *@brief 获取编码器值
     *@param motor        电机对象
     *@param EncoderValue 编码器值
     *@return void
    */

    EXPORT_API void robot_motor_get_EncoderValue(RobotMotor* motor, unsigned int* EncoderValue);

     /**
     *@brief 获取电机使能状态
     *@param motor        电机对象
     *@param world        使能状态
     *@return 1 返回成功，0 返回失败
    */

    EXPORT_API int robot_motor_get_EnableValue(RobotMotor* motor, unsigned int* world);


 


    /**
   * @brief 设置开环强脱电压 
   * @param motor       电机对象
   * @param Voltage     电压
   * @return  1 返回成功，0 返回失败
   */

    EXPORT_API int robot_motor_set_SetVoltageValue(RobotMotor* motor, float Voltage);




    /**
    * @brief  设置电机ID
    * @param motor 电机对象
    * @param Id  电机ID
    * @return  1 返回成功，0 返回失败， -1 电机对象为nullptr
    */

    EXPORT_API int  robot_motor_set_motor_id(RobotMotor* motor,unsigned short Id);




    /**
     * @brief 设置心跳关闭
     * @param motor 电机对象
     * @param state 设置心跳周期
     * @return 1 返回成功，0 返回失败 -1 电机对象为nullptr
     */

    EXPORT_API int robot_motor_setHeartbeat(RobotMotor* motor,unsigned short state);


    /**
    * @brief  获取电机ID
    * @param ctx 电机对象
    * @param canid can id
    * @param lineid line id
    * @return  void
    */

    EXPORT_API void  robot_motor_get_motor_id(RobotMotor* motor, unsigned short* canid, unsigned short* lineid);


    /**
    * @brief 一键ota主板升级
    * @param ctx 主板对象
    * @param filename 文件的绝对路径
    * @param func 返回进度及错误码，1为成功，-1为握手失败，-2 发送开始传输标志失败，-3发送传输文件失败，-4停止传输失败
    * @return  1 返回成功
    */

    EXPORT_API int  robot_mother_board_one_click_ota_upgradeing(RobotCtx* ctx, const char* filename, progress_callback func);

   /**
    * @brief 查看主板固件版本号
    * @param ctx   主板对象
    * @param version   固件版本号
    * @return 返回 1 为成功,0为失败
   */

    EXPORT_API int  robot_motor_get_mother_board_firmware_version(RobotCtx* ctx,char* version);

    /**
    * @brief 一键ota电机升级
    * @param motor    电机对象
    * @param filename 文件的绝对路径
    * @param func 返回进度及错误码，1为成功，-1为握手失败，-2 发送开始传输标志失败，-3发送传输文件失败，-4停止传输失败
    * @return   1 返回成功
   */

    EXPORT_API int  robot_motor_one_click_ota_upgradeing(RobotMotor* motor, const char* filename, progress_callback func);

    /**
    * @brief 查看电机固件版本号
    * @param motor   电机对象
    * @param version 固件版本号
    * @return   1 返回成功 0 返回失败
    */

    EXPORT_API int  robot_motor_get_motor_firmware_version(RobotMotor* motor, char* version);

  
    /**
    * @brief 速度模式设置电机速度控制参数
    * @param motor 电机对象
    * @param Vel 速度设定值 单位:rad/s
    * @return void
    */

    EXPORT_API void robot_motor_set_vel(RobotMotor* motor, float Vel);



    /**
     * @brief 电流模式设置电机电流控制参数
     * @param motor 电机对象
     * @param Cur 电流设定值 单位:A
     * @return void
     */
    EXPORT_API void robot_motor_set_cur(RobotMotor* motor, float Cur);

    /**
    * @brief 设置pd模式 PD控制参数
    * @param motor 电机对象
    * @param PDKp PD环比例系数
    * @param PDKd PD环微分系数
    * @return 1 成功，0 失败
    */

    EXPORT_API int robot_motor_set_pd(RobotMotor* motor, float PDKp, float PDKd);
    
     /**
    * @brief 获取PD环PD控制参数
    * @param motor 电机对象
    * @param PDKp PD环比例系数
    * @param PDKd PD环微分系数
    * @return void
    */

    EXPORT_API void robot_motor_get_pd(RobotMotor* motor, float* PDKp, float* PDKd);



     /**
     * @brief 设置速度环 PID控制参数
     * @param motor 电机对象
     * @param VelKp 速度环 比例系数P
     * @param VelKi 速度环 比例系数I
     * @param VelKd 速度环 积分系数D
     * @return 1 成功，0 失败
     */

    EXPORT_API int robot_motor_setVelPID(RobotMotor* motor, float VelKp, float VelKi, float VelKd);



     /**
     * @brief 获取速度环 PID控制参数
     * @param motor 电机对象
     * @param VelKp 速度环 比例系数P
     * @param VelKi 速度环 比例系数I
     * @param VelKd 速度环 积分系数D
     * @return 1 成功，0 失败
     */

    EXPORT_API int robot_motor_getVelPID(RobotMotor* motor, float* VelKp, float* VelKi, float* VelKd);


     /**
     * @brief 设置位置环 PID控制参数
     * @param motor 电机对象
     * @param PosKp 位置环比例系数P
     * @param PosKi 位置环比例系数I
     * @param PosKd 位置环积分系数D
     * @return 1 成功，0 失败
     */

    EXPORT_API int robot_motor_setPosPID(RobotMotor* motor, float PosKp, float PosKi, float PosKd);



     /**
     * @brief 获取位置环 PID控制参数
     * @param motor 电机对象
     * @param PosKp 位置环比例系数P
     * @param PosKi 位置环比例系数I
     * @param PosKd 位置环积分系数D
     * @return 1 成功，0 失败
     */

    EXPORT_API int robot_motor_getPosPID(RobotMotor* motor, float* PosKp, float* PosKi, float* PosKd);

     /**
   * @brief 升级前握手（擦falsh）
   * @param  motor 电机指针
   * @return 主板准备是否成功 1成功，0 失败
   * @warning
   */
    EXPORT_API int  robot_motor_start_OTA_upgrade(RobotMotor* motor);


    /**
   * @brief 传输文件
   * @param  motor 电机指针
   * @param  filename 电机固件
   * @return 主板准备是否成功 成功 1 ， 失败 -1 发送升级开始标志失败、-2 传输文件失败
   * @warning
   */
    EXPORT_API int  robot_motor_send_OTA_upgrade_data(RobotMotor* motor,char* filename);


     /**
   * @brief 停止传输
   * @param  motor 电机指针
   * @param  filename 电机固件
   * @return 主板准备是否成功 1成功，0 失败
   * @warning
   */
    EXPORT_API int  robot_motor_stop_OTA_upgrade_sign(RobotMotor* motor);



    /**
   * @brief 升级前握手（擦falsh）
   * @param  ctx 主板对象
   * @return 主板准备是否成功 1成功，0 失败
   * @warning
   */
    EXPORT_API int  robot_mother_board_start_OTA_upgrade(RobotCtx* ctx);


    /**
   * @brief  传输文件
   * @param  ctx 主板对象
   * @param  filename 电机固件
   * @return 主板准备是否成功 成功 1， 失败 -1 发送升级开始标志失败、-2 传输文件失败
   * @warning
   */
    EXPORT_API int  robot_mother_board_send_OTA_upgrade_data(RobotCtx* ctx,char* filename);


     /**
   * @brief 停止传输
   * @param  ctx 主板对象
   * @param  filename 电机固件
   * @return 主板准备是否成功 1成功，0 失败
   * @warning
   */
    EXPORT_API int  robot_mother_board_stop_OTA_upgrade_sign(RobotCtx* ctx);
    



        
    /**
    * @brief 释放单电机结构体
    * @param  ctx           主板对象
    * @param  motor         电机指针
    * @return void
    */

    EXPORT_API void robot_destroy_motor(RobotCtx* ctx, RobotMotor* motor);

    /**
   * @brief 获取sdk版本
   * @return char*  sdk版本号
   */

    EXPORT_API char* get_sdk_version();


    /**
     * @brief 保存电机参数设置
     * @param motor 电机对象
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_sys_save_to_flash(RobotMotor* motor);

    
     /**
     * @brief 设置电机执行器参数
     * @param motor 电机对象
     * @param type 执行器类别
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_actuator_type_param(RobotMotor* motor,ActuatorType_e type);


     /**
     * @brief 获取电机执行器参数
     * @param motor 电机对象
     * @param type 执行器类别
     * @return 1 设置成功 0 设置失败
     */

     

    EXPORT_API int robot_motor_get_actuator_type_param(RobotMotor* motor,ActuatorType_e* type);

     /**
     * @brief 设置电机 执行器基础配置
     * @param motor 电机对象
     * @param param 执行器基础配置
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_actuator_bsae_param(RobotMotor* motor,ActuatorBsaeParam_t param);

     /**
     * @brief 获取电机 执行器基础配置
     * @param motor 电机对象
     * @param param 执行器基础配置
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_get_actuator_bsae_param(RobotMotor* motor,ActuatorBsaeParam_t* param);


      /**
     * @brief 设置电机 执行器传感器参数
     * @param motor 电机对象
     * @param param 执行器传感器参数
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_actuator_sensor_param1(RobotMotor* motor,ActuatorSensorParam1_t param);

     /**
     * @brief 获取电机 执行器传感器参数
     * @param motor 电机对象
     * @param param 执行器传感器参数
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_get_actuator_sensor_param1(RobotMotor* motor,ActuatorSensorParam1_t* param);

      /**
     * @brief 设置电机 执行器传感器参数
     * @param motor 电机对象
     * @param param 执行器传感器参数
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_actuator_sensor_param2(RobotMotor* motor,ActuatorSensorParam2_t param);

     /**
     * @brief 获取电机 执行器传感器参数
     * @param motor 电机对象
     * @param param 执行器传感器参数
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_get_actuator_sensor_param2(RobotMotor* motor,ActuatorSensorParam2_t* param);


     /**
     * @brief 设置电机 执行器控制参数 
     * @param motor 电机对象
     * @param param 执行器控制参数 
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_actuator_control_params(RobotMotor* motor,ActuatorControlParam_t param);

    /**
     * @brief 获取电机 执行器控制参数 
     * @param motor 电机对象
     * @param param 执行器控制参数 
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_get_actuator_control_params(RobotMotor* motor,ActuatorControlParam_t* param);


     /**
     * @brief 设置电机 执行器功能配置 
     * @param motor 电机对象
     * @param param 执行器功能配置 
     * @return  1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_actuator_func_param(RobotMotor* motor,ActuatorFuncParam_t param);

     /**
     * @brief 获取电机 执行器功能配置 
     * @param motor 电机对象
     * @param param 执行器功能配置 
     * @return  1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_get_actuator_func_param(RobotMotor* motor,ActuatorFuncParam_t* param);
    
     /**
     * @brief 设置电机 电机参数
     * @param motor 电机对象
     * @param param 电机参数 
     * @return  1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_motor_param(RobotMotor* motor,MotorParam_t param);

    /**
     * @brief 获取电机 电机参数
     * @param motor 电机对象
     * @param param 电机参数 
     * @return  1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_get_motor_param(RobotMotor* motor,MotorParam_t* param);

    /**
     * @brief 设置电机 硬件参数
     * @param motor 电机对象
     * @param param 硬件参数 
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int  robot_motor_set_hardware_param(RobotMotor* motor,HardwareParam_t param);

     /**
     * @brief 获取电机 硬件参数
     * @param motor 电机对象
     * @param param 硬件参数 
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int  robot_motor_get_hardware_param(RobotMotor* motor,HardwareParam_t* param);

     /**
     * @brief 设置电机 强拖控制参数
     * @param motor 电机对象
     * @param param 强拖控制参数 
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int  robot_motor_set_force_param(RobotMotor* motor,ForceParam_t param);

    /**
     * @brief 获取电机 强拖控制参数
     * @param motor 电机对象
     * @param param 强拖控制参数 
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int  robot_motor_get_force_param(RobotMotor* motor,ForceParam_t* param);

    /**
     * @brief 设置电机 无感控制参数
     * @param motor 电机对象
     * @param param 无感控制参数
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_fulxObs_param(RobotMotor* motor,FulxObsParam_t param);

     /**
     * @brief 获取电机 无感控制参数
     * @param motor 电机对象
     * @param param 无感控制参数
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_get_fulxObs_param(RobotMotor* motor,FulxObsParam_t* param);

    /**
     * @brief 设置电机 电流力矩转换三次多项式系数
     * @param motor 电机对象
     * @param param 电流力矩转换三次多项式系数
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_torqueCalib_param(RobotMotor* motor,TorqueCalibParam_t param);

    /**
     * @brief 获取电机 电流力矩转换三次多项式系数
     * @param motor 电机对象
     * @param param 电流力矩转换三次多项式系数
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_get_torqueCalib_param(RobotMotor* motor,TorqueCalibParam_t* param);


      /**
     * @brief 设置电机 摩擦参数
     * @param motor 电机对象
     * @param param 摩擦参数
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_set_friction_param(RobotMotor* motor,FrictionParam_t param);

      /**
     * @brief 设置电机 摩擦参数
     * @param motor 电机对象
     * @param param 摩擦参数
     * @return 1 设置成功 0 设置失败
     */

    EXPORT_API int robot_motor_get_friction_param(RobotMotor* motor,FrictionParam_t* param);

    /**
     * @brief 设置电机 摄氏度电流系数
     * @param motor 电机对象
     * @param param 摄氏度电流系数
     * @return 设置成功返回true，失败返回false
     */

    EXPORT_API int robot_motor_set_tempCur_param(RobotMotor* motor,TempCurParam_t param);

     /**
     * @brief 获取电机 摄氏度电流系数
     * @param motor 电机对象
     * @param param 摄氏度电流系数
     * @return 设置成功返回true，失败返回false
     */

    EXPORT_API int robot_motor_get_tempCur_param(RobotMotor* motor,TempCurParam_t* param);

     /**
     * @brief 设置电机 安全保护模块
     * @param motor 电机对象
     * @param param 安全保护模块
     * @return 设置成功返回true，失败返回false
     */

    EXPORT_API int robot_motor_set_errDect_param(RobotMotor* motor,ErrDectParam_t param);

    /**
     * @brief 获取电机 安全保护模块
     * @param motor 电机对象
     * @param param 安全保护模块
     * @return 设置成功返回true，失败返回false
     */

    EXPORT_API int robot_motor_get_errDect_param(RobotMotor* motor,ErrDectParam_t* param);


#ifdef __cplusplus  
}
#endif



#endif