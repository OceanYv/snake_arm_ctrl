/**
 * @file motor_commu.h
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 电机通讯类，只关注电机、CAN分析仪，不关注关节、机械臂等
 * @version 0.1
 * @date 2022-05-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MOTOR_COMMU_H
#define MOTOR_COMMU_H

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "controlcan.h"
#include "common_define.h"
#include "common_function.h"

/*******************/
/* 电机命令相关宏定义 */
/*******************/
#define MOTOR_READ_PID 0x30// 读取PI参数
#define MOTOR_WRITE_PID_RAM 0x31// 写入PI参数到RAM，断电丢失
#define MOTOR_WRITE_PID_ROM 0x32// 写入PI参数到ROM，断电保存
#define MOTOR_READ_ACC 0x33// 读取加速度
#define MOTOR_WRITE_ACC 0x34// 写入加速度到RAM，断电丢失

#define MOTOR_CLOSE 0x80// 电机关闭（清除之前的运动状态、控制指令）
#define MOTOR_STOP 0x81// 电机停止(但是不清除状态)
#define MOTOR_WORK 0x88// 电机运行（恢复停止之前的控制方式）

#define MOTOR_READ_ECD 0x90// 读取编码器
#define MOTOR_SET_ZERO_ECD 0x91// 将编码器值写入ROM，作为电机零点
#define MOTOR_SET_ZERO_CUR 0x19// 将当前位置写入ROM，作为电机零点
#define MOTOR_READ_MULT_ANG 0x92// 读取多圈角度
#define MOTOR_READ_SING_ANG 0x94// 读取单圈角度
// #define MOTOR_CLR_ANG 0x95// 清除电机角度（设置电机初始位置）

#define MOTOR_READ_ST1 0x9A// 读取电机状态1命令(温度、电压、错误信息)
#define MOTOR_CLR_ERRO 0x9B// 清除电机错误标志命令
#define MOTOR_READ_ST2 0x9C// 读取电机状态2命令（温度、电流、转速、编码器）
#define MOTOR_READ_ST3 0x9D// 读取电机状态3命令（温度、三相电流）

// #define MOTOR_XXXXXXX 0xA1// 转矩闭环控制
#define MOTOR_SPEED_CONTROL 0xA2// 速度闭环控制
#define MOTOR_MULT_POS 0xA3// 位置闭环控制命令1：使电机运动到指定多圈角度
#define MOTOR_MULT_POS_LIM 0xA4// 位置闭环控制命令2：使电机运动到指定多圈角度，且限定最大速度
#define MOTOR_SING_POS 0xA5// 位置闭环控制命令3：使电机运动到指定单圈角度
#define MOTOR_SING_POS_LIM 0xA6// 位置闭环控制命令4：使电机运动到指定单圈角度，且限定最大速度
#define MOTOR_RELA_POS 0xA7// 位置闭环控制命令5：使电机运动指定角度（相对位置）
#define MOTOR_RELA_POS_LIM 0xA8// 位置闭环控制命令6：使电机运动指定角度（相对位置），且限定最大速度

/****************/
/* 自定义数据类型 */
/****************/

// 通过该结构体，可以定位到某个分析仪设备的某个CAN端口上
typedef struct _CAN_NUMBER_VECTOR
{
    DWORD DeviceType; // 设备类型，对应不同的产品型号，CANalyst-II对应4，或者取宏定义【VCI_USBCAN2】
    DWORD DeviceInd;  // 设备索引，从0开始分别对应计算机上挂载的各个分析仪
    DWORD CANInd;      // CAN通道索引。第几路 CAN。即对应卡的CAN通道号,CAN1为0,CAN2为1
} CAN_NUMBER_VECTOR;

/***********/
/* 类的定义 */
/***********/

class motor_commu
{
public:

    /****************/
    /* 构造、析构函数 */
    /****************/

    /**
     * @brief Construct a new motor commu object
     * 只进行了各种配置参数的预读取，没有进行任何实质性的通讯和配置工作
     * 之后要需要自行调用can_device_init函数，才能够使用串口
     */
    motor_commu(ros::NodeHandle *nh);

    /**
     * @brief Destroy the motor commu object
     */
    ~motor_commu();

    /*****************/
    /* 通讯设备操作方法 */
    /*****************/

    /**
     * @brief CAN设备的初始化，以及电机的可通讯情况检测
     */
    bool can_device_init();

    /**
     * @brief 电机初始化函数，在对象被创建之后，需要调用该函数来进行初始化
     * @param Init_MotorRota_Array 机械臂启动时，通过关节角计算的电机初始位置
     * @return  1: 初始化成功；
     *         -1: 硬件设备连接失败；
     *         -2：零点配置文件缺失，需要重启修复；
     *         -4：电机实际位置与配置文件中值不符；
     *         -5：电机校准判停异常；
     *         -6：机械臂需要上电重启；
     */
    int motor_commu_init(Arrayf_3_Joint Init_MotorRota_Array);

    /**
     * @brief CAN分析仪复位，一般用于出现异常的情况;调用该函数之后应当再调用motor_commu_init()函数
     * @return true 复位成功。若出现失败通常是硬件连接问题或配置问题。
     */
    bool Reset_CANanalyst();

    /*****************/
    /* 高级电机操作方法 */
    /*****************/
    
    /**
     * @brief 同时指定多个电机的多圈目标位置，运动并判停
     * 
     * @param MotorRota_Array 目标位置对应的电机多圈角度，单位:度
     * @param delta_tolerance 判停的允许偏差，单位：degree，默认值：8degree
     * @return true 运动并判停成功
     */
    bool multmotors_mult_pos_lim(Arrayf_3_Joint MotorRota_Array, float delta_tolerance = 8);

    /**
     * @brief 通过打开位置环使所有电机自锁
     * 通过调用motor_rela_pos_lim实现，之后直接清空消息缓存区，无回信解析过程
     */
    void motors_selflock();

    /**
     * @brief 调用motor_stop使电机停止自锁，以减少电机发热
     */
    bool motors_selfunlock();

    /**
     * @brief 获取当前各个电机的多圈角度，并更新到Cur_MotorRota_Array中
     * 
     * @return Arrayf_3_Joint 存放电机当前的多圈位置,单位：度
     */
    Arrayf_3_Joint get_Cur_MotorMultAng();

    /**
     * @brief Get the Cur MotorSingAng object
     * @return Arrayf_3_Joint 存放电机当前的单圈位置,单位：度,范围[-180，180)
     */
    Arrayf_3_Joint get_Cur_MotorSingAng();

    /**************/
    /* 电机相关变量 */
    /**************/
    Arrayf_3_Joint Cur_MotorRota_Array ; // 存放电机当前的电机多圈角度值,单位：度。每次调用motor_read_mult_ang后（实际上在程序里，只有get_Cur_MotorMultAng中用到了该函数），通过motor_ack_parse函数解析回复包以更新；
    Arrayf_3_Joint Next_MotorRota_Array; // 存放电机当前控制结束后的电机多圈角度值,单位：度。每次调用motor_mult_pos_lim / motor_rela_pos_lim后更新；

    Arrayf_3_Joint MotorRota_Compensate_Array; // 存放对电机绝对位置的补偿,单位：度，每次执行初始化函数时更新。电机重新上电后，多圈位置会清空复位为单圈绝对位置，导致从电机读到的多圈位置与机械臂零点处对应的电机多圈位置不一致，为此引入该补偿。

private:

    ros::NodeHandle *_nh;

    /*******************/
    /* CAN分析仪相关变量 */
    /*******************/
    VCI_BOARD_INFO devices_info[CANalyst_NUM + 1]; //存放连接到计算机的CANalyst设备信息
    CAN_NUMBER_VECTOR can_numbervector; // 存放用于控制电机的CAN的端口与分析仪编号
    VCI_INIT_CONFIG InitConfig; //存放对应CAN端口的初始化信息

    /**************/
    /* 电机相关变量 */
    /**************/

    // 电机通讯相关变量
    VCI_CAN_OBJ frame_trans;     // 发送帧数据格式
    VCI_CAN_OBJ frame_recv[CAN_RECV_ARRY_SIZE];  // 接收帧数据
    UINT recv_len;               // frame_recv数组中有数据的帧数，每次调用can_recv函数之后会更新

    // 电机控制相关变量
    int Motor_Online_Num; // 可通讯的电机数，每次初始化时进行判断，通过对Motor_Online_Flag求和计算，因此更新该值前应先更新Motor_Online_Flag；
    Eigen::Array<int,3,JOINT_NUM> Motor_Online_Flag; // 存放电机在线标识，每次初始化时通过motor_work函数及其回复报文的解析进行判断
    Eigen::Array<int,3,JOINT_NUM> Motor_Stop_Flag; // 存放电机判停标识，-1为离线，0为未停止，1为判停成功

    std::string MotorZero_savefile; // 存放软零点补偿值的文件名，该文件在每次执行零点标定函数时更新;储存数据单位：度

    Arrayf_3_Joint Cur_MotorSingAng_Array; // 存放电机当前的单圈位置,单位：度,范围[-180，180)。每次调用motor_read_sing_ang后更新；
    Arrayf_3_Joint MotorZero_Compensate_Array; // 电机软零点补偿。单位：度。其值从文件MotorZero_savefile读取，值的范围应为[-180，180）。

    float StopJudge_Period;    // 电机判停时的问讯周期，单位：s
    float StopJudge_Timeout;   // 电机判停超时时间，单位：s。
    int time_out_cnt_max;      // time_out_cnt_max = StopJudge_Timeout / StopJudge_Period;

    /*****************/
    /* 基础电机操作方法 */
    /*****************/

    /**
     * @brief 电机关闭（清除之前的运动状态、控制指令）；
     *        命令号：0x80；返回数据：无；解析动作：无。
     * @param slave_ID 控制电机ID
     */
    bool motor_close(UINT slave_ID);

    /**
     * @brief 电机停止(但是不清除状态)。注意与motor_stop_judge函数的区别；
     *        命令号：0x81；返回数据：无；解析动作：无。
     * @param slave_ID 控制电机ID
     */
    bool motor_stop(UINT slave_ID);

    /**
     * @brief 电机运行（恢复停止之前的控制方式）,与motor_stop组合使用；
     *        也应该在初始化的时候调用该函数，以启动电机及判断电机在线情况；
     *        命令号：0x88；返回数据：无；解析动作：更新对应电机的在线标志位。
     * @param slave_ID 控制电机ID
     */
    bool motor_work(UINT slave_ID);

    /**
     * @brief 读取多圈角度。注意：每次重新上电之后，多圈角度会清除，且与单圈角度保持一致（处于-180～180之间）。
     *        命令号：0x92；返回数据：角度；解析动作：将角度更新到Cur_MotorRota_Array中。
     * @param slave_ID 控制电机ID
     */
    bool motor_read_mult_ang(UINT slave_ID);

    /**
     * @brief 读取单圈角度（处于[-180，180)范围内）
     *        命令号：0x94；返回数据：角度；解析动作：将角度更新到Cur_MotorSingAng_Array中。
     * @param slave_ID 控制电机ID
     */
    bool motor_read_sing_ang(UINT slave_ID);

    /**
     * @brief 速度控制；
     *        命令号：0xA2；返回数据：电机温度、转矩电流、电机速度、编码器位置；解析动作：杂碎信息的屏幕输出。
     * @param slave_ID 控制电机ID
     * @param speedControl 单位：degree/second
     * @return true 
     */
    bool motor_speed_control(UINT slave_ID, float speedControl);

    /**
     * @brief 基于can_trans()和电机控制通讯协议,进一步封装的多圈角度控制，且限定最大速度；
     *        命令号：0xA4；返回数据：电机温度、转矩电流、电机速度、编码器位置；解析动作：更新Next_MotorRota_Array。
     * @param slave_ID 控制电机ID
     * @param max_speed 最大转动速度，单位 1dps/LSB 
     * @param angle_control 多圈绝对角度值，单位 0.01degree/LSB
     * @return bool true: 发送成功， flase: 发送失败
     */
    bool motor_mult_pos_lim(UINT slave_ID, uint16_t max_speed, int32_t angle_control);

    /**
     * @brief 基于can_trans()和电机控制通讯协议,进一步封装的相对位置控制。向指定电机发送相对运动指令。
     *        命令号：0xA8；返回数据：电机温度、转矩电流、电机速度、编码器位置；解析动作：更新Next_MotorRota_Array。
     * @param slave_ID 控制电机ID
     * @param max_speed 最大转动速度，单位 1dps/LSB 
     * @param angle_control 相对于当前位置的运动量，单位 0.01degree/LSB
     * @return bool true: 发送成功， flase: 发送失败
     */
    bool motor_rela_pos_lim(UINT slave_ID, uint16_t max_speed, int32_t angle_control);

    /**
     * @brief 电机判停函数，对所有在线电机进行判停
     *        【注意：判停操作会清空缓存区】
     * @param delta_tolerance 判停的允许偏差，单位：degree
     * @return true 停止
     * @return false 未停止
     */
    bool motors_stop_judge(float delta_tolerance = 1.0);

    /**
     * @brief 对当前frame_recv中储存的所有数据帧进行解析和输出
     *        另外，将接收到的数据存入相应的成员变量中
     */
    void motor_ack_parse();

    /**************/
    /* CAN操作方法 */
    /**************/

    /**
     * @brief 获取计算机上连接的CAN分析仪信息、连接，并对其进行初始化配置;
     *        流程：VCI_FindUsbDevice2 -> VCI_OpenDevice -> VCI_InitCAN -> VCI_StartCAN
     * @arg devices_info 将会储存搜索到的分析仪信息；
     * @arg can_numbervector 储存连接的目标CAN端口编号；
     * @arg InitConfig 储存用于CAN初始化的参数；
     * @return true 连接成功
     * @return false 连接失败
     */
    bool start_CANanalyst();

    /**
     * @brief 向指定设备发送指定数据，函数内部通过延时限制帧率为4000帧/秒
     * 
     * @param slave_ID 从机设备ID，设备号应在0x161~0x180
     * @param Data 8字节数组
     * @return true 发送成功
     * @return false 反复尝试3次后仍发送失败
     */
    bool can_trans(UINT slave_ID, BYTE *Data);

    /**
     * @brief 接收指定数量的帧。若指定数量超出frame_recv数组大小，多出来的部分将会被丢弃。
     *        每次调用该函数，会覆盖上次调用中接收到的数据;
     *        该函数只负责将包接收到消息数组中，
     * @param expect_frame_num 
     * @return UINT 
     */
    UINT can_recv(UINT expect_frame_num);

    /**
     * @brief 清除CAN分析仪中未读入上位机的缓存数据包
     * @return true
     */
    bool can_buff_clear();
};

#endif