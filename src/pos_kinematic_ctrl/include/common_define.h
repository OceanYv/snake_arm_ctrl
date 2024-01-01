/**
 * @file common_define.h
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 一些与硬件相关、基本上不会变动的宏定义
 * @version 0.1
 * @date 2022-05-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef COMMON_DEFINE
#define COMMON_DEFINE

    #define M_PI_12 0.26179938779914943 // PI/12

    /************/
    /* 机械臂参数 */
    /************/

    #define JOINT_NUM 8 //臂段个数，一个臂段包含两个关节
    #define MOTOR_NUM (3*JOINT_NUM) //电机个数


    /*******************/
    /* 机械臂运动控制参数 */
    /*******************/
    #define MAX_JOINTANGLE (M_PI/6) // 允许的最大转角，弧度

    /************/
    /* 机械臂尺寸 */
    /************/

    #define LINE_DIAM 2.5 // 钢丝绳直径，mm
    #define COIL_SHAFT_PITHC 6 // 卷线盘上线槽的螺距，mm

    #define ARM_BASE_HEIGHT 1045.0 // 机械臂基部相对于小车地面的高度，mm
    #define ARM_BASE_X_OFFSET 120.0 // 机械臂基部相对于底座中心在x方向上的偏移，mm
    #define ARRANGE_LINE_RADIUS 40.0 // 机械臂上，各驱动绳在端面上分布的圆的半径，mm
    #define FIX_POLE_PART_LENGTH 89.0 // 机械臂圆筒段长度,mm
    #define ROT_SHAFT_PART_LENGTH 61.0 // 机械臂转轴段长度,mm

    #define END_POSITION_X 114.0 // 末端中心到最后一个关节中心x方向上的距离，mm
    #define END_POSITION_Y   0.0 // 末端中心到最后一个关节中心y方向上的距离，mm
    #define END_POSITION_Z   0.0 // 末端中心到最后一个关节中心z方向上的距离，mm
    
    #define COIL_SHAFT_DIAM_1 27 // 第1节对应卷线轴直径，30-3mm
    #define COIL_SHAFT_DIAM_2 39 // 第2节对应卷线轴直径，42-3mm
    #define COIL_SHAFT_DIAM_3 55 // 第3节对应卷线轴直径，58-3mm
    #define COIL_SHAFT_DIAM_4 67 // 第4节对应卷线轴直径，70-3mm
    #define COIL_SHAFT_DIAM_5 67 // 第5节对应卷线轴直径，70-3mm
    #define COIL_SHAFT_DIAM_6 67 // 第6节对应卷线轴直径，70-3mm
    #define COIL_SHAFT_DIAM_7 64 // 第7节对应卷线轴直径，67-3mm
    #define COIL_SHAFT_DIAM_8 67 // 第8节对应卷线轴直径，70-3mm

    #define MOTOR_REDUC_RATIO_1 25 // 第1节对应电机减速比
    #define MOTOR_REDUC_RATIO_2 28 // 第2节对应电机减速比
    #define MOTOR_REDUC_RATIO_3 28 // 第3节对应电机减速比
    #define MOTOR_REDUC_RATIO_4 25 // 第4节对应电机减速比
    #define MOTOR_REDUC_RATIO_5 16 // 第5节对应电机减速比
    #define MOTOR_REDUC_RATIO_6 7 // 第6节对应电机减速比
    #define MOTOR_REDUC_RATIO_7 4 // 第7节对应电机减速比
    #define MOTOR_REDUC_RATIO_8 4 // 第8节对应电机减速比

    /***********/
    /* 电机参数 */
    /***********/
    #define MOTOR_BASE_ADDRESS 0x141 // 电机CAN通讯的基地址
    #define MOROR_SELFLOCK_WAITTIME 0.1 // 发送自锁命令后，制动器释放前的等待时间（等待电机响应&施加电流），单位：秒
    #define BRAKE_HOLD_WAITTIME 0.15 // 发送制动器锁紧命令后、电机释放命令前，需要等待的时间
    #define MAX_SPEED 50 // 电机最大速度，单位：dps/LSB
    /***********/
    /* 系统参数 */
    /***********/

    #define CANalyst_NUM 1 // CANalyst设备个数
    #define CAN_RECV_ARRY_SIZE 80 // 用于接收can帧的数组大小，对应可以储存的帧数。官方文档中建议大小为3000，但是考虑到本使用场景中与24个电机的通讯基本上是“收发收发”形式，所以仅设置了80。

#endif