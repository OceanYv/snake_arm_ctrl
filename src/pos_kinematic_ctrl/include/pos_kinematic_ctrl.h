/**
 * @file pos_kinematic_ctrl.h
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 机械臂正运动学控制器，输入各个关节角，输出机械臂的姿态、速度等状态，并于具体硬件交互
 * @version 0.1
 * @date 2022-05-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef POS_KINEMATIC_CTRL
#define POS_KINEMATIC_CTRL

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "motor_commu.h"
#include "serial_commu.h"
#include "common_define.h"
#include "common_function.h"

/****************/
/* 自定义数据类型 */
/****************/

enum SNAKEARM_STATE
{
    IDLE    = 1   ,
    CORRECT       ,
    WAIT          ,
    MOVE_AUTO     ,
    MOVE_DEBUG    ,
    ERROR
};

/***********/
/* 类的定义 */
/***********/

class pos_kinematic_ctrl
{
public:

    /****************/
    /* 构造、析构函数 */
    /****************/

    /**
     * @brief Construct a new pos kinematic ctrl object
     * 
     */
    pos_kinematic_ctrl(ros::NodeHandle *nh);

    /**
     * @brief Destroy the pos kinematic ctrl object
     * 
     */
    ~pos_kinematic_ctrl();

    /*****************/
    /* 硬件设备对象例化 */
    /*****************/

    motor_commu *_motor_device; // 电机通讯实例，通过该对象可以连接CAN分析仪并与各个电机进行通讯
    serial_commu *_serial_device; // 串口通讯实例，通过该对象可以实现串口连接和抱闸的控制

    /*****************/
    /* 正运动学相关变量 */
    /*****************/

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SNAKEARM_STATE SnakeArm_State; // 机械臂状态
    bool pose_update_reg;          // 自上次读取后，机械臂的当前位姿值是否被更新过，属于read-clear类型寄存器；每运动1小步即置1；
    Arrayf_2_Joint Last_JointAngle_Array;  // 存放上一步运动后各关节角。单位：弧度。调用move_to_posture时，每运动一小步进行一次更新并拉高pose_update_reg，以供node读取并更新到tftree中

    /**************/
    /* 运动控制方法 */
    /**************/

    /**
     * @brief 姿态运动控制，是本类的核心函数之一。其内部操作在主要包括：
     *          1. 获取期望运动到的姿态（关节角，JointAngle_Array）
     *          2. 调用规划函数进行分步（包括释放余量、角度划分、收紧余量）
     *          3. 为每步调用JointAngle2MotorAngle函数、_motor_device对象，以对电机进行驱动。
     *              这个过程中还
     * @param Target_JointAngle_Array 期望运动到的姿态，单位：弧度
     * @return true 
     * @return false 
     */
    bool move_to_posture(Arrayf_2_Joint Target_JointAngle_Array);

    /**
     * @brief 手动电机控制
     *        每次输入控制指令后，电机运动前后程序会内部执行制动器的开关
     */
    bool Manual_MotorCtrl();

    /*******************/
    /* 机械臂状态控制方法 */
    /*******************/

    /**
     * @brief 机械臂重新启动函数
     * 
     * @return true 
     * @return false 
     */
    bool SnakeArm_Reboot();

    /**
     * @brief 关闭机械臂，最主要的是当前机械臂状态、电机状态的保存
     * 
     * @return true 
     */
    bool SnakeArm_Close();

    /**
     * @brief 每完成1小步后将当前关节角写入到文件中
     * 
     * @return true 
     * @return false 
     */
    bool state_save();

    /**
     * @brief 自上次被调用后，机械臂的当前位姿值是否被更新过。如果发生过更新，返回1。
     * 
     * @return true 
     * @return false 
     */
    bool pose_update();

private:

    ros::NodeHandle *_nh;

    /*****************/
    /* 正运动学相关变量 */
    /*****************/

    // 配置参数，基本上都是从yaml文件中获取
    Arrayf_2_Joint Origin_JointAngle;   // 存放原点关节角。单位：弧度

    float Max_StepLength;                   // 最大单步转角。单位：弧度
    float Line_Allowance;                   // 余量释放量相对步长的比例系数，过大会极大影响运动精度且无所裨益，过小可能会使运动过程中的电机有一定卡顿。
    std::string JointAngle_Savefile;        // 存放Last_JointAngle_Array的文件名,储存数据单位：度
    std::string MotorZero_savefile;         // 存放软零点补偿值的文件名，该文件在每次执行零点标定函数时更新;储存数据单位：度

    // 非过程性状态描述参数
    Arrayf_3_Joint MotorZero_Compensate_Array; // 电机软零点补偿。单位：度。其值从文件MotorZero_savefile读取，值的范围应为[-180，180）。只有在执行零点标定时才会更新。

    Arrayf_2_Joint Next_JointAngle_Array;  // 存放下一个运动周期结束后各关节角。单位：弧度
    // Arrayf_3_Joint Last_LineLength_Array;  // 存放上一步运动后各驱动绳相对零点位姿的长度变化（仅考虑机械臂部分）。单位：mm
    // Arrayf_3_Joint Next_LineLength_Array;  // 存放下一个运动周期结束后各驱动绳相对零点位姿的长度变化（仅考虑机械臂部分）。单位：mm

    bool StopFlag;                         // 暂停标志位

    // 一些用于加快运行效率的查找表，其在构造函数初始化时就会定下来
    Eigen::Array<Eigen::Array3f,JOINT_NUM,3> LinePos_FindTab;        // 存放各驱动绳在机械臂传线盘上的起止点，在其所处位置对应的坐标系中的位置
    Eigen::Array<float, 2, JOINT_NUM>        ColiShaft_line_FindTab; // 卷线处部分线段的初始长度，用于之后补偿【由于卷线过程带来端点的轴相移动，引入轻微的偏差】
    Eigen::Array<float, 1, JOINT_NUM>        Slope_Coil_FindTab;     // 转一圈实际收放的线长，考虑卷线过程中端点的轴相移动
    Eigen::Array<float, 1, JOINT_NUM>        Motor_Effi_FindTab;     // 收放1mm，电机实际转过的角度，单位:度
    Eigen::Array<float, 3, JOINT_NUM>        Line_Allowance_Ang;     // Line_Allowance对应的各电机转角，单位:度。

    /**************************/
    /* 位姿空间-关节空间-驱动空间 */
    /**************************/

    /**
     * @brief 计算指定绝对关节角下，理论上各线相对于绝对零点的长度、各电机的绝对转角。
     *        不考虑软零点补偿
     * @param JointAngle_Array 输入参数，存放按步分解之后的关节角。单位：弧度
     * @param MotorRota_Array 输出参数，存放卷轴转角。单位:度 
     */
    void JointAngle2MotorAngle(Arrayf_2_Joint *JointAngle_Array, Arrayf_3_Joint *MotorRota_Array);

    /**
     * @brief 计算给定关节角下，第JointID节的三根驱动绳在第UnitID节范围内的长度
     *          计算长度的方法：获取在同一坐标系下，绳子起点、终点的位置，即绳子的向量表达，然后求模长。
     *                       起点位置依次经过一次旋转、一次平移、一次旋转，即可得到终点位置。
     *                       坐标系定义：x轴沿轴向指向末端，z轴向上，y轴通过右手系确定。
     *                       驱动绳位置编号：沿x轴方向看，从正上方开始顺时针编号为1-1、2-1、3-1、4-1...8-1、1-2...、8-3。
     * @param JointAngle_Array 存放关节角。单位：弧度
     * @param JointID 要计算的绳子驱动的关节，0～JOINT_NUM-1
     * @param UnitID 要计算的绳长所处的位置，0～JOINT_NUM-1，UnitID<=JointID
     * @return Eigen::Array3f 出现异常时返回[-1,-1,-1]
     */
    Eigen::Array3f Cal_LineUnitLength(Arrayf_2_Joint *JointAngle_Array_d, uint8_t JointNum, uint8_t UnitNum);

    /*******************/
    /* 机械臂状态控制方法 */
    /*******************/

    /**
     * @brief 机械臂初始化函数
     * 
     * @return true 
     * @return false 
     */
    bool SnakeArm_Init();

    /**
     * @brief 手动零点标定函数
     * 
     */
    bool Manual_ZeroCalib();
};
#endif