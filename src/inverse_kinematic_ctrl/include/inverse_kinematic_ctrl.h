/**
 * @file inverse_kinematic_ctrl.h
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 机械臂逆运动学控制器，输入目标位姿和环境信息，输出机械臂的关节角，并将关节角发布出去
 * @version 0.1
 * @date 2022-11-22
 * 
 * @copyright Copyright (c) 2022 OceanYv
 * 
 */

#ifndef INVERSE_KINEMATIC_CTRL
#define INVERSE_KINEMATIC_CTRL

// # define TEST

#include <queue>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sisl.h"
#include "GoReadWrite.h"
#include "common_define.h"
#include "common_function.h"

/****************/
/* 自定义数据类型 */
/****************/
#define PATHCURVE_CTRLPOINTS_MAXNUM (JOINT_NUM*3)//脊线曲线控制点个数最大值

/***********/
/* 类的定义 */
/***********/

class inverse_kinematic_ctrl
{

public:

    /****************/
    /* 构造、析构函数 */
    /****************/

    /**
     * @brief Construct a new inverse_kinematic_ctrl object
     * 读取param；
     * 从文件读入/初始化脊线曲线；
     * 初始化Joint2World_Trans和End2Joint_Trans；
     * 进入命令行模式；
     */
    inverse_kinematic_ctrl(ros::NodeHandle *nh);

    /**
     * @brief Destroy the inverse_kinematic_ctrl object
     * 
     */
    ~inverse_kinematic_ctrl();

    /***************/
    /*   对象例化   */
    /***************/
    bool STATE;

    // 当前位置
    double BasePosiCur;            // 当前基部相对全剧坐标系位置，初始化自文件，单位：mm
    Arrayf_2_Joint JointsAngleCur; // 当前关节角，初始化自文件，单位：弧度

    // 用于线程间通信的变量
    bool simulation;          // 仿真模式标志位    
    bool CallService_Flag;    // 完成逆运动学解算、需要调用正运动学包和base控制包的运动服务时拉高。节点中的函数监测到其为高时进行service调用，并在完成调用后拉低
    double Call_BasePosi_Exp; // 期望的基座运动位置，单位：mm
    Arrayf_2_Joint Call_JointsAngle_Exp;  // 期望的机械臂关节角运动位置，单位：rad

    /*****************/
    /* 高级运动控制函数 */
    /*****************/

    /**
     * @brief 通过命令行进行机械臂运动控制的入口函数
     */
    bool inverse_kinematic_ctrl_cmd();

    /**
     * @brief 提供了三种调整脊线曲线的形式。
     *           1. 根据提供的机械臂姿态（两个输入参数）重新生成曲线
     *           2. 通过控制机械臂进行FTL运动，在末端进行曲线的延长或收缩
     *           3. 对每个点的位置进行调整，并可在末端进行点的增减
     * @param BasePosi in 提供的机械臂基部位置参考值，用于可能的初始曲线生成
     * @param JointsAngle in 提供的机械臂关节角参考值，用于可能的初始曲线生成
     * @return bool 
     */
    bool Adjust_Curve(double BasePosi, Arrayf_2_Joint JointsAngle);

    /**
     * @brief FTL控制函数，为调用 FTLInvKine_EndMove函数、FTLInvKine_BaseMove函数 提供统一的cmd接口；
     *        控制值通过service请求发出。
     * @param BasePosi inout 机械臂基座的初始位置，如果函数执行成功，则会将该值更新
     * @param JointsAngle inout 机械臂关节角的初始位置，如果函数执行成功，则会将该值更新
     * @return bool 
     */
    bool FTLInvKine(double& BasePosi, Arrayf_2_Joint& JointsAngle);

    /**
     * @brief 指定末端在曲线上的位移，进行整体关节角的解算。
     *        通过循环调用FTLInvKine_BaseMove函数实现。
     *        注意，只进行解算，不进行控制。若要进行控制，需要在函数外处理
     * @param BasePosi in 机械臂基座的初始位置, 单位：mm
     * @param Part_Num in 参与FTL的臂段数(一个臂段包含两个正交的关节)，最大为JOINT_NUM.
     * @param EndDistExp in  期望末端运动的位移, 单位：mm
     * @param BasePosi_Exp out 期望位移对应的基部位置, 单位：mm
     * @param JointsAngle_Exp out 期望位移对应的机械臂关节角，单位：弧度
     * @return bool 
     */
    bool FTLInvKine_EndMove(double BasePosi, int Part_Num, double EndDistExp,
                            double& BasePosi_Exp, Arrayf_2_Joint& JointsAngle_Exp);

    /**
     * @brief 指定基部的位移，进行整体关节角的解算。
     *        若机械臂原本没有在脊线上，可以通过该函数进行一次基部进给为0的运动，将机械臂拟合上去。
     *        注意，只进行解算，不进行控制。若要进行控制，需要在函数外处理。
     * @param BasePosi in 机械臂基座的初始位置, 单位：mm
     * @param Part_Num in 参与FTL的臂段数(一个臂段包含两个正交的关节)，最大为JOINT_NUM.
     * @param BaseDist_Exp in 期望基部运动的位移, 单位：mm
     * @param BasePosi_Exp out 期望位移对应的基部位置, 单位：mm
     * @param JointsAngle_Exp out 期望位移对应的机械臂关节角，单位：弧度
     * @return bool 
     */
    bool FTLInvKine_BaseMove(double BasePosi, int Part_Num, double BaseDist_Exp,
                             double& BasePosi_Exp, Arrayf_2_Joint& JointsAngle_Exp);

    /**
     * @brief 基于 雅可比矩阵微分迭代法/混合法 进行机械臂的运动控制cmd接口函数，
     *        控制值通过service请求发出，也会更新BasePosiCur、JointsAngleCur的值
     *        雅可比矩阵微分迭代法/混合法的选择通过 传入参数Hybrid来确定。
     * @param Hybrid in 是否通过混合法进行运动
     * @return bool 
     */
    bool JacobianDiffInvKine(bool Hybrid=false);

    /**
     * @brief 基于雅可比矩阵微分迭代法进行机械臂的逆运动学解算。函数思路见论文Algorithm2。
     *        注意，只进行解算，不进行控制。若要进行控制，需要在函数外处理
     * @param EndPoseExp in 期望的末端位姿
     * @param BasePosi_Exp out 期望末端位姿对应的基部位置，单位：mm
     * @param JointsAngle_Exp out 期望末端位姿对应的机械臂关节角，单位：rad
     * @param MoveBase_Flag in 机械臂运动过程中是否移动底座，默认为否
     * @return bool 
     */
    bool JacobianDiffInvKine_ToPose( Eigen::Matrix4d EndPoseExp, 
                                     double& BasePosi_Exp, 
                                     Arrayf_2_Joint& JointsAngle_Exp, 
                                     bool MoveBase_Flag = false);

    /**
     * @brief 基于混合法进行机械臂的逆运动学解算
     *        注意，只进行解算，不进行控制。若要进行控制，需要在函数外处理
     * @param EndPoseExp in 期望的末端位姿
     * @param BasePosi_Exp out 期望末端位姿对应的基部位置，单位：mm
     * @param JointsAngle_Exp out 期望末端位姿对应的机械臂关节角，单位：rad
     * @return bool 
     */
    bool HybridInvKine_ToPose( Eigen::Matrix4d EndPoseExp, 
                               double& BasePosi_Exp, 
                               Arrayf_2_Joint& JointsAngle_Exp);

    /****************/
    /* 基础运动学函数 */
    /***************/

    /**
     * @brief 基于BasePosi、JointsAngle计算机械臂各个关节坐标系相对全局坐标系的位姿, 求解结结果通过Joint2World_Trans输出
     * 
     * @param BasePosi 基部相对全剧坐标系位置, 单位：mm
     * @param JointsAngle 存放关节角的2*8数组，单位：弧度
     * @param Joint2World_Trans 存放world到各个坐标系（base、joint0～joint16、end）的变换。位移向量的单位：mm
     * @return bool 
     */
    bool Position_Solution(double BasePosi, Arrayf_2_Joint& JointsAngle, Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3>& Joint2World_Trans);

    /****************/
    /* 可视化相关函数 */
    /***************/

    /**
     * @brief 将Joint2World_Trans或Joint2World_Trans_Virtual中的坐标系通过topic发布，以在rviz中进行观察机械臂姿态；
     *        基坐标系为world。
     * 
     * @param W2J_Trans 存放从world到各个坐标系（base、joint0～joint16、end）的变换，单位为mm；
     * @param hold_on 是否保留之前绘制的机械臂姿态，默认false；
     * @param color 线条颜色，rgbwk分别为红绿蓝白黑，默认黑；
     * @return bool 
     */
    bool Arm_Position_Pub(Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3>& W2J_Trans, bool hold_on=false, char color='k');

    /**
     * @brief 基于Curve对象，调用s1227()生成一系列的点，并通过topic发布，在rviz中订阅观察；
     * 
     * @param curve Curve对象
     * @param curve_name Curve发布时Marker的名称
     * @param frame_name 曲线所在坐标系的名称,可能使用的是"world" "arm_end_comp"
     * @param color 线条颜色，rgbwk分别为红绿蓝白黑
     * @param hold_on 是否保留之前发布的曲线
     * @param plot_num 曲线上采样点的个数
     * @return bool 成功与否
     */
    bool Curve_Pub(SISLCurve* curve, std::string curve_name, std::string frame_name, char color = 'r', bool hold_on=false, std::size_t plot_num=100);

    /**
     * @brief 将Curve的控制点序列PathCurve_CtrlPoints通过topic发布，在rviz中订阅观察；
     * 
     * @param Points 存放点的数组
     * @param point_num 点的个数
     * @param points_name 
     * @param frame_name 曲线所在坐标系的名称,可能使用的是"world" "arm_end_comp"
     * @param color 点的颜色，rgbwk分别为红绿蓝白黑
     * @return bool 成功与否
     */
    bool Points_Pub(double* Points, int point_num, std::string points_name, std::string frame_name, char color='g');

    bool Listen_mode; // 是否处于监听模式，只有在该模式下才对外界的service请求进行响应。以此可以确保不会有两个控制任务同时执行，避免冲突。

int cnt_test;
#ifdef TEST
    // 测试
    double error_test;

    // 当前基部相对全剧坐标系位置,每次启动后为0，单位：mm
    double get_BasePosiCur(); 
    // 虚拟基部位置, 用于仿真/控制过程中的可视化, 单位：mm
    double get_BasePosi_Virtual(); 
    // 当前关节角，初始化自对topic的订阅，单位：弧度
    Arrayf_2_Joint get_JointsAngleCur(); 
    // 虚拟关节角，用于仿真/控制过程中的可视化，单位：弧度
    Arrayf_2_Joint get_JointsAngle_Virtual(); 

    // 通过动态矩阵存放雅可比矩阵，其大小可以通过resize调整
    Eigen::MatrixXd get_JacobianMat(); 
    // 脊线曲线
    SISLCurve* get_PathCurve();
    
    // 存放从末端坐标系到各个坐标系（world、base、joint0～joint16）的变换，用于雅可比矩阵的求解;
    Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3> get_End2Joint_Trans(); 
    //存放从各个坐标系（base、joint0～joint16、end）到world的变换，用于输出机械臂姿态;
    Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3> get_Joint2World_Trans(); 
    //存放从各个坐标系（base、joint0～joint16、end）到world的变换，用于输出机械臂姿态;
    Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3> get_Joint2World_Trans_Virtual(); 

    // 雅可比矩阵法中，一次迭代允许的最大转角，单位：弧度
    void set_max_step_ang(double max_step_ang_new);      
    // 雅可比矩阵法中，一次迭代允许的最大位移，单为：mm
    void set_max_step_posi(double max_step_posi_new);      
    // 雅可比矩阵法中，迭代的步长系数，应大于1，越大收敛越稳定但是速度慢；
    void set_step_coef_min(double step_coef_min_new);      
    // 雅可比矩阵法中，判断是否接近奇异值的系数, 越大收敛越稳定但是速度慢；
    void set_singular_coef(double singular_coef_new);      
    // 雅可比矩阵法中的默认权重
    void set_weight_def(Eigen::Matrix<double,2*JOINT_NUM+1,1> weight_def_new);    
    // 雅可比矩阵法中，减小关节权重的角度阈值，在数学上为允许的关节角单次增加量最大值;
    void set_weight_joint_threshold(double weight_joint_threshold_new);  
    // 雅可比矩阵法中，迭代计算允许的最大次数
    void set_JacoIterateCNTMax(int JacoIterateCNTMax_new);    
    // 在更新自适应权重后，是否将权重代入雅克比矩阵进行重新求解
    void set_regression_solu_on(bool regression_solu_on_new);

    // 脊线法中，控制机械臂末端运动时，迭代停止的最大位置误差，单位：mm
    void set_Ftl_End_Toler(double Ftl_End_Toler_new);     
    // 脊线法中，控制机械臂末端运动时，计算基部运动的比例因子，越小越稳定。
    void set_FeedCoef(double FeedCoef_new);          
    // 脊线法中，控制机械臂基部运动时，迭代计算允许的最大次数
    void set_FTLBaseIterateCNTMax(int FTLBaseIterateCNTMax_new); 
    // 脊线法中，控制机械臂末端运动时，迭代计算允许的最大次数
    void set_FTLEndIterateCNTMax(int FTLEndIterateCNTMax_new);  
    // 脊线法中，通过FTL法控制的基部臂段数，应介于1～JOINT_NUM.
    void set_FTL_Part_Num(int FTL_Part_Num_new);         

    // 混合法中，虚拟关节的权重
    void set_VirtualJointWeight_def(double VirtualJointWeight_def_new); 
    // 混合法中，判断是否接近奇异值的系数, 越大收敛越稳定但是速度慢；
    void set_Hybird_singular_coef(double Hybird_singular_coef_new);   
    // 混合法中，通过FTL法控制的基部臂段数，应介于0～JOINT_NUM-3.
    void set_Hyb_Part_Num(int Hyb_Part_Num_new);              
#endif

private:
    
    /**************/
    /* 对象变量定义 */
    /**************/

    ros::NodeHandle *_nh;
    std::string BasePosi_Savefile;            // 存放基部位置的文件名,储存数据单位：mm
    std::string JointAngle_Savefile;           // 存放Last_JointAngle_Array的文件名,储存数据单位：度
    std::string PathCurve_CtrlPoints_Savefile; // 存放脊线曲线控制点序列的文件名,储存数据单位：mm

    SISLCurve *PathCurve; // 脊线曲线
    double PathCurve_CtrlPoints[PATHCURVE_CTRLPOINTS_MAXNUM*3]; // 脊线曲线控制点序列，每3各元素表示一个点，单位: mm
    int PathCurve_CtrlPoints_type[PATHCURVE_CTRLPOINTS_MAXNUM]; // 脊线曲线控制点类型，若值为1则表示坐标值
    int PathCurve_CtrlPoints_Num;                               // 脊线曲线控制点个数，最大为PATHCURVE_CTRLPOINTS_MAXNUM

    // 用于计算的过程变量，不用太管
    Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3> End2Joint_Trans;   // 存放从末端坐标系到各个坐标系（world、base、joint0～joint16）的变换，本意是用来算雅可比矩阵的，但后来发现在其它地方也用得到,所以弄成成员变量了;
    Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3> Joint2World_Trans_Virtual; // 存放从各个坐标系（base、joint0～joint16、end）到world的变换，主要用于逆运动学计算以及计算过程可视化;

    // 一些控制参数的定义
    // -------------------------
    double EndPose_Tolerance_ang;  // 雅可比矩阵法中，末端位姿偏差二范数容差
    double EndPose_Tolerance_posi; // 雅可比矩阵法中，末端位姿偏差二范数容差
    double max_step_ang;           // 雅可比矩阵法中，一次迭代允许的最大转角，单位：弧度
    double max_step_posi;          // 雅可比矩阵法中，一次迭代允许的最大位移，单为：mm
    double step_coef_min;          // 雅可比矩阵法中，迭代的步长系数，应大于1，越大收敛越稳定但是速度慢；
    double singular_coef;          // 雅可比矩阵法中，判断是否接近奇异值的系数, 越大收敛越稳定但是速度慢；
    Eigen::Matrix<double,2*JOINT_NUM+1,1> weight_def;    // 雅可比矩阵法中的默认权重
    double weight_joint_threshold; // 雅可比矩阵法中，减小关节权重的角度阈值，在数学上为允许的关节角单次增加量最大值;
    int    JacoIterateCNTMax;      // 雅可比矩阵法中，迭代计算允许的最大次数
    bool   regression_solu_on;     // 在更新自适应权重后，是否将权重代入雅克比矩阵进行重新求解

    double WrappedBall_Min;        // 脊线法中，包被单节机械臂工作空间的小球面半径，单位：mm
    double WrappedBall_Max;        // 脊线法中，包被单节机械臂工作空间的大球面半径，单位：mm
    double Inter_Toler;            // 脊线法中，计算单段机械臂工作曲线与曲线交点时允许的误差，单位：mm
    double Ftl_End_Toler;          // 脊线法中，控制机械臂末端运动时，迭代停止的最大位置误差，单位：mm
    double FeedCoef;               // 脊线法中，控制机械臂末端运动时，计算基部运动的比例因子，越小越稳定。
    int    FTLBaseIterateCNTMax;   // 脊线法中，控制机械臂基部运动时，迭代计算允许的最大次数
    int    FTLEndIterateCNTMax;    // 脊线法中，控制机械臂末端运动时，迭代计算允许的最大次数
    double VirtualPathCurvMax;     // 通过虚拟路径进行FTL时，路径允许的最大曲率, 单位：1/mm
    int    FTL_Part_Num;           // 脊线法中，通过FTL法控制的基部臂段数，应介于1～JOINT_NUM.

    double VirtualJointWeight_def; // 混合法中，虚拟关节的权重
    double Hybird_singular_coef;   // 混合法中，判断是否接近奇异值的系数, 越大收敛越稳定但是速度慢；
    double last_int_param;         // 存放FTLInvKine_BaseMove中获取的最后一个交点的样条曲线参数，用于求斜率
    int    Hyb_Part_Num;           // 混合法中，通过FTL法控制的基部臂段数，应介于0～JOINT_NUM-3.

    // 用于msg发布的变量定义
    // -------------------------

    // 发布者
    ros::Publisher Curve_Publisher;  // 用于PathCurve的发布
    ros::Publisher Arm_Publisher;  // 用于机械臂折线的发布
    ros::Publisher Points_Publisher; // 用于点的发布
    ros::Publisher Poses_Publisher;  // 用于机械臂姿态的发布

    // 发布对象的变量
    visualization_msgs::Marker Curve_Marker;  // 用于发布曲线/虚拟曲线
    visualization_msgs::Marker Arm_Marker;  // 用于发布机械臂折线
    visualization_msgs::Marker Points_Marker; // 用于发布单点或多点
    geometry_msgs::PoseArray Poses_Array;    // 用于发布机械臂各个坐标系的姿态

    /****************/
    /* 基础运动学函数 */
    /***************/

    /**
     * @brief 基于BasePosi、JointsAngle计算机械臂各个关节坐标系到末端坐标系的位姿, 求解结结果通过End2Joint_Trans输出
     *        这个输出主要用于雅可比矩阵的计算
     * @param BasePosi 基部相对全剧坐标系位置, 单位：mm
     * @param JointsAngle 存放关节角的2*8数组，单位：弧度
     * @param End2Joint_Trans 存放各个坐标系到world（world、base、joint0～joint16）的变换。位移向量的单位：mm
     * @return bool 
     */
    bool Position_Solution_Joint2End(double BasePosi, Arrayf_2_Joint& JointsAngle, Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3>& End2Joint_Trans);

    /**
     * @brief 进行雅可比矩阵的计算。计算关节start_joint～end_joint运动对于末端位姿的偏导。
     * 
     * @param JointsAngle 关节角
     * @param JacobianMat 输出雅可比矩阵，其行数为6，列数为[end_joint-start_joint+1]
     * @param start_coor 开始的关节，0<=start_joint<=end_joint，其中joint0对应的是机械臂基座的位移
     * @param end_coor 终止的关节,start_joint<=end_joint<=JOINT_NUM*2
     * @return bool 
     */
    bool GetJacobian(double BasePosi, Arrayf_2_Joint JointsAngle, Eigen::MatrixXd& JacobianMat, int start_joint, int end_joint);

    /*************/
    /***辅助函数***/
    /*************/

    /**
     * @brief 根据雅克比矩阵微分法封装的方程组求解函数，用于求解Ax=y中的x
              阻尼系数的设置会影响其中解方程的方法
     * @return bool 
    */
    bool Pseudo_inverse_solution(Eigen::MatrixXd& A, Eigen::VectorXd& x, Eigen::Matrix<double, 6, 1>& y);

    /**
     * @brief 根据当前机械臂姿态重新生成脊线，并将新的控制点更新到指定文件。控制点采样策略如下：
     *        joint0、1、3、5...15、end中心位置，以及basemove为0时joint0的位置。
              在这几个点的基础上，前后各延伸450mm，间隔控制在150mm附近。
     * @param BasePosi 基部相对全剧坐标系位置, 单位：mm
     * @param JointsAngle 存放关节角的2*8数组，单位：弧度
     * @param write_points 写控制点存储g2文件的指针
     * @return bool 
     */
    bool ReGeneTheCurve(double BasePosi, Arrayf_2_Joint JointsAngle);

    /**
     * @brief 求球面与曲线的交点，并返回曲线最前面一个点的位置；
     *        当没有交点或有两个以上交点时会报错；
     * @param sphere_centre 球心在world坐标系下的坐标，单位：mm
     * @param radius 半径，单位：mm
     * @param PathCurve 曲线对象
     * @param int_param 所求点的参数值
     * @param int_point 交点位置，在world坐标系下的坐标，单位：mm
     * @return bool
     */
    bool cal_int_point(double *sphere_centre, double radius, SISLCurve *PathCurve, double &int_param, Eigen::Vector4d &int_point);

public:
    /**
     * @brief 经过简单封装的s1356函数，用于生成3次3阶样条曲线
     * 
     * @param CtrlPoints 曲线经过的控制点，三个元素构成一个点
     * @param CtrlPoints_Num 控制点的个数，需要与CtrlPoints中点的个数保持一致
     * @param CtrlPoints_type 控制点的类型，1表示坐标，一般来说都是1
     * @param Curve 生成的曲线
     * @return bool 
     */
    bool my_s1356(double *CtrlPoints,int CtrlPoints_Num,int* CtrlPoints_type, SISLCurve* &Curve);

};
#endif