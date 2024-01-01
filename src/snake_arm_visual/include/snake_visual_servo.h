/**
 * @file snake_visual_servo.h
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 视觉伺服模块
 * @version 0.1
 * @date 2023-04-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SNAKE_VISUAL_SERVO
#define SNAKE_VISUAL_SERVO

// # define TEST

#include <ctime>
#include <queue>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
// ROS功能相关的头文件
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <realsense2_camera/DeviceInfo.h>

// PCL点云获取、处理与聚类相关的头文件
#include <pcl/io/pcd_io.h>                           // 用于pcd文件输入与输出
#include <pcl/point_types.h>                         // PCL中对各种点类型的定义
#include <pcl/filters/voxel_grid.h>                  // 用于体素滤波
#include <pcl/registration/icp_nl.h>                 // 用于ICP_NL配准
#include <pcl/filters/passthrough.h>                 // 用于点云范围切割
#include <pcl/filters/statistical_outlier_removal.h> // 游离点滤除
#include <pcl_conversions/pcl_conversions.h>         // PCL到ROS的接口

#include "common_function.h"

/****************/
/* 自定义数据类型 */
/****************/


/***********/
/* 类的定义 */
/***********/

class snake_visual_servo
{

public:

    /****************/
    /* 构造、析构函数 */
    /****************/

    /**
     * @brief Construct a new snake_visual_servo object
     * 读取param；
     * 等待深度相机初始化；
     * 通过文件或相机获取初始点云
     * 调用Visual_Servo_Loop持续进行伺服控制
     */
    snake_visual_servo(ros::NodeHandle *nh);

    /**
     * @brief Destroy the snake visual servo object
     * 
     */
    ~snake_visual_servo();

    /***************/
    /*   对象例化   */
    /***************/

    bool STATE;
    Eigen::Matrix4f EndposeError;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 使用这个宏的理由：http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html

    /***********/
    /* 功能函数 */
    /***********/


    /**
     * @brief 视觉伺服: 在指定机械臂的期望末端位姿后,根据相机获得的点云图像进行末端位姿解算,并将偏差值作为控制量发布给机械臂运动控制模块
     * 
     * @return bool 
     */
    bool Visual_Servo_Loop();

private:

    /**************/
    /* 对象变量定义 */
    /**************/

    // 话题与服务相关变量
    ros::NodeHandle *_nh;
    ros::CallbackQueue pointcloud_queue;   // 获取点云的服务队列
    ros::Subscriber pointcloud_sub;        // 获取点云的订阅者
    tf::TransformListener pose_listener;   // 获取tf变换的订阅者

    // 点云捕获的配置参数
    std::string data_save_path;             // 存放点云数据及相应位姿数据的路径
    std::string EndposeError_Savefile;      // 存放运动偏差的文件
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr parents_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr capture_cloud;

    Eigen::Isometry3f end_trans_camera;

    // 点云预处理过程中的各种配置参数
    int dist_min;                               // 获取点云的距离下限，单位：mm
    int dist_max;                               // 获取点云的距离上限，单位：mm
    double parents_resloution;                  // 配准中基准点云的分辨率，单位：mm
    int capture_sample    ;                  // 捕获点云时随机降采样的倍数，整数
    double pre_icp_resloution;                  // ICP之前再次体素滤波的分辨率，单位：mm
    double statistical_filter_StddevMulThresh;  // 通过相机捕获点云时，过滤游离点的阈值

    double icp_Epsilon;                         // ICP拼接算法收敛条件，越小精度越大，收敛也越慢
    double icp_MaxCorrespondenceDistance;       // ICP拼接算法配准时考虑的最大偏差距离,根据自己数据采集的情况确定
    int    icp_MaximumIterations;               // ICP拼接算法允许的最大迭代次数
    double ICPTf_ang_Max;                       // 允许的ICP计算得到的旋转矩阵范数最大值，如果超过该值则认为此次配准异常，不采用其数据
    double ICPTf_posi_Max;                      // 允许的ICP计算得到的平移向量范数最大值，如果超过该值则认为此次配准异常，不采用其数据，单位:m

    /*************/
    /***回调函数***/
    /*************/

    /**
     * @brief 获取一帧点云的回调函数，获取的点云保存在类成员capture_cloud中
     */
    void PointsCloudGet_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    /*************/
    /***辅助函数***/
    /*************/

    /** 
     * @brief 发布点云为sensor_msgs::PointCloud2类型的topic。模板函数必须定义在头文件中
     * @param source 类型为 pcl::PointCloud<PointT>::Ptr 或 普通指针
     * @param topic_name 发布点云的话题名
     * @param frame_id 为点云发布的坐标系
     */
    template <typename PointT>
    void PointCloud_Pub(PointT source, std::string topic_name, std::string frame_id = "world"){
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*source, output);
        output.header.frame_id = frame_id;
        ros::Publisher pub_Merge  = _nh->advertise<sensor_msgs::PointCloud2>(topic_name, 1);
        ros::Rate wait(5);
        for(int j=0; j<5;j++){
            pub_Merge.publish(output);
            wait.sleep();
        }
    }

    /**
     * @brief 用于实现类成员函数回调的订阅者配置函数
     *        参考自 https://blog.csdn.net/luoyang7891/article/details/110799622
     * @param topic 订阅的话题名
     * @param queue_size 订阅队列的长度
     * @param fp 回调函数名
     * @param obj 回调函数所属的类的对象，一般都是this吧
     * @param queue 自定义的接收队列，用于实现该队列的单独spin
     */
    template <class M, class T>
    ros::SubscribeOptions getSubscribeOptions(
        const std::string &topic, uint32_t queue_size,
        void (T::*fp)(const boost::shared_ptr<M const> &),
        T *obj,
        ros::CallbackQueueInterface *queue,
        const ros::TransportHints &transport_hints = ros::TransportHints())
    {
        ros::SubscribeOptions ops;
        ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
        ops.callback_queue = queue;
        ops.transport_hints = transport_hints;
        return ops;
    }

};

#endif