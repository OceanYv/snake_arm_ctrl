/**
 * @file snake_arm_visual.h
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 机械臂视觉模块
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef SNAKE_ARM_VISUAL
#define SNAKE_ARM_VISUAL

// # define TEST
# define PLOT_CURVATURE

#include <ctime>
#include <queue>
#include <time.h>
#include <random>
#include <sys/stat.h>
#include <sys/types.h>
// ROS功能相关的头文件
#include <std_srvs/SetBool.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <realsense2_camera/DeviceInfo.h>

// PCL点云获取、处理与聚类相关的头文件
#include <pcl/io/pcd_io.h>                           // 用于pcd文件输入与输出
#include <pcl/point_types.h>                         // PCL中对各种点类型的定义
#include <pcl/filters/voxel_grid.h>                  // 用于体素滤波
#include <pcl/features/normal_3d.h>                  // 用于估算法线
#include <pcl/features/principal_curvatures.h>       // 用于计算主曲率
#include <pcl/registration/icp_nl.h>                 // 用于ICP_NL配准
#include <pcl/filters/passthrough.h>                 // 用于点云范围切割
#include <pcl/filters/statistical_outlier_removal.h> // 游离点滤除
#include <pcl_conversions/pcl_conversions.h>         // PCL到ROS的接口
#include <conditional_euclidean_clustering_my.h>     // 用于聚类,自己根据需求改编了一点

// PCL基于点云的曲面重构与路径规划相关的头文件
#include <pcl/surface/on_nurbs/triangulation.h>         // 
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>   // 
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h> // 

#include "sisl.h"
#include "GoReadWrite.h"
#include "common_function.h"
#include <snake_arm_msg/EndPoseGo.h>

/****************/
/* 自定义数据类型 */
/****************/


/***********/
/* 类的定义 */
/***********/

class snake_arm_visual
{

public:

    /****************/
    /* 构造、析构函数 */
    /****************/

    /**
     * @brief 构造函数，主要完成以下几件事情：
     *   1. 通过参数服务器获取参数配置
     *   2. 初始化各种变量
     *   3. 等待相机设备唤醒（若使用）
     */
    snake_arm_visual(ros::NodeHandle *nh);

    /**
     * @brief 析构函数，主要完成以下几件事情：
     *   1. 关闭相机设备（若使用）
     */
    ~snake_arm_visual();

    /***********/
    /* 对象例化 */
    /***********/
    bool STATE;

    /***********/
    /* 控制函数 */
    /***********/

    /**
     * @brief 命令行入口函数，可在完成对象创建后调用
     *        实现 点云获取与曲面重建、路径规划、视觉伺服控制 函数的命令调用
     */
    bool snake_arm_visual_cmd();

    /**
     * @brief 点云获取，依次完成以下几件事情：
     *    1. 通过相机设备获取点云数据，并将对应位姿信息存储为文件
     *    2. 调用函数PointCloud_Merge，通过ICP算法进行已捕获点云的拼接（可以是上一步获取的，也可以是来自已有文件的），并将拼接结果存储为文件
     */
    bool ObjCapture();

   /**
    * @brief 选择指定聚类结果作为扫差对象，对其进行曲面重构与路径规划，依次完成以下几件事情：
    *     1. 对指定对象（ObjCapture获得的聚类结果点云）进行B样条曲面规划
    *     2. 基于探头横向与纵向的孔径参数，根据B样条曲面进行路径离散、点离散
    *     3. 根据点离散结果进行各个扫差点末端位姿的文件输出
    */
    bool BuildSurf_PathPlan();

    /**
     * @brief 对BuildSurf_PathPlan规划得到的位姿序列进行可达性检测
     * 
     * @return void 
     */
    void poses_array_reachable_check();

private:
    
    /**************/
    /* 对象变量定义 */
    /**************/

    // 话题与服务相关变量
    ros::NodeHandle *_nh;
    ros::CallbackQueue pointcloud_queue;    // 获取点云的服务队列
    ros::Subscriber pointcloud_sub;         // 获取点云的订阅者
    tf::TransformListener pose_listener;    // 获取tf变换的订阅着

    // 点云捕获的配置参数
    int data_save_cnt;                      // 捕获点云图像的张数
    bool from_file;                         // 从data_save_path中读取原始点云文件,而非使用相机进行采集
    std::string data_save_path;             // 存放点云数据及相应位姿数据的路径
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr capture_cloud;

    // 点云后处理过程中的各种配置参数
    double capture_resloution;                  // 通过相机捕获点云时,体素滤波的分辨率
    double merge_resloution;                    // 完成点云拼接后,对拼接结果体素滤波的分辨率
    double statistical_filter_StddevMulThresh;  // 通过相机捕获点云时，过滤游离点的阈值

    double icp_Epsilon;                     // ICP拼接算法收敛条件，越小精度越大，收敛也越慢
    double icp_MaxCorrespondenceDistance_1; // ICP拼接算法配准时考虑的最大偏差距离,根据自己数据采集的情况确定
    double icp_MaxCorrespondenceDistance_2; // ICP拼接算法配准时考虑的最大偏差距离,根据自己数据采集的情况确定
    int icp_MaximumIterations;              // ICP拼接算法允许的最大迭代次数

    double PassThrough_xp;  // 云范围限制参数,用于ICP拼接后、聚类之前的点云切割
    double PassThrough_xn;  // 云范围限制参数,用于ICP拼接后、聚类之前的点云切割
    double PassThrough_yp;  // 云范围限制参数,用于ICP拼接后、聚类之前的点云切割
    double PassThrough_yn;  // 云范围限制参数,用于ICP拼接后、聚类之前的点云切割
    double PassThrough_zp;  // 云范围限制参数,用于ICP拼接后、聚类之前的点云切割
    double PassThrough_zn;  // 云范围限制参数,用于ICP拼接后、聚类之前的点云切割

    double PrincipalCurvatures_RadiusSearch; // 评估主曲率时的搜索半径，越大效率越低
    double cluster_ClusterTolerance;         // 近邻搜索的搜索半径
    int cluster_MinClusterSize;              // 最小聚类尺寸（点的个数）
    int cluster_MaxClusterSize;              // 最大聚类尺寸（点的个数）
    int Normal_search_num;                   // 计算用于聚类的切线之前，需要先计算法线。这个参数指定了计算法线时周围搜索点的个数

    // 曲面重建过程中的各种配置参数
    double remove_rate_x;  // 曲面拟合前边缘去毛刺时，包围盒x轴方向上的去除率
    double remove_rate_y;  // 曲面拟合前边缘去毛刺时，包围盒y轴方向上的去除率

    int surface_order ; // B样条曲面的阶数
    int surface_fit_refinement; // 曲面拟合中，求精迭代的次数，其中每插入一个迭代控制点，b样条曲面的每个参数方向上的控制点大约翻倍
    int surface_fit_iterations; // 曲面拟合中，求精迭代完成后，继续优化的迭代数量

    double surface_fit_interior_smoothness; // 内部表面的光滑度
    double surface_fit_interior_weight    ; // 用于表面内部优化的权重
    double surface_fit_boundary_smoothness; // 表面边界的平滑度
    double surface_fit_boundary_weight    ; // 表面边界优化的权重

    // 路径规划中的各种配置参数
    double scan_path_dist  ;  // 往复路径的间隔，与相控阵探头的长度有关
    double scan_step_length;  // 路径上离散点的间隔。对于线阵，与相控阵探头垂直阵列排布方向的波束角有关；对于面阵，相控阵探头的宽度有关；
    int ScanPnt_Flt_NeighborNum;        // 过滤不在面上的扫查点时，求取近邻距离平均值时近邻点的个数
    double ScanPnt_Flt_StddevMulThresh; // 过滤不在面上的扫查点时，求取近邻距离平均值时的距离阈值

    /*************/
    /***回调函数***/
    /*************/

    /**
     * @brief 通过相机获取图像阶段，键盘输入捕获命令时预处理并保存点云的回调函数
     *        由订阅者pointcloud_sub调用
     *    流程为： 获取原始点云
     *         -> 调用StatisticalOutlierRemoval滤除离群点
     *         -> 调用VoxelGrid进行降采样
     *         -> 调用removeNaNFromPointCloud去除无效点
     *         -> 将预处理后的点云输出到文件中
     */
    void PointsCloudSave_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    /**
     * @brief 获取一帧点云的回调函数，获取的点云保存在类成员
     */
    void PointsCloudGet_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    /*************/
    /*点云操作函数**/
    /*************/

    /**
     * @brief 给定存放文件的路径和点云文件个数,进行点云的数据合并。合并结果放回路径下的capture_merge.pcd文件中
     *     流程为：读取各个点云文件和位姿文件
     *         -> 根据已有位姿信息进行点云的坐标变换
     *         -> 通过ICP进行精配准 （初期尝试了在ICP之前进行SAC粗配准，但效果不好且效率低。果断放弃！果断放弃！果断放弃！）
     *         -> 可视化与保存
     * 注意：路径中文件的格式必须为: 位姿文件命名为capture_pose.txt, 点云文件命名为capture_x.pcd (x从1开始连续计数到file_num)
    */
    bool PointCloud_Merge(std::string path);

    /**
     * @brief 聚类函数，通过欧氏距离、法向量和切向量对输入点云进行聚类
     *        聚类结果保存在指定路径下的cluster_X.pcd文件和cluster_all.pcd文件中
     *     流程为：在xyz方向上进行裁剪
     *        -> 计算法向量与、主曲率与切线特征
     *        -> 通过ConditionalEuclideanClustering方法进行聚类
     *        -> 将聚类结果分离、可视化并保存
     */
    bool PointCloud_Cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source);

    /**
     * @brief 对点云进行曲面拟合，拟合结果保存在指定路径下的SurfaceFit_X.g2文件中
     *        参考：https://pcl.readthedocs.io/projects/tutorials/en/master/bspline_fitting.html#bspline-fitting
     *     流程为：通过主成分分析和包围盒进行曲面初始化
     *        -> 通过曲面精细化进行拟合
     *        -> 通过曲面调整进行拟合优化
     *        -> 将点云边线提取为样条曲线，作为曲面的边界
     *        -> 将拟合结果可视化与存储
     * @param source_cloud 要拟合的点云
     * @param num 点云的编号，用于输出文件命名
     */    
    bool Cloud_Surface_fit(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, int num);

    /**
     * @brief 对指定曲面进行表面路径规划
     *     流程为： 
     *        ->  
     * @param num 点云的编号，用于输出文件命名
     */
    bool Path_plan(int num);

    /** 
     * @brief 通过PCA、OBB等算法，获取输入点云的重心、特征向量、特征值、包围盒大小等参数
     *        参考自 pcl/surface/src/on_nurbs/fitting_surface_pdm.cpp initNurbsPCABoundingBox函数
     * @param source_cloud 类型为 pcl::PointCloud<PointT>::Ptr 或 普通指针
     * @param mean         点云重心
     * @param eigenvectors 特征向量（有三个列向量）
     * @param eigenvalues  特征值（有三个）
     * @param obb_range     包围盒范围
     * @return Eigen::Vector3f 包围盒的长宽高
     */
    void PCAAndOBB_Get(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                       Eigen::Vector3d&          mean                  ,
                       Eigen::Matrix3d&          eigenvectors          ,
                       Eigen::Vector3d&          eigenvalues           ,
                       Eigen::Matrix<float,3,2>& obb_range             );

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
     * @brief 发布ON_NurbsSurface为sensor_msgs::PointCloud2类型的topic。模板函数必须定义在头文件中
     *        参考自  pcl/surface/on_nurbs/triangulation.cpp 中的 convertSurface2Vertices 函数
     * @param surface_in 输入曲面
     * @param obb_size 曲面包围盒的大小，用于确定采样点的个数,间隔0.003
     * @param topic_name 发布点云的话题名
     * @param frame_id 为点云发布的坐标系，默认 world
     */
    void ON_NurbsSurface_pub(ON_NurbsSurface &surface_in, Eigen::Vector3f obb_size, std::string topic_name, std::string frame_id = "world");

    /** 
     * @brief 通过marker发布包围盒和主成分分析获得的坐标系，以进行可视化
     * @param mean         点云重心
     * @param eigenvectors 特征向量（有三个列向量）
     * @param eigenvalues  特征值（有三个）
     * @param obb_range     包围盒范围
     * @param topic_name 发布点云的话题名
     * @param frame_id 为点云发布的坐标系，默认 world
     * @return Eigen::Vector3f 包围盒的长宽高
     */
    void PCAAndOBB_Pub( Eigen::Vector3d           mean              ,
                        Eigen::Matrix3d           eigenvectors      ,
                        Eigen::Vector3d           eigenvalues       ,
                        Eigen::Matrix<float,3,2>& obb_range         ,
                        std::string               topic_name        , 
                        std::string               frame_id = "world");

    /**
     * @brief 基于Curve对象，调用s1227()生成一系列的点，并通过topic发布，在rviz中订阅观察；
     * 
     * @param curve Curve对象
     * @param topic_name 话题名
     * @param frame_name 曲线所在坐标系的名称,可能使用的是"world" "arm_end_comp"
     * @param color 线条颜色，rgbwk分别为红绿蓝白黑
     * @param hold_on 是否保留之前发布的曲线
     * @param plot_num 曲线上采样点的个数
     * @return bool 成功与否
     */
    bool Curve_Pub(SISLCurve* curve, std::string topic_name, std::string frame_name, char color = 'r', bool hold_on=false, std::size_t plot_num=100);

    /** 
     * @brief 将ON_NurbsSurface类型的曲面保存为sisl的g2格式
     *        可通过SISL中提供的可视化工具进行查看（详见doc/SISL B样条库/SISL库.g2文件格式说明.txt）
     * @param m_nurbs      曲面对象
     * @param eigenvectors 文件名（及路径）
     * @return bool 成功与否
     */
    bool ON_NurbsSurface2g2file(ON_NurbsSurface& m_nurbs, std::string file_name);

    /** 
     * @brief 将ON_NurbsCurve类型的曲面保存为sisl的g2格式
     *        可通过SISL中提供的可视化工具进行查看（详见doc/SISL B样条库/SISL库.g2文件格式说明.txt）
     * @param m_nurbs      曲线对象
     * @param eigenvectors 文件名（及路径）
     * @return bool 成功与否
     */
    bool ON_NurbsCurve2g2file(ON_NurbsCurve& m_nurbs, std::string file_name);

    /**
     * @brief SISL中用于获取指定点坐标的函数接口太繁琐了，自己封装了一下
     * @param surface   曲面对象
     * @param pointpar  长度为二的数组，存储需要索引的点的u、v
     * @param posi      长度为三的数组，返回的坐标
     */
    bool my_s1424(SISLSurf* surface, double* pointpar, double* posi);

    /**
     * @brief SISL中用于获取指定点坐标的函数接口太繁琐了，自己封装了一下
     * @param curve     曲线对象
     * @param parvalue  存储需要索引的点的参数
     * @param point     长度为三的数组，返回的坐标
     */
    bool my_s1227(SISLCurve* curve, double parvalue, double* point);

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