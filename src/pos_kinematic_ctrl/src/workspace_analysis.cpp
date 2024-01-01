/**
 * @file workspace_analysis.cpp
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2023-06-11
 * 
 * @copyright Copyright (c) 2023 OceanYv
 * 
 */

/************/
/* 头文件声明 */
/************/

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>                  // 用于体素滤波
#include <pcl_conversions/pcl_conversions.h>         // PCL到ROS的接口

#include <sensor_msgs/PointCloud2.h>

#include "common_define.h"
#include "common_function.h"


/**********/
/* 函数声明 */
/**********/

/**
 * @brief 基于BasePosi、JointsAngle计算机械臂各个关节坐标系相对全局坐标系的位姿, 求解结结果通过Joint2World_Trans输出
 * 
 * @param BasePosi 基部相对全剧坐标系位置, 单位：mm
 * @param JointsAngle 存放关节角的2*8数组，单位：弧度
 * @param Joint2World_Trans 存放world到各个坐标系end的变换。位移向量的单位：mm
 * @return bool 
 */
bool Position_Solution(double BasePosi, Arrayf_2_Joint& JointsAngle, Eigen::Isometry3d& Joint2World_Trans);


/** 
 * @brief 发布点云为sensor_msgs::PointCloud2类型的topic。模板函数必须定义在头文件中
 * @param source 类型为 pcl::PointCloud<PointT>::Ptr 或 普通指针
 * @param topic_name 发布点云的话题名
 * @param frame_id 为点云发布的坐标系
 */
template <typename PointT>
void PointCloud_Pub(ros::NodeHandle& nh, PointT source, std::string topic_name, std::string frame_id = "world"){
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*source, output);
    output.header.frame_id = frame_id;
    ros::Publisher pub_Merge  = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
    ros::Rate wait(5);
    for(int j=0; j<5;j++){
        pub_Merge.publish(output);
        wait.sleep();
    }
}

/*************/
/* 节点主函数 */
/************/

int main(int argc, char *argv[]){

    ros::init(argc, argv, "workspace_analysis");
    ros::NodeHandle nh;

    // 声明变量
    int point_num = 10000000;
    std::string point_cloud_name = "/home/ocici/ROS_CODE/3_snake_arm/src/pos_kinematic_ctrl/data/pointcloud_2.pcd";
    double MaxAngle = M_PI / 180 * 30;

    double BasePosi = 0;
    Arrayf_2_Joint JointsAngle;
    Eigen::Isometry3d Joint2World_Trans;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Workspace_PointCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 开始蒙特卡洛
    pcl::PointXYZ workspace_point;
    for(int i=0;i<point_num;i++){
        // 生成随机数
        JointsAngle = Arrayf_2_Joint::Random() * MaxAngle;

        // 调用正运动学
        Position_Solution(BasePosi, JointsAngle, Joint2World_Trans);

        // 压入点云
        workspace_point.x = Joint2World_Trans.translation().x()/1000.0;
        workspace_point.y = Joint2World_Trans.translation().y()/1000.0;
        workspace_point.z = Joint2World_Trans.translation().z()/1000.0;
        Workspace_PointCloud->push_back (workspace_point);

        // 输出提示信息
        if(i % 10000 == 9999){
            ROS_INFO("i = %d", i+1);
        }

    }

    // 降采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(0.005, 0.005,0.005);
    voxel_filter.setInputCloud(Workspace_PointCloud);
    voxel_filter.filter(*Workspace_PointCloud);    
    ROS_INFO("Number of point is %d", (int)(Workspace_PointCloud->size()));

    // 保存点云
    pcl::io::savePCDFileBinary(point_cloud_name.c_str(), *Workspace_PointCloud);

    ROS_INFO("Finish!!!");
    return 1;
}


/***********/
/* 函数实现 */
/***********/

bool Position_Solution(double BasePosi, Arrayf_2_Joint& JointsAngle, Eigen::Isometry3d& Joint2World_Trans)
{
    Eigen::Isometry3d pos = Eigen::Isometry3d::Identity();

    for (int i = 0; i < 2*JOINT_NUM+3; i++){
        if (i == 0){ // world to base
            pos.translate(Eigen::Vector3d(BasePosi, 0, 0));
        }
        else if (i == 1){ // base to joint0
            pos.translate(Eigen::Vector3d(ARM_BASE_X_OFFSET, 0, ARM_BASE_HEIGHT));
        }
        else if (i == 2){ // joint0 to joint1
            pos.translate(Eigen::Vector3d(FIX_POLE_PART_LENGTH, 0, 0));
            pos.rotate(Eigen::AngleAxisd(JointsAngle(0), Eigen::Vector3d(0, 0, 1)));
        }
        else if(i == 2*JOINT_NUM+2){ // joint16 to end
            pos.rotate(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(1, 0, 0)));
            pos.translate(Eigen::Vector3d(END_POSITION_X, END_POSITION_Y, END_POSITION_Z)); // 末端位置
        }
        else{ // joint_i-2 to joint_i-1
            pos.rotate(Eigen::AngleAxisd((i % 2) ? M_PI / 2 : -M_PI / 2, Eigen::Vector3d(1, 0, 0)));
            pos.translate(Eigen::Vector3d((i % 2) ? ROT_SHAFT_PART_LENGTH: FIX_POLE_PART_LENGTH, 0, 0));
            pos.rotate(Eigen::AngleAxisd(JointsAngle(i - 2), Eigen::Vector3d(0, 0, 1)));
        }
    }

    Joint2World_Trans = pos;
    return true;
}

