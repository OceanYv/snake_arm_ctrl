#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <snake_arm_msg/EndPoseGo.h>

#include <pcl/io/pcd_io.h>                           // 用于pcd文件输入与输出
#include <pcl/point_types.h>                         // PCL中对各种点类型的定义
#include <pcl_conversions/pcl_conversions.h>         // PCL到ROS的接口

#include "common_define.h"
#include "common_function.h"

/**************/
/* 辅助函数声明 */
/**************/

/** 
 * @brief 发布点云为sensor_msgs::PointCloud2类型的topic。模板函数必须定义在头文件中
 * @param source 类型为 pcl::PointCloud<PointT>::Ptr 或 普通指针
 * @param topic_name 发布点云的话题名
 * @param frame_id 为点云发布的坐标系
 */
template <typename PointT>
void PointCloud_Pub(PointT source, std::string topic_name, ros::NodeHandle &nh){
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*source, output);
    output.header.frame_id = "world";
    ros::Publisher pub_Merge  = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
    ros::Rate wait(5);
    for(int j=0; j<5;j++){
        pub_Merge.publish(output);
        wait.sleep();
    }
}

/**************/
/* 声明全局变量 */
/**************/


/*************/
/* 节点主函数 */
/************/

int main(int argc, char *argv[]){
    ros::init(argc, argv, "poses_array_pub_node");
    ros::NodeHandle nh;


    // 创建订阅者、发布者等
    ros::ServiceClient EndPoseReq = nh.serviceClient<snake_arm_msg::EndPoseGo>("EndposeExp");

    // 参数初始化
    std::string data_save_path = nh.param<std::string>("/snake_arm_visual/build_surface/data_save_path", "/home/ocici/ROS_CODE/3_snake_arm/src/snake_arm_visual/data/default");
    snake_arm_msg::EndPoseGo EndPoseReqMsg;

    int num = 0;
    while(num!=-1){


        // 读取文件，并获取其中的位姿个数,以及其可达性
        std::ifstream fposes(data_save_path+"/ScanPoses_2.txt");
        if (!fposes) {
            std::cerr << "[poses_array_pub_node] cannot find pose file" << std::endl;
            return false;
        }

        int poses_num = 0;
        std::string temp;
        while(std::getline(fposes, temp,'\n')) poses_num++;
        fposes.close(); fposes.open(data_save_path+"/ScanPoses_2.txt");
    
        std::ifstream fReachable(data_save_path+"/ScanPoses_Reachable_2.txt");
        if (!fReachable) {
            std::cerr << "[poses_array_pub_node] cannot find Reachable file" << std::endl;
            return false;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr reachable_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 
        printf("range 1~%d, -1 to exit: ", poses_num);
        std::cin >> num;

        // 开始循环发起请求
        int succ_cnt=0;
        for(int i = 0; i < poses_num ; i++ ){
            

            ROS_INFO("[poses_array_pub_node] EndPoseReqMsg Request %d/%d", i+1, poses_num);

            // 获取位姿及其可达性
            float data[7] = {0};
            int reachable = 0;
            for (int i = 0; i < 7; i++) fposes >> data[i];
            fReachable >> reachable;
            if(!reachable) continue;


            Eigen::Quaternionf q(data[3], data[4], data[5], data[6]);
            Eigen::Isometry3f Trans(q);
            Trans.pretranslate(Eigen::Vector3f(data[0], data[1], data[2]));

            EndPoseReqMsg.request.Poses.position.x = data[0];
            EndPoseReqMsg.request.Poses.position.y = data[1];
            EndPoseReqMsg.request.Poses.position.z = data[2];
            EndPoseReqMsg.request.Poses.orientation.w = data[3];
            EndPoseReqMsg.request.Poses.orientation.x = data[4];
            EndPoseReqMsg.request.Poses.orientation.y = data[5];
            EndPoseReqMsg.request.Poses.orientation.z = data[6];    

            pcl::PointXYZRGB point_rgb;
            if((i+1) == num){
                if(EndPoseReq.call(EndPoseReqMsg)){
                    ROS_DEBUG("[poses_array_pub_node] EndPoseReqMsg Request: succeed!");
                    succ_cnt++;
                    point_rgb.r = 0; point_rgb.g = 255;
                }
            }
            else{
                point_rgb.r = 0; point_rgb.g = 255;
            }

            
            // 将可达性压入点云以可视化
            point_rgb.x = data[0]; point_rgb.y = data[1]; point_rgb.z = data[2]; 
            point_rgb.b = 0;
            reachable_cloud->points.push_back(point_rgb);

            if(((i+1) == num)) {
                 PointCloud_Pub(reachable_cloud, "reachable_cloud", nh);
                 break;
            }
        }
        ROS_INFO("[poses_array_pub_node] Reachabale Points: %d/%d", succ_cnt, poses_num);
    }

    return 1;
}