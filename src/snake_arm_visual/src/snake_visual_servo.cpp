#include <snake_visual_servo.h>

snake_visual_servo::snake_visual_servo(ros::NodeHandle *nh){

    /**************/
    /* 获取配置参数 */
    /**************/
    STATE = true;
    _nh = nh;

    printf("\n");
    ROS_INFO(">>>snake_visual_servo config information");
    ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

    // 获取参数
    // -------------------------------------------------------------
    data_save_path        = _nh->param<std::string>("/snake_visual_servo/data_save_path", "/home/ocici/ROS_CODE/3_snake_arm/src/snake_arm_visual/data/default");
    EndposeError_Savefile = _nh->param<std::string>("/snake_visual_servo/EndposeError_Savefile", "/home/ocici/ROS_CODE/3_snake_arm/src/snake_arm_visual/config/EndposeError.txt");
    
    dist_min           = _nh->param<int>("/snake_visual_servo/dist_min", 300);
    dist_max           = _nh->param<int>("/snake_visual_servo/dist_max", 900);
    parents_resloution = _nh->param<double>("/snake_visual_servo/parents_resloution", 0.003);
    capture_sample     = _nh->param<int>("/snake_visual_servo/capture_sample", 15) * 2;
    pre_icp_resloution = _nh->param<double>("/snake_visual_servo/pre_icp_resloution", 0.0035);
    statistical_filter_StddevMulThresh = _nh->param<double>("/snake_visual_servo/statistical_filter_StddevMulThresh", 0.01);

    icp_Epsilon = _nh->param<double>("/snake_visual_servo/icp_Epsilon", 1e-8);
    icp_MaxCorrespondenceDistance = _nh->param<double>("/snake_visual_servo/icp_MaxCorrespondenceDistance", 0.02);
    icp_MaximumIterations = _nh->param<int>("/snake_visual_servo/icp_MaximumIterations", 100);

    ICPTf_ang_Max  = _nh->param<double>("/snake_visual_servo/ICPTf_ang_Max", 10.0);
    ICPTf_posi_Max = _nh->param<double>("/snake_visual_servo/ICPTf_posi_Max", 0.05);

    // 判断参数是否合法

    // 输出参数到终端
    printf("\n");
    ROS_INFO_STREAM("data_save_path                     : " << data_save_path                );
    ROS_INFO_STREAM("EndposeError_Savefile              : " << EndposeError_Savefile         );

    ROS_INFO_STREAM("dist_min                           : " << dist_min                      );
    ROS_INFO_STREAM("dist_max                           : " << dist_max                      );

    ROS_INFO_STREAM("parents_resloution                 : " << parents_resloution            );
    ROS_INFO_STREAM("capture_sample                     : " << capture_sample / 2            );
    ROS_INFO_STREAM("pre_icp_resloution                 : " << pre_icp_resloution            );
    ROS_INFO_STREAM("statistical_filter_StddevMulThresh : " << statistical_filter_StddevMulThresh);

    ROS_INFO_STREAM("icp_Epsilon                        : " << icp_Epsilon                   );
    ROS_INFO_STREAM("icp_MaxCorrespondenceDistance      : " << icp_MaxCorrespondenceDistance );
    ROS_INFO_STREAM("icp_MaximumIterations              : " << icp_MaximumIterations         );
    ROS_INFO_STREAM("ICPTf_ang_Max                      : " << ICPTf_ang_Max                 );
    ROS_INFO_STREAM("ICPTf_posi_Max                     : " << ICPTf_posi_Max                );
    printf("\n");

    ICPTf_ang_Max = 2 * sqrt(1 - cos(ICPTf_ang_Max / 180.0 * M_PI));

    /************/
    /* 初始化对象 */
    /************/

    // 创建相机点云的订阅者，订阅结果存放在类成员capture_cloud中
    ros::SubscribeOptions ops = getSubscribeOptions("/camera/depth/color/points",1,&snake_visual_servo::PointsCloudGet_callback,this,&pointcloud_queue);
    pointcloud_sub = _nh->subscribe(ops);

    // 读取之前存储的偏差，并发布
    if(file_to_array(EndposeError_Savefile, EndposeError) != true){
        ROS_ERROR_STREAM("[snake_visual_servo] Fail to open EndposeError_Savefile, EndposeError will be set as Identity.");
        if(!Confirm_Interaction()){ STATE = false; return; }
        EndposeError.setIdentity();
    }

    Eigen::Quaternionf q(EndposeError.block<3,3>(0,0));
    ros::Publisher EndposeError_pub = _nh->advertise<geometry_msgs::Transform>("EndposeError", 1);
    geometry_msgs::Transform EndposeErrorMsg;
    EndposeErrorMsg.translation.x = EndposeError(0,3);
    EndposeErrorMsg.translation.y = EndposeError(1,3);
    EndposeErrorMsg.translation.z = EndposeError(2,3);
    EndposeErrorMsg.rotation.w    = q.w();
    EndposeErrorMsg.rotation.x    = q.x();
    EndposeErrorMsg.rotation.y    = q.y();
    EndposeErrorMsg.rotation.z    = q.z();
    for(int i=0;i<5;i++) {
        EndposeError_pub.publish(EndposeErrorMsg);
        usleep(1000*100);
    }
    sleep(1);

    // 读取已有的点云文件
    if(access(data_save_path.c_str(), 0) == -1) // unlink(data_save_path.c_str());
        mkdir(data_save_path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);

    parents_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    capture_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    if(pcl::io::loadPCDFile(data_save_path + "/capture_merge.pcd", *parents_cloud) == -1){ // 如果读取文件失败，则用此刻相机获取的点云作为初始值
        ROS_WARN_STREAM("[snake_visual_servo] Fail to read file " << data_save_path << "/capture_merge.pcd");
        ROS_WARN_STREAM("[snake_visual_servo] Will capture a frame from camera to init the parents_cloud.");

        // 获取此时的机械臂位姿
        ros::Time now=ros::Time(0);
        tf::StampedTransform transform;
        pose_listener.waitForTransform("world","camera_aligned_depth_to_color_frame",now,ros::Duration(1.0));
        pose_listener.lookupTransform("/world", "/camera_aligned_depth_to_color_frame", now, transform);

        // 获取一帧点云
        if(pointcloud_queue.callOne(ros::WallDuration(1)) != 0){
            ROS_ERROR("[snake_visual_servo] Failed to call pointcloud_queue.");
            STATE = false; return;            
        }
        else{
            ROS_INFO("[snake_visual_servo] Got a pointcoud to init the parent cloud.");

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            int points_num = 0;
            for(int i=0;i<capture_cloud->points.size();i++){
                if( capture_cloud->points[i].z < 0) continue; // 被压紧随动机构遮挡的部分

                double dist = sqrt( std::pow(capture_cloud->points[i].x,2) + std::pow(capture_cloud->points[i].y,2) + std::pow(capture_cloud->points[i].z,2) );
                if(dist < dist_max && dist > dist_min){
                    points_num++;
                    result_cloud->points.push_back(capture_cloud->points[i]);
                }
            }
            result_cloud->height = 1;
            result_cloud->width = points_num;
            result_cloud->is_dense = false;
            result_cloud->swap(*capture_cloud);

            pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
            voxel_filter.setLeafSize(parents_resloution, parents_resloution, parents_resloution);
            voxel_filter.setInputCloud(capture_cloud);
            voxel_filter.filter(*parents_cloud);

            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
            statistical_filter.setMeanK(20);                // 这个值对代码运行效率的影响并不大，大概是kdtree的特性
            statistical_filter.setStddevMulThresh(parents_resloution);
            statistical_filter.setInputCloud(parents_cloud);
            statistical_filter.filter(*parents_cloud);

            // 一个奇奇怪怪的相机坐标系定义带来的坐标变换
            Eigen::Isometry3d Pose_Corr_Trans = Eigen::Isometry3d::Identity();
            Pose_Corr_Trans.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 0, 1)));
            Pose_Corr_Trans.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1, 0, 0)));
            pcl::transformPointCloud(*parents_cloud, *parents_cloud, Pose_Corr_Trans.matrix());

            // 从相机坐标系变换到世界坐标系
            Eigen::Quaternionf q(transform.getRotation().getW(), transform.getRotation().getX(),
                                 transform.getRotation().getY(), transform.getRotation().getZ());
            Eigen::Isometry3f Trans(q);
            Trans.pretranslate(Eigen::Vector3f(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ()));
            pcl::transformPointCloud(*parents_cloud, *parents_cloud, Trans.matrix());
        }
    }
    PointCloud_Pub(parents_cloud, "Parents_PointCloud");

    // 获取end_trans_camera，作为更新偏差中的一个参数
    ros::Time now = ros::Time::now();
    tf::StampedTransform transform;
    pose_listener.waitForTransform("arm_end_comp", "camera_aligned_depth_to_color_frame" , now, ros::Duration(1.0));
    pose_listener.lookupTransform("/arm_end_comp", "/camera_aligned_depth_to_color_frame", now, transform);
    q = Eigen::Quaternionf(transform.getRotation().getW(), transform.getRotation().getX(),
                           transform.getRotation().getY(), transform.getRotation().getZ());
    end_trans_camera = Eigen::Isometry3f(q);
    end_trans_camera.pretranslate(Eigen::Vector3f(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ()));
    ROS_INFO_STREAM("end_trans_camera = \n" << end_trans_camera.matrix());
    
    /*********/
    /* debug */
    /*********/
    
    ROS_INFO("snake_visual_servo Construct successfully.\n");
    return;
}

snake_visual_servo::~snake_visual_servo(){
    return;
}

bool snake_visual_servo::Visual_Servo_Loop(){
    ROS_INFO("[Visual_Servo_Loop] begin ========= ");

    // 发布者
    ros::Publisher EndposeError_pub = _nh->advertise<geometry_msgs::Transform>("EndposeError", 1);
    geometry_msgs::Transform EndposeErrorMsg;

    // 一个奇奇怪怪的相机坐标系定义带来的坐标变换
    Eigen::Isometry3d Pose_Corr_Trans = Eigen::Isometry3d::Identity();
    Pose_Corr_Trans.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 0, 1)));
    Pose_Corr_Trans.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1, 0, 0)));

    // 创建ICP对象,并进行参数配置
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp_obj;
    icp_obj.setTransformationEpsilon(icp_Epsilon);
    icp_obj.setMaxCorrespondenceDistance(icp_MaxCorrespondenceDistance);
    icp_obj.setMaximumIterations(icp_MaximumIterations);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_icp_src(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_icp_tgt(new pcl::search::KdTree<pcl::PointXYZRGB>());
    icp_obj.setSearchMethodSource(tree_icp_src);
    icp_obj.setSearchMethodTarget(tree_icp_tgt);

    // 降采样器对象
    pcl::VoxelGrid<pcl::PointXYZRGB> parent_voxel_filter;
    parent_voxel_filter.setLeafSize(parents_resloution, parents_resloution, parents_resloution);
    parent_voxel_filter.setInputCloud(parents_cloud);

    // 离群点滤除器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
    statistical_filter.setMeanK(20);                // 这个值对代码运行效率的影响并不大，大概是kdtree的特性
    statistical_filter.setStddevMulThresh(0.004);

    // 循环计算迭代控制量
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr regis_result(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sample_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int cnt = 0;
    while(1){

        // 获取此时的机械臂位姿
        ros::Time now = ros::Time::now();
        tf::StampedTransform world_trans_camera;
        tf::StampedTransform end_transform;
        pose_listener.waitForTransform("world", "camera_aligned_depth_to_color_frame", now, ros::Duration(0.5));
        pose_listener.lookupTransform("/world", "/camera_aligned_depth_to_color_frame", now, world_trans_camera);
        // 获取此时的点云
        if(pointcloud_queue.callOne(ros::WallDuration(0.1)) != 0){
            ROS_WARN("[ObjCapture] Failed to call pointcloud_queue.");
            continue;
        }

        /************/
        /* 点云预处理 */
        /************/

        // 范围滤波兼随机采样
        // ros::Time begin = ros::Time::now();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        int points_num = 0;
        int sample_cnt = 0; // 简单的随机降采样
        int sample_step = rand() % capture_sample + 1;
        for(int i=0;i<capture_cloud->points.size();i++){
            if( capture_cloud->points[i].z < 0) continue; // 被压紧随动机构遮挡的部分

            double dist = sqrt( std::pow(capture_cloud->points[i].x,2) + std::pow(capture_cloud->points[i].y,2) + std::pow(capture_cloud->points[i].z,2) );
            if(dist < dist_max && dist > dist_min){
                sample_cnt++;
                if(sample_cnt == sample_step){
                    points_num++;
                    result_cloud->points.push_back(capture_cloud->points[i]);
                    sample_cnt = 0;
                    sample_step = rand() % capture_sample+1;
                }
            }
        }
        result_cloud->height = 1;
        result_cloud->width = points_num;
        result_cloud->is_dense = false;
        result_cloud->swap(*capture_cloud);
        // ROS_INFO("Points number of Cloud after range_filter is %d", points_num);
        // ROS_DEBUG("range filter takes %d ms.", (int)((ros::Time::now()-begin).toNSec()/1000000));

        // 滤除离群点，执行时间与输入点云点的个数成正比，与capture_resloution成反比（capture_resloution0.001mm-180ms，0.003mm，40-60ms）
        // begin = ros::Time::now();
        statistical_filter.setInputCloud(capture_cloud);
        statistical_filter.filter(*capture_cloud);
        // ROS_DEBUG("statistical_filter takes %f ms.", (ros::Time::now()-begin).toSec()*1000);

        // 一个奇奇怪怪的相机坐标系定义带来的坐标变换
        pcl::transformPointCloud(*capture_cloud, *capture_cloud, Pose_Corr_Trans.matrix());
        // PointCloud_Pub(capture_cloud, "test");

        /***********/
        /* 点云匹配 */
        /***********/
        
        // 获取点云的坐标变换以粗配准
        Eigen::Quaternionf q(world_trans_camera.getRotation().getW(), world_trans_camera.getRotation().getX(),
                             world_trans_camera.getRotation().getY(), world_trans_camera.getRotation().getZ());
        Eigen::Isometry3f Trans(q); // world to camera
        Trans.pretranslate(Eigen::Vector3f(world_trans_camera.getOrigin().getX(), world_trans_camera.getOrigin().getY(), world_trans_camera.getOrigin().getZ())); 
        pcl::transformPointCloud(*capture_cloud, *sample_cloud, Trans.matrix());

        // 配准
        // begin = ros::Time::now();
        icp_obj.setInputSource(sample_cloud);
        icp_obj.setInputTarget(parents_cloud);
        icp_obj.align(*regis_result);
        // ROS_DEBUG("ICP takes %d ms.", (int)((ros::Time::now()-begin).toNSec()/1000000));

        if(((icp_obj.getFinalTransformation().block<3,3>(0,0)-Eigen::Matrix3f::Identity()).norm() > ICPTf_ang_Max) || ( icp_obj.getFinalTransformation().block<3,1>(0,3).norm() > ICPTf_posi_Max)){
            ROS_DEBUG("Normal of transform that gotten by ICP is out of range. Rot norm: %f, Posi norm: %f.", (icp_obj.getFinalTransformation().block<3,3>(0,0)-Eigen::Matrix3f::Identity()).norm(), icp_obj.getFinalTransformation().block<3,1>(0,3).norm());
            continue;
        }

        // 更新偏差，发布并存储
        Eigen::Matrix4f world_trans_end = ( Trans * (end_trans_camera.inverse()) ).matrix();
        EndposeError = EndposeError * (world_trans_end.inverse()) * icp_obj.getFinalTransformation() * world_trans_end;
        // ROS_DEBUG_STREAM("[Visual_Servo_Loop] EndposeError = \n" << EndposeError);

        Eigen::Quaternionf qq(EndposeError.block<3,3>(0,0));
        EndposeErrorMsg.translation.x = EndposeError(0,3);
        EndposeErrorMsg.translation.y = EndposeError(1,3);
        EndposeErrorMsg.translation.z = EndposeError(2,3);
        EndposeErrorMsg.rotation.w    = qq.w();
        EndposeErrorMsg.rotation.x    = qq.x();
        EndposeErrorMsg.rotation.y    = qq.y();
        EndposeErrorMsg.rotation.z    = qq.z();
        EndposeError_pub.publish(EndposeErrorMsg);

        array_to_file(EndposeError_Savefile ,EndposeError);

        // // 扩展parents_cloud
        // if(++cnt %3 == 0){
        //     (*parents_cloud) += (*regis_result);
        //     if(cnt == 6){
        //         parent_voxel_filter.filter(*parents_cloud);
        //     }
        //     if(cnt == 18){
        //         PointCloud_Pub(parents_cloud, "Parents_PointCloud"); // debug
        //         cnt = 1;
        //     }
        // }
    }

    return true;
}

void snake_visual_servo::PointsCloudGet_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
    // ROS_DEBUG("Have captured a frame, filter...");

    pcl::fromROSMsg(*cloud_msg, *capture_cloud);
    capture_cloud->is_dense = false;

    // 去除无效点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*capture_cloud, indices);
    capture_cloud->is_dense = true;

    return;
}