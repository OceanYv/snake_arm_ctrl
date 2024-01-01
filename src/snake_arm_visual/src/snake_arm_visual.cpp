#include <snake_arm_visual.h>

snake_arm_visual::snake_arm_visual(ros::NodeHandle *nh){
    
    /**************/
    /* 获取配置参数 */
    /**************/
    STATE = true;
    _nh = nh;

    printf("\n");
    ROS_INFO(">>>snake_arm_visual config information");
    ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

    // 获取参数
    // -------------------------------------------------------------
    from_file          = _nh->param<bool>("/snake_arm_visual/pointcloud_from_file", false);
    capture_resloution = _nh->param<double>("/snake_arm_visual/build_surface/capture_resloution", 0.002);
    merge_resloution   = _nh->param<double>("/snake_arm_visual/build_surface/merge_resloution", 0.004);
    statistical_filter_StddevMulThresh = _nh->param<double>("/snake_arm_visual/build_surface/statistical_filter_StddevMulThresh", 0.01);

    icp_Epsilon = _nh->param<double>("/snake_arm_visual/build_surface/icp_Epsilon", 1e-10);
    icp_MaxCorrespondenceDistance_1 = _nh->param<double>("/snake_arm_visual/build_surface/icp_MaxCorrespondenceDistance_1", 0.02);
    icp_MaxCorrespondenceDistance_2 = _nh->param<double>("/snake_arm_visual/build_surface/icp_MaxCorrespondenceDistance_2", 0.005);
    icp_MaximumIterations = _nh->param<int>("/snake_arm_visual/build_surface/icp_MaximumIterations", 1000);

    PassThrough_xp = _nh->param<double>("/snake_arm_visual/build_surface/PassThrough_xp",  10.0 );
    PassThrough_xn = _nh->param<double>("/snake_arm_visual/build_surface/PassThrough_xn", -10.0 );
    PassThrough_yp = _nh->param<double>("/snake_arm_visual/build_surface/PassThrough_yp",  10.0 );
    PassThrough_yn = _nh->param<double>("/snake_arm_visual/build_surface/PassThrough_yn", -10.0 );
    PassThrough_zp = _nh->param<double>("/snake_arm_visual/build_surface/PassThrough_zp",   5.0 );
    PassThrough_zn = _nh->param<double>("/snake_arm_visual/build_surface/PassThrough_zn",   0.0 );

    Normal_search_num = nh->param<double>("/snake_arm_visual/build_surface/Normal_search_num", 30 );
    PrincipalCurvatures_RadiusSearch = _nh->param<double>("/snake_arm_visual/build_surface/PrincipalCurvatures_RadiusSearch",  0.005);
    cluster_ClusterTolerance =_nh->param<double>("/snake_arm_visual/build_surface/cluster_ClusterTolerance",  0.001);
    cluster_MinClusterSize = _nh->param<int>("/snake_arm_visual/build_surface/cluster_MinClusterSize", 100);
    cluster_MaxClusterSize = _nh->param<int>("/snake_arm_visual/build_surface/cluster_MaxClusterSize", 100000);

    double CRG_plane_curvature = nh->param<double>("/snake_arm_visual/build_surface/CRG_plane_curvature",  1.0 );
    double CRG_max_radius_dela = nh->param<double>("/snake_arm_visual/build_surface/CRG_max_radius_dela", 0.02 );
    double CRG_max_angle = nh->param<double>("/snake_arm_visual/build_surface/CRG_max_angle",  10.0);

    remove_rate_x  = nh->param<double>("/snake_arm_visual/build_surface/remove_rate_x", 0.95 );
    remove_rate_y  = nh->param<double>("/snake_arm_visual/build_surface/remove_rate_y", 0.90 );
    surface_order  = _nh->param<int>("/snake_arm_visual/build_surface/surface_order"  , 3    );
    surface_fit_refinement = _nh->param<int>("/snake_arm_visual/build_surface/surface_fit_refinement" , 2);
    surface_fit_iterations = _nh->param<int>("/snake_arm_visual/build_surface/surface_fit_iterations" , 5);

    surface_fit_interior_smoothness = _nh->param<double>("/snake_arm_visual/build_surface/surface_fit_interior_smoothness" , 0.2);
    surface_fit_interior_weight     = _nh->param<double>("/snake_arm_visual/build_surface/surface_fit_interior_weight"     , 1.0);
    surface_fit_boundary_smoothness = _nh->param<double>("/snake_arm_visual/build_surface/surface_fit_boundary_smoothness" , 0.2);
    surface_fit_boundary_weight     = _nh->param<double>("/snake_arm_visual/build_surface/surface_fit_boundary_weight"     , 0.0);

    scan_path_dist                  = _nh->param<double>("/snake_arm_visual/path_plan/scan_path_dist"     , 0.05);
    scan_step_length                = _nh->param<double>("/snake_arm_visual/path_plan/scan_step_length"   , 0.01);
    ScanPnt_Flt_NeighborNum         = _nh->param<int>("/snake_arm_visual/path_plan/ScanPnt_Flt_NeighborNum",25  );
    ScanPnt_Flt_StddevMulThresh     = _nh->param<double>("/snake_arm_visual/path_plan/ScanPnt_Flt_StddevMulThresh",0.05);


    time_t nowtime = time(NULL); //获取日历时间
    std::string temp = ctime(&nowtime)+4;
    if(from_file)
        data_save_path = _nh->param<std::string>("/snake_arm_visual/build_surface/data_save_path", "/home/ocici/ROS_CODE/3_snake_arm/src/snake_arm_visual/data/default");
    else
        data_save_path = _nh->param<std::string>("/snake_arm_visual/build_surface/data_save_path", "/home/ocici/ROS_CODE/3_snake_arm/src/snake_arm_visual/data/"+temp);
        
    // 判断参数是否合法
    if(PassThrough_xp <= PassThrough_xn) {ROS_WARN("Configuration parameter is illegal! PassThrough_xp=%f, PassThrough_xn=%f.",PassThrough_xp ,PassThrough_xn); PassThrough_xp = 10.0; PassThrough_xn = -10.0; }
    if(PassThrough_yp <= PassThrough_yn) {ROS_WARN("Configuration parameter is illegal! PassThrough_yp=%f, PassThrough_yn=%f.",PassThrough_yp ,PassThrough_yn); PassThrough_yp = 10.0; PassThrough_yn = -10.0; }
    if(PassThrough_zp <= PassThrough_zn) {ROS_WARN("Configuration parameter is illegal! PassThrough_zp=%f, PassThrough_zn=%f.",PassThrough_zp ,PassThrough_zn); PassThrough_zp =  5.0; PassThrough_zn =   0.0; }

    // 输出参数到终端
    printf("\n");
    ROS_INFO_STREAM("data_save_path                     : " << data_save_path            );
    ROS_INFO_STREAM("from_file                          : " << (from_file?"true":"false"));
    ROS_INFO_STREAM("capture_resloution                 : " << capture_resloution        );
    ROS_INFO_STREAM("merge_resloution                   : " << merge_resloution          );
    ROS_INFO_STREAM("statistical_filter_StddevMulThresh : " << statistical_filter_StddevMulThresh);

    ROS_INFO_STREAM("icp_Epsilon                        : " << icp_Epsilon               );
    ROS_INFO_STREAM("icp_MaxCorrespondenceDistance_1    : " << icp_MaxCorrespondenceDistance_1);
    ROS_INFO_STREAM("icp_MaxCorrespondenceDistance_2    : " << icp_MaxCorrespondenceDistance_2);
    ROS_INFO_STREAM("icp_MaximumIterations              : " << icp_MaximumIterations     );

    ROS_INFO_STREAM("PassThrough_xp                     : " << PassThrough_xp          );
    ROS_INFO_STREAM("PassThrough_xn                     : " << PassThrough_xn          );
    ROS_INFO_STREAM("PassThrough_yp                     : " << PassThrough_yp          );
    ROS_INFO_STREAM("PassThrough_yn                     : " << PassThrough_yn          );
    ROS_INFO_STREAM("PassThrough_zp                     : " << PassThrough_zp          );
    ROS_INFO_STREAM("PassThrough_zn                     : " << PassThrough_zn          );

    ROS_INFO_STREAM("Normal_search_num                  : " << Normal_search_num       );
    ROS_INFO_STREAM("PrincipalCurvatures_RadiusSearch   : " << PrincipalCurvatures_RadiusSearch);
    ROS_INFO_STREAM("cluster_ClusterTolerance           : " << cluster_ClusterTolerance);
    ROS_INFO_STREAM("cluster_MinClusterSize             : " << cluster_MinClusterSize  );
    ROS_INFO_STREAM("cluster_MaxClusterSize             : " << cluster_MaxClusterSize  );
    ROS_INFO_STREAM("CRG_plane_curvature                : " << CRG_plane_curvature     );
    ROS_INFO_STREAM("CRG_max_radius_dela                : " << CRG_max_radius_dela     );
    ROS_INFO_STREAM("CRG_max_angle                      : " << CRG_max_angle           );

    ROS_INFO_STREAM("remove_rate_x                      : " << remove_rate_x           );
    ROS_INFO_STREAM("remove_rate_y                      : " << remove_rate_y           );
    ROS_INFO_STREAM("surface_order                      : " << surface_order           );
    ROS_INFO_STREAM("surface_fit_refinement             : " << surface_fit_refinement  );
    ROS_INFO_STREAM("surface_fit_iterations             : " << surface_fit_iterations  );

    ROS_INFO_STREAM("surface_fit_interior_smoothness    : " << surface_fit_interior_smoothness);
    ROS_INFO_STREAM("surface_fit_interior_weight        : " << surface_fit_interior_weight    );
    ROS_INFO_STREAM("surface_fit_boundary_smoothness    : " << surface_fit_boundary_smoothness);
    ROS_INFO_STREAM("surface_fit_boundary_weight        : " << surface_fit_boundary_weight    );

    ROS_INFO_STREAM("scan_path_dist                     : " << scan_path_dist             );
    ROS_INFO_STREAM("scan_step_length                   : " << scan_step_length           );
    ROS_INFO_STREAM("ScanPnt_Flt_NeighborNum            : " << ScanPnt_Flt_NeighborNum    );
    ROS_INFO_STREAM("ScanPnt_Flt_StddevMulThresh        : " << ScanPnt_Flt_StddevMulThresh);

    printf("\n");

    /************/
    /* 初始化对象 */
    /************/
    if(access(data_save_path.c_str(), 0) == -1) //unlink(data_save_path.c_str());
        mkdir(data_save_path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);

    capture_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    /*********/
    /* debug */
    /*********/

    #ifdef TEST
        ROS_INFO("[snake_arm_visual] ==test temp==");

        // ObjCapture();

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // if(pcl::io::loadPCDFile(data_save_path + "/capture_merge.pcd", *source_pointcloud) == -1){
        //     ROS_ERROR_STREAM("Fail to read file " << data_save_path << "/capture_merge.pcd");
        //     return;
        // }
        // if(!PointCloud_Cluster(source_pointcloud)){
        //     ROS_ERROR("Fail to call PointCloud_Cluster.");
        //     return;
        // }

        // BuildSurf_PathPlan();

    #endif

    ROS_INFO("snake_arm_visual Construct successfully.");
    return;
}

snake_arm_visual::~snake_arm_visual(){
    // 释放对象
    
    return;
}

bool snake_arm_visual::snake_arm_visual_cmd(){

    if(!STATE) return false;

    printf("\n=========================================================\n");
    printf("====================Command Mode=========================\n");
    printf("=========================================================\n");

    std::string cmd;
    while (1){
        printf(" \n======== Select the Control Mode, or exit the program ======== \n");
        printf(" === [CP] to capture the object pointcloud.\n");
        printf(" === [CL] to cluster the object pointcloud.\n");
        printf(" === [PL] to build surface of the seleted feature and plan the scan path.\n");
        printf(" === [RC] to check accessibility of each pose.\n");
        printf(" === [Q ] to exit the snake_arm_visual program.\n");
        printf("\tCOMMAND: ");
        std::cin >> cmd;

        if (cmd == "CP"){
            if (!ObjCapture())
                ROS_WARN("[snake_arm_visual_cmd] Fail to call function ObjCapture.");
        }
        else if (cmd == "CL"){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (pcl::io::loadPCDFile(data_save_path + "/capture_merge.pcd", *source_pointcloud) == -1){
                ROS_WARN_STREAM("[snake_arm_visual_cmd] Fail to read file " << data_save_path << "/capture_merge.pcd");
                continue;
            }
            if (!PointCloud_Cluster(source_pointcloud)){
                ROS_WARN("[snake_arm_visual_cmd] Fail to call PointCloud_Cluster.");
                continue;
            }
        }
        else if (cmd == "PL"){
            if (!BuildSurf_PathPlan())
                ROS_WARN("[snake_arm_visual_cmd] Fail to call function BuildSurf_PathPlan.");
        }
        else if (cmd == "RC"){
            ROS_WARN("This function can only be run under simulation mode!!!");
            ROS_WARN("This function can only be run under simulation mode!!!");
            ROS_WARN("This function can only be run under simulation mode!!!");
            printf("'y' to continue, or 'n' to break, ");
            if (!Confirm_Interaction()) continue;
            poses_array_reachable_check();
        }
        else if (cmd == "Q"){
            ROS_INFO("Exiting...");
            break;
        }
        else
            ROS_WARN("No such command!!!");
    }
    STATE = false;
    return true;
}

bool snake_arm_visual::ObjCapture(){
    data_save_cnt = 0;

    ros::SubscribeOptions ops = getSubscribeOptions("/camera/depth/color/points",1,&snake_arm_visual::PointsCloudSave_callback,this,&pointcloud_queue);
    pointcloud_sub = _nh->subscribe(ops);

    ////////////////
    // 获取点云数据 //
    ////////////////
    if(!from_file){
        std::ofstream fout(data_save_path+"/capture_pose.txt",std::ios::out);
        std::string cmd;
        while(1){
            printf(" \n==================================================\n");
            printf(" ===================[ObjCapture]=================\n");
            printf(" === [C] to capture a pointcloud with the time and pose.\n");
            printf(" === [Q] to exit and splicing the pointcloud.\n");
            printf("\tCOMMAND: "); std::cin >> cmd;
            if(cmd == "C"){
                // 获取此时的机械臂位姿
                ros::Time now=ros::Time(0);
                tf::StampedTransform transform;
                pose_listener.waitForTransform("world","camera_aligned_depth_to_color_frame",now,ros::Duration(1.0));
                pose_listener.lookupTransform("/world", "/camera_aligned_depth_to_color_frame", now, transform);

                // 获取点云并存储
                if(pointcloud_queue.callOne(ros::WallDuration(0.05)) != 0)
                    ROS_WARN("[ObjCapture] Failed to call pointcloud_queue.");
                else
                {
                    data_save_cnt++;

                    Eigen::Matrix<double,1,7> pose_vec;
                    pose_vec(0) = transform.getOrigin().getX();
                    pose_vec(1) = transform.getOrigin().getY();
                    pose_vec(2) = transform.getOrigin().getZ();
                    pose_vec(3) = transform.getRotation().getW();
                    pose_vec(4) = transform.getRotation().getX();
                    pose_vec(5) = transform.getRotation().getY();
                    pose_vec(6) = transform.getRotation().getZ();
                    fout << pose_vec << std::endl;
                }
            }
            else if(cmd == "Q"){
                fout.close();
                break;
            }
            else ROS_WARN("No such command!!!");
        }
    }

    if(!PointCloud_Merge(data_save_path)){
        ROS_ERROR("Fail to call PointCloud_Merge.");
        return false;
    }

    return true;
}

bool snake_arm_visual::PointCloud_Merge(std::string path){
    ROS_INFO("[PointCloud_Merge] begin ========= ");

    bool first_frame = true;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Merged_PointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    /**************/
    /* 获取位姿文件 */
    /**************/
    std::ifstream fin(path+"/capture_pose.txt");
    if (!fin) {
        std::cerr << "[ObjCapture] cannot find pose file" << std::endl;
        return false;
    }

    int file_num = 0;
    std::string temp;
    while(std::getline(fin, temp,'\n')) file_num++;
    fin.close(); fin.open(path+"/capture_pose.txt");

    /********************************/
    /* 读取文件中的点云,并进行配准与融合 */
    /*******************************/

    // 创建ICP对象,并进行参数配置
    // pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB> icp_obj; // 虽说是ICP的优化算法，但实际体验感觉差不多而且更慢，就不打算用了
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp_obj;
    icp_obj.setTransformationEpsilon(icp_Epsilon);                        // 设置收敛判断条件，越小精度越大，收敛也越慢
    icp_obj.setMaximumIterations(icp_MaximumIterations);                  // 设置允许的最大迭代次数
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_icp_src(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_icp_tgt(new pcl::search::KdTree<pcl::PointXYZRGB>());
    icp_obj.setSearchMethodSource(tree_icp_src);
    icp_obj.setSearchMethodTarget(tree_icp_tgt);

    // 创建过程中的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final_2(new pcl::PointCloud<pcl::PointXYZRGB>);

    ROS_INFO("[PointCloud_Merge] Merging ... ");
    for(int i = 1; i <= file_num; i++) {

        // 读取文件
        if(pcl::io::loadPCDFile(path + "/capture_" + std::to_string(i) + ".pcd", *source_cloud) == -1){
            ROS_WARN_STREAM("Fail to read file " << path + "/capture_" + std::to_string(i) + ".pcd");
            continue;
        }

        // // 除去压紧随动机构的遮挡部分
        // PointCloud_Pub(source_cloud,"in_PointCloud"); // DEBUG
        // ROS_INFO("[PointCloud_Cluster] PassThrough ...");
        // pcl::PassThrough<pcl::PointXYZRGB> pass;
        // pass.setInputCloud (source_cloud);
        // pass.setFilterFieldName ("z");
        // pass.setFilterLimits (0,FLT_MAX);
        // pass.filter (*source_cloud);
        // PointCloud_Pub(source_cloud,"in_PointCloud"); // DEBUG

        // 获取位姿变换矩阵,并将点云变换到全局坐标系下
        float data[7] = {0};
        for (int i = 0; i < 7; i++) fin >> data[i];
        Eigen::Quaternionf q(data[3], data[4], data[5], data[6]);
        Eigen::Isometry3f Trans(q);
        Trans.pretranslate(Eigen::Vector3f(data[0], data[1], data[2]));
        // ROS_DEBUG_STREAM("Trans = \n" << Trans.matrix());
        pcl::transformPointCloud(*source_cloud, *source_cloud, Trans.matrix());
        PointCloud_Pub(source_cloud,"in_PointCloud"); // DEBUG
        // if(!Confirm_Interaction()) return false;

        // 配准与融合
        if(first_frame){
            first_frame = false;
            (*Merged_PointCloud) += (*source_cloud);
            (*target_cloud) = (*source_cloud);
        }
        else{
            // 通过 ICP 进行粗配准
            icp_obj.setMaxCorrespondenceDistance(icp_MaxCorrespondenceDistance_1);
            ROS_INFO("[PointCloud_Merge] === Executing ICP registration (%d/%d)...",i ,file_num);
            icp_obj.setInputSource(source_cloud);
            icp_obj.setInputTarget(target_cloud);
            icp_obj.align(*Final_1);
            if(!icp_obj.hasConverged())
                ROS_WARN("[PointCloud_Merge] First ICP Fail to convergence. i = %d, score = %f", i, icp_obj.getFitnessScore());
            else
                ROS_INFO_STREAM("[PointCloud_Merge] First  ICP Convergence. score = " << icp_obj.getFitnessScore() /*<< ", FinalTransformation = \n" << icp_obj.getFinalTransformation()*/);

            // 提高精度要求，二次迭代优化
            icp_obj.setMaxCorrespondenceDistance(icp_MaxCorrespondenceDistance_2);
            icp_obj.setInputSource(Final_1);
            icp_obj.setInputTarget(target_cloud);
            icp_obj.align(*Final_2);
            if(!icp_obj.hasConverged()){
                ROS_INFO("Second ICP Fail to convergence. i = %d, score = %f", i, icp_obj.getFitnessScore());
                (*Merged_PointCloud) += (*Final_1); // 更新点云
                target_cloud.swap(Final_1);
            }
            else {
                ROS_INFO_STREAM("[PointCloud_Merge] Second ICP Convergence! score = " << icp_obj.getFitnessScore() /*<< ", FinalTransformation = \n" << icp_obj.getFinalTransformation()*/);
                (*Merged_PointCloud) += (*Final_2);
                target_cloud.swap(Final_2);
            }
        }
        ROS_INFO("[PointCloud_Merge] finish %d/%d",i,file_num);
        PointCloud_Pub(Merged_PointCloud,"Merged_PointCloud");
    }

    // 对合并结果进行体素滤波,以消除重叠部分
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setLeafSize(merge_resloution, merge_resloution, merge_resloution);
    voxel_filter.setInputCloud(Merged_PointCloud);
    voxel_filter.filter(*Merged_PointCloud);

    // 将合并结果写入到文件capture_merge.pcd中
    std::string file_name = data_save_path + "/capture_merge.pcd";
    pcl::io::savePCDFileBinary(file_name.c_str(), *Merged_PointCloud);
    
    return true;
}

// 用于ConditionalEuclideanClustering的自定义判断函数。这里只简单的注释了一下思路，详细的可以看毕业论文。跟论文里的内容略有出入。
// source_curvatures用于存放主曲率数据，用于聚类计算
// 主要是类的成员函数的回调太麻烦了，所以迫不得已这样定义了全局变量和全局函数，强迫症抱头痛哭QAQ
pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr source_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
bool customRegionGrowing(const pcl::PointNormal &point_a, const pcl::PointNormal &point_b, float squared_distance, unsigned int point_a_index, unsigned int point_b_index)
{
    static ros::NodeHandle nh;
    static double plane_curvature = nh.param<double>("/snake_arm_visual/build_surface/CRG_plane_curvature", 0.04 );   // 当曲率小于该值时，认为是平面，使用法线方向夹角进行同面的判断
    static double max_radius_dela = nh.param<double>("/snake_arm_visual/build_surface/CRG_max_radius_dela", 0.1 );    // 对于球面,当曲率半径差小于该值时，认为共面
    static double max_angle = nh.param<double>("/snake_arm_visual/build_surface/CRG_max_angle",  12.0) / 180.0 * M_PI; // 当切线或法线夹角小于该值时，认为共面

    pcl::PrincipalCurvatures PC_a = source_curvatures->points[point_a_index];
    pcl::PrincipalCurvatures PC_b = source_curvatures->points[point_b_index];

    // 当面比较平的时候，使用法线方向夹角
    if( (std::max(std::abs(PC_a.pc1),std::abs(PC_a.pc2)) < plane_curvature) || (std::max(std::abs(PC_b.pc1),std::abs(PC_b.pc2)) < plane_curvature) ) {
        Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap();
        Eigen::Map<const Eigen::Vector3f> point_b_normal = point_b.getNormalVector3fMap();
        double angle_cos = point_a_normal.dot(point_b_normal) / (point_a_normal.norm() * point_b_normal.norm());
        if( std::abs(std::acos(angle_cos)) < max_angle/2 || std::abs(M_PI - std::acos(angle_cos)) < max_angle/2)  //法线夹角小于一定值
            return (true);
        else
            return (false);
    }
    else { // 当面曲率比较大的时候，曲面可能相切，因此可通过主切线的方向向量夹角进行判断
        // 当一个点的两个主曲率相等时，说明该点附近是个球面。若球面与另一个曲面共面，则应当有相同的曲率
        if(abs(1.0/PC_a.pc1 - 1.0/PC_a.pc2) < max_radius_dela){
            if( (abs(1.0/PC_a.pc1 - 1.0/PC_b.pc1)<max_radius_dela) || (abs(1.0/PC_a.pc1 - 1.0/PC_b.pc2)<max_radius_dela) ) return (true);
            else return (false);
        }
        else if(abs(1.0/PC_b.pc1 - 1.0/PC_b.pc2)<max_radius_dela){
            if( (abs(1.0/PC_b.pc1 - 1.0/PC_a.pc1)<max_radius_dela) || (abs(1.0/PC_b.pc1 - 1.0/PC_a.pc2)<max_radius_dela) ) return (true);
            else return (false);
        }

        Eigen::Vector3f point_a_tangent; point_a_tangent << PC_a.principal_curvature_x, PC_a.principal_curvature_y, PC_a.principal_curvature_z;
        Eigen::Vector3f point_b_tangent; point_b_tangent << PC_b.principal_curvature_x, PC_b.principal_curvature_y, PC_b.principal_curvature_z;
        double angle_cos = point_a_tangent.dot(point_b_tangent) / (point_a_tangent.norm() * point_b_tangent.norm());
        // ROS_DEBUG_STREAM("point_a_tangent = " << point_a_tangent.transpose() << ", point_b_tangent = " << point_b_tangent.transpose() << ", angle = " << std::abs(std::acos(angle_cos)));
        // ROS_DEBUG("PC_a.pc1 = %f, PC_a.pc2 = %f, PC_b.pc1 = %f, PC_b.pc2 = %f", PC_a.pc1, PC_a.pc2, PC_b.pc1, PC_b.pc2);
        if( std::abs(std::acos(angle_cos)) < max_angle || std::abs(M_PI - std::acos(angle_cos)) < max_angle)
            return (true);
        else
            return (false);
    }
}

bool snake_arm_visual::PointCloud_Cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source){
    ROS_INFO("[PointCloud_Cluster] Begin ========= ");

    /*********/
    /* 预处理 */
    /*********/

    // 除去地面等无关的面
    ROS_INFO("[PointCloud_Cluster] PassThrough ...");
    pcl::PassThrough<pcl::PointXYZRGB> pass;

    pass.setInputCloud (source);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (PassThrough_xn,PassThrough_xp);
    pass.filter (*source);

    pass.setInputCloud (source);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (PassThrough_yn,PassThrough_yp);
    pass.filter (*source);

    pass.setInputCloud (source);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (PassThrough_zn,PassThrough_zp);
    pass.filter (*source);

    PointCloud_Pub(source,"Merged_PointCloud");
    ROS_INFO("[PointCloud_Cluster] Source PointCloud has %d points.", (int)(source->points.size()));
    
    // 计算法向量特征
    ROS_INFO("[PointCloud_Cluster] Cal Normals ...");
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> normal_cal;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointNormal>::Ptr source_normals(new pcl::PointCloud<pcl::PointNormal>); // 含法线的点数据，用于ICP临时计算

    normal_cal.setSearchMethod(tree_normal);
    normal_cal.setKSearch(Normal_search_num);
    normal_cal.setInputCloud(source);
    normal_cal.compute(*source_normals);
    pcl::copyPointCloud(*source, *source_normals);

    // 计算点云主曲率
    // 看了一下源码，代码里的计算方法为：将邻域中的法线投影到待求点处的切平面上，并对所有投影向量进行PCA，所得 最大特征值除以邻域中点的个数 及 对应特征向量，即为最大主曲率及相应主切线
	if (pcl::io::loadPCDFile<pcl::PrincipalCurvatures>(data_save_path+"/curvatures.pcd", *source_curvatures) == -1) {
		ROS_WARN("Couldn't read file curvatures.pcd.");
        ROS_INFO("[PointCloud_Cluster] Cal PrincipalCurvatures ...");
        pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PrincipalCurvatures> pc_cal;
        pc_cal.setInputCloud(source);
        pc_cal.setInputNormals(source_normals);
        pc_cal.setSearchMethod(tree_normal);
        pc_cal.setRadiusSearch(PrincipalCurvatures_RadiusSearch);
        pc_cal.compute(*source_curvatures);
        pcl::io::savePCDFileASCII(data_save_path+"/curvatures.pcd", *source_curvatures);
	}       

    // DEBUG: show PrincipalCurvatures
    #ifdef PLOT_CURVATURE
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr PrincipalCurvatures_plot(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointXYZRGB point_Curvatures;
        for(int i=0;i<source->points.size();i++){
            point_Curvatures.x = source->points[i].x;
            point_Curvatures.y = source->points[i].y;
            point_Curvatures.z = source->points[i].z;

            pcl::PrincipalCurvatures PC_a = source_curvatures->points[i];
            double PC = std::min(std::abs(1.0/PC_a.pc1), std::abs(1.0/PC_a.pc1));
            double radia_max = 25;
            PC = (PC > radia_max) ? radia_max : PC;
            point_Curvatures.r = (int)( (PC)/ radia_max * 255);
            point_Curvatures.g = (int)( (PC)/ radia_max * 255);
            point_Curvatures.b = (int)( (PC)/ radia_max * 255);

            PrincipalCurvatures_plot->points.push_back(point_Curvatures);
        }
        PointCloud_Pub(PrincipalCurvatures_plot,"Curvatures_PointCloud");
    #endif

    /*******/
    /* 聚类 */
    /*******/

    // 参考：https://zhuanlan.zhihu.com/p/458450122
    // PCL中的聚类算法主要有: EuclideanClusterExtraction(是用于对欧几里得聚类（Euclidean Clustering）的实现)
    //                     ConditionalEuclideanClustering(在前一个方法的基础上，增加了自定义判据的功能)
    ROS_INFO("[PointCloud_Cluster] Cluster ...");
    pcl::IndicesClustersPtr result_clusters(new pcl::IndicesClusters);
    pcl::IndicesClustersPtr small_clusters(new pcl::IndicesClusters);
    pcl::IndicesClustersPtr large_clusters(new pcl::IndicesClusters);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_cluster(new pcl::search::KdTree<pcl::PointNormal>);
    tree_cluster->setInputCloud(source_normals);

    bool extract_removed_clusters = true; // 是否获取大小超范围的簇
    pcl::ConditionalEuclideanClustering_my<pcl::PointNormal> Cluster_obj(extract_removed_clusters);
    Cluster_obj.setInputCloud(source_normals);
    Cluster_obj.setConditionFunction(&customRegionGrowing);
    Cluster_obj.setMinClusterSize(cluster_MinClusterSize);     // 设置最小聚类尺寸
    Cluster_obj.setMaxClusterSize(cluster_MaxClusterSize);     // 设置最大聚类尺寸
    Cluster_obj.setClusterTolerance(cluster_ClusterTolerance); // 设置近邻搜索的搜索半径

    Cluster_obj.segment(*result_clusters);
    Cluster_obj.getRemovedClusters(small_clusters, large_clusters);

    ROS_INFO("[PointCloud_Cluster] result_clusters_num = %d", (int)(result_clusters->size()));
    ROS_INFO("[PointCloud_Cluster] small_clusters_num  = %d", (int)(small_clusters->size()));
    ROS_INFO("[PointCloud_Cluster] large_clusters_num  = %d", (int)(large_clusters->size()));
    
    if(!result_clusters->size()) return false; // 如果聚类结果为空，则返回

    /*****************/
    /* 结果输出与可视化 */
    /*****************/

    ROS_INFO("[PointCloud_Cluster] Output the result ...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_rgb(new pcl::PointCloud<pcl::PointXYZRGB>); // 用于可视化的点云

    output_rgb->width = 0;
    int cluster_num = 0;      // 簇的编号
    pcl::PointXYZRGB  point; // 获取每一个点的值,并在赋予颜色后压入output_rgb
    std::ofstream fout(data_save_path+"/cluster_info.txt",std::ios::out);

    // 遍历每一个簇
    for(pcl::IndicesClusters::const_iterator it = result_clusters->begin(); it != result_clusters->end (); ++it){
        // 不同簇中的点赋予不同的颜色
        cluster_num++;
        point.r = rand()%256; point.g = rand()%256; point.b = rand()%256; 

        // 遍历每个簇中的每一个点
        pcl::PointCloud<pcl::PointNormal>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointNormal>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            point.x = source_normals->points[*pit].x;
            point.y = source_normals->points[*pit].y;
            point.z = source_normals->points[*pit].z;
            output_rgb->points.push_back(point);
            cloudCluster->points.push_back(source_normals->points[*pit]);
        }
        ROS_INFO("[PointCloud_Cluster] PointCloud cluster_%d has %d points.",cluster_num, (int)(cloudCluster->points.size()));
        fout << std::setiosflags(std::ios::left)  << std::setw(2) << std::setfill(' ') << "PointCloud cluster_" << cluster_num;
        fout << std::setiosflags(std::ios::right) << std::setw(5) << std::setfill(' ') << " has " << (int)(cloudCluster->points.size()) << " points." << std::endl;

        output_rgb->width += cloudCluster->points.size ();
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        std::string file_name = data_save_path + "/cluster_" + std::to_string(cluster_num) + ".pcd";
        pcl::io::savePCDFileBinary(file_name.c_str(), *cloudCluster);
    }

    fout.close();
    output_rgb->height  = 1;
    output_rgb->is_dense = true;
    std::string file_name = data_save_path + "/cluster_all.pcd";
    pcl::io::savePCDFileBinary(file_name.c_str(), *output_rgb);
    PointCloud_Pub(output_rgb, "Cluster_PointCloud");

    // printf("Please confirm the result, ");
    // return Confirm_Interaction();
    return true;
}

bool snake_arm_visual::BuildSurf_PathPlan(){
    ROS_INFO("[BuildSurf_PathPlan] begin ========= ");

    while(1){

        /*******************/
        /* 获取已有对象的信息 */
        /*******************/
        std::ifstream fin(data_save_path+"/cluster_info.txt");
        if (!fin) {
            ROS_ERROR("[BuildSurf_PathPlan] cannot find cluster_info file");
            return false;
        }

        int file_num = 0; // 对象数量
        std::string temp;
        while(std::getline(fin, temp,'\n')){
            std::cout << temp << std::endl;
            file_num++;
        }
        if(!file_num){ROS_ERROR("[BuildSurf_PathPlan] Don't have any cluster object file!!!"); return false;}

        // 选择需要处理的对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        int cluster_select_num;
        bool get_right_num = false, break_flag = false; // 获取正确的编号、跳出该函数的循环
        do{
            usleep(1000*500);
            std::cout << "[BuildSurf_PathPlan] Select an object to deal with(1~" << file_num << ", '0' to break.): ";
            std::cin >> cluster_select_num;
            if(cluster_select_num == 0) {
                break_flag = true;
                break;
            }
            else if( (cluster_select_num < 1) || (cluster_select_num > file_num) )
                ROS_WARN("[BuildSurf_PathPlan] Given number out of range!!!");
            else{
                if(pcl::io::loadPCDFile(data_save_path + "/cluster_" + std::to_string(cluster_select_num) + ".pcd", *source_cloud) == -1)
                    ROS_WARN_STREAM("[BuildSurf_PathPlan] Fail to read file " << data_save_path + "/cluster_" + std::to_string(cluster_select_num) + ".pcd");
                else
                    get_right_num = true;
            }
        }while(!get_right_num);

        if(break_flag) break;

        PointCloud_Pub(source_cloud,"BuildSurf_Obj");

        /************/
        /* 对象预处理 */
        /************/
        // 边缘去毛刺：求取包围盒，然后删除包围盒x、y方向上最外围的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ranged(new pcl::PointCloud<pcl::PointXYZ>); // 存放去毛刺后的点云
        cloud_ranged->width  = 0;
        cloud_ranged->height = 1;
        cloud_ranged->is_dense = true;

        Eigen::Vector3d mean;               // 点云重心
        Eigen::Matrix3d eigenvectors;       // 特征向量（有三个列向量）
        Eigen::Vector3d eigenvalues;        // 特征值（有三个）
        Eigen::Matrix<float,3,2> obb_range; // 包围盒范围
        PCAAndOBB_Get(source_cloud, mean, eigenvectors, eigenvalues, obb_range);

        double x_max = remove_rate_x * obb_range(0,0);
        double x_min = remove_rate_x * obb_range(0,1);
        double y_max = remove_rate_y * obb_range(1,0);
        double y_min = remove_rate_y * obb_range(1,1);

        Eigen::Matrix3d eigenvectors_inv = eigenvectors.inverse ();
        for (unsigned i = 0; i < static_cast<unsigned> (source_cloud->points.size ()); i++) {
            Eigen::Vector3d point_vec = Eigen::Vector3d(source_cloud->points[i].x, source_cloud->points[i].y, source_cloud->points[i].z);
            Eigen::Vector3d p (eigenvectors_inv * (point_vec - mean)); // 将点转移到PCA建立的局部坐标系下

            if(p(0)<x_max && p(0)>x_min && p(1)<y_max && p(1)>y_min){
                cloud_ranged->points.push_back(source_cloud->points[i]);
                cloud_ranged->width++;
            }
        }
        PointCloud_Pub(cloud_ranged,"BuildSurf_Obj");

        /*************/
        /* 拟合与规划 */
        /************/

        // 曲面拟合
        if(!Cloud_Surface_fit(cloud_ranged, cluster_select_num)){
            ROS_ERROR("Fail to call Cloud_Surface_fit.");
        }

        // 路径规划
        if(!Path_plan(cluster_select_num)){
            ROS_ERROR("Fail to call Cloud_Surface_fit.");
        }
    }

    return true;
}

bool snake_arm_visual::Cloud_Surface_fit(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, int num){
    ROS_INFO("[Cloud_Surface_fit] Begin ========= ");

    // STEP0: 获取对象PCA与OBB。OBB大小也会在后面被用于可视化时的分辨率设置
    Eigen::Vector3d mean;               // 点云重心
    Eigen::Matrix3d eigenvectors;       // 特征向量（有三个列向量）
    Eigen::Vector3d eigenvalues;        // 特征值（有三个）
    Eigen::Matrix<float,3,2> obb_range; // 包围盒范围

    PCAAndOBB_Get(source_cloud, mean, eigenvectors, eigenvalues, obb_range);
    PCAAndOBB_Pub(mean, eigenvectors, eigenvalues, obb_range, "PCA_And_OBB");

    Eigen::Matrix<float,3,1> obb_size; // 包围盒大小
    obb_size(0) = obb_range(0,0) - obb_range(0,1);
    obb_size(1) = obb_range(1,0) - obb_range(1,1);
    obb_size(2) = obb_range(2,0) - obb_range(2,1);

    // STEP1: 通过PCA获取初始曲面对象
    //   将点云形式转换为适用于pcl::on_nurbs的形式，并通过initNurbsPCABoundingBox方法获取初始曲面
    //   source_cloud必须是没有无效点的
    ROS_INFO("[Cloud_Surface_fit] Init the Surface by PCABoundingBox...");
    pcl::on_nurbs::NurbsDataSurface data;
    for (unsigned i = 0; i < source_cloud->size(); i++) {
        pcl::PointXYZ &p = source_cloud->at(i);
        data.interior.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }

    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(surface_order, &data); // 获取初始曲面对象
    ON_NurbsSurface_pub(nurbs,obb_size,"fit_surface");
    usleep(1000*500);

    // STEP2: 创建拟合器对象和控制参数，并初始化
    ROS_INFO("[Cloud_Surface_fit] Creating the FittingSurface object...");
    pcl::on_nurbs::FittingSurface fit(&data, nurbs); // 含初始曲面的拟合器对象
    pcl::on_nurbs::FittingSurface::Parameter params; // 拟合器参数
    params.interior_smoothness = surface_fit_interior_smoothness; // 内部表面的光滑度
    params.interior_weight     = surface_fit_interior_weight    ; // 用于表面内部优化的权重
    params.boundary_smoothness = surface_fit_boundary_smoothness; // 表面边界的平滑度
    params.boundary_weight     = surface_fit_boundary_weight    ; // 表面边界优化的权重

    // STEP3: 表面细化。每次细化都会增大控制点个数（差不多是翻倍）
    for (unsigned i = 0; i < surface_fit_refinement; i++) {
        ROS_INFO("[Cloud_Surface_fit] Step One : refine %d/%d ...", i+1, surface_fit_refinement);
        fit.refine(0); // 通过在每一个元素之间插入一个节点来进行曲面的细化，这里细化的是第一个维度
        fit.refine(1); // 通过在每一个元素之间插入一个节点来进行曲面的细化，这里细化的是第二个维度
        fit.assemble(params); // 输入初始化参数，需在solve之前调用一次，会比较费时
        fit.solve();   // 求解

        ON_NurbsSurface_pub(fit.m_nurbs,obb_size,"fit_surface");
        usleep(1000*100);
    }

    // STEP4: 表面优化。在保持点的个数不变的情况下进行拟合的优化
    for (unsigned i = 0; i < surface_fit_iterations; i++) {
        ROS_INFO("[Cloud_Surface_fit] Step Two : surface_fit_iterations %d/%d ...", i+1, surface_fit_iterations);
        fit.assemble(params);
        fit.solve();

        ON_NurbsSurface_pub(fit.m_nurbs,obb_size,"fit_surface");
        usleep(1000*100);
    }

    // 以上过程建立了拟合点云的曲面，当时曲面的面积是大于点云的面积的
    // 所以需要通过点云的边界进行曲面裁剪，以获得重合的区域
    // 这里的做法是：将点云中离曲面最近的点投影到曲面的参数域中（曲面有两个参数，于是问题就转变成了二维的）
    //             接着在参数域中进行B样条曲线的拟合。
    // Ps：这个曲线算出来之后，后面的代码其实并没有用到，因为感觉好难懒得搞了（没错我就是废物！！理直气壮.jpg）

    // 参数配置
    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
    curve_params.addCPsAccuracy = 0.05;   // 曲线的支持区域到最近数据点的距离必须低于该值，否则将插入控制点
    curve_params.addCPsIteration = 3;     // 不插入控制点的内部迭代
    curve_params.maxCPs = 4;            // 控制点的最大总数
    curve_params.accuracy = 0.1;          // 曲线的拟合精度
    curve_params.iterations = 100;        // 最大迭代次数

    curve_params.param.closest_point_resolution = 0; // 必须位于支持区域内的控制点的数量
    curve_params.param.closest_point_weight = 1.0;   // 曲线拟合到最近点的权值
    curve_params.param.closest_point_sigma2 = 0.1;   // 最近点的阈值（不考虑离曲线更远的点）
    curve_params.param.interior_sigma2 = 0.00001;    // 内部点的阈值（用于忽略远离曲线并位于曲线内的点）
    curve_params.param.smooth_concavity = 1.0;       // 导致曲线向内弯曲的值(0 =不弯曲;< 0向内弯曲;> 0向外弯曲
    curve_params.param.smoothness = 1.0;             // 平滑项的权值

    // STEP1: 创建初始曲线对象，
    //        并通过一个点云的包被圆进行曲线的初始化。圆的中心位于点云重心处
    ROS_INFO("[Cloud_Surface_fit] Curve init...");
    ON_NurbsCurve curve_nurbs;                  // 初始曲线对象
    pcl::on_nurbs::NurbsDataCurve2d curve_data; // 参数对象
    curve_data.interior = data.interior_param;
    curve_data.interior_weight_function.push_back (true); // 对所有点启用内部加权

    curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (surface_order, curve_data.interior);

    // STEP2: 拟合优化
    ROS_INFO("[Cloud_Surface_fit] Curve fitting...");
    pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs); // 拟合器对象
    curve_fit.fitting(curve_params); // 执行拟合

    // 将曲面和曲线保存为.g2文件
    ROS_INFO("[Cloud_Surface_fit] Surface & Curve Output...");
    std::string file_name = data_save_path + "/SurfaceFit_" + std::to_string(num) + ".g2";
    ON_NurbsSurface2g2file(fit.m_nurbs, file_name);
    file_name = data_save_path + "/CurveFit_" + std::to_string(num) + ".g2";
    ON_NurbsCurve2g2file(curve_fit.m_nurbs, file_name);

    return true;
}

bool snake_arm_visual::Path_plan(int num){
    ROS_INFO("[Path_plan] Begin ========= ");

    // 读取曲面与相应点云
    std::string surface_name = data_save_path + "/SurfaceFit_" + std::to_string(num) + ".g2";
    std::ifstream surface_rd(surface_name.c_str());
    SISLSurf* surface = readGoSurface(surface_rd);
    surface_rd.close();

    std::string pointcloud_name = data_save_path + "/cluster_" + std::to_string(num) + ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(pointcloud_name, *source_cloud) == -1){
        ROS_WARN_STREAM("[Path_plan] Fail to read file " << pointcloud_name);
        return false;
    }

    // debug : calculate fit error
    std::string error_file_name = data_save_path + "/SurfaceFitError_" + std::to_string(num) + ".txt";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr error_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::ofstream data_write(error_file_name.c_str());
    double distance;

    for(int i=0;i<source_cloud->points.size();i++){
        double point[3];
        int numclopt = 0;
        double* pointpar = NULL;
        int numclocr = 0;
        SISLIntcurve **clocurves =NULL;
        int stat = 0;

        point[0] = source_cloud->points[i].x; point[1] = source_cloud->points[i].y; point[2] = source_cloud->points[i].z;
        s1954(surface, point, 3, 1e-9, 1e-6, &numclopt, &pointpar, &numclocr, &clocurves, &stat);
        if(numclopt >0){
            double pointpar_1[2]; double point_cls[3];
            pointpar_1[0] = pointpar[0]; pointpar_1[1] = pointpar[1];
            if(!my_s1424(surface,pointpar_1,point_cls)) ROS_WARN("[Path_plan] Fail to call my_s1424");
            distance = sqrt( pow(point[0]-point_cls[0],2) + pow(point[1]-point_cls[1],2) + pow(point[2]-point_cls[2],2) );
        }
        else
            distance = 0;

        data_write << i << " " << distance << std::endl;

        pcl::PointXYZRGB point_rgb;
        point_rgb.x = point[0]; point_rgb.y = point[1]; point_rgb.z = point[2]; 
        double max_error = 0.001;
        point_rgb.r = 255;
        point_rgb.g = ((distance>max_error)?0:max_error-distance) / max_error * 255;
        point_rgb.b = ((distance>max_error)?0:max_error-distance) / max_error * 255;
        error_cloud->points.push_back(point_rgb);
    }
    PointCloud_Pub(error_cloud,"error_cloud");
    data_write.close();

    /*********************/
    /* 获取整体的主切线方向 */
    /*********************/

    // 在扫查中，相控阵探头应尽量沿表面曲率最小的方向摆放，以获得最好的耦合效果
    // 并将曲率最大的方向作为扫查运动中的主要进给方向
    // 此处通过随机获取平面上多个点的主曲率的方法，来进行曲面整体主切线方向的判定

    // 用于生成随机数
    double knot_max[2] = {surface->et1[surface->ik1 + surface->in1 -1], surface->et2[surface->ik2 + surface->in2 -1]}; // u v
    double knot_min[2] = {surface->et1[0], surface->et2[0]}; // u v
    std::uniform_real_distribution<double> u_rand(knot_min[0], knot_max[0]);
    std::uniform_real_distribution<double> v_rand(knot_min[1], knot_max[1]);
    std::default_random_engine e(time(NULL));

    // 求切线向量
    int first_direc; // 0:u, 1:v
    double d_sum[2] = {0,0};
    for(int i=0;i<15;i++){ // 15个点
        double parvalue[2] = {u_rand(e), v_rand(e)}; // 点的索引参数
        int leftknot1=0, leftknot2=0;                // 在顺序搜索中用于加快搜索，在该随机场景中无效
        double k1,k2;                                // 主曲率
        double d1[2],d2[2];                 // 切线向量（参数域下）
        int stat;
        s2542(surface, 0, 0, 0, parvalue, &leftknot1, &leftknot2, &k1, &k2, d1, d2, &stat); // 求解主曲率及其切线

        // 对切向量进行粗暴的求平均（其实只是求和，不过比平均少了个除法）
        if(abs(k2)>abs(k1)){
            d1[0] = d2[0];  d1[1] = d2[1];
        }
        double d_length = sqrt( d1[0]*d1[0] + d1[1]*d1[1] ); // 化为单位向量
        d_sum[0] += std::abs(d1[0]) / d_length;
        d_sum[1] += std::abs(d1[1]) / d_length;
        ROS_DEBUG("k1 = %f, k2 = %f", k1, k2);
        ROS_DEBUG("d1[0] = %f, d1[1] = %f", std::abs(d1[0]) / d_length, std::abs(d1[1]) / sqrt( d1[0]*d1[0] + d1[1]*d1[1] ));
        ROS_DEBUG("d2[0] = %f, d2[1] = %f", std::abs(d2[0]) / d_length, std::abs(d2[1]) / sqrt( d2[0]*d2[0] + d2[1]*d2[1] ));
    }
    ROS_DEBUG("d_sum[0] = %f, d_sum[1] = %f", d_sum[0], d_sum[1]);

    if(d_sum[0] > d_sum[1]){
        first_direc = 0;
        ROS_INFO("[Path_plan] First direction is u.");
    }
    else{
        first_direc = 1;
        ROS_INFO("[Path_plan] First direction is v.");
    }

    /*************************/
    /* 沿first_direc路径的获取 */
    /*************************/

    ROS_INFO("[Path_plan] Getting the paths along the first direction ...");
    
    // 变量定义
    double knot_1st_mid;
    double knot_2nd_last, knot_2nd_delta;
    double dist_exp;
    double posi_last[3], posi_new[3], posi_end[3]; // 上一个点，新点，终点
    std::vector<double> knot_2nd; // 存放沿2nd方向按scan_path_dist搜索获得的各路径上节点序列

    double pointpar[2];
    bool first_search = true;
    
    // 初始化
    knot_2nd.clear();
    knot_1st_mid = (knot_min[first_direc] + knot_max[first_direc])/2;
    knot_2nd_last = knot_min[1-first_direc];

    pointpar[first_direc] = knot_1st_mid;
    pointpar[1-first_direc] = knot_min[1-first_direc];
    if(!my_s1424(surface,pointpar,posi_last)) ROS_WARN("[Path_plan] Fail to call my_s1424");

    pointpar[1-first_direc] = knot_max[1-first_direc];
    if(!my_s1424(surface,pointpar,posi_end)) ROS_WARN("[Path_plan] Fail to call my_s1424");

    // 开始求解,每次一个点
    while(1){
        // 估算 knot_2nd_delta
        double dist_remain = sqrt( std::pow(posi_last[0]-posi_end[0],2) + std::pow(posi_last[1]-posi_end[1],2) + std::pow(posi_last[2]-posi_end[2],2) );

        if(dist_remain < scan_path_dist){ // 搜索到接近边界，搜索结束
            knot_2nd.push_back((knot_max[1-first_direc] + knot_2nd_last)/2);
            break;
        }
        else if(dist_remain < 1.5*scan_path_dist) // 为避免最后一步极其靠近边缘且步进极小，对剩余行程进行近似的均分
            knot_2nd_delta = (knot_max[1-first_direc] - knot_2nd_last) / std::ceil(dist_remain / scan_path_dist);
        else
            knot_2nd_delta = (knot_max[1-first_direc] - knot_2nd_last) * scan_path_dist / dist_remain;

        // 设置搜索距离
        if(first_search){
            first_search = false;
            knot_2nd_delta /= 3;
            dist_exp = scan_path_dist / 3;
        }
        else if(dist_remain < 1.5*scan_path_dist) // 为避免最后一步极其靠近边缘且步进极小，对剩余行程进行近似的均分
            dist_exp = dist_remain / std::ceil(dist_remain / scan_path_dist);
        else
            dist_exp = scan_path_dist;

        // 迭代求解单个点
        while(1){
            pointpar[1-first_direc] = knot_2nd_last + knot_2nd_delta;
            if(!my_s1424(surface,pointpar,posi_new)) ROS_WARN("[Path_plan] Fail to call my_s1424");
            double dist_now = sqrt( std::pow(posi_last[0]-posi_new[0],2) + std::pow(posi_last[1]-posi_new[1],2) + std::pow(posi_last[2]-posi_new[2],2) );

            if( abs(dist_now-dist_exp) < 2e-4){ // 求解收敛，更新数据并返回
                posi_last[0] = posi_new[0]; posi_last[1] = posi_new[1]; posi_last[2] = posi_new[2];
                knot_2nd_last += knot_2nd_delta;
                knot_2nd.push_back(knot_2nd_last);
                break;
            }
            else
                knot_2nd_delta *= (dist_exp / dist_now);
        }
    }
    ROS_INFO("[Path_plan]     Got %d paths along the first direction!", (int)(knot_2nd.size()));

    // 调用 SISL s1439 获取各个曲线，保存并可视化
    std::string file_name = data_save_path + "/SurfacePathPlan_" + std::to_string(num) + ".g2";
    std::ofstream output_PathPlan(file_name.c_str());
    std::vector<SISLCurve*> curve_list;
    for(int i=0;i<knot_2nd.size();i++){
        SISLCurve *rcurve = NULL;
        int stat=0;

        s1439(surface, knot_2nd[i], 2-first_direc, &rcurve, &stat);
        if(stat != 0)
            ROS_ERROR("[Path_plan] Fail to call s1439.");
        else{
            curve_list.push_back(rcurve);
            writeGoCurve(rcurve, output_PathPlan);
            Curve_Pub(rcurve,"scan_path","world",'b',(i==0)?false:true);
        }
    }
    output_PathPlan.close();

    /******************/
    /* Discrete paths */
    /******************/
    // 由于拟合出的曲面可能大于实际曲面，因此需要对离散出的扫差点进行判断：
    //   判断获得的点对于原始点云是否是游离点，方法参考于statistical_outlier_removal类

    ROS_INFO("[Path_plan] Making paths discrete ...");

    // 变量定义
    pcl::search::Search<pcl::PointXYZ>::Ptr filter_tree(new pcl::search::KdTree<pcl::PointXYZ>); // 用于过滤超出点云区域的扫差点
    filter_tree->setInputCloud(source_cloud);
    std::vector<int> indices(ScanPnt_Flt_NeighborNum);  // 存放临近点搜索的索引
    std::vector<float> distes(ScanPnt_Flt_NeighborNum); // 存放临近点距离

    pcl::PointCloud<pcl::PointXYZ>::Ptr plan_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<Eigen::Vector2f> param_vec; Eigen::Vector2f param_temp;// 存放所有采样点在样条曲面上的参数值
    plan_points->width = 0;
    plan_points->height = 1;
    plan_points->is_dense = true;

    bool inverse = false; // 是否沿着曲线逆向运动（应用于S形扫差路径中）

    // 依次对每一条曲线进行离散
    for(int i=0;i<curve_list.size();i++){
        ROS_INFO("[Path_plan]     Making paths %d/%d discrete ...",i+1 ,(int)(curve_list.size()));

        // 初始化
        double knot_min  = curve_list[i]->et[0];
        double knot_max  = curve_list[i]->et[curve_list[i]->ik + curve_list[i]->in -1];
        double knot_last = inverse?knot_max:knot_min;
        double knot_delta;

        double dist_exp;
        bool first_search = true;
        
        double point_last[3], point_new[3], point_end[3];
        pcl::PointXYZ plan_point;

        if(!my_s1227(curve_list[i], knot_last, point_last)) ROS_WARN("[Path_plan] Fail to call my_s1227.");
        if(!my_s1227(curve_list[i], inverse?knot_min:knot_max, point_end )) ROS_WARN("[Path_plan] Fail to call my_s1227.");

        // 开始求解,每次一个点
        int cnt=0;
        while(1){
            // 估算 knot_delta
            double dist_remain = sqrt( std::pow(point_last[0]-point_end[0],2) + std::pow(point_last[1]-point_end[1],2) + std::pow(point_last[2]-point_end[2],2) );
            // ROS_DEBUG("dist_remain = %f, knot_last = %f", dist_remain , knot_last);
            if(dist_remain < scan_step_length){ // 搜索到接近边界，搜索结束
                if(!my_s1227(curve_list[i], ((inverse?knot_min:knot_max) + knot_last)/2,  point_new )) ROS_WARN("[Path_plan] Fail to call my_s1227.");
                plan_point.x = point_new[0]; plan_point.y = point_new[1]; plan_point.z = point_new[2]; 

                // 检验获得的点是否属于点云的游离点
                int get_num = filter_tree->nearestKSearch(plan_point, ScanPnt_Flt_NeighborNum, indices, distes);
                if( get_num == 0)
                    ROS_WARN("[Path_plan] Searching for the closest %d neighbors failed.", ScanPnt_Flt_NeighborNum);
                else{
                    double dist_sum = 0; for (int j = 1; j < get_num; ++j)  dist_sum += sqrt (distes[j]); //求和

                    if(dist_sum/get_num < ScanPnt_Flt_StddevMulThresh){ // 压入点
                        plan_points->points.push_back(plan_point); cnt++;
                        param_temp(1-first_direc) = knot_2nd[i]; param_temp(first_direc) = ((inverse?knot_min:knot_max) + knot_last)/2;
                        param_vec.push_back(param_temp);
                    }
                    else ROS_DEBUG("[Path_plan] Scan Point out of range.");
                }
                break;
            }
            else
                knot_delta = ((inverse?knot_min:knot_max) - knot_last) * scan_step_length / dist_remain;

            // 设置搜索距离
            if(first_search){
                first_search = false;
                knot_delta /= 3;
                dist_exp = scan_step_length / 3;
            }
            else
                dist_exp = scan_step_length;

            // 迭代求解单个点
            while(1){
                // ROS_DEBUG("knot_delta = %f", knot_delta);
                if(!my_s1227(curve_list[i], knot_last + knot_delta, point_new)) ROS_WARN("[Path_plan] Fail to call my_s1227.");
                double dist_now = sqrt( std::pow(point_last[0]-point_new[0],2) + std::pow(point_last[1]-point_new[1],2) + std::pow(point_last[2]-point_new[2],2) );

                if( abs(dist_now-dist_exp) < 2e-4){ // 求解收敛，更新数据并返回
                    point_last[0] = point_new[0]; point_last[1] = point_new[1]; point_last[2] = point_new[2];
                    plan_point.x  = point_new[0]; plan_point.y  = point_new[1]; plan_point.z  = point_new[2]; 
                    knot_last += knot_delta;

                    // 检验获得的点是否属于点云的游离点
                    int get_num = filter_tree->nearestKSearch(plan_point, ScanPnt_Flt_NeighborNum, indices, distes); // 临近点的数量
                    if( get_num == 0){ ROS_WARN("[Path_plan] Searching for the closest %d neighbors failed.", ScanPnt_Flt_NeighborNum); break; }

                    double dist_sum = 0; for (int j = 1; j < get_num; ++j)  dist_sum += sqrt (distes[j]); // 距离求和
                    if(dist_sum/get_num < ScanPnt_Flt_StddevMulThresh){ // 有效点，压入
                        plan_points->points.push_back(plan_point);
                        param_temp(1-first_direc) = knot_2nd[i]; param_temp(first_direc) = knot_last;
                        param_vec.push_back(param_temp);
                        cnt++; 
                    }
                    else ROS_DEBUG("[Path_plan] Scan Point out of range.");
                    
                    break;
                }
                else
                    knot_delta *= ((dist_exp / dist_now) + 1)/2;
            }
        }
        plan_points->width += cnt;
        if(inverse) inverse = false;
        else inverse = true;
    }
    ROS_INFO("[Path_plan] Got %d points in total.", (int)(plan_points->points.size()));

    // 输出与可视化
    ROS_INFO("[Path_plan] Plotting scan points and path...");
    file_name = data_save_path + "/SurfacePathDisc_" + std::to_string(num) + ".pcd";
    pcl::io::savePCDFileBinary(file_name.c_str(), *plan_points);
    PointCloud_Pub(plan_points, "Plan_Points");

    visualization_msgs::Marker Path_Marker;
    ros::Publisher Path_Publisher;
    Path_Publisher = _nh->advertise<visualization_msgs::Marker>("scan_path", 1);

    static int surface_cnt = 0;  surface_cnt++;
    Path_Marker.id = surface_cnt;
    Path_Marker.scale.x = 0.002;
    Path_Marker.color.a = 1.0;
    Path_Marker.type = visualization_msgs::Marker::LINE_STRIP;
    Path_Marker.pose.orientation.w = 1.0;
    Path_Marker.pose.orientation.x = 0.0;
    Path_Marker.pose.orientation.y = 0.0;
    Path_Marker.pose.orientation.z = 0.0;
    Path_Marker.color.r = 0.0f; Path_Marker.color.g = 0.0f; Path_Marker.color.b = 1.0f;
    Path_Marker.header.frame_id = "world";
    Path_Marker.header.stamp = ros::Time::now();
    Path_Marker.ns = "scan_path";
    Path_Marker.action = visualization_msgs::Marker::ADD;
    Path_Marker.lifetime = ros::Duration();

    geometry_msgs::Point sample_points;
    Path_Marker.points.clear();
    for (int i = 0; i < plan_points->points.size(); i++){
        sample_points.x = plan_points->points[i].x;
        sample_points.y = plan_points->points[i].y;
        sample_points.z = plan_points->points[i].z;
        Path_Marker.points.push_back(sample_points);
    }

    for(int i=0;i<5;i++){
        Path_Publisher.publish(Path_Marker);
        usleep(1000*100);
    }

    /**************************/
    /* Get pose at each point */
    /**************************/
    ROS_INFO("[Path_plan] Calculating pose of each points...");
    geometry_msgs::PoseArray poses;
    ros::Publisher Pub;
    Pub = _nh->advertise<geometry_msgs::PoseArray>("Scan_Pose", 1);

    std::string poses_file_name = data_save_path + "/ScanPoses_" + std::to_string(num) + ".txt";
    std::ofstream output_ScanPoses(poses_file_name.c_str());

    for(int i=0;i<param_vec.size();i++){
        double parvalue[2] = {(param_vec[i])(0), (param_vec[i])(1)}; // 点的索引参数
        static int leftknot1=0, leftknot2=0;                         // 在顺序搜索中用于加快搜索
        double k1,k2;                                                // 主曲率
        double d1[2],d2[2];                                          // 切线向量（参数域下）
        double derive[9];                                            // 位置与导数
        double normal[3];                                            // 法向量
        int stat;
        s2542(surface, 0, 0, 0, parvalue, &leftknot1, &leftknot2, &k1, &k2, d1, d2, &stat); // 求解主曲率及其切线
        s1421(surface, 1, parvalue, &leftknot1, &leftknot2, derive, normal, &stat);         // 求解导数与法向量
        // 获取旋转矩阵
        Eigen::Matrix3d RotMat;

        // x轴
        int inv = (normal[0]>0)?1:-1;
        double normal_length = std::sqrt(std::pow(normal[0],2) + std::pow(normal[1],2) + std::pow(normal[2],2) );
        RotMat(0,0) = inv * normal[0] / normal_length;
        RotMat(1,0) = inv * normal[1] / normal_length;
        RotMat(2,0) = inv * normal[2] / normal_length;

        // z轴
        if(abs(k1) < abs(k2)){
            RotMat(0,2) = -(inv * d1[0]*derive[3] + d1[1]*derive[6]);
            RotMat(1,2) = -(inv * d1[0]*derive[4] + d1[1]*derive[7]);
            RotMat(2,2) = -(inv * d1[0]*derive[5] + d1[1]*derive[8]);
        }
        else{
            RotMat(0,2) = -(inv * d2[0]*derive[3] + d2[1]*derive[6]);
            RotMat(1,2) = -(inv * d2[0]*derive[4] + d2[1]*derive[7]);
            RotMat(2,2) = -(inv * d2[0]*derive[5] + d2[1]*derive[8]);
        }

        // y轴
        RotMat(0,1) = RotMat(1,2) * RotMat(2,0) - RotMat(2,2) * RotMat(1,0);
        RotMat(1,1) = RotMat(2,2) * RotMat(0,0) - RotMat(0,2) * RotMat(2,0);
        RotMat(2,1) = RotMat(0,2) * RotMat(1,0) - RotMat(1,2) * RotMat(0,0);
        
        Eigen::Quaterniond q = Eigen::Quaterniond(RotMat);//旋转矩阵转为四元数
        q.normalize();

        geometry_msgs::Pose pose;
        pose.position.x = derive[0];
        pose.position.y = derive[1];
        pose.position.z = derive[2];
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();

        poses.poses.push_back(pose);
        output_ScanPoses << derive[0] << " " << derive[1] << " " << derive[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    }
    output_ScanPoses.close();

    poses.header.frame_id = "world";
    poses.header.stamp = ros::Time::now();
    for(int k=0;k<5;k++){
        Pub.publish(poses);
        usleep(1000*100);
    }

    if(surface) freeSurf(surface); // 释放曲面对象
    for(int i=0;i<curve_list.size();i++) if(curve_list[i]) freeCurve(curve_list[i]); // 释放所有曲线对象
    std::cout << std::endl;
    return true;
}

void snake_arm_visual::poses_array_reachable_check(){
    ROS_INFO("[Reachable_check] begin ========= ");
    snake_arm_msg::EndPoseGo EndPoseReqMsg;
    ros::ServiceClient EndPoseReq = _nh->serviceClient<snake_arm_msg::EndPoseGo>("EndposeExp");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr reachable_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    while(1){

        // 选择需要处理的对象
        int select_num;
        bool get_right_num = false, break_flag = false; // 获取正确的编号、跳出该函数的循环
        std::ifstream fposes;
        do{
            usleep(1000*500);
            std::cout << "[Reachable_check] Select an object to deal with('0' to break.): ";
            std::cin >> select_num;
            if(select_num == 0) {
                break_flag = true;
                break;
            }
            else{
                fposes.open(data_save_path+"/ScanPoses_" + std::to_string(select_num) + ".txt");
                if (!fposes) {
                    std::cerr << "[Reachable_check] cannot find pose file" << std::endl;
                    continue;
                }
                else
                    get_right_num = true;
            }
        }while(!get_right_num);
        if(break_flag) break;

        int poses_num = 0;
        std::string temp;
        while(std::getline(fposes, temp,'\n')) poses_num++;
        fposes.close(); fposes.open(data_save_path+"/ScanPoses_" + std::to_string(select_num) + ".txt");
    
        // 开始循环发起请求
        int succ_cnt=0;
        std::string poses_file_name = data_save_path + "/ScanPoses_Reachable_" + std::to_string(select_num) + ".txt";
        std::ofstream output_ScanPoses(poses_file_name.c_str());
        for(int i = 0; i < poses_num ; i++ ){

            ROS_INFO("[Reachable_check] EndPoseReqMsg Request %d/%d", i+1, poses_num);

            // 获取位姿
            float data[7] = {0};
            for (int i = 0; i < 7; i++) fposes >> data[i];
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
            if(EndPoseReq.call(EndPoseReqMsg)){
                ROS_DEBUG("[Reachable_check] EndPoseReqMsg Request: succeed!");
                output_ScanPoses << "1" << std::endl;
                succ_cnt++;

                point_rgb.r = 0; point_rgb.g = 255;
            }
            else{
                ROS_WARN("[Reachable_check] EndPoseReqMsg Request: fail!");
                output_ScanPoses << "0" << std::endl;

                point_rgb.r = 255; point_rgb.g = 0;
            }

            // 将可达性压入点云以可视化
            point_rgb.x = data[0]; point_rgb.y = data[1]; point_rgb.z = data[2]; 
            point_rgb.b = 0;
            reachable_cloud->points.push_back(point_rgb);

            if(i%20 == 0) PointCloud_Pub(reachable_cloud, "reachable_cloud");
        }
        ROS_INFO("[Reachable_check] Reachabale Points: %d/%d", succ_cnt, poses_num);
        pcl::io::savePCDFileBinary((data_save_path+"/reachable_cloud_2.pcd").c_str(), *reachable_cloud);
    }

    return;
}

void snake_arm_visual::ON_NurbsSurface_pub(ON_NurbsSurface &surface_in, Eigen::Vector3f obb_size, std::string topic_name, std::string frame_id){
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    mesh_cloud->clear ();

    if (surface_in.KnotCount (0) <= 1 || surface_in.KnotCount (1) <= 1){
        ROS_WARN("[ON_NurbsSurface_pub] ON knot vector empty.");
        return;
    }

    // 生成采样配置
    double x0 = surface_in.Knot (0, 0); // 曲面u向起点
    double y0 = surface_in.Knot (1, 0); // 曲面v向起点

    double width  = surface_in.Knot (0, surface_in.KnotCount (0) - 1) - x0; // 曲面u方向的控制向量长度
    double height = surface_in.Knot (1, surface_in.KnotCount (1) - 1) - y0; // 曲面v方向的控制向量长度
    int resolution_x = obb_size(0) / 0.002; // u向采样的点数
    int resolution_y = obb_size(1) / 0.002; // v向采样的点数
    float dx = width  / resolution_x; // u向采样间隔
    float dy = height / resolution_y; // v向采样间隔

    // 进行采样，获得点云
    pcl::PointXYZ sample_point;
    double point[9];
    for (unsigned j = 0; j <= resolution_y; j++){
        for (unsigned i = 0; i <= resolution_x; i++) {
            surface_in.Evaluate (x0 + i * dx, y0 + j * dy, 1, 3, point);
            sample_point.x = static_cast<float> (point[0]);
            sample_point.y = static_cast<float> (point[1]);
            sample_point.z = static_cast<float> (point[2]);
            mesh_cloud->push_back (sample_point);
        }
    }

    // 发布
    PointCloud_Pub(mesh_cloud,topic_name);
}

void snake_arm_visual::PCAAndOBB_Get(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                     Eigen::Vector3d&          mean                  ,
                                     Eigen::Matrix3d&          eigenvectors          ,
                                     Eigen::Vector3d&          eigenvalues           ,
                                     Eigen::Matrix<float,3,2>& obb_range             ){
                                        
    // 将点云转化为PCA API需要的形式
    pcl::on_nurbs::NurbsDataSurface data;
    for (unsigned i = 0; i < source_cloud->size(); i++) {
        pcl::PointXYZ &p = source_cloud->at(i);
        data.interior.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }
    
    // 调用PCA函数进行主成分分析
    pcl::on_nurbs::NurbsTools::pca(data.interior, mean, eigenvectors, eigenvalues);
    eigenvalues = eigenvalues / static_cast<unsigned> (data.interior.size ()); // seems that the eigenvalues are dependent on the number of points (???)
    // ROS_INFO_STREAM(" mean = " << mean.transpose() << " , eigenvalues = " << eigenvalues.transpose());
    // ROS_INFO_STREAM(" eigenvectors =  \n" << eigenvectors);

    // 基于以上信息，获取包围盒
    Eigen::Vector3d v_max (-DBL_MAX, -DBL_MAX, -DBL_MAX);
    Eigen::Vector3d v_min (DBL_MAX, DBL_MAX, DBL_MAX);
    Eigen::Matrix3d eigenvectors_inv = eigenvectors.inverse ();
    for (unsigned i = 0; i < static_cast<unsigned> (data.interior.size ()); i++) {
        Eigen::Vector3d p (eigenvectors_inv * (data.interior[i] - mean)); // 将点转移到PCA建立的局部坐标系下

        if (p (0) > v_max (0)) v_max (0) = p (0);
        if (p (1) > v_max (1)) v_max (1) = p (1);
        if (p (2) > v_max (2)) v_max (2) = p (2);
        if (p (0) < v_min (0)) v_min (0) = p (0);
        if (p (1) < v_min (1)) v_min (1) = p (1);
        if (p (2) < v_min (2)) v_min (2) = p (2);
    }

    obb_range(0,0) = v_max(0) ; obb_range(0,1) = v_min (0);
    obb_range(1,0) = v_max(1) ; obb_range(1,1) = v_min (1);
    obb_range(2,0) = v_max(2) ; obb_range(2,1) = v_min (2);
    // ROS_INFO("obb_range : x = %f (%f~%f), y = %f (%f~%f), z = %f (%f~%f)", v_max(0)-v_min(0), v_min(0), v_max(0), v_max(1)-v_min(1), v_min(1), v_max(1), v_max(2)-v_min(2), v_min(2), v_max(2));

    return;
}

void snake_arm_visual::PCAAndOBB_Pub( Eigen::Vector3d           mean          ,
                                      Eigen::Matrix3d           eigenvectors  ,
                                      Eigen::Vector3d           eigenvalues   ,
                                      Eigen::Matrix<float,3,2>& obb_range     ,
                                      std::string               topic_name    , 
                                      std::string               frame_id      ){
    
    visualization_msgs::Marker PCA_OBB_Marker;
    ros::Publisher PCA_OBB_Publisher;

    PCA_OBB_Publisher = _nh->advertise<visualization_msgs::Marker>(topic_name, 1);
    PCA_OBB_Marker.ns = "PCA_And_OBB";
    PCA_OBB_Marker.header.frame_id = frame_id;

    // 清空之前的形状
    PCA_OBB_Marker.action = visualization_msgs::Marker::DELETEALL;
    for(int i=0;i<5;i++){
        PCA_OBB_Publisher.publish(PCA_OBB_Marker);
        usleep(1000*70);
    }

    // 发布PCA结果
    PCA_OBB_Marker.scale.x = 0.003; // 箭头粗细
    PCA_OBB_Marker.scale.y = 0.006; // 头部粗细
    PCA_OBB_Marker.scale.z = 0.02 ; // 头部长度
    PCA_OBB_Marker.color.a = 1;
    PCA_OBB_Marker.pose.orientation.w = 1;
    PCA_OBB_Marker.pose.orientation.x = 0;
    PCA_OBB_Marker.pose.orientation.y = 0;
    PCA_OBB_Marker.pose.orientation.z = 0;
    PCA_OBB_Marker.type     = visualization_msgs::Marker::ARROW;
    PCA_OBB_Marker.action   = visualization_msgs::Marker::ADD; // 增加一个对象，或修改已有对象
    PCA_OBB_Marker.lifetime = ros::Duration();

    for(int i=0;i<3;i++){
        PCA_OBB_Marker.id = i;
        PCA_OBB_Marker.color.r = (i==0) ? 1.0f : 0.0f; // 三个轴取不同的颜色
        PCA_OBB_Marker.color.g = (i==1) ? 1.0f : 0.0f; // 三个轴取不同的颜色
        PCA_OBB_Marker.color.b = (i==2) ? 1.0f : 0.0f; // 三个轴取不同的颜色

        geometry_msgs::Point key_point;
        PCA_OBB_Marker.points.clear();
        key_point.x = mean(0);
        key_point.y = mean(1);
        key_point.z = mean(2);
        PCA_OBB_Marker.points.push_back(key_point); // 坐标轴原点
        key_point.x = mean(0) + eigenvectors(0,i) * std::pow(eigenvalues(i),0.2) / 2 ; // 只是为了把长度差异可视化出来
        key_point.y = mean(1) + eigenvectors(1,i) * std::pow(eigenvalues(i),0.2) / 2 ; // 只是为了把长度差异可视化出来
        key_point.z = mean(2) + eigenvectors(2,i) * std::pow(eigenvalues(i),0.2) / 2 ; // 只是为了把长度差异可视化出来
        PCA_OBB_Marker.points.push_back(key_point); // 坐标轴顶点
        PCA_OBB_Marker.header.stamp = ros::Time::now();
        for(int j=0;j<5;j++){
            PCA_OBB_Publisher.publish(PCA_OBB_Marker);
            usleep(1000*50);
        }
    }

    // 发布OBB结果
    PCA_OBB_Marker.id++;
    PCA_OBB_Marker.type = visualization_msgs::Marker::CUBE;
    PCA_OBB_Marker.scale.x = obb_range(0,0) - obb_range(0,1);
    PCA_OBB_Marker.scale.y = obb_range(1,0) - obb_range(1,1);
    PCA_OBB_Marker.scale.z = obb_range(2,0) - obb_range(2,1);
    PCA_OBB_Marker.color.a = 0.2;

    Eigen::Quaterniond q = Eigen::Quaterniond(eigenvectors);//旋转矩阵转为四元数
    q.normalize();
    PCA_OBB_Marker.points.clear();
    PCA_OBB_Marker.pose.position.x    = mean(0) + (obb_range(0,0) + obb_range(0,1))/2;
    PCA_OBB_Marker.pose.position.y    = mean(1) + (obb_range(1,0) + obb_range(1,1))/2;
    PCA_OBB_Marker.pose.position.z    = mean(2) + (obb_range(2,0) + obb_range(2,1))/2;
    PCA_OBB_Marker.pose.orientation.w = q.w()  ;
    PCA_OBB_Marker.pose.orientation.x = q.x()  ;
    PCA_OBB_Marker.pose.orientation.y = q.y()  ;
    PCA_OBB_Marker.pose.orientation.z = q.z()  ;
    PCA_OBB_Marker.header.stamp = ros::Time::now();
    for(int i=0;i<5;i++){
        PCA_OBB_Publisher.publish(PCA_OBB_Marker);
        usleep(1000*50);
    }

    return ;
}

bool snake_arm_visual::Curve_Pub(SISLCurve* curve, std::string topic_name, std::string frame_name, char color, bool hold_on, std::size_t plot_num){ 

    visualization_msgs::Marker Curve_Marker;
    ros::Publisher Curve_Publisher;  // 用于PathCurve的发布
    Curve_Publisher = _nh->advertise<visualization_msgs::Marker>(topic_name, 1);

    Curve_Marker.scale.x = 0.002;
    Curve_Marker.color.a = 1.0;
    Curve_Marker.type = visualization_msgs::Marker::LINE_STRIP;
    Curve_Marker.pose.orientation.w = 1.0;
    Curve_Marker.pose.orientation.x = 0.0;
    Curve_Marker.pose.orientation.y = 0.0;
    Curve_Marker.pose.orientation.z = 0.0;

    static int Curve_Marker_id=0;
    if(hold_on){
        Curve_Marker_id++;
    }
    else if(Curve_Marker_id!=0){
        Curve_Marker_id = 0;
        Curve_Marker.action = visualization_msgs::Marker::DELETEALL;
        for(int i=0;i<5;i++){
            Curve_Publisher.publish(Curve_Marker);
            usleep(1000*80);
        }
    }
    Curve_Marker.id = Curve_Marker_id;

    // 通过markers的形式进行话题发布 
    Curve_Marker.header.frame_id = frame_name;
    Curve_Marker.header.stamp = ros::Time::now();
    Curve_Marker.ns = topic_name;
    Curve_Marker.action = visualization_msgs::Marker::ADD; // 增加一个对象，或修改已有对象
    Curve_Marker.lifetime = ros::Duration();

    // 进行点的采样
    // --------------------
    geometry_msgs::Point sample_points;
    double point[3];

    // 调用s1227
    Curve_Marker.points.clear();
    for (int i = 0; i < plot_num; i++){
        double t = curve->et[curve->ik - 1] + (curve->et[curve->in] - curve->et[curve->ik - 1]) * i / (plot_num - 1.0);
        if(!my_s1227(curve, t, point)){
            ROS_WARN("[Curve_Pub ] Fail to call my_s1227.");
            return false;
        }

        sample_points.x = point[0];
        sample_points.y = point[1];
        sample_points.z = point[2];
        Curve_Marker.points.push_back(sample_points);
    }

    // 颜色
         if(color == 'r'){ Curve_Marker.color.r = 1.0f; Curve_Marker.color.g = 0.0f; Curve_Marker.color.b = 0.0f; }
    else if(color == 'g'){ Curve_Marker.color.r = 0.0f; Curve_Marker.color.g = 1.0f; Curve_Marker.color.b = 0.0f; }
    else if(color == 'b'){ Curve_Marker.color.r = 0.0f; Curve_Marker.color.g = 0.0f; Curve_Marker.color.b = 1.0f; }
    else if(color == 'w'){ Curve_Marker.color.r = 1.0f; Curve_Marker.color.g = 1.0f; Curve_Marker.color.b = 1.0f; }
    else if(color == 'k'){ Curve_Marker.color.r = 0.0f; Curve_Marker.color.g = 0.0f; Curve_Marker.color.b = 0.0f; }
    else                 { Curve_Marker.color.r = 0.0f; Curve_Marker.color.g = 0.0f; Curve_Marker.color.b = 0.0f; }

    // 发布topic
    for(int i=0;i<5;i++){
        Curve_Publisher.publish(Curve_Marker);
        usleep(1000*100);
    }
}

bool snake_arm_visual::ON_NurbsSurface2g2file(ON_NurbsSurface& m_nurbs, std::string file_name){
    std::ofstream fout;
    if(m_nurbs.IsValid()){
        fout.open(file_name.c_str(),std::ios::out);

        fout << 200 << ' ' << 1 << ' ' << 0 << " 0\n"; // SURFACE_INSTANCE_TYPE MAJOR_VERSION MINOR_VERSION
        fout << m_nurbs.Dimension() << ' ' << m_nurbs.m_is_rat << '\n';

        fout << m_nurbs.m_cv_count[0] << ' ' << m_nurbs.m_order[0] << '\n';
        fout << (m_nurbs.m_knot[0])[0] << ' ';
        for(int i=0;i<m_nurbs.m_cv_count[0]+m_nurbs.m_order[0]-2;i++)
            fout << (m_nurbs.m_knot[0])[i] << ' ';
        fout << (m_nurbs.m_knot[0])[m_nurbs.m_cv_count[0]+m_nurbs.m_order[0]-3] << ' ';
        fout << '\n';

        fout << m_nurbs.m_cv_count[1] << ' ' << m_nurbs.m_order[1] << '\n';
        fout << (m_nurbs.m_knot[1])[0] << ' ';
        for(int i=0;i<m_nurbs.m_cv_count[1]+m_nurbs.m_order[1]-2;i++)
            fout << (m_nurbs.m_knot[1])[i] << ' ';
        fout << (m_nurbs.m_knot[1])[m_nurbs.m_cv_count[1]+m_nurbs.m_order[1]-3] << ' ';
        fout << '\n';

        int cv_count = m_nurbs.m_cv_count[0] * m_nurbs.m_cv_count[1] * (m_nurbs.Dimension() + m_nurbs.m_is_rat);
        for(int i=0;i<cv_count;i++){
            fout << m_nurbs.m_cv[i] << ' ';
        }
        fout << '\n';

        fout.close();
        ROS_INFO("Surface saved: %s", file_name.c_str());
        return true;
    }
    else{
        ROS_WARN("[ON_NurbsSurface2g2file] Input surface is empty!!!");
        return false;
    }
}

bool snake_arm_visual::ON_NurbsCurve2g2file(ON_NurbsCurve& m_nurbs, std::string file_name){
    std::ofstream fout;
    if(m_nurbs.IsValid()){
        fout.open(file_name.c_str(),std::ios::out);

        fout << 100 << ' ' << 1 << ' ' << 0 << " 0\n"; // CURVE_INSTANCE_TYPE MAJOR_VERSION MINOR_VERSION
        fout << m_nurbs.Dimension() << ' ' << m_nurbs.m_is_rat << '\n';

        fout << m_nurbs.m_cv_count << ' ' << m_nurbs.m_order << '\n';
        fout << (m_nurbs.m_knot)[0] << ' ';
        for(int i=0;i<m_nurbs.m_cv_count+m_nurbs.m_order-2;i++)
            fout << (m_nurbs.m_knot)[i] << ' ';
        fout << (m_nurbs.m_knot)[m_nurbs.m_cv_count+m_nurbs.m_order-3] << ' ';
        fout << '\n'; 

        int cv_count = m_nurbs.m_cv_count * (m_nurbs.Dimension() + m_nurbs.m_is_rat);
        for(int i=0;i<cv_count;i++){
            fout << m_nurbs.m_cv[i] << ' ';
        }
        fout << '\n';

        fout.close();
        ROS_INFO("Fit curve result saved: %s", file_name.c_str());
        return true;
    }
    else{
        ROS_WARN("[ON_NurbsCurve2g2file] Input surface is empty!!!");
        return false;
    }
}

void snake_arm_visual::PointsCloudSave_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
    ROS_INFO("Have captured a frame, filter...");

    // 类型转换
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *colorcloud);
    colorcloud->is_dense = false;

    // 滤除离群点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
    statistical_filter.setMeanK(50);
    statistical_filter.setStddevMulThresh(statistical_filter_StddevMulThresh);
    statistical_filter.setInputCloud(colorcloud);
    statistical_filter.filter(*tmp);

    // 体素滤波
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setLeafSize(capture_resloution, capture_resloution, capture_resloution);
    voxel_filter.setInputCloud(tmp);
    voxel_filter.filter(*colorcloud);

    // 去除无效点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*colorcloud, indices);
    colorcloud->is_dense = true;

    // 一个奇奇怪怪的相机坐标系定义带来的坐标变换
    Eigen::Isometry3d Trans = Eigen::Isometry3d::Identity();
    Trans.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 0, 1)));
    Trans.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1, 0, 0)));
    pcl::transformPointCloud(*colorcloud, *colorcloud, Trans.matrix());
    
    // 输出
    std::string file_name = data_save_path + "/capture_" + std::to_string(data_save_cnt+1) + ".pcd";
    pcl::io::savePCDFileBinary(file_name.c_str(), *colorcloud);
    return;
}

bool snake_arm_visual::my_s1424(SISLSurf* surface, double* pointpar, double* posi){

    static int interval_u=0;
    static int interval_v=0;
    int jstat;

    s1424(surface,          // input surface
          0,                // evaluate position only (no derivatives in u-param)
          0,                // evaluate position only (no derivatives in v-param)
          pointpar,         // parameter values
          &interval_u,      // returns used u-interval
          &interval_v,      // returns used v-interval
          posi,             // the calculated position
          &jstat);          // status variable 

    if(jstat != 0) return false;
    else return true;
}

bool snake_arm_visual::my_s1227(SISLCurve* curve, double parvalue, double* point){

    static int leftknot=0;
    int jstat;

    s1227(curve,            // input surface
          0,                // evaluate position only (no derivatives)
          parvalue,         // parameter values
          &leftknot,        // returns used u-interval
          point,            // the calculated position
          &jstat);          // status variable 

    if(jstat != 0) return false;
    else return true;
}
