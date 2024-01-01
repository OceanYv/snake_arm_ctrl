/**
 * @file inverse_kinematic_ctrl.cpp
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2023-02-20
 * 
 * @copyright Copyright (c) 2023 OceanYv
 * 
 */

#include <inverse_kinematic_ctrl.h>

inverse_kinematic_ctrl::inverse_kinematic_ctrl(ros::NodeHandle *nh){
    
    /**************/
    /* 获取配置参数 */
    /**************/
    STATE = true;
    _nh = nh;

    printf("\n");
    ROS_INFO(">>>inverse_kinematic_ctrl config information");
    ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

    // 获取参数
    // -------------------------------------------------------------

    // 系统参数
    BasePosi_Savefile   = _nh->param<std::string>("/base_posi_savefile", "/home/ocici/ROS_CODE/3_snake_arm/src/snake_base_ctrl/config/base_posi_savefile.txt");
    JointAngle_Savefile = _nh->param<std::string>("/pos_kinematic/SnakeArm/JointAngle_Savefile", "/home/ocici/ROS_CODE/3_snake_arm/src/pos_kinematic_ctrl/config/joint_angle_savefile.txt");
    PathCurve_CtrlPoints_Savefile = _nh->param<std::string>("/inverse_kinematic_ctrl/SnakeArm/PathCurve_CtrlPoints_Savefile", "/home/ocici/ROS_CODE/3_snake_arm/src/inverse_kinematic_ctrl/config/PathCurve_CtrlPoints.g2");
    simulation          = _nh->param<bool>("/inverse_kinematic_ctrl/simulation",true);
    
    // 雅可比算法参数
    EndPose_Tolerance_ang  = _nh->param<float>("/inverse_kinematic_ctrl/Jacobian/EndPose_Tolerance_ang",0.15);
    EndPose_Tolerance_posi = _nh->param<float>("/inverse_kinematic_ctrl/Jacobian/EndPose_Tolerance_posi",1.0);
    max_step_ang           = _nh->param<float>("/inverse_kinematic_ctrl/Jacobian/max_step_ang",20.0) / 180.0 * M_PI;
    max_step_posi          = _nh->param<float>("/inverse_kinematic_ctrl/Jacobian/max_step_posi",200.0);
    step_coef_min          = _nh->param<float>("/inverse_kinematic_ctrl/Jacobian/step_coef_min",1.1);
    singular_coef          = _nh->param<float>("/inverse_kinematic_ctrl/Jacobian/singular_coef",0.1);
    weight_def             = parseVVF<Eigen::Matrix<double, 2*JOINT_NUM+1, 1>>(_nh->param<std::string>("/inverse_kinematic_ctrl/Jacobian/weight_def","[[1]]"));
    weight_joint_threshold = _nh->param<float>("/inverse_kinematic_ctrl/Jacobian/weight_joint_threshold", 90*MAX_JOINTANGLE/M_PI) / 180.0 * M_PI;
    JacoIterateCNTMax = _nh->param<int>("/inverse_kinematic_ctrl/Jacobian/JacoIterateCNTMax",200);
    regression_solu_on     = _nh->param<bool>("/inverse_kinematic_ctrl/Jacobian/regression_solu_on",true);

    // FTL法
    WrappedBall_Min      = std::sqrt(std::pow(FIX_POLE_PART_LENGTH,2) + std::pow(ROT_SHAFT_PART_LENGTH,2) + 2*FIX_POLE_PART_LENGTH*ROT_SHAFT_PART_LENGTH*std::cos(MAX_JOINTANGLE));
    WrappedBall_Max      = FIX_POLE_PART_LENGTH + ROT_SHAFT_PART_LENGTH;
    Inter_Toler          = _nh->param<float>("/inverse_kinematic_ctrl/FTL/Inter_Toler",0.005);
    Ftl_End_Toler        = _nh->param<float>("/inverse_kinematic_ctrl/FTL/Ftl_End_Toler",0.01);
    FeedCoef             = _nh->param<float>("/inverse_kinematic_ctrl/FTL/FeedCoef",1.0);
    FTLBaseIterateCNTMax = _nh->param<int>("/inverse_kinematic_ctrl/Jacobian/FTLBaseIterateCNTMax",100);
    FTLEndIterateCNTMax  = _nh->param<int>("/inverse_kinematic_ctrl/Jacobian/FTLEndIterateCNTMax",200);
    VirtualPathCurvMax   = 1.0/(_nh->param<float>("/inverse_kinematic_ctrl/FTL/VirtualPathCurvMax",237.0));
    FTL_Part_Num         = (_nh->param<int>("/inverse_kinematic_ctrl/FTL/FTL_Part_Num",JOINT_NUM));

    // 混合法
    VirtualJointWeight_def = _nh->param<float>("/inverse_kinematic_ctrl/Hybrid/VirtualJointWeight_def",55);
    Hybird_singular_coef   = _nh->param<float>("/inverse_kinematic_ctrl/Hybrid/Hybird_singular_coef",0.1);
    Hyb_Part_Num           = _nh->param<int>("/inverse_kinematic_ctrl/FTL/Hyb_Part_Num",JOINT_NUM-4);

    // 判断参数是否合法
    if(FTL_Part_Num > JOINT_NUM) FTL_Part_Num = JOINT_NUM;
    else if(FTL_Part_Num < 1) FTL_Part_Num = 1;

    if(Hyb_Part_Num > JOINT_NUM-3) Hyb_Part_Num = JOINT_NUM-3;
    else if(Hyb_Part_Num < 0) Hyb_Part_Num = 0;

    if(Ftl_End_Toler > EndPose_Tolerance_posi/2) Ftl_End_Toler = EndPose_Tolerance_posi/2; // 为了确保混合法的收敛，必须使FTL法的运动精度高于Jaco法
    if(Inter_Toler > Ftl_End_Toler) Inter_Toler =Ftl_End_Toler;

    // 输出参数到终端
    printf("\n");
    ROS_INFO_STREAM("BasePosi_Savefile: " << BasePosi_Savefile);
    ROS_INFO_STREAM("JointAngle_Savefile: " << JointAngle_Savefile);
    ROS_INFO_STREAM("PathCurve_CtrlPoints_Savefile: " << PathCurve_CtrlPoints_Savefile);
    ROS_INFO_STREAM("simulation         : " << simulation );

    printf("\n");
    ROS_INFO("EndPose_Tolerance_ang  = %10.5f", EndPose_Tolerance_ang );
    ROS_INFO("EndPose_Tolerance_posi = %10.5f", EndPose_Tolerance_posi);
    ROS_INFO("max_step_ang         = %10.5f", max_step_ang       );
    ROS_INFO("max_step_posi        = %10.5f", max_step_posi      );
    ROS_INFO("step_coef_min        = %10.5f", step_coef_min      );
    ROS_INFO("singular_coef        = %10.5f", singular_coef      );
    ROS_INFO_STREAM("weight_def: " << weight_def.transpose()     );
    ROS_INFO("weight_joint_threshold = %10.5f", weight_joint_threshold);
    ROS_INFO("JacoIterateCNTMax    = %10d"  , JacoIterateCNTMax  );
    ROS_INFO("regression_solu_on   = %10d"  , regression_solu_on );
    printf("\n");
    ROS_INFO("WrappedBall_Min      = %10.5f", WrappedBall_Min    );
    ROS_INFO("WrappedBall_Max      = %10.5f", WrappedBall_Max    );
    ROS_INFO("Inter_Toler          = %10.5f", Inter_Toler        );
    ROS_INFO("Ftl_End_Toler        = %10.5f", Ftl_End_Toler      );
    ROS_INFO("FeedCoef             = %10.5f", FeedCoef           );
    ROS_INFO("FTLBaseIterateCNTMax = %10d", FTLBaseIterateCNTMax );
    ROS_INFO("FTLEndIterateCNTMax  = %10d", FTLEndIterateCNTMax  );
    ROS_INFO("VirtualPathCurvMax   = %10.5f", 1.0/VirtualPathCurvMax );
    ROS_INFO("FTL_Part_Num         = %10d", FTL_Part_Num         );
    printf("\n");
    ROS_INFO("VirtualJointWeight_def  = %10.5f", VirtualJointWeight_def);
    ROS_INFO("Hybird_singular_coef    = %10.5f", Hybird_singular_coef  );
    ROS_INFO("Hyb_Part_Num            = %10d"  , Hyb_Part_Num          );

    /************/
    /* 初始化对象 */
    /************/

    CallService_Flag = false;

    // 话题发布相关变量
    // ------------------------------------
    Curve_Publisher = _nh->advertise<visualization_msgs::Marker>("Curve_Pub", 1);
    Arm_Publisher = _nh->advertise<visualization_msgs::Marker>("Arm_line_Pub", 1);
    Points_Publisher = _nh->advertise<visualization_msgs::Marker>("Points_Pub", 1);
    Poses_Publisher = _nh->advertise<geometry_msgs::PoseArray>("Virtual_Poses_Pub", 1);

    Curve_Marker.scale.x = 0.005;
    Curve_Marker.color.a = 1.0;
    Curve_Marker.type = visualization_msgs::Marker::LINE_STRIP;
    Curve_Marker.pose.orientation.w = 1.0;
    Curve_Marker.pose.orientation.x = 0.0;
    Curve_Marker.pose.orientation.y = 0.0;
    Curve_Marker.pose.orientation.z = 0.0;
    Curve_Marker.header.frame_id = "world";

    Arm_Marker.scale.x = 0.003;
    Arm_Marker.color.a = 1.0;
    Arm_Marker.type = visualization_msgs::Marker::LINE_STRIP;
    Arm_Marker.pose.orientation.w = 1.0;
    Arm_Marker.pose.orientation.x = 0.0;
    Arm_Marker.pose.orientation.y = 0.0;
    Arm_Marker.pose.orientation.z = 0.0;
    Arm_Marker.header.frame_id = "world";

    Points_Marker.scale.x = 0.01;
    Points_Marker.scale.y = 0.01;
    Points_Marker.color.a = 1.0;
    Points_Marker.type = visualization_msgs::Marker::POINTS;

    // 初始化机械臂关节参数与基座位置参数
    // --------------------------------------
    ROS_INFO("[inverse_kinematic_ctrl] Reading Arm joint data and base position.");

    // 从base_ctrl包中读取上次停机时的基座位置
    std::ifstream BasePosi_Read(BasePosi_Savefile);
    if(!BasePosi_Read){
        ROS_WARN_STREAM("Fail to open file : "<< BasePosi_Savefile);
        BasePosiCur = 0;
    }
    else {BasePosi_Read >> BasePosiCur; BasePosiCur *= 1000;}
    BasePosi_Read.close();

    // 从pos_kinematic_ctrl包中读取上次停机时的机械臂关节角
    if(file_to_array(JointAngle_Savefile, JointsAngleCur) != true) // 若文件不存在，则默认为0
        JointsAngleCur = Arrayf_2_Joint::Zero();
    JointsAngleCur = JointsAngleCur/180*M_PI;

    std::cout << "BasePosiCur: 1 mm * " << BasePosiCur << std::endl;
    std::cout << "JointsAngleCur: 1 degree * \n" << JointsAngleCur*180/M_PI << std::endl << std::endl;

    // 初始化Joint2World_Trans
    sleep(2);
    ROS_INFO("[inverse_kinematic_ctrl] Calculating Joint2World_Trans.");
    Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3> Joint2World_Trans; //存放从各个坐标系（base、joint0～joint16、end）到world的变换，用于可视化
    Position_Solution(BasePosiCur, JointsAngleCur, Joint2World_Trans);
    Arm_Position_Pub(Joint2World_Trans);

    // 从文件读入曲线参数，或通过Curve_Points_Init获取曲线参数
    // ----------------------------------------------------
    ROS_INFO("[inverse_kinematic_ctrl] Reading curve/curve_points config file.");
    std::ifstream read_points(PathCurve_CtrlPoints_Savefile.c_str());
    for(int i=0;i<PATHCURVE_CTRLPOINTS_MAXNUM;i++)PathCurve_CtrlPoints_type[i]=1;

    // 判断文件是否打开成功，如果出问题的话需要给出警告
    if (!read_points) {
        ROS_WARN("Unable to open curve/curve_points config file!");
        read_points.close();
    }

    // // 由用户选择使用文件中的曲线数据，还是根据当前机械臂姿态重新生成初始曲线
    if(!read_points) { 
        ROS_INFO("File to read curve from file, generate Curve with current arm pose(y) or exit the program(n)?");
        if(Confirm_Interaction())
            ReGeneTheCurve(BasePosiCur,JointsAngleCur);
        else {
            STATE = false;
            return;
        }
    }
    else {
        std::vector<double> points;
        readGoPoints(points, read_points);
        PathCurve_CtrlPoints_Num = int(points.size()) / 3;
        for (int i = 0; i < 3*PathCurve_CtrlPoints_Num; ++i)
            PathCurve_CtrlPoints[i] = points[i];
        if(!my_s1356(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,PathCurve_CtrlPoints_type,PathCurve))
            ROS_WARN("Fail to gen the curve.");
    }
        
    printf("\n");ROS_INFO("PathCurve_CtrlPoints_Num: %d",PathCurve_CtrlPoints_Num);

    // 将曲线发布以通过rviz显示；
    sleep(1);
    Curve_Pub(PathCurve,"PathCurve","world");
    Points_Pub(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,"PathCurve_CtrlPoints","world");

    Listen_mode = false;

    /*********/
    /* debug */
    /*********/

    #ifdef TEST
        ROS_INFO("[inverse_kinematic_ctrl] ==test temp==");
    #endif

    /********/
    /* test */
    /********/
    
}

inverse_kinematic_ctrl::~inverse_kinematic_ctrl(){
    // 释放曲线对象
    if (PathCurve) freeCurve(PathCurve);
}

bool inverse_kinematic_ctrl::inverse_kinematic_ctrl_cmd(){

    printf("\n=========================================================\n");
    printf("====================Command Mode=========================\n");
    printf("=========================================================\n");
    std::string cmd;

    while(1){
        printf(" \n======== Select the Control Mode, or exit the program ======== \n");
        printf(" === [GC ] to adjust the Ridge Curve.\n");
        printf(" === [JK ] to adopt method \"Jacobian Diff\" , which controls the pose of the arm end.\n");
        printf(" === [FTL] to adopt method \"Follow The Leader\", by which the arm can move along the ridge curve.\n");
        printf(" === [HY ] to adopt method Hybrid, which controls the pose of the arm end while arm moves along the ridge curve.\n");
        printf(" === [RR ] Arm Respond to requests from ros service.\n");
        printf(" === [Q  ] to exit the inverse_kinematic_ctrl program.\n");
        printf("\tCOMMAND: "); std::cin >> cmd;

        if(cmd == "GC"){
            Adjust_Curve(BasePosiCur, JointsAngleCur);
        }
        else if(cmd == "JK"){
            JacobianDiffInvKine();
        }
        else if(cmd == "FTL"){
            FTLInvKine(BasePosiCur, JointsAngleCur);
        }
        else if(cmd == "HY"){
            JacobianDiffInvKine(true);
        }
        else if(cmd == "RR"){
            ROS_INFO("Enter listening mode ... ");
            Listen_mode = true;

            std::string rr_cmd;
            do{
                sleep(1);
                ROS_INFO("To exit the listening mode, input \"Q\".");
                std::cin >> rr_cmd;
            }while(rr_cmd != "Q");

            Listen_mode = false;
        }
        else if(cmd == "Q"){
            ROS_INFO("Exiting...");
            break;
        }
        else ROS_WARN("No such command!!!");
    };
    STATE = false;
    return true;
}

bool inverse_kinematic_ctrl::Adjust_Curve(double BasePosi, Arrayf_2_Joint JointsAngle){

    std::string cmd;
    std::ofstream write_points;

    Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3> Joint2World_Trans; //存放从各个坐标系（base、joint0～joint16、end）到world的变换，用于输出机械臂姿态;
    Position_Solution(BasePosi, JointsAngle, Joint2World_Trans);

    while(1){
        printf(" \n==================================================\n");
        printf(" ===================[Adjust_Curve]=================\n");
        printf(" === [GEN] to generate new curve with curent arm psoe.\n");
        printf(" === [CUR] to adjust the curve by control move of the end.\n");
        printf(" === [PNT] to adjust the curve by adjust selected control point.\n");
        printf(" === [Q  ] to exit the Adjust_Curve program.\n");
        printf("\tCOMMAND: "); std::cin >> cmd;

        if(cmd == "GEN"){
            ReGeneTheCurve(BasePosi,JointsAngle);
        }
        else if(cmd == "CUR"){
            
            double BasePosi_Virtual;            // 虚拟基部位置, 用于调整过程中的可视化，不影响实际值, 单位：mm
            Arrayf_2_Joint JointsAngle_Virtual; // 虚拟关节角  ，用于调整过程中的可视化，不影响实际值，单位：弧度

            double BasePosi_temp;
            Arrayf_2_Joint JointsAngle_temp = Arrayf_2_Joint::Zero();

            // 先确保机械臂在脊线上
            if(!FTLInvKine_BaseMove(0, FTL_Part_Num, 0, BasePosi_Virtual, JointsAngle_Virtual)){ 
                ROS_WARN("[Adjust_Curve] Fail to call function FTLInvKine_BaseMove.");
                continue;
            }
            // 运动到末端
            Position_Solution(BasePosi_Virtual, JointsAngle_Virtual, Joint2World_Trans_Virtual); // 获取目前的末端位置
            double dist = sqrt( pow(( (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(0,3) - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-3] ), 2) + 
                                pow(( (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(1,3) - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-2] ), 2) + 
                                pow(( (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(2,3) - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-1] ), 2) );
            if (!FTLInvKine_EndMove(BasePosi_Virtual, FTL_Part_Num, dist, BasePosi_temp, JointsAngle_temp)) {
                ROS_WARN("[Adjust_Curve] Fail to call function FTLInvKine_EndMove.");
                continue;
            }
            BasePosi_Virtual = BasePosi_temp;  JointsAngle_Virtual = JointsAngle_temp;
            Position_Solution(BasePosi_Virtual, JointsAngle_Virtual, Joint2World_Trans_Virtual);
            Arm_Position_Pub(Joint2World_Trans_Virtual);

            // 初始化虚拟路径为末端前方100mm的线段，并加入临时控制点、虚拟控制点
            PathCurve_CtrlPoints_Num += 2;
            PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-6] = PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-9] + 50*(Joint2World_Trans_Virtual(2*FTL_Part_Num+2).matrix())(0,0);
            PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-5] = PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-8] + 50*(Joint2World_Trans_Virtual(2*FTL_Part_Num+2).matrix())(1,0);
            PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-4] = PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-7] + 50*(Joint2World_Trans_Virtual(2*FTL_Part_Num+2).matrix())(2,0);
            PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-3] = PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-6] + 50*(Joint2World_Trans_Virtual(2*FTL_Part_Num+2).matrix())(0,0);
            PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-2] = PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-5] + 50*(Joint2World_Trans_Virtual(2*FTL_Part_Num+2).matrix())(1,0);
            PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-1] = PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-4] + 50*(Joint2World_Trans_Virtual(2*FTL_Part_Num+2).matrix())(2,0);

            // 生成曲线并可视化
            if(!my_s1356(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,PathCurve_CtrlPoints_type,PathCurve)){
                ROS_ERROR("Fail to gen the curve.");
                continue;
            }
            else{
                Curve_Pub(PathCurve,"PathCurve","world");
                Points_Pub(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,"PathCurve_CtrlPoints","world");
            }

            // 进入命令行状态
            // -----------------------------
            bool quit_flag = false, change_flag = false, move_flag = false;
            double curvature = 0;
            int speed = 10, pitch = 0, length = 100, BaseDist_Exp=0;
            printf("\n === [w/s] to control the arm move along the path.\n");
            printf(" === [a/d] to control the pitch of the virtual path, 0~360.\n");
            printf(" === [e/q] to control the curvature of the virtual path.\n");
            printf(" === [u/j] to control the length of the virtual path, min 10.\n");
            printf(" === [ X ] to set distance per press.\n");
            printf(" === [ Q ] to exit the curve expend program.\n");
            while(1){
                printf("Curve Expend Command:");
                switch(scanKeyboard()){
                    case('w'):  BaseDist_Exp =  speed; move_flag=true; change_flag=true; break;
                    case('s'):  BaseDist_Exp = -speed; move_flag=true; change_flag=true; break;
                    case('a'):  pitch += speed; ROS_INFO("pitch = %d deg. ", pitch); change_flag=true; break;
                    case('d'):  pitch -= speed; ROS_INFO("pitch = %d deg. ", pitch); change_flag=true; break;
                    case('e'):  curvature += 0.0001*speed; ROS_INFO("curvature = %f. ", curvature); change_flag=true; break;
                    case('q'):  curvature -= 0.0001*speed; ROS_INFO("curvature = %f. ", curvature); change_flag=true; break;
                    case('u'):  length += speed; change_flag=true; break;
                    case('j'):  length -= speed; change_flag=true; break;
                    case('X'):
                        printf("Current speed: %d mm/deg/0.0001m^-1 per press, input the new speed(int 1~100):",speed);
                        std::cin >> speed;
                        if(speed>100) speed = 100;
                        if(speed>(length*0.8)) speed = length*0.8; else if(speed<1) speed=1;
                        printf("Current speed: %d.\n",speed);
                        break;
                    case('Q'):
                        // 生成最终曲线，并可视化
                        PathCurve_CtrlPoints_Num -= 1;
                        if(!my_s1356(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,PathCurve_CtrlPoints_type,PathCurve)){
                            ROS_ERROR("Fail to gen the curve.");
                            break;
                        }
                        Curve_Pub(PathCurve,"PathCurve","world");
                        Points_Pub(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,"PathCurve_CtrlPoints","world");
                        write_points.open(PathCurve_CtrlPoints_Savefile.c_str(),std::ios::out);
                        writeGoPoints(PathCurve_CtrlPoints_Num, PathCurve_CtrlPoints, write_points);
                        write_points.close();
                        quit_flag = true; break;
                    default:
                        change_flag=false; ROS_WARN("No such command!!!\n");
                }
                if(quit_flag) break;

                if(move_flag){
                    // 运动
                    if(!FTLInvKine_BaseMove(BasePosi_Virtual, FTL_Part_Num, BaseDist_Exp, BasePosi_temp, JointsAngle_temp)){
                        ROS_WARN("[FTLInvKine_EndMove] Fail to call function FTLInvKine_BaseMove. BaseDist_Exp = %f",(double)BaseDist_Exp);
                        continue;
                    }
                    BasePosi_Virtual = BasePosi_temp;  JointsAngle_Virtual = JointsAngle_temp;
                    Position_Solution(BasePosi_Virtual, JointsAngle_Virtual, Joint2World_Trans_Virtual);
                    Arm_Position_Pub(Joint2World_Trans_Virtual);

                    move_flag = false;
                }

                // 更新临时控制点和虚拟控制点
                if(change_flag){
                    // 检验各个配置是否超范围
                    if(length<10) length = 10;
                    if(pitch>=360) pitch -= 360; else if(pitch<0) pitch +=360;
                    if(curvature<-VirtualPathCurvMax) curvature=-VirtualPathCurvMax;
                    else if(curvature>VirtualPathCurvMax) curvature=VirtualPathCurvMax;

                    // 检查末端点与曲线（除临时控制点、虚拟控制点外）最后一个控制点的位置关系，以确定是否进行点的增减
                    double dist1,dist2,dist3;
                    dist1 = sqrt( pow(( (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(0,3) - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-6] ), 2) + 
                                  pow(( (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(1,3) - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-5] ), 2) + 
                                  pow(( (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(2,3) - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-4] ), 2) );
                    dist2 = sqrt( pow(( (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(0,3) - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-9] ), 2) + 
                                  pow(( (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(1,3) - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-8] ), 2) + 
                                  pow(( (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(2,3) - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-7] ), 2) );
                    dist3 = sqrt( pow(( PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-6] - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-9] ), 2) + 
                                  pow(( PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-5] - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-8] ), 2) + 
                                  pow(( PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-4] - PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-7] ), 2) );

                    if(dist1 > dist2 && (dist1 - dist3)>-1 ) {
                        PathCurve_CtrlPoints_Num--; // 需要删除点
                        ROS_INFO("PathCurve_CtrlPoints_Num--, Num = %d",PathCurve_CtrlPoints_Num);
                    }
                    else if(dist2 > 200) {
                        PathCurve_CtrlPoints_Num++; // 需要增加点，直接将本次运动前末端的位置作为新的点 
                        ROS_INFO("PathCurve_CtrlPoints_Num++, Num = %d",PathCurve_CtrlPoints_Num);
                    }

                    // 用末端位置作为临时控制点
                    PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-6] = (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(0,3);
                    PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-5] = (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(1,3);
                    PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-4] = (Joint2World_Trans_Virtual(2*FTL_Part_Num+2))(2,3);

                    // 计算虚拟路径末端点在末端坐标系下的坐标值
                    Eigen::Vector4d sample_point; sample_point(3)=1;
                    if(abs(curvature)<0.0001){ // 曲率为零时的奇异情况
                        sample_point(0) = length;
                        sample_point(1) = 0;
                        sample_point(2) = 0;
                    }
                    else{
                        sample_point(0) =  sin(length * curvature)  / curvature;
                        double temp = (1 - cos(length * curvature)) / curvature;
                        sample_point(1) = cos(pitch/180.0*M_PI) * temp;
                        sample_point(2) = sin(pitch/180.0*M_PI) * temp;
                    }

                    // 变换到world坐标系下,并作为虚拟控制点加入曲线
                    sample_point = Joint2World_Trans_Virtual(2*FTL_Part_Num+2)* sample_point;
                    PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-3] = sample_point(0);
                    PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-2] = sample_point(1);
                    PathCurve_CtrlPoints[3*PathCurve_CtrlPoints_Num-1] = sample_point(2);

                    // 生成曲线并可视化
                    if(!my_s1356(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,PathCurve_CtrlPoints_type,PathCurve))
                        ROS_ERROR("Fail to gen the curve.");
                    else{
                        Curve_Pub(PathCurve,"PathCurve","world");
                        Points_Pub(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,"PathCurve_CtrlPoints","world");
                    }
                    change_flag = false;
                }
            }
        }
        else if(cmd == "PNT"){ // adjust points start
            bool quit_flag = false, move_flag = false;
            int choose_point_num = PathCurve_CtrlPoints_Num;
            int speed=1;
            double point_old[3]={PathCurve_CtrlPoints[choose_point_num*3-3],PathCurve_CtrlPoints[choose_point_num*3-2],PathCurve_CtrlPoints[choose_point_num*3-1]};
            printf("\n === [ c ] to choose a point to adjust.\n");
            printf(" === [w/s] to control the point move along x axis(world coord).\n");
            printf(" === [a/d] to control the point move along y axis(world coord).\n");
            printf(" === [e/q] to control the point move along z axis(world coord).\n");
            printf(" === [A/D] to add a new point at the end/ delete the last point.\n");
            printf(" === [ X ] to set distance per press.\n");
            printf(" === [ C ] cancel change of the current selected point.\n");
            printf(" === [ Q ] to exit the point adjust program.\n");
            while(1){
                printf("Point Adjust Command:");
                switch(scanKeyboard()){
                    case('c'): 
                        // get new point number
                        printf("Current choosen point: %d, input the new point(int 1~%d):",choose_point_num, PathCurve_CtrlPoints_Num);
                        std::cin >> choose_point_num;
                        if(choose_point_num > PathCurve_CtrlPoints_Num)
                            choose_point_num =PathCurve_CtrlPoints_Num;
                        else if(choose_point_num < 1)
                            choose_point_num=1;
                        printf("Current choosen point: %d.\n",choose_point_num);

                        // save old data, for cancel function
                        point_old[0] = PathCurve_CtrlPoints[choose_point_num*3-3];
                        point_old[1] = PathCurve_CtrlPoints[choose_point_num*3-2];
                        point_old[2] = PathCurve_CtrlPoints[choose_point_num*3-1];

                        Points_Pub(PathCurve_CtrlPoints+choose_point_num*3-3,1,"Adjust Point","world",'r'); // Visual
                        break;
                    case('w'): PathCurve_CtrlPoints[choose_point_num*3-3] = PathCurve_CtrlPoints[choose_point_num*3-3] + speed; move_flag=true; break;
                    case('s'): PathCurve_CtrlPoints[choose_point_num*3-3] = PathCurve_CtrlPoints[choose_point_num*3-3] - speed; move_flag=true; break;
                    case('a'): PathCurve_CtrlPoints[choose_point_num*3-2] = PathCurve_CtrlPoints[choose_point_num*3-2] + speed; move_flag=true; break;
                    case('d'): PathCurve_CtrlPoints[choose_point_num*3-2] = PathCurve_CtrlPoints[choose_point_num*3-2] - speed; move_flag=true; break;
                    case('e'): PathCurve_CtrlPoints[choose_point_num*3-1] = PathCurve_CtrlPoints[choose_point_num*3-1] + speed; move_flag=true; break;
                    case('q'): PathCurve_CtrlPoints[choose_point_num*3-1] = PathCurve_CtrlPoints[choose_point_num*3-1] - speed; move_flag=true; break;
                    case('A'): 
                        if(choose_point_num == PathCurve_CtrlPoints_Num) choose_point_num++;
                        PathCurve_CtrlPoints_Num++;
                        PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-3] = PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-6] + 150;
                        PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-2] = PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-5];
                        PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-1] = PathCurve_CtrlPoints[PathCurve_CtrlPoints_Num*3-4];
                        move_flag=true; break;
                    case('D'): 
                        if(choose_point_num == PathCurve_CtrlPoints_Num) choose_point_num--;
                        PathCurve_CtrlPoints_Num--;
                        move_flag=true; break;
                    case('X'):
                        printf("Current speed: %dmm per press, input the new speed(int 1~20):",speed);
                        std::cin >> speed;
                        if(speed>20) speed =20; else if(speed<1) speed=1;
                        printf("Current speed: %d.\n",speed);
                        break;
                    case('C'): 
                        PathCurve_CtrlPoints[choose_point_num*3-3] = point_old[0];
                        PathCurve_CtrlPoints[choose_point_num*3-2] = point_old[1];
                        PathCurve_CtrlPoints[choose_point_num*3-1] = point_old[2];
                        move_flag=true; break;
                    case('Q'): 
                        write_points.open(PathCurve_CtrlPoints_Savefile.c_str(),std::ios::out);
                        writeGoPoints(PathCurve_CtrlPoints_Num, PathCurve_CtrlPoints, write_points);
                        write_points.close();
                        quit_flag = true; break;
                    default:
                        move_flag=false; ROS_WARN("No such command!!!\n");
                }
                if(quit_flag) break;

                if(move_flag){
                    if(!my_s1356(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,PathCurve_CtrlPoints_type,PathCurve))
                        ROS_WARN("Fail to gen the curve.");
                    Curve_Pub(PathCurve,"PathCurve","world");
                    Points_Pub(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,"PathCurve_CtrlPoints","world");
                    Points_Pub(PathCurve_CtrlPoints+choose_point_num*3-3,1,"Adjust Point","world",'r'); // Visual
                    move_flag=false;
                }
            }
        }  // adjust points end
        else if(cmd == "Q"){
            ROS_INFO("Exiting...");
            Arm_Position_Pub(Joint2World_Trans);
            break;
        }
        else ROS_WARN("No such command!!!");
    };
}

bool inverse_kinematic_ctrl::ReGeneTheCurve(double BasePosi, Arrayf_2_Joint JointsAngle){
    
    // 机械臂姿态解算
    Position_Solution(BasePosi,JointsAngle,Joint2World_Trans_Virtual);

    // 确定点的个数
    double base_extend_dist = ( BasePosi>0?BasePosi:0 ) + 450;
    int num = floor(base_extend_dist/150.0);
    if(num > JOINT_NUM+4){ num = JOINT_NUM+5; ROS_WARN("The arm hase been too far. Maybe face some bugs.");}
    PathCurve_CtrlPoints_Num = num + 1 + JOINT_NUM + 1 + 3;
    // 写入joint0及其之前的点
    for(int i=0;i<=num;i++){
        PathCurve_CtrlPoints[3*i  ] = ARM_BASE_X_OFFSET + BasePosi - base_extend_dist + base_extend_dist / num * i;
        PathCurve_CtrlPoints[3*i+1] = 0;
        PathCurve_CtrlPoints[3*i+2] = ARM_BASE_HEIGHT;
    }
    // 写入joint1～joint15、end的点
    for(int i=1;i<=JOINT_NUM+1;i++){
        PathCurve_CtrlPoints[3*(i+num)  ] = (Joint2World_Trans_Virtual(2*i).matrix())(0,3);
        PathCurve_CtrlPoints[3*(i+num)+1] = (Joint2World_Trans_Virtual(2*i).matrix())(1,3);
        PathCurve_CtrlPoints[3*(i+num)+2] = (Joint2World_Trans_Virtual(2*i).matrix())(2,3);
    }
    // 写入end之后的3个点
    for(int i=1;i<=3;i++){
        PathCurve_CtrlPoints[3*(num+1+JOINT_NUM+i)  ] = PathCurve_CtrlPoints[3*(num+JOINT_NUM+i)  ] + 150*(Joint2World_Trans_Virtual(2*JOINT_NUM+2).matrix())(0,0);
        PathCurve_CtrlPoints[3*(num+1+JOINT_NUM+i)+1] = PathCurve_CtrlPoints[3*(num+JOINT_NUM+i)+1] + 150*(Joint2World_Trans_Virtual(2*JOINT_NUM+2).matrix())(1,0);
        PathCurve_CtrlPoints[3*(num+1+JOINT_NUM+i)+2] = PathCurve_CtrlPoints[3*(num+JOINT_NUM+i)+2] + 150*(Joint2World_Trans_Virtual(2*JOINT_NUM+2).matrix())(2,0);
    }
    // 生成曲线并可视化
    if(!my_s1356(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,PathCurve_CtrlPoints_type,PathCurve))
        ROS_ERROR("Fail to gen the curve.");
    else{
        std::ofstream write_points(PathCurve_CtrlPoints_Savefile.c_str(),std::ios::out);
        writeGoPoints(PathCurve_CtrlPoints_Num, PathCurve_CtrlPoints, write_points);
        write_points.close();
        Curve_Pub(PathCurve,"PathCurve","world");
        Points_Pub(PathCurve_CtrlPoints,PathCurve_CtrlPoints_Num,"PathCurve_CtrlPoints","world");
    }
    return true;
}

bool inverse_kinematic_ctrl::FTLInvKine(double& BasePosi, Arrayf_2_Joint& JointsAngle){

    int speed = 50;
    bool quit_flag=false;
    bool move_flag=false, state = false;

    double BasePosi_Exp;
    Arrayf_2_Joint JointsAngle_Exp = Arrayf_2_Joint::Zero();

    printf(" ==================================================\n");
    printf(" ====================[FTLInvKine]==================\n");
    printf(" === [w/s] to control the move of base.\n");
    printf(" === [d/a] to control the move of end.\n");
    printf(" === [ X ] to set distance per press.\n");
    printf(" === [ N ] to set number of Arm part to control(1 part has 2 joint).\n");
    printf(" === [ Q ] to exit the FTLInvKine program.\n");

    while(1){
        printf("FTLInvKine Command:");
        // command explain
        switch(scanKeyboard()){
            case('w'): state = FTLInvKine_BaseMove(BasePosi,FTL_Part_Num, speed/10.0,BasePosi_Exp,JointsAngle_Exp); move_flag=true; break;
            case('s'): state = FTLInvKine_BaseMove(BasePosi,FTL_Part_Num,-speed/10.0,BasePosi_Exp,JointsAngle_Exp); move_flag=true; break;
            case('d'): state = FTLInvKine_EndMove (BasePosi,FTL_Part_Num, speed/10.0,BasePosi_Exp,JointsAngle_Exp); move_flag=true; break;
            case('a'): state = FTLInvKine_EndMove (BasePosi,FTL_Part_Num,-speed/10.0,BasePosi_Exp,JointsAngle_Exp); move_flag=true; break;
            case('X'):
                printf("Current speed: %d * (0.1mm) per press, input the new speed(int 0~1000):",speed);
                std::cin >> speed;
                if(speed>1000) speed =1000; else if(speed<0) speed=0;
                printf("Current speed: %d.\n",speed); break;
            case('N'):
                printf("Current part number: %d, input the new speed(%d MAX):",FTL_Part_Num, JOINT_NUM);
                std::cin >> FTL_Part_Num; if(FTL_Part_Num>JOINT_NUM) speed =JOINT_NUM;
                printf("Current part number: %d\n",FTL_Part_Num); break;
            case('Q'): quit_flag = true; break;
            default:
                move_flag=false; ROS_WARN("No such command!!!\n");
        }
        if(quit_flag) break;

        // 运动后处理
        if(move_flag == true && state==true){

            // 拉高服务请求标志位，并等待服务server响应
            if(!simulation){
                // 置高服务请求flag
                Call_BasePosi_Exp = BasePosi_Exp;
                Call_JointsAngle_Exp = JointsAngle_Exp;
                CallService_Flag = true;
            }
            
            int wait_cnt = 0;
            while(CallService_Flag){
                if((wait_cnt % 30) == 0)
                    ROS_DEBUG("[FTLInvKine] Waitfor Server finish.");
                usleep(1000*100);

                wait_cnt++;
                if(wait_cnt >= 300){ // 30s
                    ROS_ERROR("[FTLInvKine] Fail to Call the Server.");
                    break;
                }
            }

            // 运动完成后，进行更新，并可视化
            BasePosi = BasePosi_Exp;
            JointsAngle = JointsAngle_Exp;
            Position_Solution(BasePosi_Exp,JointsAngle_Exp,Joint2World_Trans_Virtual);
            Arm_Position_Pub(Joint2World_Trans_Virtual);
            // Arm_Position_Pub(Joint2World_Trans_Virtual,true);

            move_flag = false;
            state = false;
        }
    }
}

bool inverse_kinematic_ctrl::FTLInvKine_EndMove(double BasePosi, int Part_Num, double EndDistExp,
                                                double& BasePosi_Exp, Arrayf_2_Joint& JointsAngle_Exp){
    if(Part_Num>JOINT_NUM) ROS_ERROR("[FTLInvKine_BaseMove] Part_Num out of range!!!");
    if(!FTLInvKine_BaseMove(BasePosi, Part_Num, 0, BasePosi_Exp, JointsAngle_Exp)){ // 先确保机械臂在脊线上
        ROS_ERROR("[FTLInvKine_EndMove] Fail to call function FTLInvKine_BaseMove at the begin of FTLInvKine_EndMove.");
        return false;
    }

    // 计算指定关节末端中心的初始位置
    // -------------------------------------
    Eigen::Isometry3d sphere_centre_mat;
    Eigen::Vector3d sphere_centre_vec_0,sphere_centre_vec;

    sphere_centre_mat = Eigen::Isometry3d::Identity();
    sphere_centre_mat.translate(Eigen::Vector3d(BasePosi_Exp + ARM_BASE_X_OFFSET + FIX_POLE_PART_LENGTH, 0, ARM_BASE_HEIGHT));
    for(int i=0;i<Part_Num;i++){
        sphere_centre_mat.rotate(Eigen::AngleAxisd(JointsAngle_Exp(2*i), Eigen::Vector3d(0, 0, 1)));
        sphere_centre_mat.translate(Eigen::Vector3d(ROT_SHAFT_PART_LENGTH, 0, 0));
        sphere_centre_mat.rotate(Eigen::AngleAxisd(-JointsAngle_Exp(2*i+1), Eigen::Vector3d(0, 1, 0)));
        if(i!=(JOINT_NUM-1))
            sphere_centre_mat.translate(Eigen::Vector3d(FIX_POLE_PART_LENGTH, 0, 0));
        else
            sphere_centre_mat.translate(Eigen::Vector3d(END_POSITION_X, END_POSITION_Y, END_POSITION_Z));
    }
    sphere_centre_vec_0 = sphere_centre_mat.translation();

    // 调用FTLInvKine_BaseMove
    // -------------------------------------
    int cnt = 0;
    double BaseDist_Exp = 0;
    double EndDistDelta = EndDistExp; // 指定关节末端中心期望位移与当前位移之差
    while(abs(EndDistDelta) > Ftl_End_Toler){
        BaseDist_Exp += EndDistDelta * FeedCoef;
        if(!FTLInvKine_BaseMove(BasePosi, Part_Num, BaseDist_Exp, BasePosi_Exp, JointsAngle_Exp)){
            ROS_ERROR("[FTLInvKine_EndMove] Fail to call function FTLInvKine_BaseMove. BaseDist_Exp = %f, cnt = %d",BaseDist_Exp,cnt);
            return false;
        };

        // update EndDistDelta
        sphere_centre_mat = Eigen::Isometry3d::Identity();
        sphere_centre_mat.translate(Eigen::Vector3d(BasePosi_Exp + ARM_BASE_X_OFFSET + FIX_POLE_PART_LENGTH, 0, ARM_BASE_HEIGHT));
        for(int i=0;i<Part_Num;i++){
            sphere_centre_mat.rotate(Eigen::AngleAxisd(JointsAngle_Exp(2*i), Eigen::Vector3d(0, 0, 1)));
            sphere_centre_mat.translate(Eigen::Vector3d(ROT_SHAFT_PART_LENGTH, 0, 0));
            sphere_centre_mat.rotate(Eigen::AngleAxisd(-JointsAngle_Exp(2*i+1), Eigen::Vector3d(0, 1, 0)));
            if(i!=(JOINT_NUM-1))
                sphere_centre_mat.translate(Eigen::Vector3d(FIX_POLE_PART_LENGTH, 0, 0));
            else
                sphere_centre_mat.translate(Eigen::Vector3d(END_POSITION_X, END_POSITION_Y, END_POSITION_Z));
        }
        sphere_centre_vec = sphere_centre_mat.translation();

        double EndDist = (sphere_centre_vec - sphere_centre_vec_0).norm();
        if(EndDistExp > 0)
            EndDistDelta = EndDistExp - EndDist;
        else 
            EndDistDelta = EndDistExp + EndDist;
        // ROS_DEBUG("[FTLInvKine_EndMove] EndDistDelta = %f", EndDistDelta);

        // 如果不收敛，需要采取一定的措施
        if(++cnt > FTLEndIterateCNTMax){
            ROS_WARN("[FTLInvKine_EndMove] CNT OUT OF RANGE!!!!");
            return false;
        }
        if(cnt == FTLEndIterateCNTMax/3){
            FeedCoef = FeedCoef / 1.5;
            ROS_WARN("[FTLInvKine_EndMove] CNT is too big, change FeedCoef to %f!!!!", FeedCoef);
        }

    }

    if(cnt >= FTLEndIterateCNTMax/3) FeedCoef = _nh->param<float>("/inverse_kinematic_ctrl/FTL/FeedCoef",0.8);
    // ROS_DEBUG("[FTLInvKine_EndMove] finished, cnt = %d, error = %f\n",cnt ,abs(EndDistDelta));
    return true;
}

bool inverse_kinematic_ctrl::FTLInvKine_BaseMove(double BasePosi, int Part_Num, double BaseDist_Exp,
                                                 double& BasePosi_Exp, Arrayf_2_Joint& JointsAngle_Exp){
    if(Part_Num>JOINT_NUM) ROS_ERROR("[FTLInvKine_BaseMove] Part_Num out of range!!!");
                                                    
    double sphere_centre[3]; // 当前臂段的基部中心，作为后续求交点时的球心
    Eigen::Isometry3d sphere_centre_mat = Eigen::Isometry3d::Identity(); // 当前臂段的基部中心位姿

    // 更新基部位置
    BasePosi_Exp = BasePosi + BaseDist_Exp;
    sphere_centre_mat.translate(Eigen::Vector3d(BasePosi_Exp + ARM_BASE_X_OFFSET + FIX_POLE_PART_LENGTH, 0, ARM_BASE_HEIGHT));

    // 逐节更新关节角
    for(int i=0;i<Part_Num;i++){
        // ROS_DEBUG("[DEBUG] Arm part [%d]",i+1);
        // 通过球面交点获取初始点
        // -------------------------------
        Eigen::Vector4d point_posi[3]; // 点的位置
        double int_param[3]; // 曲线上点的参数
        sphere_centre[0]=(sphere_centre_mat.translation())(0);
        sphere_centre[1]=(sphere_centre_mat.translation())(1);
        sphere_centre[2]=(sphere_centre_mat.translation())(2);
        // ROS_DEBUG_STREAM("sphere_centre_mat: \n" << sphere_centre_mat.matrix());
        if(!cal_int_point(sphere_centre,WrappedBall_Min,PathCurve,int_param[0],point_posi[0]) ||
           !cal_int_point(sphere_centre,WrappedBall_Max,PathCurve,int_param[1],point_posi[1])){
            ROS_ERROR("[FTLInvKine_BaseMove] Arm part [%d]. Fail to calculate intersection point of the sphere and curve.",i+1);
            cnt_test=0; return false;
        }
        // ROS_DEBUG("int_param[0] =%f, int_param[1]=%f", int_param[0], int_param[1]);
        
        // 计算两个球面的交点到曲面的距离
        // ---------------------------------------
        double distance_x1,distance_x2,temp;

        // 小球面的交点
        point_posi[0] = sphere_centre_mat.inverse() * point_posi[0]; // 将点从world坐标系下变换到当前坐标系下
        // 检验z轴方向是否超限，若无则进行第一步求解
        if( abs((point_posi[0])(2)) > abs((i!=(JOINT_NUM-1))?FIX_POLE_PART_LENGTH:END_POSITION_X)-1e-4 ){ 
            ROS_WARN("[FTLInvKine_BaseMove] Arm part [%d]. Distance is cannot be calculated, z of intersection point 1 is [%f]",i+1 ,(point_posi[0])(2));
            cnt_test=0; return false; }
        temp = ROT_SHAFT_PART_LENGTH + sqrt(pow((i!=(JOINT_NUM-1))?FIX_POLE_PART_LENGTH:END_POSITION_X,2) - pow((point_posi[0])(2),2));
        // 检验y轴方向是否超限，若无则进行第二步求解
        if( abs((point_posi[0])(1)) > temp-1e-4 ){ 
            ROS_WARN("[FTLInvKine_BaseMove] Arm part [%d]. Distance is cannot be calculated, y of intersection point 1 is [%f]",i+1 ,(point_posi[0])(1));
            cnt_test=0; return false; }
        temp = sqrt(temp*temp - pow((point_posi[0])(1),2));
        distance_x1 = (point_posi[0])(0) - temp;
        
        // 大球面的交点
        point_posi[1] = sphere_centre_mat.inverse() * point_posi[1]; // 将点从world坐标系下变换到当前坐标系下
        // 检验z轴方向是否超限，若无则进行第一步求解
        if( abs((point_posi[1])(2)) > abs((i!=(JOINT_NUM-1))?FIX_POLE_PART_LENGTH:END_POSITION_X)-1e-4 ){ 
            ROS_WARN("[FTLInvKine_BaseMove] Arm part [%d]. Distance is cannot be calculated, z of intersection point 2 is [%f]",i+1 ,(point_posi[1])(2));
            cnt_test=0; return false; }
        temp = ROT_SHAFT_PART_LENGTH + sqrt(pow((i!=(JOINT_NUM-1))?FIX_POLE_PART_LENGTH:END_POSITION_X,2) - pow((point_posi[1])(2),2));
        // 检验y轴方向是否超限，若无则进行第二步求解
        if( abs((point_posi[1])(1)) > temp-1e-4 ){ 
            ROS_WARN("[FTLInvKine_BaseMove] Arm part [%d]. Distance is cannot be calculated, y of intersection point 2 is [%f]",i+1 ,(point_posi[1])(1));
            cnt_test=0; return false; }
        temp = sqrt(temp*temp - pow((point_posi[1])(1),2)); // 以及上一行，用于计算曲面上y、z与交点相同的点 的x的值
        distance_x2 = (point_posi[1])(0) - temp;
        // ROS_DEBUG("intersection points[0] = %f, distance_1 = %f", int_param[0], distance_x1);
        // ROS_DEBUG("intersection points[1] = %f, distance_2 = %f", int_param[1], distance_x2);

        // 迭代计算新的点
        // ---------------------------------------
        int cnt =0;
        while(std::min(abs(distance_x1),abs(distance_x2)) > Inter_Toler){
            // 新的点的参数
            int_param[2] = (int_param[0]*distance_x2 - int_param[1]*distance_x1) / (distance_x2-distance_x1);

            // 用于通过节点参数对曲线上的点进行采样
            int left=0; double point[3]; int stat = 0;
            s1227(PathCurve, 0, int_param[2], &left, point, &stat); 
            (point_posi[2])(0) = point[0]; (point_posi[2])(1) = point[1]; (point_posi[2])(2) = point[2]; (point_posi[2])(3) = 1;
            
            // 计算距离
            point_posi[2] = sphere_centre_mat.inverse() * point_posi[2]; // 将点从world坐标系下变换到当前坐标系下
            // 检验z轴方向是否超限，若无则进行第一步求解
            if( abs((point_posi[2])(2)) > abs((i!=(JOINT_NUM-1))?FIX_POLE_PART_LENGTH:END_POSITION_X)-1e-4 ){ 
                ROS_WARN("[FTLInvKine_BaseMove] Arm part [%d]. Distance is cannot be calculated, z of intersection point 3 is [%f]",i+1 ,(point_posi[2])(2));
                cnt_test=0; return false; }
            temp = ROT_SHAFT_PART_LENGTH + sqrt(pow((i!=(JOINT_NUM-1))?FIX_POLE_PART_LENGTH:END_POSITION_X,2) - pow((point_posi[2])(2),2));
            // 检验y轴方向是否超限，若无则进行第二步求解
            if( abs((point_posi[2])(1)) > temp-1e-4 ){ 
                ROS_WARN("[FTLInvKine_BaseMove] Arm part [%d]. Distance is cannot be calculated, y of intersection point 3 is [%f]",i+1 ,(point_posi[2])(1));
                cnt_test=0; return false; }
            temp = sqrt(temp*temp - pow((point_posi[2])(1),2)); // 以及上一行，用于计算曲面上y、z与交点相同的点 的x的值
            temp = (point_posi[2])(0) - temp;
            // ROS_DEBUG("intersection points = %f, distance = %f", int_param[2], temp);

            // 更新点
            if(abs(distance_x2) > abs(distance_x1)){
                distance_x2 = temp; point_posi[1] = point_posi[2]; int_param[1] = int_param[2];
            }
            else{
                distance_x1 = temp; point_posi[0] = point_posi[2]; int_param[0] = int_param[2];
            }

            // 判断是否可以收敛
            if(++cnt > FTLBaseIterateCNTMax){
                ROS_WARN("[FTLInvKine_BaseMove]Arm part [%d]. CNT OUT OF RANGE!!!!", i+1);
                return false;
            }
        }

        // 将求得交点的坐标值放入point_posi[0]，并将其代入曲面表达式求解关节角
        // ------------------------------------------------------------
        if(abs(distance_x1) > abs(distance_x2)) {
            point_posi[0] = point_posi[1]; 
            int_param[0]  = int_param[1];
        }
        JointsAngle_Exp(2*i+1) = asin(point_posi[0](2)/((i!=(JOINT_NUM-1))?FIX_POLE_PART_LENGTH:END_POSITION_X));
        JointsAngle_Exp(2*i  ) = asin(point_posi[0](1)/(ROT_SHAFT_PART_LENGTH + cos(JointsAngle_Exp(2*i+1)) * ((i!=(JOINT_NUM-1))?FIX_POLE_PART_LENGTH:END_POSITION_X)));
        
        // 为了混合控制算法，需要提供一下第Part_Num个关节中心的int_param
        last_int_param = int_param[0];

        // 限位
        // --------
        if(JointsAngle_Exp(2*i) > MAX_JOINTANGLE) JointsAngle_Exp(2*i) = MAX_JOINTANGLE;
        else if(JointsAngle_Exp(2*i) < -MAX_JOINTANGLE) JointsAngle_Exp(2*i) = -MAX_JOINTANGLE;

        if(JointsAngle_Exp(2*i+1) > MAX_JOINTANGLE) JointsAngle_Exp(2*i+1) = MAX_JOINTANGLE;
        else if(JointsAngle_Exp(2*i+1) < -MAX_JOINTANGLE) JointsAngle_Exp(2*i+1) = -MAX_JOINTANGLE;

        // 完成本关节求解后，计算下一个关节中心点的位姿
        // -----------------------------------------
        if(i != JOINT_NUM-1){
            sphere_centre_mat.rotate(Eigen::AngleAxisd(JointsAngle_Exp(2*i), Eigen::Vector3d(0, 0, 1)));
            sphere_centre_mat.translate(Eigen::Vector3d(ROT_SHAFT_PART_LENGTH, 0, 0));
            sphere_centre_mat.rotate(Eigen::AngleAxisd(-JointsAngle_Exp(2*i+1), Eigen::Vector3d(0, 1, 0)));
            sphere_centre_mat.translate(Eigen::Vector3d(FIX_POLE_PART_LENGTH, 0, 0));
        }
    }
    // ROS_INFO("[FTLInvKine_BaseMove] finished.");
    return true;
}

bool inverse_kinematic_ctrl::cal_int_point(double *sphere_centre, double radius, SISLCurve *PathCurve, double &int_param, Eigen::Vector4d &int_point){

    int numintpt=0, numintcu=0, stat=0; // 交点个数,交线个数,s1371运行状态指示
    double *intps; // 指向交点参数的指针数组
    SISLIntcurve** intcurve;// 指向交线的指针数组

    // 调用sisl s1371进行计算交点
    s1371(PathCurve, sphere_centre, radius, 3, 1e-9, 1e-2, &numintpt, &intps, &numintcu, &intcurve, &stat);

    if(stat!=0){
        ROS_WARN("[cal_int_point] Fail to calculate intersection points with WrappedBall_Min, point_num=%d",numintpt);
        return false;
    }
    // ROS_DEBUG("intersection points point_num = %d",numintpt);
    // for(int i=0;i<numintpt;i++)
    //     ROS_DEBUG("intersection points[%d]: %f",i+1, intps[i]);

    // 找其中最靠前的点
    if(numintpt == 1) int_param = intps[0];
    else if(numintpt == 2) int_param = std::max(intps[0],intps[1]);
    else{
        ROS_WARN("Intersection point number out of range, num = %d",numintpt);
        return false;
    }

    // 获取该点在world坐标系下的坐标
    double point[3];
    int left = 0;
    s1227(PathCurve, 0, int_param, &left, point, &stat); // 用于通过节点参数对曲线上的点进行采样
    if(stat!=0){
        ROS_WARN("[cal_int_point] Fail to calculate point with param [%f] on PathCurve.",int_param);
        return false;
    }

    int_point(0) = point[0]; int_point(1) = point[1]; int_point(2) = point[2]; int_point(3) = 1; 
    return true;
}

bool inverse_kinematic_ctrl::JacobianDiffInvKine(bool Hybrid){

    Position_Solution(BasePosiCur, JointsAngleCur, Joint2World_Trans_Virtual);
    Eigen::Isometry3d End_pose_Exp = Joint2World_Trans_Virtual(2 * JOINT_NUM + 2);
    Eigen::Isometry3d End_pose_save = End_pose_Exp;
    double BasePosi_Exp;
    Arrayf_2_Joint JointsAngle_Exp;

    int speed = 120;
    bool quit_flag=false;
    bool move_flag=false;
    bool move_base_flag=true;

    while(1){

        printf(" ===========================================================\n");
        printf(" ==========[ JacobianDiffInvKine / HybridInvKine] ==========\n");
        printf(" === [u/j] to control the end move along x axis(front/back).\n");
        printf(" === [h/k] to control the end move along y axis(front/back).\n");
        printf(" === [y/i] to control the end move along z axis(front/back).\n");
        printf(" === [w/s] to control the end rot along y axis.\n");
        printf(" === [a/d] to control the end rot along z axis.\n");
        printf(" === [q/e] to control the end rot along x axis.\n");
        printf(" === [B/b] do/don't move base(only for JacobianDiffInvKine method).\n");
        printf(" === [ P ] to control the joints num controlled by FTL mode.\n");
        printf(" === [ X ] to set distance per press.\n");
        printf(" === [ Q ] to exit the JacobianDiffInvKine program.\n");
        printf("\n");
        printf(" === Current speed: %d * (0.1mm or 0.02degree) per press.\n", speed);
        printf(" === Current FTL Joints: %d * 2, Free Joints: %d * 2.\n", Hyb_Part_Num, JOINT_NUM - Hyb_Part_Num);

        printf("JacobianDiffInvKine Command:");
        // command explain
        switch(scanKeyboard()){
            case('u'): End_pose_Exp.translate(Eigen::Vector3d( speed/10.0, 0, 0)); move_flag=true; break;
            case('j'): End_pose_Exp.translate(Eigen::Vector3d(-speed/10.0, 0, 0)); move_flag=true; break;
            case('h'): End_pose_Exp.translate(Eigen::Vector3d(0,  speed/10.0, 0)); move_flag=true; break;
            case('k'): End_pose_Exp.translate(Eigen::Vector3d(0, -speed/10.0, 0)); move_flag=true; break;
            case('i'): End_pose_Exp.translate(Eigen::Vector3d(0, 0,  speed/10.0)); move_flag=true; break;
            case('y'): End_pose_Exp.translate(Eigen::Vector3d(0, 0, -speed/10.0)); move_flag=true; break;

            case('w'): End_pose_Exp.rotate(Eigen::AngleAxisd(-speed/2827.0, Eigen::Vector3d(0, 1, 0))); move_flag=true; break;
            case('s'): End_pose_Exp.rotate(Eigen::AngleAxisd( speed/2827.0, Eigen::Vector3d(0, 1, 0))); move_flag=true; break;
            case('d'): End_pose_Exp.rotate(Eigen::AngleAxisd(-speed/2827.0, Eigen::Vector3d(0, 0, 1))); move_flag=true; break;
            case('a'): End_pose_Exp.rotate(Eigen::AngleAxisd( speed/2827.0, Eigen::Vector3d(0, 0, 1))); move_flag=true; break;
            case('q'): End_pose_Exp.rotate(Eigen::AngleAxisd(-speed/2827.0, Eigen::Vector3d(1, 0, 0))); move_flag=true; break;
            case('e'): End_pose_Exp.rotate(Eigen::AngleAxisd( speed/2827.0, Eigen::Vector3d(1, 0, 0))); move_flag=true; break;

            case('B'): move_base_flag = true;  ROS_INFO("move_base ON.");  break;
            case('b'): move_base_flag = false; ROS_INFO("move_base OFF."); break;
            case('P'):
                printf("Current FTL Joints: %d * 2, input the new num(input 0~%d): 2 * ", Hyb_Part_Num, JOINT_NUM-3);
                std::cin >> Hyb_Part_Num;
                if(Hyb_Part_Num>JOINT_NUM-3) Hyb_Part_Num=JOINT_NUM-3; else if(Hyb_Part_Num<0) Hyb_Part_Num=0;
                printf("Current FTL Joints: %d * 2.\n",Hyb_Part_Num);
                break;
            case('X'):
                printf("Current speed: %d * (0.1mm or 0.02degree) per press, input the new speed(int 1~1000):",speed);
                std::cin >> speed;
                if(speed>1000) speed =1000; else if(speed<1) speed=1;
                printf("Current speed: %d.\n",speed);
                break;
            case('Q'): quit_flag = true; break;
            default:
                move_flag=false; ROS_WARN("No such command!!!\n");
        }
        // ROS_DEBUG_STREAM("End_pose_2:\n" << End_pose.matrix());

        if(quit_flag) break;

        // 运动
        if(move_flag == true){
            // ROS_DEBUG_STREAM("End_pose_exp: \n"<<End_pose_Exp.matrix());
            bool state;
            if(!Hybrid || (Hyb_Part_Num==0)){
                state = JacobianDiffInvKine_ToPose(End_pose_Exp.matrix(), BasePosi_Exp ,JointsAngle_Exp, move_base_flag);
            }
            else{
                state = HybridInvKine_ToPose(End_pose_Exp.matrix(), BasePosi_Exp ,JointsAngle_Exp);
            }

            if (!state){
                ROS_WARN("JacobianDiffInvKine_ToPose / HybridInvKine_ToPose false.");
                ROS_WARN_STREAM("BasePosi = " << BasePosiCur <<", JointsAngle = \n" << JointsAngleCur);
                ROS_WARN_STREAM("End_pose_exp = \n" << End_pose_Exp.matrix());
                End_pose_Exp = End_pose_save; // 运动失败，恢复到之前的值
            }
            else{
                ROS_INFO_STREAM("BasePosi_Exp = 1 mm * " << BasePosi_Exp<< ", JointsAngle_Exp = 1 degree * \n" << JointsAngle_Exp * 180 / M_PI);

                // 拉高服务请求标志位，并等待服务server响应
                if(!simulation){
                    // 置高服务请求flag
                    Call_BasePosi_Exp = BasePosi_Exp;
                    Call_JointsAngle_Exp = JointsAngle_Exp;
                    CallService_Flag = true;
                }
                
                int wait_cnt = 0;
                while(CallService_Flag){
                    if((wait_cnt % 30) == 0)
                        ROS_DEBUG("[JacobianDiffInvKine] Waitfor Server finish.");
                    usleep(1000*100);

                    wait_cnt++;
                    if(wait_cnt >= 300){ // 30s
                        ROS_ERROR("[JacobianDiffInvKine] Fail to Call the Server.");
                        break;
                    }
                }

                // 更新运动之后的值
                End_pose_save  = End_pose_Exp;
                BasePosiCur    = BasePosi_Exp;
                JointsAngleCur = JointsAngle_Exp;
            }
            #ifndef TEST
                Arm_Position_Pub(Joint2World_Trans_Virtual);
            #endif
            move_flag = false;
        }
    }
}

bool inverse_kinematic_ctrl::JacobianDiffInvKine_ToPose( Eigen::Matrix4d EndPoseExp, 
                                                         double& BasePosi_Exp, 
                                                         Arrayf_2_Joint& JointsAngle_Exp, 
                                                         bool MoveBase_Flag){
    ROS_DEBUG_STREAM("Begin the JacobianDiffInvKine_ToPose ================");

    // 一些变量的定义
    Eigen::Matrix<double, 17, 1> weight_adaptive; weight_adaptive.setOnes(); // 自适应权重
    double step_coef;   // 步长系数

    Eigen::Isometry3d *EndPose;
    double BasePosi = BasePosiCur;
    Arrayf_2_Joint JointsAngle = JointsAngleCur;

    double delta_base;
    Eigen::Matrix<double,2*JOINT_NUM,1> delta_JointsAngle;

    // 计算末端位置偏差
    Position_Solution(BasePosi,JointsAngle,Joint2World_Trans_Virtual);
    EndPose = &Joint2World_Trans_Virtual(2*JOINT_NUM+2);
    double error_ang = (EndPose->matrix()-EndPoseExp).block<3,3>(0,0).norm();
    double error_posi = (EndPose->matrix()-EndPoseExp).block<3,1>(0,3).norm();
    ROS_DEBUG_STREAM("Initial error: " << error_ang*81 << "deg, " << error_posi << "mm.");

    // 迭代
    int cnt=0;
    double max_step_ang_coef = 2*sqrt(1-cos(max_step_ang));
    Eigen::MatrixXd JacobianMat; // 通过动态矩阵存放雅可比矩阵，其大小可以通过resize调整
    while((error_ang > EndPose_Tolerance_ang) || (error_posi > EndPose_Tolerance_posi)){

        // 计算步长系数
        double distance_ang = (EndPose->rotation() - EndPoseExp.block<3, 3>(0, 0)).norm();
        double distance_posi = (EndPose->translation() - EndPoseExp.block<3, 1>(0, 3)).norm();
        step_coef = std::max(std::max(distance_ang / max_step_ang_coef, distance_posi / max_step_posi), step_coef_min);

        // 计算本次迭代中期望的末端微分运动
        Eigen::Isometry3d delta_pose_mat;
        Eigen::Matrix<double,6,1> delta_pose_vec;

        delta_pose_mat = EndPose->inverse() * EndPoseExp;
        Eigen::AngleAxisd rot_vec(delta_pose_mat.rotation());
        delta_pose_vec.block<3, 1>(0, 0) = delta_pose_mat.translation() / step_coef;
        delta_pose_vec.block<3, 1>(3, 0) = rot_vec.axis() * rot_vec.angle() / step_coef;

        // 计算雅可比矩阵，并将默认权重添加到其中,并通过雅克比矩阵进行解算
        Eigen::VectorXd delta;
        GetJacobian(BasePosi, JointsAngle, JacobianMat, MoveBase_Flag ? 0 : 1, JOINT_NUM * 2);
        if(MoveBase_Flag)
            JacobianMat = JacobianMat * weight_def.asDiagonal();
        else    
            JacobianMat = JacobianMat * ((weight_def.block<2*JOINT_NUM,1>(1,0)).asDiagonal());
        
        Pseudo_inverse_solution(JacobianMat, delta, delta_pose_vec);
        if(MoveBase_Flag){
            delta_JointsAngle = delta.block<2*JOINT_NUM,1>(1,0);
        }
        else{
            delta_JointsAngle = delta.block<2*JOINT_NUM,1>(0,0);
        }

        // 若通过自适应权重进行回归解算，则执行以下内容
        if(regression_solu_on){ 
            // 计算自适应权重
            bool regression_flag = false; // 是否在更新自适应权重后进行回归计算
            for (int i = 0; i < 2 * JOINT_NUM; i++) { // 关节角
                if ((std::abs(JointsAngle(i)) > weight_joint_threshold) && ((JointsAngle(i) > 0) == (delta_JointsAngle(i) > 0))){
                    weight_adaptive(i+1) = (MAX_JOINTANGLE - std::abs(JointsAngle(i))) / weight_joint_threshold;
                    regression_flag = true;
                }
                else
                    weight_adaptive(i+1) = 1;
            }
            // ROS_DEBUG_STREAM("%weight_adaptive = " << weight_adaptive.transpose());

            // 计算雅可比矩阵，并将默认权重添加到其中,并通过雅克比矩阵进行解算
            if(regression_flag){
                // 雅克比矩阵中已经添加了weight_def，这里再添加weight_adaptive
                if(MoveBase_Flag)
                    JacobianMat = JacobianMat * weight_adaptive.asDiagonal();
                else
                    JacobianMat = JacobianMat * ((weight_adaptive.block<2*JOINT_NUM,1>(1,0)).asDiagonal());

                Pseudo_inverse_solution(JacobianMat, delta, delta_pose_vec);
                if(MoveBase_Flag)
                    delta_JointsAngle = delta.block<2*JOINT_NUM,1>(1,0);
                else
                    delta_JointsAngle = delta.block<2*JOINT_NUM,1>(0,0);
            }
        }

        // 根据解算到的变化量，进行关节角更新
        if(MoveBase_Flag){
            BasePosi += weight_def(0) * delta(0);
        }
        for (int i = 0; i < 2 * JOINT_NUM; i++) {
            JointsAngle(i) += weight_def(i+1) * weight_adaptive(i+1) * delta_JointsAngle(i);
            // ROS_DEBUG("WEIGHT%d = %f %f", i+1 , weight_def(i+1), weight_adaptive(i+1) );
            // 关节角限位
            if (JointsAngle(i) > MAX_JOINTANGLE) JointsAngle(i) = MAX_JOINTANGLE;
            else if (JointsAngle(i) < -MAX_JOINTANGLE) JointsAngle(i) = -MAX_JOINTANGLE;
        }

        // 通过更新之后的关节角进行位姿解算，并计算偏差
        // ROS_DEBUG_STREAM("BasePosi = " << BasePosi);
        // ROS_DEBUG_STREAM("JointsAngle = \n" << JointsAngle * 180 / M_PI);
        Position_Solution(BasePosi,JointsAngle,Joint2World_Trans_Virtual);
        // ROS_DEBUG_STREAM("EndPose = \n" << EndPose->matrix());
        // ROS_DEBUG_STREAM("EndPoseExp = \n" << EndPoseExp);
        error_ang = (EndPose->matrix()-EndPoseExp).block<3,3>(0,0).norm();
        error_posi = (EndPose->matrix()-EndPoseExp).block<3,1>(0,3).norm();
        // Confirm_Interaction();

        // 超限
        ROS_DEBUG_STREAM("[JacobianDiffInvKine_ToPose] cnt = "<< cnt+1 << ", error: " << error_ang*81 << "deg, " << error_posi << "mm.");
        if(++cnt>JacoIterateCNTMax) {
            ROS_WARN("[JacobianDiffInvKine_ToPose] CNT OUT OF RANGE!!!!");
            Position_Solution(BasePosiCur, JointsAngleCur, Joint2World_Trans_Virtual); // 若超限，则恢复之前的值
            return false;
        }
    }
    ROS_DEBUG("cnt = %d", cnt+1);
    ROS_DEBUG_STREAM("Finish the JacobianDiffInvKine_ToPose ================");
    JointsAngle_Exp = JointsAngle;
    BasePosi_Exp = BasePosi;
    return true;
}

bool inverse_kinematic_ctrl::Pseudo_inverse_solution(Eigen::MatrixXd& A, Eigen::VectorXd& x, Eigen::Matrix<double, 6, 1>& y){
    // 奇异值分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // 计算关节变化量
    // ROS_DEBUG("    Min singularValues is: %f.\n", svd.singularValues().minCoeff());
    if (svd.singularValues().minCoeff() > singular_coef){
        x = svd.solve(y);
    }
    else{
        // ROS_DEBUG("Closed to the singular position!");
        x = (A.transpose()) *
            ((A * (A.transpose()) + singular_coef * Eigen::Matrix<double, 6, 6>::Identity()).inverse()) *
            y;
        // ROS_DEBUG_STREAM("A = \n" << A << " \n x.transpose() = \n" << x.transpose() << "\n y.transpose() = \n" << y.transpose());
    }
    // ROS_DEBUG_STREAM("delta (base &) theta= " << x.transpose());
    return true;
}

bool inverse_kinematic_ctrl::HybridInvKine_ToPose(Eigen::Matrix4d EndPoseExp, 
                                                  double& BasePosi_Exp, 
                                                  Arrayf_2_Joint& JointsAngle_Exp){
    ROS_DEBUG_STREAM("[HybridInvKine_ToPose] Begin the HybridInvKine_ToPose ================");
    // Arm_Position_Pub(Joint2World_Trans_Virtual); // debug

    // 一些变量的定义
    Eigen::VectorXd weight_adaptive; 
    weight_adaptive.resize(2*(JOINT_NUM-Hyb_Part_Num)+1,1);
    weight_adaptive.setOnes(); // 自适应权重

    double step_coef;   // 步长系数
    bool TryChangeWeight_Flag = false;
    double Ftl_End_Toler_store = Ftl_End_Toler; // 存放原始Ftl_End_Toler，以备临时降低FTLInvKine_EndMove的运动精度要求，使大步运动时不花过多的时间在迭代上

    Eigen::Isometry3d *EndPose;
    double BasePosi = BasePosiCur;
    Arrayf_2_Joint JointsAngle = JointsAngleCur;

    // 存放关节变化值（含虚拟关节）、存放关节权重向量
    Eigen::VectorXd delta, weight_def_Hybr;
    weight_def_Hybr.resize(2*(JOINT_NUM-Hyb_Part_Num)+1,1);

    weight_def_Hybr(0) = VirtualJointWeight_def;
    weight_def_Hybr.block(1,0,2*(JOINT_NUM-Hyb_Part_Num),1) = weight_def.block(2*Hyb_Part_Num+1,0,2*(JOINT_NUM-Hyb_Part_Num),1);
    // ROS_DEBUG_STREAM("weight_def_Hybr = " << weight_def_Hybr.transpose());

    // 先确保机械臂基部的几个关节在脊线上，以此对JointsAngle的之前几个关节进行更新
    if(!FTLInvKine_BaseMove(BasePosi, Hyb_Part_Num, 0, BasePosi, JointsAngle)){
        ROS_ERROR("[HybridInvKine_ToPose] Fail to call function FTLInvKine_BaseMove.");
        return false;
    }

    // 计算末端位置偏差
    Position_Solution(BasePosi,JointsAngle,Joint2World_Trans_Virtual);
    EndPose = &Joint2World_Trans_Virtual(2*JOINT_NUM+2);
    double error_ang = (EndPose->matrix()-EndPoseExp).block<3,3>(0,0).norm();
    double error_posi = (EndPose->matrix()-EndPoseExp).block<3,1>(0,3).norm();
    ROS_DEBUG_STREAM("[HybridInvKine_ToPose] Initial error: " << error_ang*81 << "deg, " << error_posi << "mm.");

    #ifdef TEST
        Arm_Position_Pub(Joint2World_Trans_Virtual); // 清空之前的轨迹
    #endif

    // 迭代
    int cnt=0;
    double max_step_ang_coef = 2*sqrt(1-cos(max_step_ang));
    Eigen::Isometry3d TNB2World, End2TNB;
    Eigen::MatrixXd JacobianMat; // 通过动态矩阵存放雅可比矩阵，其大小可以通过resize调整

    // 用于TNB坐标系的建立
    int state=0;
    double curvature; // 曲率
    double Frenet_P[3], Frenet_T[3], Frenet_N[3], Frenet_B[3]; // 分别为frenet frame的原点坐标 T N B
    while((error_ang > EndPose_Tolerance_ang) || (error_posi > EndPose_Tolerance_posi)){

        // 计算步长系数
        double distance_ang = (EndPose->rotation() - EndPoseExp.block<3, 3>(0, 0)).norm();
        double distance_posi = (EndPose->translation() - EndPoseExp.block<3, 1>(0, 3)).norm();
        step_coef = std::max(std::max(2 * distance_ang / max_step_ang_coef, 2 * distance_posi / max_step_posi), step_coef_min);

        // 计算本次迭代中期望的末端微分运动
        Eigen::Isometry3d delta_pose_mat;
        Eigen::Matrix<double,6,1> delta_pose_vec;

        delta_pose_mat = EndPose->inverse() * EndPoseExp;
        Eigen::AngleAxisd rot_vec(delta_pose_mat.rotation());
        delta_pose_vec.block<3, 1>(0, 0) = delta_pose_mat.translation() / step_coef;
        delta_pose_vec.block<3, 1>(3, 0) = rot_vec.axis() * rot_vec.angle() / step_coef;
        // ROS_DEBUG_STREAM("delta_pose_vec: " << delta_pose_vec.transpose());

        // 计算转动关节的雅可比矩阵
        // -------------------------------------
        GetJacobian(BasePosi, JointsAngle, JacobianMat, Hyb_Part_Num * 2, JOINT_NUM * 2); // 雅可比矩阵宽度比2*(JOINT_NUM-Hyb_Part_Num)多1，第一列留给虚拟关节
        // ROS_DEBUG_STREAM("JacobianMat_1: \n" << JacobianMat);

        // 计算虚拟关节对应的雅可比矩阵值
        // ----------------------------------

        // 建立TNB frame
        s2559( PathCurve, &last_int_param, 1, Frenet_P, Frenet_T, Frenet_N, Frenet_B, &state); // 获取TNB
        if(state!=0){
            ROS_WARN("[HybridInvKine_ToPose] Fail to calculate PTN with param [%f] on PathCurve.",last_int_param);
            return false;
        }
        s2550( PathCurve, &last_int_param, 1, &curvature, &state); // 获取曲率
        if(state!=0){
            ROS_WARN("[HybridInvKine_ToPose] Fail to calculate curvature with param [%f] on PathCurve.",last_int_param);
            return false;
        }
        // ROS_DEBUG("Curvature = %f, radius = %f", curvature, 1.0/curvature);

        // // 发布曲率圆半径以可视化
        // Curve_Marker.header.stamp = ros::Time::now();
        // Curve_Marker.ns = "curvature_centre_point";
        // geometry_msgs::Point sample_points;
        // Curve_Marker.points.clear();
        // sample_points.x = Frenet_P[0]/1000;
        // sample_points.y = Frenet_P[1]/1000;
        // sample_points.z = Frenet_P[2]/1000;
        // Curve_Marker.points.push_back(sample_points);
        // sample_points.x = (Frenet_P[0] + std::min(1.0/abs(curvature),2000.0) * Frenet_N[0] )/1000;
        // sample_points.y = (Frenet_P[1] + std::min(1.0/abs(curvature),2000.0) * Frenet_N[1] )/1000;
        // sample_points.z = (Frenet_P[2] + std::min(1.0/abs(curvature),2000.0) * Frenet_N[2] )/1000;
        // Curve_Marker.points.push_back(sample_points);
        // Curve_Publisher.publish(Curve_Marker);

        TNB2World.matrix() <<  Frenet_T[0], Frenet_N[0], Frenet_B[0], Frenet_P[0],
                               Frenet_T[1], Frenet_N[1], Frenet_B[1], Frenet_P[1],
                               Frenet_T[2], Frenet_N[2], Frenet_B[2], Frenet_P[2],
                                        0 ,          0 ,          0 ,          1 ;

        // 建立TNB到末端的变换，并计算相应的Jacobian关系
        End2TNB = TNB2World.inverse() * End2Joint_Trans(0);
        // ROS_DEBUG_STREAM("TNB2World: \n" << TNB2World.matrix());
        // ROS_DEBUG_STREAM("End2Joint_Trans: \n" << End2Joint_Trans(0).matrix());
        // ROS_DEBUG_STREAM("End2TNB: \n " << End2TNB.matrix());
        JacobianMat(0,0) = curvature * (End2TNB(0,3)*End2TNB(1,0) - End2TNB(1,3)*End2TNB(0,0)) + End2TNB(0,0);
        JacobianMat(1,0) = curvature * (End2TNB(0,3)*End2TNB(1,1) - End2TNB(1,3)*End2TNB(0,1)) + End2TNB(0,1);
        JacobianMat(2,0) = curvature * (End2TNB(0,3)*End2TNB(1,2) - End2TNB(1,3)*End2TNB(0,2)) + End2TNB(0,2);
        JacobianMat(3,0) = curvature *  End2TNB(2,0);
        JacobianMat(4,0) = curvature *  End2TNB(2,1);
        JacobianMat(5,0) = curvature *  End2TNB(2,2);
        // ROS_DEBUG_STREAM("JacobianMat without weight: \n" << JacobianMat);

        // 将权重添加进来，并求解
        JacobianMat = JacobianMat * weight_def_Hybr.asDiagonal();
        Pseudo_inverse_solution(JacobianMat, delta, delta_pose_vec);

        // 若通过自适应权重进行回归解算，则执行以下内容
        if(regression_solu_on){ 
            // 计算自适应权重
            bool regression_flag = false; // 是否在更新自适应权重后进行回归计算
            for (int i = 0; i < 2*(JOINT_NUM-Hyb_Part_Num); i++) {
                // ROS_INFO("JointsAngle(%d)=%f, delta=%f",i+2*Hyb_Part_Num,JointsAngle(i+2*Hyb_Part_Num)*180/M_PI,delta(i+1));
                if ((std::abs(JointsAngle(i+2*Hyb_Part_Num)) > weight_joint_threshold) && ( (JointsAngle(i+2*Hyb_Part_Num)>0)==(abs(delta(i+1)>0)) )){
                    weight_adaptive(i+1) = (MAX_JOINTANGLE - std::abs(JointsAngle(i+2*Hyb_Part_Num))) / weight_joint_threshold;
                    weight_adaptive(i+1) *= weight_adaptive(i+1);
                    regression_flag = true;
                }
                else if(std::abs(JointsAngle(i+2*Hyb_Part_Num)) > MAX_JOINTANGLE-0.2){
                    weight_adaptive(i+1) = 0.3;
                }
                else
                    weight_adaptive(i+1) = 1;
            }
            // ROS_DEBUG_STREAM("weight_adaptive = " << weight_adaptive.transpose());

            // 计算雅可比矩阵，并将默认权重添加到其中,并通过雅克比矩阵进行解算
            if(regression_flag){
                // 雅克比矩阵中已经添加了weight_def，这里再添加weight_adaptive
                JacobianMat = JacobianMat * weight_adaptive.asDiagonal();
                // ROS_DEBUG_STREAM("JacobianMat with all weight: \n" << JacobianMat);
                Pseudo_inverse_solution(JacobianMat, delta, delta_pose_vec);
                // ROS_DEBUG_STREAM("regression delta = 1 degree * " << delta.transpose()*180/M_PI);
                // ROS_DEBUG_STREAM("[HybridInvKine_ToPose] regression ...");
            }
        }

        // 根据解算到的变化量，进行关节角更新
        for (int i = 2*Hyb_Part_Num; i < 2*JOINT_NUM; i++) {
            JointsAngle(i) += weight_def_Hybr(i-2*Hyb_Part_Num+1) * weight_adaptive(i-2*Hyb_Part_Num+1) * delta(i-2*Hyb_Part_Num+1);
            // ROS_DEBUG("WEIGHT%d = %f %f", i+1 , weight_def_Hybr(i-2*Hyb_Part_Num+1), weight_adaptive(i-2*Hyb_Part_Num+1) );
            // 关节角限位
            if (JointsAngle(i) > MAX_JOINTANGLE) JointsAngle(i) = MAX_JOINTANGLE;
            else if (JointsAngle(i) < -MAX_JOINTANGLE) JointsAngle(i) = -MAX_JOINTANGLE;
        }

        // 通过虚拟关节值的更新调用FTLInvKine_EndMove，计算相应的角度值的变化，并更新到BasePosi、JointsAngle中
        Ftl_End_Toler = std::max(weight_def_Hybr(0) * weight_adaptive(0) * delta(0) / 10.0, Ftl_End_Toler_store); // 临时降低FTLInvKine_EndMove的运动精度要求，使大步运动时不花过多的时间在迭代上
        Ftl_End_Toler = std::min(Ftl_End_Toler,1.0);
        if (!FTLInvKine_BaseMove(BasePosi, Hyb_Part_Num, 1.1 * weight_def_Hybr(0) * weight_adaptive(0) * delta(0), BasePosi_Exp, JointsAngle_Exp)){ // 这样的效率好像更高，1.1是一个可以加快收敛速度的系数
            ROS_WARN("[HybridInvKine_ToPose] Fail to call function FTLInvKine_EndMove.");
            return false;
        }
        BasePosi = BasePosi_Exp;
        JointsAngle.block(0, 0, 2, Hyb_Part_Num) = JointsAngle_Exp.block(0, 0, 2, Hyb_Part_Num);
        // ROS_DEBUG_STREAM("BasePosi: " << BasePosi);
        // ROS_DEBUG_STREAM("JointsAngle \n" << JointsAngle*180/M_PI);

        // 通过更新之后的关节角进行位姿解算，并计算偏差
        // -----------------------------------------
        Position_Solution(BasePosi, JointsAngle, Joint2World_Trans_Virtual);
        // Arm_Position_Pub(Joint2World_Trans_Virtual,true); // debug
        error_ang = (EndPose->matrix()-EndPoseExp).block<3,3>(0,0).norm();
        error_posi = (EndPose->matrix()-EndPoseExp).block<3,1>(0,3).norm();
        // ROS_DEBUG_STREAM("[HybridInvKine_ToPose] cnt = "<< cnt+1 << ", error: " << error_ang*81 << "deg, " << error_posi << "mm.");

        #ifdef TEST
            Eigen::Matrix<double,1,16> joint_out;
            for(int kk=0;kk<16;kk++) joint_out(kk) = JointsAngle(kk)*180/M_PI; 
            ROS_DEBUG_STREAM("[HybridInvKine_ToPose] joints = "<< joint_out);
            if(error>100 || (cnt%10)==0)
                Arm_Position_Pub(Joint2World_Trans_Virtual,true); // debug
        #endif

        // 超限
        if(++cnt>JacoIterateCNTMax) {
            // 改变权重再给一次机会
            if(TryChangeWeight_Flag){
                ROS_WARN("[HybridInvKine_ToPose] CNT OUT OF RANGE!!!!");
                Position_Solution(BasePosiCur, JointsAngleCur, Joint2World_Trans_Virtual);
                #ifdef TEST
                    cnt_test = cnt;
                    error_test = error;
                #endif
                return false;
            }
            else{
                ROS_WARN("[HybridInvKine_ToPose] CNT OUT OF RANGE! Change Weight and try again...");
                cnt = ((JacoIterateCNTMax - 100) > 0) ? (JacoIterateCNTMax - 100) : 0;
                weight_def_Hybr(0) = weight_def_Hybr(0) + 1;
                TryChangeWeight_Flag = true;
            }
        }
    }
    ROS_DEBUG("cnt = %d", cnt+1);
    ROS_DEBUG_STREAM("Finish the HybridInvKine_ToPose ================");
    Ftl_End_Toler = Ftl_End_Toler_store; // 恢复精度
    JointsAngle_Exp = JointsAngle;
    BasePosi_Exp = BasePosi;
    return true;
}

bool inverse_kinematic_ctrl::Position_Solution(double BasePosi, Arrayf_2_Joint& JointsAngle, Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3>& Joint2World_Trans)
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
        Joint2World_Trans(i) = pos;
    }
    return true;
}

bool inverse_kinematic_ctrl::Position_Solution_Joint2End(double BasePosi, Arrayf_2_Joint& JointsAngle, Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3>& End2Joint_Trans)
{
    Eigen::Isometry3d pos = Eigen::Isometry3d::Identity();

    for (int i = 2*JOINT_NUM+2; i >= 0; i--){
        if (i == 0){ // world to base
            pos.pretranslate(Eigen::Vector3d(BasePosi, 0, 0));
        }
        else if (i == 1){ // base to joint0
            pos.pretranslate(Eigen::Vector3d(ARM_BASE_X_OFFSET, 0, ARM_BASE_HEIGHT));
        }
        else if (i == 2){ // joint0 to joint1
            pos.prerotate(Eigen::AngleAxisd(JointsAngle(0), Eigen::Vector3d(0, 0, 1)));
            pos.pretranslate(Eigen::Vector3d(FIX_POLE_PART_LENGTH, 0, 0));
        }
        else if(i == 2*JOINT_NUM+2){ // joint16 to end
            pos.pretranslate(Eigen::Vector3d(END_POSITION_X, END_POSITION_Y, END_POSITION_Z)); // 末端位置
            pos.prerotate(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(1, 0, 0)));
        }
        else{ // joint_i-2 to joint_i-1
            pos.prerotate(Eigen::AngleAxisd(JointsAngle(i - 2), Eigen::Vector3d(0, 0, 1)));
            pos.pretranslate(Eigen::Vector3d((i % 2) ? ROT_SHAFT_PART_LENGTH: FIX_POLE_PART_LENGTH, 0, 0));
            pos.prerotate(Eigen::AngleAxisd((i % 2) ? M_PI / 2 : -M_PI / 2, Eigen::Vector3d(1, 0, 0)));
        }
        End2Joint_Trans(i) = pos;
    }
    return true;
}

bool inverse_kinematic_ctrl::GetJacobian(double BasePosi, Arrayf_2_Joint JointsAngle, Eigen::MatrixXd& JacobianMat, int start_joint, int end_joint){
    // check input
    if(start_joint<0 || start_joint>end_joint || end_joint>JOINT_NUM*2){
        ROS_WARN("Value of start_joint or end_joint is illegal!");
        return false;
    }

    // 更新End2Joint_Trans
    Position_Solution_Joint2End(BasePosi, JointsAngle, End2Joint_Trans); 

    // 计算JacobianMat
    JacobianMat.resize(6,end_joint-start_joint+1);
    JacobianMat.setZero();
    for(int i=start_joint;i<=end_joint;i++){
        if(i==0){ // 平动关节,
            JacobianMat(0,i-start_joint) = End2Joint_Trans(1)(0,0);
            JacobianMat(1,i-start_joint) = End2Joint_Trans(1)(0,1);
            JacobianMat(2,i-start_joint) = End2Joint_Trans(1)(0,2);
            JacobianMat(3,i-start_joint) = 0.0;
            JacobianMat(4,i-start_joint) = 0.0;
            JacobianMat(5,i-start_joint) = 0.0;
        }
        else{ // 转动关节
            JacobianMat(0,i-start_joint) = End2Joint_Trans(i+2)(0,3)*End2Joint_Trans(i+2)(1,0)-End2Joint_Trans(i+2)(1,3)*End2Joint_Trans(i+2)(0,0);
            JacobianMat(1,i-start_joint) = End2Joint_Trans(i+2)(0,3)*End2Joint_Trans(i+2)(1,1)-End2Joint_Trans(i+2)(1,3)*End2Joint_Trans(i+2)(0,1);
            JacobianMat(2,i-start_joint) = End2Joint_Trans(i+2)(0,3)*End2Joint_Trans(i+2)(1,2)-End2Joint_Trans(i+2)(1,3)*End2Joint_Trans(i+2)(0,2);
            JacobianMat(3,i-start_joint) = End2Joint_Trans(i+2)(2,0);
            JacobianMat(4,i-start_joint) = End2Joint_Trans(i+2)(2,1);
            JacobianMat(5,i-start_joint) = End2Joint_Trans(i+2)(2,2);
        }
    }
}

bool inverse_kinematic_ctrl::Arm_Position_Pub(Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3>& W2J_Trans, bool hold_on, char color){
    // 通过geometry_msgs::PoseArray的形式进行坐标系的发布
    // --------------------------------------------------
    // static 
    Poses_Array.header.frame_id = "world";
    Poses_Array.header.seq = 1;
    Poses_Array.header.stamp = ros::Time::now();

    Poses_Array.poses.clear();
    geometry_msgs::Pose pose_temp;
    for(int i=0;i<2*JOINT_NUM+3;i++){
        pose_temp.orientation.x = Eigen::Quaterniond(W2J_Trans(i).rotation()).x();
        pose_temp.orientation.y = Eigen::Quaterniond(W2J_Trans(i).rotation()).y();
        pose_temp.orientation.z = Eigen::Quaterniond(W2J_Trans(i).rotation()).z();
        pose_temp.orientation.w = Eigen::Quaterniond(W2J_Trans(i).rotation()).w();
        pose_temp.position.x = W2J_Trans(i).translation().x()/1000.0;
        pose_temp.position.y = W2J_Trans(i).translation().y()/1000.0;
        pose_temp.position.z = W2J_Trans(i).translation().z()/1000.0;

        Poses_Array.poses.push_back(pose_temp);
    }

    // 通过visualization_msgs::Marker的形式进行连杆的发布
    // --------------------------------------------------
    static int Arm_Marker_id = 0;
    if(hold_on){
        Arm_Marker_id++;
    }
    else if(Arm_Marker_id!=0){
        Arm_Marker_id = 0;
        Arm_Marker.action = visualization_msgs::Marker::DELETEALL;
        Arm_Publisher.publish(Arm_Marker);
    }
    Arm_Marker.id = Arm_Marker_id;

    Arm_Marker.header.stamp = ros::Time::now();
    // Arm_Marker.ns = "Virtual_Arm_temp";
    Arm_Marker.ns = "Virtual_Arm";
    Arm_Marker.action = visualization_msgs::Marker::ADD; // 增加一个对象，或修改已有对象
    Arm_Marker.lifetime = ros::Duration();
    Arm_Marker.points.clear();
    geometry_msgs::Point sample_points;

    // 颜色
         if(color == 'r'){ Arm_Marker.color.r = 1.0f; Arm_Marker.color.g = 0.0f; Arm_Marker.color.b = 0.0f; }
    else if(color == 'g'){ Arm_Marker.color.r = 0.0f; Arm_Marker.color.g = 1.0f; Arm_Marker.color.b = 0.0f; }
    else if(color == 'b'){ Arm_Marker.color.r = 0.0f; Arm_Marker.color.g = 0.0f; Arm_Marker.color.b = 1.0f; }
    else if(color == 'w'){ Arm_Marker.color.r = 1.0f; Arm_Marker.color.g = 1.0f; Arm_Marker.color.b = 1.0f; }
    else if(color == 'k'){ Arm_Marker.color.r = 0.0f; Arm_Marker.color.g = 0.0f; Arm_Marker.color.b = 0.0f; }
    else                 { Arm_Marker.color.r = 0.0f; Arm_Marker.color.g = 0.0f; Arm_Marker.color.b = 0.0f; }

    // 压入world坐标系所在的点
    sample_points.x = 0.0; sample_points.y = 0.0; sample_points.z = 0.0;
    Arm_Marker.points.push_back(sample_points);
    
    // 压入各个子坐标系所在的点
    for (int i = 0; i < 2*JOINT_NUM+3; i++){
        sample_points.x = W2J_Trans(i).translation().x()/1000.0;
        sample_points.y = W2J_Trans(i).translation().y()/1000.0;
        sample_points.z = W2J_Trans(i).translation().z()/1000.0;
        Arm_Marker.points.push_back(sample_points);
    }
    
    // 发布topic
    // --------------------------------------------------
    Poses_Publisher.publish(Poses_Array);
    Arm_Publisher.publish(Arm_Marker);
}

bool inverse_kinematic_ctrl::Curve_Pub(SISLCurve* curve, std::string curve_name, std::string frame_name, char color, bool hold_on, std::size_t plot_num){ 

    static int Curve_Marker_id=0;
    if(hold_on){
        Curve_Marker_id++;
    }
    else if(Curve_Marker_id!=0){
        Curve_Marker_id = 0;
        Curve_Marker.action = visualization_msgs::Marker::DELETEALL;
        Curve_Publisher.publish(Curve_Marker);
    }
    Curve_Marker.id = Curve_Marker_id;

    // 通过markers的形式进行话题发布 
    Curve_Marker.header.frame_id = frame_name;
    Curve_Marker.header.stamp = ros::Time::now();
    Curve_Marker.ns = curve_name;
    Curve_Marker.action = visualization_msgs::Marker::ADD; // 增加一个对象，或修改已有对象
    Curve_Marker.lifetime = ros::Duration();

    // 进行点的采样
    // --------------------
    // ROS_DEBUG("[Curve_Pub] Sample points...================");
    geometry_msgs::Point sample_points;
    double point[3];

    // 涉及到函数s1227的使用，可以参考sisl的example03，以及sisl中的函数draw_all_curves
    int stat, left = 0;
    Curve_Marker.points.clear();
    for (int i = 0; i < plot_num; i++){
        double t = curve->et[curve->ik - 1] + (curve->et[curve->in] - curve->et[curve->ik - 1]) * i / (plot_num - 1.0);
        s1227(curve, 0, t, &left, point, &stat); // 用于通过节点参数t对曲线上的点进行采样
        if (stat != 0) {
            ROS_WARN("[Curve_Pub - s1227] s1227 returned ERROR.");
            return false;
        }
        // ROS_DEBUG("[Curve_Pub - s1227] t = %f, point = [%f %f %f]",t, point[0], point[1], point[2]);

        sample_points.x = point[0]/1000.0;
        sample_points.y = point[1]/1000.0;
        sample_points.z = point[2]/1000.0;
        Curve_Marker.points.push_back(sample_points);
    }
    // ROS_DEBUG("[Curve_Pub] Sample points finished================.");

    // 颜色
         if(color == 'r'){ Curve_Marker.color.r = 1.0f; Curve_Marker.color.g = 0.0f; Curve_Marker.color.b = 0.0f; }
    else if(color == 'g'){ Curve_Marker.color.r = 0.0f; Curve_Marker.color.g = 1.0f; Curve_Marker.color.b = 0.0f; }
    else if(color == 'b'){ Curve_Marker.color.r = 0.0f; Curve_Marker.color.g = 0.0f; Curve_Marker.color.b = 1.0f; }
    else if(color == 'w'){ Curve_Marker.color.r = 1.0f; Curve_Marker.color.g = 1.0f; Curve_Marker.color.b = 1.0f; }
    else if(color == 'k'){ Curve_Marker.color.r = 0.0f; Curve_Marker.color.g = 0.0f; Curve_Marker.color.b = 0.0f; }
    else                 { Curve_Marker.color.r = 0.0f; Curve_Marker.color.g = 0.0f; Curve_Marker.color.b = 0.0f; }

    // 发布topic
    Curve_Publisher.publish(Curve_Marker);
}

bool inverse_kinematic_ctrl::Points_Pub(double* Points, int point_num, std::string points_name, std::string frame_name, char color){
    // 通过markers的形式进行话题发布 
    Points_Marker.header.frame_id = frame_name;
    Points_Marker.header.stamp = ros::Time::now();

    Points_Marker.ns = points_name;
    Points_Marker.id = 0;

    Points_Marker.action = visualization_msgs::Marker::ADD; // 增加一个对象，或修改已有对象
    Points_Marker.lifetime = ros::Duration();

    // 进行点的压入
    // --------------------
    geometry_msgs::Point sample_points;
    Points_Marker.points.clear();

    for (int i = 0; i < point_num; i++){
        sample_points.x = Points[3*i  ]/1000.0;
        sample_points.y = Points[3*i+1]/1000.0;
        sample_points.z = Points[3*i+2]/1000.0;
        Points_Marker.points.push_back(sample_points);
    }

    // 颜色
         if(color == 'r'){ Points_Marker.color.r = 1.0f; Points_Marker.color.g = 0.0f; Points_Marker.color.b = 0.0f; }
    else if(color == 'g'){ Points_Marker.color.r = 0.0f; Points_Marker.color.g = 1.0f; Points_Marker.color.b = 0.0f; }
    else if(color == 'b'){ Points_Marker.color.r = 0.0f; Points_Marker.color.g = 0.0f; Points_Marker.color.b = 1.0f; }
    else if(color == 'w'){ Points_Marker.color.r = 1.0f; Points_Marker.color.g = 1.0f; Points_Marker.color.b = 1.0f; }
    else if(color == 'k'){ Points_Marker.color.r = 0.0f; Points_Marker.color.g = 0.0f; Points_Marker.color.b = 0.0f; }
    else                 { Points_Marker.color.r = 0.0f; Points_Marker.color.g = 0.0f; Points_Marker.color.b = 0.0f; }

    // 发布topic
    Points_Publisher.publish(Points_Marker);
}

bool inverse_kinematic_ctrl::my_s1356(double *CtrlPoints,int CtrlPoints_Num,int* CtrlPoints_type, SISLCurve* &Curve){
    double cendpar;
    double *gpar = 0;
    int jnbpar;
    int jstat = 0;

    // ROS_DEBUG("[my_s1356] CtrlPoints:");
    // for(int i=0;i<CtrlPoints_Num*3;i++){
    //     printf("%8.3f ",CtrlPoints[i]);
    //     if(i%3==2) printf("\n");
    // }

    s1356(CtrlPoints,      // pointer to where the point coordinates are stored
          CtrlPoints_Num,  // number of points to be interpolated
          3,               // the dimension
          CtrlPoints_type, // what type of information is stored at a particular point
          0,               // no additional condition at start point
          0,               // no additional condition at end point
          1,               // open curve
          3,               // order of the spline curve to be produced
          0,               // parameter value to be used at start of curve
          &cendpar,        // parameter value at the end of the curve (to be determined)
          &Curve,          // the resulting spline curve (to be determined)
          &gpar,           // pointer to the parameter values of the points in the curve
                           // (to be determined)
          &jnbpar,         // number of unique parameter values (to be determined)
          &jstat);         // status message
    
    if (jstat < 0)
        ROS_ERROR("Error occured inside call to SISL routine.");
    else if (jstat > 0)
        ROS_ERROR("WARNING: warning occured inside call to SISL routine. \n");

    return true;
}

#ifdef TEST
    double inverse_kinematic_ctrl::get_BasePosiCur(){return BasePosiCur;}
    double inverse_kinematic_ctrl::get_BasePosi_Virtual(){return BasePosi_Virtual;}
    Arrayf_2_Joint inverse_kinematic_ctrl::get_JointsAngleCur(){return JointsAngleCur;}
    Arrayf_2_Joint inverse_kinematic_ctrl::get_JointsAngle_Virtual(){return JointsAngle_Virtual;} 

    Eigen::MatrixXd inverse_kinematic_ctrl::get_JacobianMat(){return JacobianMat;}
    SISLCurve* inverse_kinematic_ctrl::get_PathCurve(){return PathCurve;};

    Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3> inverse_kinematic_ctrl::get_End2Joint_Trans(){return End2Joint_Trans;}
    Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3> inverse_kinematic_ctrl::get_Joint2World_Trans(){return Joint2World_Trans;}
    Eigen::Array<Eigen::Isometry3d, 1, 2*JOINT_NUM+3> inverse_kinematic_ctrl::get_Joint2World_Trans_Virtual(){return Joint2World_Trans_Virtual;}

    void inverse_kinematic_ctrl::set_max_step_ang(double max_step_ang_new){max_step_ang = max_step_ang_new;}
    void inverse_kinematic_ctrl::set_max_step_posi(double max_step_posi_new){max_step_posi = max_step_posi_new;}
    void inverse_kinematic_ctrl::set_step_coef_min(double step_coef_min_new){step_coef_min = step_coef_min_new;}
    void inverse_kinematic_ctrl::set_singular_coef(double singular_coef_new){singular_coef = singular_coef_new;}
    void inverse_kinematic_ctrl::set_weight_def(Eigen::Matrix<double,2*JOINT_NUM+1,1> weight_def_new){weight_def = weight_def_new;}
    void inverse_kinematic_ctrl::set_weight_joint_threshold(double weight_joint_threshold_new){weight_joint_threshold = weight_joint_threshold_new;}
    void inverse_kinematic_ctrl::set_JacoIterateCNTMax(int JacoIterateCNTMax_new){JacoIterateCNTMax = JacoIterateCNTMax_new;}
    void inverse_kinematic_ctrl::set_regression_solu_on(bool regression_solu_on_new){regression_solu_on = regression_solu_on_new;}

    void inverse_kinematic_ctrl::set_Ftl_End_Toler(double Ftl_End_Toler_new){Ftl_End_Toler = Ftl_End_Toler_new;}
    void inverse_kinematic_ctrl::set_FeedCoef(double FeedCoef_new){FeedCoef = FeedCoef_new;}
    void inverse_kinematic_ctrl::set_FTLBaseIterateCNTMax(int FTLBaseIterateCNTMax_new){FTLBaseIterateCNTMax = FTLBaseIterateCNTMax_new;}
    void inverse_kinematic_ctrl::set_FTLEndIterateCNTMax(int FTLEndIterateCNTMax_new){FTLEndIterateCNTMax = FTLEndIterateCNTMax_new;}
    void inverse_kinematic_ctrl::set_FTL_Part_Num(int FTL_Part_Num_new){FTL_Part_Num = FTL_Part_Num_new;}

    void inverse_kinematic_ctrl::set_VirtualJointWeight_def(double VirtualJointWeight_def_new){VirtualJointWeight_def = VirtualJointWeight_def_new;}
    void inverse_kinematic_ctrl::set_Hybird_singular_coef(double Hybird_singular_coef_new){Hybird_singular_coef = Hybird_singular_coef_new;}
    void inverse_kinematic_ctrl::set_Hyb_Part_Num(int Hyb_Part_Num_new){Hyb_Part_Num = Hyb_Part_Num_new;}

#endif