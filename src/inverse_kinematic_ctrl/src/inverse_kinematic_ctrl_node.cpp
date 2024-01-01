/**
 * @file inverse_kinematic_ctrl_node.cpp
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 蛇形机械臂的逆运动学规划功能包顶层节点文件，用于实现功能对象的例化、与其他ROS节点的通讯等；
 * @version 0.1
 * @date 2022-11-22
 * 
 * @copyright Copyright (c) 2022 OceanYv
 * 
 */

/************/
/* 头文件声明 */
/************/

#include <thread>
#include <inverse_kinematic_ctrl.h>

#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// 需要用到的msg、srv
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <snake_arm_msg/JointsAngleGo.h>
#include <snake_arm_msg/EndPoseGo.h>
#include <snake_arm_msg/DoubleReq.h>

/**************/
/* 回调函数声明 */
/**************/

/**
 * @brief 调用逆运动学控制方法，解算运动到指定末端位姿下的过程
 * 
 * @param req ，单位m
 * @param res 
 * @return bool 
 */
bool EndposeExp_callback(snake_arm_msg::EndPoseGo::Request &req, snake_arm_msg::EndPoseGo::Response &res);

/**
 * @brief 用于期望关节角、期望基座位置请求发布循环函数
 * @param inv_kine_ctrl_obj inverse_kinematic_ctrl对象
 * @param nh ros句柄对象，用于生成ServiceClient
 */
void JointsExpReq(inverse_kinematic_ctrl& inv_kine_ctrl_obj, ros::NodeHandle& nh);

/**
 * @brief 调用BasePoseReq服务的函数，用于多线程实现
 * 
 * @param client 客户端对象
 * @param msg 请求消息
 * @param succ 调用成功的标志位
 */
void BasePoseReq_call(ros::ServiceClient& client, snake_arm_msg::DoubleReq& msg, bool& succ);

/**
 * @brief 调用JointsAngleExp服务的函数，用于多线程实现
 * 
 * @param client 客户端对象
 * @param msg 请求消息
 * @param succ 调用成功的标志位
 */
void JointsAngleReq_call(ros::ServiceClient& client, snake_arm_msg::JointsAngleGo& msg, bool& succ);

/**************/
/* 声明全局变量 */
/**************/

inverse_kinematic_ctrl* InvKineCtrl; // 创建控制器对象指针
double EndposeError_Tole_posi;       // 末端位姿偏差的容差，mm
double EndposeError_Tole_ang ;       // 末端位姿偏差的容差，弧度
bool visual_servo;

/*************/
/* 节点主函数 */
/************/

int main(int argc, char *argv[]){

    ros::init(argc, argv, "inverse_kinematic_ctrl_node");
    ros::NodeHandle nh;

    // 创建控制器对象
    InvKineCtrl = new inverse_kinematic_ctrl(&nh);

    // 创建订阅者、发布者等
    ros::ServiceServer EndposeExp_srv = nh.advertiseService("EndposeExp", EndposeExp_callback);

    // 参数初始化
    EndposeError_Tole_posi = nh.param<double>("/snake_arm_visual/visual_servo/EndposeError_Tole_posi",0.005) * 1000;
    EndposeError_Tole_ang  = nh.param<double>("/snake_arm_visual/visual_servo/EndposeError_Tole_ang" ,3.0 ) / 180.0 * M_PI;
    visual_servo = nh.param<bool>("/snake_arm_visual/visual_servo_on", false); // 是否启动视觉伺服
    printf("\n");
    ROS_INFO("EndposeError_Tole_posi  = %10.5f mm.", EndposeError_Tole_posi  );
    ROS_INFO("EndposeError_Tole_ang   = %10.5f deg.", EndposeError_Tole_ang * 180.0 / M_PI );
    ROS_INFO("visual_servo            =    %s.", visual_servo ? "true" : "false");
    EndposeError_Tole_ang = 2*sqrt(1-cos(EndposeError_Tole_ang));

    // 异步线程spin
    ros::AsyncSpinner spinner(2);  // 为spin开辟2个子线程用于1个server和1个topic的响应
    spinner.start();

    // 开启工作线程
    #ifndef TEST

        std::thread ctrl_cmd_thread(&inverse_kinematic_ctrl::inverse_kinematic_ctrl_cmd, &(*InvKineCtrl)); // 控制器对象线程
        
        if (!InvKineCtrl->simulation){
            ROS_INFO("[inverse_kinematic_ctrl_node] Start ros_request_thread.");
            std::thread ros_request_thread(JointsExpReq, std::ref(*InvKineCtrl), std::ref(nh)); // 服务请求线程
            ros_request_thread.join();
        }
        
        ctrl_cmd_thread.join();
        
    #endif

    return 1;
}

/**************/
/* 回调函数实现 */
/**************/

bool EndposeExp_callback(snake_arm_msg::EndPoseGo::Request &req, snake_arm_msg::EndPoseGo::Response &res){
    
    if(!InvKineCtrl->Listen_mode){
        ROS_WARN("InvKineCtrl IS NOT IN listening mode!!!");
        res.success = false;
        return false;
    }

    // 设置最大步长，以及最小步长因子
    double max_step_ang = 5;  // 一次迭代允许的最大转角，单位：角度
    double max_step_ang_coef = 2*sqrt(1-cos(max_step_ang / 180 * M_PI));
    double max_step_posi = 15;     // 一次迭代允许的最大位移，单为：mm
    double step_coef_min = 1.2; // 迭代的步长系数，应大于1，越大收敛越稳定但是速度慢；
    double step_coef;   // 步长系数

    // 获取期望位姿
    Eigen::Quaterniond q(req.Poses.orientation.w,
                         req.Poses.orientation.x,
                         req.Poses.orientation.y,
                         req.Poses.orientation.z);
    q.normalize();
    Eigen::AngleAxisd rot(q);

    Eigen::Isometry3d EndposeExp; // 单位mm
    EndposeExp.setIdentity();
    EndposeExp.rotate(rot);
    EndposeExp.pretranslate(Eigen::Vector3d(req.Poses.position.x, req.Poses.position.y, req.Poses.position.z) * 1000);
    ROS_DEBUG_STREAM("EndposeExp = 1 mm * \n" << EndposeExp.matrix());

    // 通过tf_tree获取当前的实际位姿
    tf::TransformListener pose_listener;
    tf::StampedTransform transform;
    Eigen::Isometry3d EndPoseCur; // 单位mm

    ros::Time now = ros::Time(0);
    pose_listener.waitForTransform("world", "arm_end_comp" , now, ros::Duration(1.0));
    pose_listener.lookupTransform("/world", "/arm_end_comp", now, transform);

    q = Eigen::Quaterniond(transform.getRotation().getW(), transform.getRotation().getX(),
                           transform.getRotation().getY(), transform.getRotation().getZ());
    EndPoseCur = Eigen::Isometry3d(q);
    EndPoseCur.pretranslate(Eigen::Vector3d(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ()) * 1000); // 变换单位为mm
    ROS_DEBUG_STREAM("EndPoseCur = 1 mm * \n" << EndPoseCur.matrix());

    // 计算期望位姿与当前实际位姿的偏差，以及偏差的范数
    Eigen::Isometry3d EndposeDelta = EndPoseCur.inverse() * EndposeExp; // 单位mm
    double EndposeError_ang  = (EndposeDelta.rotation()-Eigen::Matrix3d::Identity()).norm();
    double EndposeError_posi = EndposeDelta.translation().norm(); // 单位mm
    ROS_DEBUG_STREAM("EndposeDelta = \n" << EndposeDelta.matrix());
    ROS_DEBUG_STREAM("EndposeError_ang  = " << EndposeError_ang * 81 << "deg, EndposeError_posi = " << EndposeError_posi     << "mm." );

    // 迭代计算
    Eigen::Isometry3d EndPoseCur_Theory;
    while( (EndposeError_ang > EndposeError_Tole_ang) || (EndposeError_posi > EndposeError_Tole_posi) ){

        // 获取当前理论末端位姿
        pose_listener.waitForTransform("world", "arm_end" , now, ros::Duration(1.0));
        pose_listener.lookupTransform("/world", "/arm_end", now, transform);
        q = Eigen::Quaterniond(transform.getRotation().getW(), transform.getRotation().getX(),
                            transform.getRotation().getY(), transform.getRotation().getZ());
        EndPoseCur_Theory = Eigen::Isometry3d(q);
        EndPoseCur_Theory.pretranslate(Eigen::Vector3d(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ())*1000); // 变换单位为mm

        // 根据允许的最大步长，计算本次迭代中期望的末端运动，并转换为期望的理论末端位姿
        step_coef = std::max(std::max(EndposeError_ang / max_step_ang_coef, EndposeError_posi / max_step_posi), step_coef_min);
        ROS_DEBUG("step_coef = %f", step_coef);

        Eigen::Isometry3d delta_pose = Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd rot_vec(EndposeDelta.rotation());
        delta_pose.translate(EndposeDelta.translation() / step_coef);
        delta_pose.rotate(Eigen::AngleAxisd(rot_vec.angle() / step_coef, rot_vec.axis()));
        ROS_DEBUG_STREAM("delta_pose = \n" << delta_pose.matrix());

        Eigen::Isometry3d EndposeExpCorrect = EndPoseCur_Theory * delta_pose;
        ROS_DEBUG_STREAM("EndposeExpCorrect = \n" << EndposeExpCorrect.matrix());
        // if(!Confirm_Interaction()){ res.success = false;  return false; } //DEBUG

        // 调用逆运动学方法进行解算
        double BasePosi_Exp;
        Arrayf_2_Joint JointsAngle_Exp;
        bool state = InvKineCtrl->JacobianDiffInvKine_ToPose(EndposeExpCorrect.matrix(), BasePosi_Exp ,JointsAngle_Exp, true);
        // bool state = InvKineCtrl->HybridInvKine_ToPose(EndposeExpCorrect.matrix(), BasePosi_Exp ,JointsAngle_Exp);
        if(!state){ res.success = false; return false; } // 求解失败

        // 拉高服务请求标志位，并等待服务server响应
        if(!InvKineCtrl->simulation){
            // 置高服务请求flag
            InvKineCtrl->Call_BasePosi_Exp = BasePosi_Exp;
            InvKineCtrl->Call_JointsAngle_Exp = JointsAngle_Exp;
            InvKineCtrl->CallService_Flag = true;
        }
        
        int wait_cnt = 0;
        while(InvKineCtrl->CallService_Flag){
            if((wait_cnt % 15) == 0) ROS_DEBUG("[inverse_kinematic_ctrl_node] Waitfor Server finish.");
            usleep(1000*100);

            wait_cnt++;
            if(wait_cnt >= 300){ // 30s
                ROS_ERROR("[inverse_kinematic_ctrl_node] Fail to Call the Server.");
                printf("'y' to continue: ");
                if(!Confirm_Interaction()){
                    res.success = false;
                    return false;
                }
                else break;
            }
        }

        // 更新运动之后的值
        InvKineCtrl->BasePosiCur    = BasePosi_Exp;
        InvKineCtrl->JointsAngleCur = JointsAngle_Exp;

        // 通过tf_tree更新当前的实际位姿
        sleep(10);
        pose_listener.waitForTransform("world", "arm_end_comp" , now, ros::Duration(1.0));
        pose_listener.lookupTransform("/world", "/arm_end_comp", now, transform);

        q = Eigen::Quaterniond(transform.getRotation().getW(), transform.getRotation().getX(),
                            transform.getRotation().getY(), transform.getRotation().getZ());
        EndPoseCur = Eigen::Isometry3d(q);
        EndPoseCur.pretranslate(Eigen::Vector3d(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ()) * 1000);
        ROS_DEBUG_STREAM("EndposeExp = 1 mm * \n" << EndposeExp.matrix());
        ROS_DEBUG_STREAM("EndPoseCur = 1 mm * \n" << EndPoseCur.matrix());

        // 更新期望位姿与实际位姿的偏差，以及偏差的范数
        EndposeDelta = EndPoseCur.inverse() * EndposeExp; // 单位mm
        EndposeError_ang  = (EndposeDelta.rotation()-Eigen::Matrix3d::Identity()).norm();
        EndposeError_posi = EndposeDelta.translation().norm();
        ROS_DEBUG_STREAM("EndposeDelta = \n" << EndposeDelta.matrix());
        ROS_DEBUG_STREAM("EndposeError_ang  = " << EndposeError_ang * 81 << "deg, EndposeError_posi = " << EndposeError_posi     << "mm." );
    }

    ROS_DEBUG("[inverse_kinematic_ctrl_node] Move to EndposeExp Successfully!");
    ROS_INFO("To exit the listening mode, input \"Q\".");
    res.success = true;
    return true;
}

void JointsExpReq(inverse_kinematic_ctrl& inv_kine_ctrl_obj, ros::NodeHandle& nh){

    // 声明ServiceClient、message
    ros::ServiceClient JointsAngleReq = nh.serviceClient<snake_arm_msg::JointsAngleGo>("JointsAngleExp");
    ros::ServiceClient BasePoseReq = nh.serviceClient<snake_arm_msg::DoubleReq>("BasePoseReq");
    snake_arm_msg::JointsAngleGo JointsAngleExp;
    snake_arm_msg::DoubleReq BasePoseExp;

    bool JointsAngle_succ = false;
    bool BasePoseExp_succ = false;

    JointsAngleExp.request.JointsNumber = JOINT_NUM;
    JointsAngleExp.request.Joints.resize(JOINT_NUM);

    ros::Rate MotionReq_Rate(10);
    while (ros::ok()){

        if(inv_kine_ctrl_obj.CallService_Flag){ // 如果该位被拉高，则调用service

            // 通过两个线程并行发起两个服务请求
            //--------------------------------------------
            ROS_DEBUG("[JointsExpReq] Call two services...");

            std::thread JointsAngleReq_thread(JointsAngleReq_call, std::ref(JointsAngleReq), std::ref(JointsAngleExp), std::ref(JointsAngle_succ));
            std::thread BasePoseReq_thread(BasePoseReq_call, std::ref(BasePoseReq), std::ref(BasePoseExp), std::ref(BasePoseExp_succ));

            BasePoseReq_thread.join();
            JointsAngleReq_thread.join();

            // 服务回报信息的处理
            if( BasePoseExp_succ && JointsAngle_succ ){
                inv_kine_ctrl_obj.CallService_Flag = false;
                BasePoseExp_succ = false;
                JointsAngle_succ = false;
            }
            else
                return;
        }

        MotionReq_Rate.sleep();
        if(!inv_kine_ctrl_obj.STATE) break;
    }
}

void BasePoseReq_call(ros::ServiceClient& client, snake_arm_msg::DoubleReq& msg, bool& succ){

    msg.request.value = InvKineCtrl->Call_BasePosi_Exp/1000; // mm -> m

    if(client.call(msg)){
        ROS_DEBUG("[inverse_kinematic_ctrl_node] BasePoseReq Request: succeed!");
        succ = true;
    }
    else
        ROS_ERROR("[inverse_kinematic_ctrl_node] BasePoseReq Request: fail!");
    
    return;
}

void JointsAngleReq_call(ros::ServiceClient& client, snake_arm_msg::JointsAngleGo& msg, bool& succ){
    for(int i=0; i<JOINT_NUM; i++){
        msg.request.Joints[i].Angle1 = InvKineCtrl->Call_JointsAngle_Exp(2*i);
        msg.request.Joints[i].Angle2 = InvKineCtrl->Call_JointsAngle_Exp(2*i+1);
    }

    if(client.call(msg)){
        ROS_DEBUG("[inverse_kinematic_ctrl_node] JointsAngleExp Request: succeed!");
        succ = true;
    }
    else
        ROS_ERROR("[inverse_kinematic_ctrl_node] JointsAngleExp Request: fail!");

    return;
}