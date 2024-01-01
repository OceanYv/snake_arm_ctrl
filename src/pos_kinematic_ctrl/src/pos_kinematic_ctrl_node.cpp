/**
 * @file pos_kinematic_ctrl_node.cpp
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-05-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/************/
/* 头文件声明 */
/************/
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <pos_kinematic_ctrl.h>

// 需要用到的msg、srv
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <snake_arm_msg/JointsAngle.h>
#include <snake_arm_msg/JointsAngleGo.h>
// #include <snake_arm_msg/SnakeArmPoseReq.h>

/**************/
/* 回调函数声明 */
/**************/

// 用于控制运动的回调函数
// 主要是调用pos_kine_ctrllor中的move_to_posture方法
bool Move2ExpJointAngle_callback(snake_arm_msg::JointsAngleGo::Request &req, snake_arm_msg::JointsAngleGo::Response &res);

// 收到复位请求时的回调函数(伸直状态)
bool Reposition_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

// 收到电机stop请求时的回调函数
bool StopMove_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

/**
 * @brief 获取视觉伺服模块获取的运动偏差，以用于tf_tree的发布。得到的矩阵存放在EndposeError中；
 * @param msg 
 */
void EndposeError_Callback(const geometry_msgs::Transform::ConstPtr &msg);

/**************/
/* 声明全局变量 */
/**************/

pos_kinematic_ctrl *pos_kine_ctrllor;
geometry_msgs::Transform EndposeError; // 末端位姿偏差，由视觉伺服模块获取

// 用于程序仿真的变量
bool simulation;
Arrayf_2_Joint JointAngles_simu;

/*************/
/* 节点主函数 */
/************/

int main(int argc, char *argv[]){

    ros::init(argc, argv, "pos_kinematic_ctrl_node");
    ros::NodeHandle nh;

    // 声明Publisher、Subscriber、ServiceServer
    // ros::Publisher JointsAngleCur_pub  = nh.advertise<snake_arm_msg::JointsAngle>("JointsAngleCur", 1);
    ros::ServiceServer RepositionReq_srv = nh.advertiseService("RepositionReq", Reposition_callback);
    ros::ServiceServer MotionReq_srv     = nh.advertiseService("JointsAngleExp", Move2ExpJointAngle_callback);
    ros::ServiceServer StopMove_srv      = nh.advertiseService("ArmStopMove", StopMove_callback);
    ros::Subscriber    EndposeError_sub  = nh.subscribe("EndposeError", 1, EndposeError_Callback);

    // 参数初始化
    simulation = nh.param<bool>("/pos_kinematic/simulation",true);
    JointAngles_simu.setZero();
    EndposeError.rotation.w = 1;
    EndposeError.rotation.x = EndposeError.rotation.y = EndposeError.rotation.z = 0;
    EndposeError.translation.x = EndposeError.translation.y = EndposeError.translation.z = 0;

    // 例化对象
    if(!simulation){
        pos_kine_ctrllor = new pos_kinematic_ctrl(&nh);
        if(pos_kine_ctrllor->SnakeArm_State == ERROR) return 0;
    }
    else
        ROS_WARN("[pos_kinematic_ctrl_node] Simulation Mode.");
    
    // 用于发布tf_tree
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q;

    std::string tf_names[2 * JOINT_NUM + 3];
    tf_names[0] = "base";
    tf_names[2 * JOINT_NUM + 2] = "arm_end";
    for (int i = 1; i < 2 * JOINT_NUM + 2; i++)
        tf_names[i] = "joint" + std::to_string(i - 1);
    
    // 异步线程spin
    ros::AsyncSpinner spinner(1);  // 为spin开辟1个子线程用于3个server的响应
    spinner.start();

    // 父线程用于topic发布
    Arrayf_2_Joint JointAngles_Pub = simulation?Arrayf_2_Joint::Zero():pos_kine_ctrllor->Last_JointAngle_Array;
    ros::Rate PoseUpdate_Rate(20);
    while (ros::ok())
    {
        if(simulation){
            JointAngles_Pub = JointAngles_simu;
        }
        else if(pos_kine_ctrllor->pose_update()){
            JointAngles_Pub = pos_kine_ctrllor->Last_JointAngle_Array;
        }

        // 将各个坐标系的位姿发布到tf_tree中
        transformStamped.transform.translation.y = 0;
        transformStamped.header.stamp = ros::Time::now();
        for (int i = 0; i < 2*JOINT_NUM+2; i++){
            if(i == 0){ // base->joint0
                q.setEuler(0, 0, 0);
                transformStamped.transform.translation.x = ARM_BASE_X_OFFSET/1000.0;
                transformStamped.transform.translation.z = ARM_BASE_HEIGHT/1000.0;
            }
            else if(i == 1){ // joint0->joint1
                q.setEuler(0, 0, JointAngles_Pub(i-1));
                transformStamped.transform.translation.x = FIX_POLE_PART_LENGTH/1000.0;
                transformStamped.transform.translation.z = 0;
            }
            else if(i == 2*JOINT_NUM+1){ // joint16->arm_end
                q.setEuler(0, -M_PI_2, 0);
                transformStamped.transform.translation.x =  END_POSITION_X/1000.0; // 末端位置
                transformStamped.transform.translation.y =  END_POSITION_Z/1000.0; // 末端位置
                transformStamped.transform.translation.z = -END_POSITION_Y/1000.0; // 末端位置
            }
            else{ // jointi-1->jointi
                q.setEuler(0, (i%2)?(-M_PI_2):(M_PI_2), JointAngles_Pub(i-1));
                transformStamped.transform.translation.x = (i%2)?(FIX_POLE_PART_LENGTH/1000.0):(ROT_SHAFT_PART_LENGTH/1000.0);
            }
            transformStamped.header.frame_id      = tf_names[i];
            transformStamped.child_frame_id       = tf_names[i+1];
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            broadcaster.sendTransform(transformStamped);
        }

        // 发布偏差补偿坐标系到tf_tree中(arm_end -> arm_end_comp)
        transformStamped.header.frame_id  = tf_names[2 * JOINT_NUM + 2];
        transformStamped.child_frame_id   = "arm_end_comp";  
        transformStamped.transform = EndposeError;
        broadcaster.sendTransform(transformStamped);

        PoseUpdate_Rate.sleep();
    }

    if(!simulation) delete(pos_kine_ctrllor);

    return 0;
}

/**************/
/* 回调函数实现 */
/**************/

void EndposeError_Callback(const geometry_msgs::Transform::ConstPtr &msg){
    EndposeError = *msg;
    // ROS_DEBUG_STREAM("[pos_kinematic_ctrl_node] Gotten EndposeError = \n" << EndposeError);//
    return;
}

bool Move2ExpJointAngle_callback(snake_arm_msg::JointsAngleGo::Request &req , 
                                 snake_arm_msg::JointsAngleGo::Response &res){
    // 从req中获取关节角
    Arrayf_2_Joint Target_JointAngle_Array; // 单位：rad
    for(int i=0; i<req.JointsNumber; i++){
        Target_JointAngle_Array(0,i) = req.Joints[i].Angle1;
        Target_JointAngle_Array(1,i) = req.Joints[i].Angle2;
    }

    ROS_INFO_STREAM("[Move2ExpJointAngle_callback] Target_JointAngle_Array = 1 degree * \n." << Target_JointAngle_Array / M_PI * 180);

    // 仿真模式的代码
    if(simulation){
        // usleep(1000*50);
        JointAngles_simu = Target_JointAngle_Array;
        res.success = true;
        return true;
    }

    // 等待机械臂响应
    while(pos_kine_ctrllor->SnakeArm_State != WAIT) {
        if(pos_kine_ctrllor->SnakeArm_State == ERROR){ // 机械臂状态为错误
            ROS_ERROR("SnakeArm_State is ERROR.");
            res.success = false;
            ros::shutdown();
            return false;
        }
        ROS_DEBUG("[pos_kinematic_ctrl_node]pos_kine_ctrllor->SnakeArm_State is not WAIT, value = %d",pos_kine_ctrllor->SnakeArm_State);
        usleep(100*1000); // 0.1 s
    }

    // 调用move_to_posture函数进行运动
    ROS_INFO_STREAM("[pos_kinematic_ctrl_node] Server: Begin to Move. The desired angles are: 1 degree * \n" << Target_JointAngle_Array * 180 / M_PI);
    res.success = pos_kine_ctrllor->move_to_posture(Target_JointAngle_Array);
    return true;
}

bool Reposition_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ROS_INFO("Server: Begin to reposition.");
    Arrayf_2_Joint Target_JointAngle_Array;
    Target_JointAngle_Array.setZero();

    // 仿真模式的代码
    if(simulation){
        JointAngles_simu = Target_JointAngle_Array;
        res.success = true;
        return true;
    }

    // 等待机械臂响应
    while(pos_kine_ctrllor->SnakeArm_State != WAIT) {
        if(pos_kine_ctrllor->SnakeArm_State == ERROR){ // 机械臂状态为错误
            ROS_ERROR("SnakeArm_State is ERROR.");
            res.success = false;
            ros::shutdown();
            return false;
        }
        ROS_DEBUG("[pos_kinematic_ctrl_node Reposition_callback]pos_kine_ctrllor->SnakeArm_State is not WAIT, value = %d",pos_kine_ctrllor->SnakeArm_State);
        usleep(100*1000); // 0.5 s
    }

    // 调用move_to_posture函数进行运动
    ROS_INFO_STREAM("[pos_kinematic_ctrl_node Reposition_callback] Server: Begin to Reposition.");
    res.success = pos_kine_ctrllor->move_to_posture(Target_JointAngle_Array);

    return true;
}

bool StopMove_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ROS_INFO("Server: StopMove_callback.");

    int cnt = 0;
    while(!pos_kine_ctrllor->_serial_device->Brake_Hold()){
        if(++cnt > 10) {
            ROS_ERROR("[StopMove_callback] Fail to call serial_commu::Brake_Hold()!!!");
            ROS_ERROR("[StopMove_callback] Please do it mannually!!!");
            Confirm_Interaction();
            break;
        }
        ROS_WARN("[StopMove_callback] Fail to call serial_commu::Brake_Hold()!!!");
        usleep(1000*200);
    }

    cnt = 0;
    while(!pos_kine_ctrllor->_motor_device->motors_selfunlock()){
        if(++cnt > 10) {
            ROS_ERROR("[StopMove_callback] Fail to call motor_commu::motors_selfunlock()!!!");
            ROS_ERROR("[StopMove_callback] Please do it mannually!!!");
            Confirm_Interaction();
            break;
        }
        ROS_WARN("[StopMove_callback] Fail to call motor_commu::motors_selfunlock()!!!");
        usleep(1000*200);
    }
    
    res.success = 1;
    return true;
}