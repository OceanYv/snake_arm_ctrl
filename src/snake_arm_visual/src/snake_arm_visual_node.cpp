/**
 * @file snake_arm_visual_node.cpp
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/************/
/* 头文件声明 */
/************/
#include <thread>
#include <snake_arm_visual.h>
#include <snake_visual_servo.h>
#include <snake_arm_msg/EndPoseGo.h>

/*************/
/* 节点主函数 */
/************/

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "snake_arm_visual_node");
    ros::NodeHandle nh;

    bool from_file = nh.param<bool>("/snake_arm_visual/pointcloud_from_file", false); // 用于控制路径规划的点云来源
    bool visual_servo = nh.param<bool>("/snake_arm_visual/visual_servo_on", true); // 是否启动视觉伺服

    ROS_INFO_STREAM("from_file      : " << (from_file ? "true" : "false"));
    ROS_INFO_STREAM("visual_servo   : " << (visual_servo ? "true" : "false"));
    printf("\n");

#ifndef TEST
    // wait for camera device
    bool camera_on = false;
    if( (!from_file) || visual_servo ) {
        realsense2_camera::DeviceInfo srv;
        ros::ServiceClient client = nh.serviceClient<realsense2_camera::DeviceInfo>("/camera/realsense2_camera/device_info");

        for (size_t i = 0; i < 20; i++) {
            if(!client.call(srv)){
                if(i==19){
                    ROS_ERROR("Fail to start the camera.");
                    return false;
                }
                ROS_INFO("Call camera device information ... ");
                sleep(1);
            }
            else{
                camera_on = true;
                break;
            }
        }
        ROS_INFO_STREAM("[snake_arm_visual] camera device_name         = " << srv.response.device_name);   
        ROS_INFO_STREAM("[snake_arm_visual] camera update_id           = " << srv.response.firmware_update_id);
        ROS_INFO_STREAM("[snake_arm_visual] camera firmware_version    = " << srv.response.firmware_version);
        ROS_INFO_STREAM("[snake_arm_visual] camera sensors             = " << srv.response.sensors);
        ROS_INFO_STREAM("[snake_arm_visual] camera serial_number       = " << srv.response.serial_number);
        ROS_INFO_STREAM("[snake_arm_visual] camera usb_type_descriptor = " << srv.response.usb_type_descriptor);
    }
    sleep(3);
#endif

    // 创建控制器对象
    snake_arm_visual SnakeVisual(&nh);
    snake_visual_servo* visual_servor;
    if(visual_servo) visual_servor = new snake_visual_servo(&nh);

#ifndef TEST
    // 创建多个线程分别用于执行命令控制函数、发出运动请求
    std::thread visual_servo_thread;
    if(visual_servo) visual_servo_thread = std::thread(&snake_visual_servo::Visual_Servo_Loop, &(*visual_servor));
    std::thread ctrl_cmd_thread(&snake_arm_visual::snake_arm_visual_cmd, &SnakeVisual); 

    ctrl_cmd_thread.join();
    if(visual_servo) visual_servo_thread.join();

    // 程序结束的处理
    if(visual_servo) delete(visual_servor);

    // 关闭相机
    if(camera_on){
        std_srvs::SetBool srv;
        srv.request.data = false;
        ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("/camera/enable");
        for(int i=0;i<5;i++) {
            ROS_INFO("Close camera device ... ");
            if(client.call(srv)) break;
            sleep(1);
        }
        sleep(1);
    }

#endif

    return 0;
}

