/**
 * @file serial_commu.cpp
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-05-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <serial_commu.h>

serial_commu::serial_commu(ros::NodeHandle *nh)
{
    /**************************/
    /* 通过yaml文件初始化配置参数 */
    /**************************/
    int serial_timeout_int;
    _nh = nh;

    serial_port        = _nh->param<std::string>("/pos_kinematic/serial_config/Port","/dev/ttyUSB0");
    serial_baudrate    = _nh->param("/pos_kinematic/serial_config/BaudRate",115200);
    serial_timeout_int = _nh->param("/pos_kinematic/serial_config/TimeOut",500);
    
    printf("\n");
    ROS_INFO(">>>Serial device config information");
    ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    ROS_INFO("serial_port                 :%s ", serial_port.c_str());
    ROS_INFO("serial_baudrate             :%d ", serial_baudrate    );
    ROS_INFO("serial_timeout              :%d ", serial_timeout_int );

    serial_timeout = serial::Timeout::simpleTimeout(serial_timeout_int);
}

serial_commu::~serial_commu()
{
    my_serialport->close();
    delete(my_serialport);
}

bool serial_commu::Serial_init(){

    // 打开串口
    ROS_INFO_STREAM("Try to open " << serial_port << ".");
    ROS_INFO_STREAM("If fail, please open access of the port [sudo chmod 777 " << serial_port << "]");
    my_serialport  = new serial::Serial(serial_port, serial_baudrate, serial_timeout);
    
    if(my_serialport->isOpen() != true){
        ROS_ERROR_STREAM("Fail to open serial. Port: " << serial_port << ", serial_baudrate: " << serial_baudrate);
        return false;
    }
    else
        ROS_INFO_STREAM("Open serial " << serial_port << " successfully.");

    // 制动器释放、抱紧测试
    Brake_Release();
    ROS_INFO("Have All brakes been released?");
    if(!Confirm_Interaction()){
        ROS_ERROR("Fail to release all of the brakes.");
        return false;
    }

    Brake_Hold();
    ROS_INFO("Have All brakes been held on?");
    if(!Confirm_Interaction()){
        ROS_ERROR("Fail to hold on all of the brakes.");
        return false;
    }

    return true;
}

bool serial_commu::Serial_Reset(){
    my_serialport->close();
    delete(my_serialport);
}

bool serial_commu::Brake_Release(){
    ROS_DEBUG("Brake Release.");
    std::string msg;

    msg  = "OA()"; // 全部释放命令
    msg += "\r\n"; // 止符都为回车且换行[\r\n]

    usleep(1000*50);
    if(my_serialport->write(msg) != msg.length()){
        ROS_ERROR("Fail to send Brake_Release message.");
        return false;
    }
    return true;
}

bool serial_commu::Brake_Hold(){
    ROS_DEBUG("Brake Hold.");
    std::string msg;

    msg  = "CA()"; // 全部抱紧命令
    msg += "\r\n"; // 止符都为回车且换行[\r\n]

    if(my_serialport->write(msg) != msg.length()){
        ROS_ERROR("Fail to send Brake_Hold message.");
        return false;
    }

    usleep((int)(1000*1000*BRAKE_HOLD_WAITTIME));
    return true;
}