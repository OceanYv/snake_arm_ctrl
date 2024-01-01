#ifndef SERIAL_COMMU
#define SERIAL_COMMU

#include "common_function.h"
#include <serial/serial.h>

class serial_commu
{
public:
    /**
     * @brief Construct a new serial commu object
     *        只进行了各种配置参数的预读取，没有进行任何实质性的通讯和配置工作
     *        之后要需要自行调用Serial_init函数，才能够使用串口
     */
    serial_commu(ros::NodeHandle *nh);

    /**
     * @brief Destroy the serial commu object
     * 
     */
    ~serial_commu();

    /**
     * @brief 串口初始化，主要功能为创建Serial对象，并尝试打开串口，之后测试制动器的开关
     * @return true 成功打开串口
     */
    bool Serial_init();

    /**
     * @brief 串口复位，之后再使用的话需要调用Serial_init函数重新进行初始化
     * @return true 复位成功
     */
    bool Serial_Reset();

    /**
     * @brief 发送制动器释放命令
     * @return true 发送成功
     */
    bool Brake_Release();

    /**
     * @brief 发送制动器抱紧命令
     * @return true 发送成功
     */
    bool Brake_Hold();

private:
    ros::NodeHandle *_nh;  

    serial::Serial *my_serialport;

    std::string serial_port;
    int serial_baudrate;
    serial::Timeout serial_timeout;
    
};

#endif