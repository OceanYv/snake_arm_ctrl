/**
 * @file common_function.h
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 声明了一些基础、通用的函数
 * @version 0.1
 * @date 2022-05-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef COMMON_FUNCTION
#define COMMON_FUNCTION

#include <fstream>
#include <termio.h>
#include <ros/ros.h>
#include "controlcan.h"
#include "common_define.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


/****************/
/* 自定义数据类型 */
/****************/

typedef Eigen::Array<float,2,JOINT_NUM>  Arrayf_2_Joint; // 2行、JOINT_NUM列。存放关节角
typedef Eigen::Array<float,3,JOINT_NUM>  Arrayf_3_Joint; // 3行、JOINT_NUM列。存放驱动绳参数、电机参数等

/***********/
/* 函数声明 */
/***********/

/**
 * @brief 通过yaml文件直接输入16进制数好像会出问题。
 *        这里采用的方法是,将16进制数以字符串的形式通过yaml文件写入，再调用该函数进行字符串-数值的转换。
 * @tparam T 根据函数调用时赋值对象的类型，决定函数返回数据的类型
 * @param input_string 输入的字符串，格式需要为0xNNNN或0XNNNN
 * @return T
 */
template<typename T >
T string2hex(std::string input_string);

/** @brief Parse a vector of vector of floats from a string.
 *         该函数参考于move_base功能包源码中的【costmap_2d/src/array_parser.cpp】文件;
 *         支持空格、逗号、制表等特殊字符的识别，支持数组深度为2；
 * @param input 形如[[1.0, 2.0], [3.3, 4.4, 5.5], ...]的字符串
 * @return T Eigen::Array<float，X，Y>类型
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...]
 */
template<typename T >
T parseVVF(const std::string& input);

/**
 * @brief 将指定文件的值读入到Array数组中
 * @param file_name 待读取的文件名
 * @param array Eigen::Array<float，X，Y>类型
 * @return true 成功
 */
template<typename T >
bool file_to_array(std::string file_name, T &array);

/**
 * @brief 将Array数组的值写入到指定文件中，文件中之前的内容会被覆盖。
 *        若文件不存在，将会被自动创建
 * @param file_name 待写入的文件名
 * @param array Eigen::Array<float，X，Y>类型
 * @return true 成功
 */
template<typename T >
bool array_to_file(std::string file_name, T &array);

/**
 * @brief 确认交互函数，调用后会弹出y or n的用户确认信息
 * 
 * @return true 用户输入'y'
 * @return false 用户输入'n'
 */
bool Confirm_Interaction();

/**
 * @brief 从键盘获取数据，无需回车即返回
 * 
 * @return int 返回字符ASCII码
 */
int scanKeyboard();

/***************************************************/
/* 模板函数需要在头文件中进行实现，在Cmake中链接时才能成功 */
/***************************************************/

template<typename T >
T string2hex(std::string input_string)
{
    if (input_string[0] != '0' & (input_string[1] != 'x' | input_string[1] != 'X'))
    {
        ROS_ERROR("The input is not a number with HEX format! Input: %s", input_string.c_str());
        return -1;
    }

    T output_uchar = 0;
    for (int i = 2; i < input_string.length(); i++)
    {
        if (input_string[i] >= '0' & input_string[i] <= '9')
            output_uchar = (output_uchar << 4) + input_string[i] - '0';
        else if (input_string[i] >= 'a' & input_string[i] <= 'f')
            output_uchar = (output_uchar << 4) + input_string[i] - 'a' + 10;
        else if (input_string[i] >= 'A' & input_string[i] <= 'F')
            output_uchar = (output_uchar << 4) + input_string[i] - 'A' + 10;
        else
        {
            ROS_ERROR("The input is not a number with HEX format! Input: %s", input_string.c_str());
            return -1;
        }
    }
    // ROS_DEBUG("output_uchar is : 0x%x",output_uchar);
    return output_uchar;
}

/**
 * @brief 按列读入二维数据，支持Eigen::Matrix
 * 
 * @tparam T 
 * @param input 
 * @return T 
 */
template<typename T >
T parseVVF(const std::string& input)
{
    ROS_DEBUG_STREAM("parseVVF input:" << input);
    T result;
    int sub_1=0,sub_2=0;

    std::stringstream input_ss(input);
    int depth = 0;
    
    while (!!input_ss && !input_ss.eof()){
        switch (input_ss.peek())
        {
            case EOF:
                break;
            case '[':
                depth++;
                if (depth > 2)
                {
                    ROS_ERROR("Array depth greater than 2");
                    return result;
                }
                input_ss.get();
                break;
            case ']':
                depth--;
                if (depth < 0)
                {
                    ROS_ERROR("More close ] than open [");
                    return result;
                }
                input_ss.get();
                if (depth == 1)
                {
                    sub_1++;
                    sub_2=0;
                }
                break;
            case ',':
            case ' ':
            case '\t':
                input_ss.get();
                break;
            default:    // All other characters should be part of the numbers.
                if (depth != 2)
                {
                    std::stringstream err_ss;
                    ROS_ERROR_STREAM("Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.");
                    return result;
                }
                float value;
                input_ss >> value;
                if (!!input_ss)
                {
                    // ROS_DEBUG("value IS %f, sub_1 = %d, sub_2 = %d",value, sub_1, sub_2);
                    result(sub_2,sub_1) = value;
                    sub_2++;
                }
                break;
        }
    }

    if (depth != 0)
    {
        ROS_ERROR("Unterminated vector string.");
    }

    return result;
}

/**
 * @brief 将指定文件读入到数组中
 * 
 * @tparam T 
 * @param file_name 文件名，需提供绝对地址
 * @param array 数组，支持多种类型，元素个数须与文件中数据量保持一致
 */
template<typename T >
bool file_to_array(std::string file_name, T &array)
{
    std::ifstream fin(file_name);
    if(!fin){
        ROS_WARN_STREAM("Fail to open file : "<< file_name);
        return FALSE;
    }

    for (int j = 0; j < array.rows(); j++)
        for (int i = 0; i < array.cols(); i++)
            fin >> array(j, i);

    fin.close();
    return TRUE;
}

/**
 * @brief 将数组的内容写入到指定文件中
 * 
 * @tparam T 
 * @param file_name 文件名，需提供绝对地址
 * @param array 数组，支持多种类型
 */
template<typename T >
bool array_to_file(std::string file_name, T &array)
{
    std::ofstream fout(file_name,std::ios::out|std::ios::trunc);
    if(!fout){
        ROS_WARN_STREAM("Fail to open file : "<< file_name);
        return FALSE;
    }

    fout << array << std::endl; // std::endl用于驱动缓存区刷新

    fout.close();
    return TRUE;
}

#endif