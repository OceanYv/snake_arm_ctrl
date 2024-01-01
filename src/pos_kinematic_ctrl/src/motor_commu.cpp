/**
 * @file motor_commu.cpp
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2022-05-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <motor_commu.h>

// 对帧数据进行简单输出
void frame_print(VCI_CAN_OBJ *frame_addr){
    ROS_DEBUG("\tSlave_ID:0x%04x, Data:0x%02x%02x_%02x%02x_%02x%02x_%02x%02x", frame_addr->ID,
             frame_addr->Data[0], frame_addr->Data[1], frame_addr->Data[2], frame_addr->Data[3],
             frame_addr->Data[4], frame_addr->Data[5], frame_addr->Data[6], frame_addr->Data[7]);
}

motor_commu::motor_commu(ros::NodeHandle *nh)
{
    /******************************/
    /* 通过yaml文件初始化各个配置参数 */
    /******************************/
    _nh = nh;

    // can分析仪端口参数
    can_numbervector.DeviceType = (DWORD)(_nh->param<int>("/pos_kinematic/can_config/DeviceType", 4));
    can_numbervector.DeviceInd = (DWORD)(_nh->param<int>("/pos_kinematic/can_config/DeviceInd", 0));
    can_numbervector.CANInd = (DWORD)(_nh->param<int>("/pos_kinematic/can_config/CANInd", 0));

    // can分析仪初始化配置参数
    InitConfig.AccCode = (DWORD)(_nh->param<int>("/pos_kinematic/can_config/AccCode", 0));
    InitConfig.AccMask = string2hex<DWORD>(_nh->param<std::string>("/pos_kinematic/can_config/AccMask", "0xFFFFFFFF"));
    InitConfig.Timing0 = string2hex<UCHAR>(_nh->param<std::string>("/pos_kinematic/can_config/Timing0", "0x00"));
    InitConfig.Timing1 = string2hex<UCHAR>(_nh->param<std::string>("/pos_kinematic/can_config/Timing1", "0x14"));
    InitConfig.Filter  = (UCHAR)(_nh->param<int>("/pos_kinematic/can_config/Filter", 1));
    InitConfig.Mode    = (UCHAR)(_nh->param<int>("/pos_kinematic/can_config/Mode", 0));

    // 状态储存文件
    MotorZero_savefile  = _nh->param<std::string>("/pos_kinematic/SnakeArm/MotorZero_savefile" , "/home/ocici/ROS_CODE/3_snake_arm/src/pos_kinematic_ctrl/config/motor_zero_savefile.txt" );

    StopJudge_Period = _nh->param<float>("/pos_kinematic/motor_config/StopJudge_Period", 0.1);
    StopJudge_Timeout = _nh->param<float>("/pos_kinematic/motor_config/StopJudge_Timeout", 30);
    time_out_cnt_max = StopJudge_Timeout / StopJudge_Period;

    printf("\n");
    ROS_INFO(">>>CANalyst-II device config information");
    ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    ROS_INFO("can_numbervector.DeviceType :%d "    , can_numbervector.DeviceType);
    ROS_INFO("can_numbervector.DeviceInd  :%d "    , can_numbervector.DeviceInd );
    ROS_INFO("can_numbervector.CANInd     :%d "    , can_numbervector.CANInd    );
    ROS_INFO("InitConfig.AccMask          :0x%08x ", InitConfig.AccMask         );
    ROS_INFO("InitConfig.Timing0          :0x%02x ", InitConfig.Timing0         );
    ROS_INFO("InitConfig.Timing1          :0x%02x ", InitConfig.Timing1         );
    printf("\n");
    ROS_INFO("StopJudge_Period   : %02f ", StopJudge_Period);
    ROS_INFO("StopJudge_Timeout  : %02f ", StopJudge_Timeout);

    /*********************/
    /* 配置默认的数据帧格式 */
    /*********************/
    frame_trans.RemoteFlag = 0; // 根据电机协议，固定为0（数据帧）
    frame_trans.ExternFlag = 0; // 根据电机协议，固定为0（标准帧，即11bit ID）
    frame_trans.DataLen    = 8; // 根据电机协议，固定为8byte
    frame_trans.SendType   = 1; // 单次发送(发送失败不会自动重发)

    ROS_INFO("Constructor of motor_commu is finished. Function motor_commu_init should be called next.");

} // end of motor_commu::motor_commu()

motor_commu::~motor_commu()
{
    /**********************************************/
    /* 遍历各个电机，获取电机最终状态以存档，下发停止命令 */
    /**********************************************/

    
    /**********************************/
    /* 等待CAN分析仪通讯全部完成，关闭设备 */
    /**********************************/
    VCI_ResetCAN(can_numbervector.DeviceType, can_numbervector.DeviceInd, can_numbervector.CANInd); //复位CAN通道
    usleep(1000*100);                                                                                 //延时0.1s
    VCI_CloseDevice(VCI_USBCAN2, 0);                                                                //关闭设备。

} // end of motor_commu::~motor_commu()

bool motor_commu::can_device_init()
{
    /***************************************************/
    /* 搜索计算机上是否有连接CAN分析仪, 并进行连接和初始化配置 */
    /***************************************************/
    if (start_CANanalyst() == false){
        ROS_ERROR("Start CANalyst-II deivce error!");
        return false;
    }
    else
        ROS_INFO("Open and init deivce %d successfully!", can_numbervector.DeviceInd);

    /**********************************/
    /* 检查可通讯电机情况，计算可通讯电机数 */
    /**********************************/
    printf("\n");ROS_INFO(">>>Motors test.");
    ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

    Motor_Online_Num = 0;
    Motor_Online_Flag.setZero();

    int try_cnt = 0;
    while(try_cnt < 3){
        for (int i = 0; i < 3 * JOINT_NUM; i++){
            if (motor_work(MOTOR_BASE_ADDRESS + i) != true) return false;
        }
        can_recv(3 * JOINT_NUM); motor_ack_parse();  // 接收各个电机对请求的回复，对回复进行解析（电机可通讯情况） 
        Motor_Online_Num = Motor_Online_Flag.sum();

        if(Motor_Online_Num != 3 * JOINT_NUM) try_cnt++;
        else break;
    }

    if(Motor_Online_Num != 3 * JOINT_NUM){
        ROS_WARN("Motor_Online_Num = %d", Motor_Online_Num);
        ROS_WARN_STREAM("Motor_Online_Flag:\n" << Motor_Online_Flag << "\n");
    }
    else{
        ROS_INFO("Motor_Online_Num = %d", Motor_Online_Num);
        ROS_INFO_STREAM("Motor_Online_Flag:\n" << Motor_Online_Flag << "\n");
    }

    return Confirm_Interaction();
}

int motor_commu::motor_commu_init(Arrayf_3_Joint Init_MotorRota_Array)
{
    /****************/
    /* 一些值的初始化 */
    /****************/

    recv_len = 0;
    Cur_MotorSingAng_Array.setZero();
    MotorRota_Compensate_Array.setZero();

    /******************************************/
    /* 遍历各个电机，获取电机基本状态，下发初始化参数 */
    /******************************************/

    printf("\n");
    ROS_INFO("[motor_commu_init] Begin!!!");
    get_Cur_MotorMultAng();
    ROS_INFO_STREAM("[motor_commu_init] Cur_MotorRota_Array : 1 degree *\n" << Cur_MotorRota_Array);
    if( Cur_MotorRota_Array.abs().maxCoeff() > 180.0 ) return -6; // 需要上电重启，因为下面的操作是默认所有的电机多圈角度都在[-180,180)范围内的

    /**************/
    /* 电机姿态校正 */
    /* 这一部分有点绕，建议结合详细设计说明书的4.3.3的小节来理解*/
    /**************/

    // 接下来要根据Init_MotorRota_Array、Cur_MotorRota_Array、MotorZero_savefile，对电机位置进行校正;
    ROS_INFO_STREAM("[motor_commu_init] Init_MotorRota_Array gotten is: 1 degree *\n" << Init_MotorRota_Array);

    // 读取电机软零点
    if(file_to_array(MotorZero_savefile, MotorZero_Compensate_Array) != true){
        ROS_ERROR_STREAM("[motor_commu_init] Fail to open file [" << MotorZero_savefile << "]");
        return false;
    }
    else
        ROS_INFO_STREAM("[motor_commu_init] MotorZero_savefile gotten is: 1 degree *\n" << MotorZero_Compensate_Array);
        
    // 计算各个电机的偏差
    int MotorPoseOutofRange_cnt = 0;
    for (int i = 0; i < 3 * JOINT_NUM; i++){
        if (Motor_Online_Flag(i) != 1) continue; // 跳过不在线的电机

        // 计算机械臂上电之后电机多圈角度值的理想值（理想情况下应该与上次停机时的角度对应，且处于[-180，180)区间内）
        int32_t MotorRota_ref = (int32_t)((Init_MotorRota_Array(i) + MotorZero_Compensate_Array(i)) * 100) % 36000; // 单位：0.01degree
        if (MotorRota_ref >= 18000)      MotorRota_ref -= 36000;
        else if (MotorRota_ref < -18000) MotorRota_ref += 36000;

        // 计算理想值与实际值的偏差，认为偏差在90度范围内时是正常的
        // 考虑Cur_MotorRota_Array(i)在[-180，90)或[90，180)范围内时可能出现的MotorRota_ref跨180边界情况，并对该情况下的MotorRota_ref进行修正
        if(abs(MotorRota_ref - 100 * Cur_MotorRota_Array(i)) <= 9000){}
        else if ((Cur_MotorRota_Array(i) < -90) & (abs(MotorRota_ref - 36000 - 100 * Cur_MotorRota_Array(i)) <= 9000))
            MotorRota_ref = MotorRota_ref - 36000;
        else if ((Cur_MotorRota_Array(i) >= 90) & (abs(MotorRota_ref + 36000 - 100 * Cur_MotorRota_Array(i)) <= 9000))
            MotorRota_ref = MotorRota_ref + 36000;
        else{  // 偏差大于90度，报错
            MotorPoseOutofRange_cnt++;
            ROS_ERROR("[motor_commu_init] Motor %d: delta_MotorRota bigger than 90deg. MotorRota_ref: %fdeg, MotorRota_real: %fdeg", i + 1, MotorRota_ref / 100.0, Cur_MotorRota_Array(i));
            continue;
        }

        // 偏差小于90度，纠偏
        // MotorRota_Compensate_Array为存放对电机绝对位置的补偿,单位：度，每次执行初始化函数时更新。
        //   电机重新上电后，多圈位置会清空复位为单圈绝对位置，导致从电机读到的多圈位置与机械臂零点处对应的电机多圈位置不一致，为此引入该补偿。
        //   该补偿的计算方法为：上电时，相对机械臂零点的多圈位置 - 相对电机硬零点的多圈位置
        MotorRota_Compensate_Array(i) = Init_MotorRota_Array(i) - MotorRota_ref / 100.0;
        motor_mult_pos_lim(MOTOR_BASE_ADDRESS + i, MAX_SPEED, MotorRota_ref); 
    }

    // 判断是否有偏差过大的电机，及其数量
    if(MotorPoseOutofRange_cnt!=0){
        ROS_ERROR("[motor_commu_init] Number of motors out of range is %d", MotorPoseOutofRange_cnt);
        return -4;
    }

    // 判停
    if (motors_stop_judge(2) != true) return -5;

    ROS_INFO("[motor_commu_init] Finish!!!");
    return 1;
} // end of int motor_commu::motor_commu_init()

bool motor_commu::start_CANanalyst()
{
    printf("\n");
    ROS_INFO(">>>CANalyst-II device gotten information");
    ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

    // 调用VCI_FindUsbDevice2函数，查找计算机上连接的CAN设备，获取设备基本参数
    int device_number = VCI_FindUsbDevice2(devices_info);
    if (device_number == 0){
        ROS_ERROR("No device is detected!");
        return false;
    }
    else{ // 输出各个分析仪设备的信息
        ROS_INFO("USB2CAN device number\t:%d", device_number);
        for (int device = 0; device < device_number; device++){
            ROS_INFO("Device %d:"         , device                             );
            ROS_INFO("  Serial_Num\t\t:%s", devices_info[device].str_Serial_Num); // 输出序列号
            ROS_INFO("  HW_Type\t\t:%s"   , devices_info[device].str_hw_Type   );       // 输出设备类型
            ROS_INFO("  Firmware Version\t:V%x.%x%x", (devices_info[device].fw_Version & 0xF00) >> 8,
                                                      (devices_info[device].fw_Version & 0xF0 ) >> 4,
                                                      (devices_info[device].fw_Version & 0xF  )    ); // 输出固件版本
        }
    }

    // 尝试与CAN分析仪进行连接
    if (VCI_OpenDevice(can_numbervector.DeviceType, can_numbervector.DeviceInd, 0) == 1)
        ROS_INFO("Open deivce %d successfully!", can_numbervector.DeviceInd);
    else{
        ROS_ERROR("Open deivce error!");
        return false;
    }

    // 进行初始化，并打开端口
    if (VCI_InitCAN(can_numbervector.DeviceType, can_numbervector.DeviceInd, can_numbervector.CANInd, &InitConfig) != 1){
        VCI_CloseDevice(can_numbervector.DeviceType, can_numbervector.DeviceInd);
        ROS_ERROR("Init CAN%d of device%d error!", can_numbervector.CANInd, can_numbervector.DeviceInd);
        return false;
    }
    if (VCI_StartCAN(can_numbervector.DeviceType, can_numbervector.DeviceInd, can_numbervector.CANInd) != 1){
        VCI_CloseDevice(can_numbervector.DeviceType, can_numbervector.DeviceInd);
        ROS_ERROR("Start CAN%d of device%d error!", can_numbervector.CANInd, can_numbervector.DeviceInd);
        return false;
    }

    return true;
} // end of bool motor_commu::start_CANanalyst()

bool motor_commu::Reset_CANanalyst()
{
    // 关闭在线电机
    for (int i = 0; i < 3*JOINT_NUM; i++){
        if(Motor_Online_Flag(i)!=1) continue;  // 跳过不在线的电机
        motor_close(MOTOR_BASE_ADDRESS+i);
    }
    can_recv(Motor_Online_Num);motor_ack_parse();

    // 重置CAN通讯设备
    if(VCI_UsbDeviceReset(can_numbervector.DeviceType,can_numbervector.DeviceInd,0) == 1)
        ROS_INFO("Reset deivce %d successfully!", can_numbervector.DeviceInd);
    else
        ROS_WARN("Reset deivce error!");

    usleep(1000*1000);
    return true;
} // end of bool motor_commu::Reset_CANanalyst()

bool motor_commu::can_trans(UINT slave_ID, BYTE Data[])
{
    // 将从机ID和待发数据存入frame_trans
    frame_trans.ID = slave_ID;
    std::copy(Data,Data+frame_trans.DataLen,frame_trans.Data);

    // 进行数据发送，连续失败三次的话就退出
    int try_cnt=0;
    while(VCI_Transmit(can_numbervector.DeviceType,can_numbervector.DeviceInd,can_numbervector.CANInd,&frame_trans,1)!=1){
        try_cnt++;
        if(try_cnt>=3) {
            ROS_ERROR("Fail to transmit message."); frame_print(&frame_trans);
            return false;
        }
        usleep(1000); // 重发间隔1ms
        ROS_WARN("Fail to transmit message. Retrying..."); //frame_print(&frame_trans);
    }

    // ROS_DEBUG("Message sent successfully."); frame_print(&frame_trans);

    // CAN分析仪的最大通讯能力为8000帧/s。考虑一次通讯包括一发一收，发送命令的速率应不超过4000帧/s。
    // 即帧之间的时间间隔至少为0.25ms，考虑到软件程序运行的时间以及实际测试情况，帧之间加上0.3ms的延时可以取得不错的压力测试效果；
    usleep(300); 
                                
    return true;
}

UINT motor_commu::can_recv(UINT expect_frame_num)
{
    recv_len = 0;
    int delete_frame_num = 0;

    // 检查指定数量是否超出frame_recv数组大小，多出来的部分将会被丢弃。
    if(expect_frame_num > CAN_RECV_ARRY_SIZE){
        delete_frame_num = expect_frame_num - CAN_RECV_ARRY_SIZE;
        ROS_WARN("[can_recv      ] CAN expect frame number out of range: %d. Only %d frames will be received. %d frame(s) will be deleted!", expect_frame_num, (int)(CAN_RECV_ARRY_SIZE), delete_frame_num);
        expect_frame_num = CAN_RECV_ARRY_SIZE;
    }

    // 数据接收
    // 每1.5ms进行一次接收缓存区检查，45ms后接收帧数仍不足时，则发出超时警告
    for (int i = 0; i < 30; i++){
        recv_len += VCI_Receive(can_numbervector.DeviceType,can_numbervector.DeviceInd,can_numbervector.CANInd,frame_recv+recv_len,expect_frame_num-recv_len,100);
        // ROS_DEBUG("[can_recv      ] CAN receive try: the %dth try. recv_len = %d", i+1, recv_len);
        if(recv_len == expect_frame_num)
            break;
        usleep(1500);
    }
    if(recv_len < expect_frame_num) ROS_WARN("[can_recv      ] CAN receive timeout! %d/%d frames received!", recv_len, expect_frame_num); // 接收帧数不足，警告接收超时

    // 输出接收信息
    // for (int i = 0; i < recv_len; i++){
    //     ROS_DEBUG("CAN received frame:");
    //     frame_print(frame_recv+i);
    // }

    // 接收并丢弃多出来的帧
    if(delete_frame_num != 0){
        VCI_CAN_OBJ frame_delete;
        int delete_cnt=0;
        ROS_WARN("[can_recv      ] %d frame(s) will be deleted:", delete_frame_num);
        while(delete_cnt<delete_frame_num){
            if(VCI_Receive(can_numbervector.DeviceType,can_numbervector.DeviceInd,can_numbervector.CANInd,&frame_delete,1,100)==1){
                delete_cnt ++;
                frame_print(&frame_delete);
            }
        }
    }
    
    return recv_len;
}

bool motor_commu::can_buff_clear(){
    usleep(1000);
    int frame_num = VCI_GetReceiveNum(can_numbervector.DeviceType,can_numbervector.DeviceInd,can_numbervector.CANInd);
    if(frame_num!=0){
        ROS_DEBUG("[can_buff_clear] %d frames in buffer will be cleaned.", frame_num);
        if (VCI_ClearBuffer(can_numbervector.DeviceType, can_numbervector.DeviceInd, can_numbervector.CANInd) != 1) return false;
    }
    return true;
}

void motor_commu::motors_selflock(){
    ROS_DEBUG("Motors selflock.");
    for (int i = 0; i < 3 * JOINT_NUM; i++)
        motor_rela_pos_lim(MOTOR_BASE_ADDRESS + i, MAX_SPEED, 0);

    usleep((int)(1000*1000*MOROR_SELFLOCK_WAITTIME));
    can_buff_clear();
}

bool motor_commu::motors_selfunlock(){
    ROS_DEBUG("[motors_selfunlock] Cancel Motors' selflock.");

    for(int cnt = 0 ; cnt < 3 ; cnt ++){
        for (int i = 0; i < 3 * JOINT_NUM; i++) motor_stop(MOTOR_BASE_ADDRESS + i);
        if(can_recv(Motor_Online_Num) == Motor_Online_Num) return true;
    }
    
    ROS_ERROR("[motors_selfunlock] Fail.");
    return false;
}

bool motor_commu::multmotors_mult_pos_lim(Arrayf_3_Joint MotorRota_Array, float delta_tolerance){
    // 发送运动指令
    for (int i = 0; i < 3*JOINT_NUM; i++){
        if(Motor_Online_Flag(i)!=1) continue;  // 跳过不在线的电机
        if(!motor_mult_pos_lim(MOTOR_BASE_ADDRESS+i,MAX_SPEED,100*MotorRota_Array(i))) return false;
    }

    // 判停
    if(motors_stop_judge(delta_tolerance)!=true) return false;
    else return true;
}

Arrayf_3_Joint motor_commu::get_Cur_MotorMultAng(){
    for (int i = 0; i < 3 * JOINT_NUM; i++){
        if(Motor_Online_Flag(i)!=1) continue;  // 跳过不在线的电机
        motor_read_mult_ang(MOTOR_BASE_ADDRESS + i);
    }
    can_recv(Motor_Online_Num); motor_ack_parse();

    return Cur_MotorRota_Array;
}

Arrayf_3_Joint motor_commu::get_Cur_MotorSingAng(){
    for (int i = 0; i < 3 * JOINT_NUM; i++)
        motor_read_sing_ang(MOTOR_BASE_ADDRESS + i);
    can_recv(3 * JOINT_NUM); motor_ack_parse();
    ROS_INFO_STREAM("Cur_MotorSingAng_Array : 1 degree *\n" << Cur_MotorSingAng_Array);
    return Cur_MotorSingAng_Array;
}

bool motor_commu::motor_close(UINT slave_ID){
    BYTE send_data[8];
    send_data[0] = MOTOR_CLOSE;

    if(can_trans(slave_ID, send_data) == true){
        // ROS_DEBUG("MOTOR_CLOSE COMMAND SENT.");printf("\n");
        return true;
    }
    else{
        ROS_WARN("MOTOR_CLOSE FAIL!!!"); //printf("\n");
        return false;
    }
}

bool motor_commu::motor_stop(UINT slave_ID){
    BYTE send_data[8];
    send_data[0] = MOTOR_STOP;
    
    if(can_trans(slave_ID, send_data) == true){
        // ROS_DEBUG("[motor_stop] MOTOR_STOP COMMAND SENT.");
        return true;
    }
    else{
        ROS_WARN("[motor_stop] MOTOR_STOP FAIL!!!");
        return false;
    }
}

bool motor_commu::motor_work(UINT slave_ID){
    BYTE send_data[8];
    send_data[0] = MOTOR_WORK;
    
    if(can_trans(slave_ID, send_data) == true){
        ROS_DEBUG("MOTOR_WORK COMMAND SENT.");
        return true;
    }
    else{
        ROS_WARN("MOTOR_WORK FAIL!!!");
        return false;
    }
}

bool motor_commu::motor_read_mult_ang(UINT slave_ID){
    BYTE send_data[8];
    send_data[0] = MOTOR_READ_MULT_ANG;
    
    if(can_trans(slave_ID, send_data) == true){
        // ROS_DEBUG("[motor_read_mult_ang] MOTOR_READ_MULT_ANG COMMAND SENT.");
        return true;
    }
    else{
        ROS_WARN("MOTOR_READ_MULT_ANG FAIL!!!");printf("\n");
        return false;
    }
}

bool motor_commu::motor_read_sing_ang(UINT slave_ID){
    BYTE send_data[8];
    send_data[0] = MOTOR_READ_SING_ANG;
    
    if(can_trans(slave_ID, send_data) == true){
        ROS_DEBUG("MOTOR_READ_SING_ANG SENT."); // printf("\n");
        return true;
    }
    else{
        ROS_WARN("MOTOR_READ_SING_ANG FAIL!!!"); // printf("\n");
        return false;
    }
}

bool motor_commu::motor_speed_control(UINT slave_ID, float speedControl){
    BYTE send_data[8];
    int32_t speedControl_d = (int32_t)(speedControl *100);

    send_data[0] = MOTOR_SPEED_CONTROL;
    send_data[4] = *( uint8_t *)(&speedControl_d     );
    send_data[5] = *((uint8_t *)(&speedControl_d) + 1);
    send_data[6] = *((uint8_t *)(&speedControl_d) + 2);
    send_data[7] = *((uint8_t *)(&speedControl_d) + 3);

    if(can_trans(slave_ID, send_data) == true){
        ROS_DEBUG("MOTOR_SPEED_CONTROL SENT.");printf("\n");
        return true;
    }
    else{
        ROS_WARN("MOTOR_SPEED_CONTROL FAIL!!!");printf("\n");
        return false;
    }
}

bool motor_commu::motor_mult_pos_lim(UINT slave_ID, uint16_t max_speed, int32_t angle_control){
    BYTE send_data[8];

    send_data[0] = MOTOR_MULT_POS_LIM;
    send_data[1] = 0x00;
    send_data[2] = *( uint8_t *)(&max_speed         );
    send_data[3] = *((uint8_t *)(&max_speed)     + 1);
    send_data[4] = *( uint8_t *)(&angle_control     );
    send_data[5] = *((uint8_t *)(&angle_control) + 1);
    send_data[6] = *((uint8_t *)(&angle_control) + 2);
    send_data[7] = *((uint8_t *)(&angle_control) + 3);

    if(can_trans(slave_ID, send_data) == true){
        ROS_DEBUG("[motor_mult_pos_lim] MOTOR_MULT_POS_LIM SENT, slave_ID = %d, angle = %f", (int)slave_ID, angle_control/100.0);
        Next_MotorRota_Array(slave_ID-MOTOR_BASE_ADDRESS) = angle_control/100.0;
        return true;
    }
    else{
        ROS_WARN("[motor_mult_pos_lim] MOTOR_MULT_POS_LIM FAIL!!!");
        return false;
    }
}

bool motor_commu::motor_rela_pos_lim(UINT slave_ID, uint16_t max_speed, int32_t angle_control){

    BYTE send_data[8];

    send_data[0] = MOTOR_RELA_POS_LIM;
    send_data[1] = 0x00;
    send_data[2] = *( uint8_t *)(&max_speed         );
    send_data[3] = *((uint8_t *)(&max_speed)     + 1);
    send_data[4] = *( uint8_t *)(&angle_control     );
    send_data[5] = *((uint8_t *)(&angle_control) + 1);
    send_data[6] = *((uint8_t *)(&angle_control) + 2);
    send_data[7] = *((uint8_t *)(&angle_control) + 3);

    if(can_trans(slave_ID, send_data) == true){
        // ROS_DEBUG("[motor_rela_pos_lim] MOTOR_RELA_POS_LIM.");
        Next_MotorRota_Array(slave_ID-MOTOR_BASE_ADDRESS) = Cur_MotorRota_Array(slave_ID-MOTOR_BASE_ADDRESS) + angle_control/100.0;
        return true;
    }
    else{
        ROS_WARN("[motor_rela_pos_lim] MOTOR_RELA_POS_LIM FAIL!!!");
        return false;
    }
}

bool motor_commu::motors_stop_judge(float delta_tolerance){
    
    if(delta_tolerance<0.5) delta_tolerance=0.5;
    else if(delta_tolerance>10)  delta_tolerance=10;

    ROS_DEBUG("[motors_stop_judge] Begin ... ");
    for (int time_out_cnt = 0; time_out_cnt < time_out_cnt_max; time_out_cnt++){

        usleep(1000*1000*StopJudge_Period); // StopJudge_Period 秒

        // 清除CAN缓存区，以准备接收判停数据
        can_buff_clear();
        get_Cur_MotorMultAng();
        // ROS_DEBUG_STREAM("[motors_stop_judge] Cur_MotorRota_Array : 1 degree *\n" << Cur_MotorRota_Array);

        // 判断是否达到判停标准
        int stop_cnt = 0;
        Motor_Stop_Flag.setZero();
        for (int i = 0; i < 3*JOINT_NUM; i++){
            if(Motor_Online_Flag(i)!=1)
                Motor_Stop_Flag(i) = -1;
            else if(abs(Cur_MotorRota_Array(i)-Next_MotorRota_Array(i)) <= delta_tolerance){
                Motor_Stop_Flag(i) = 1;
                stop_cnt++;
            }
        }

        // 判停成功
        if(stop_cnt == Motor_Online_Num){
            ROS_INFO("[motors_stop_judge] Stop judge successfully.");
            return true;
        }

        // 判停暂未成功
        ROS_INFO_STREAM("[motors_stop_judge] Motor_Stop_Flag = \n" << Motor_Stop_Flag);
    }
    
    // // 判停超时，进行人工辅助判停
    // ROS_INFO_STREAM("\n[motors_stop_judge] Cur_MotorRota_Array  = 1 degree *\n" << Cur_MotorRota_Array <<
    //                 "\n[motors_stop_judge] Exp_MotorRota_Array = 1 degree *\n" <<  Next_MotorRota_Array);
    // ROS_WARN("[motors_stop_judge] Stop judge failed. If you think motors stoped successfully, print 'y':");
    // return Confirm_Interaction();
    return true;
}

void motor_commu::motor_ack_parse(){
    for (int i = 0; i < recv_len; i++) {
        switch (frame_recv[i].Data[0]) {
            case MOTOR_SPEED_CONTROL:
            case MOTOR_MULT_POS_LIM:
            case MOTOR_RELA_POS_LIM:{
                int8_t temperature = 0;
                int16_t current    = 0;
                int16_t speed      = 0;
                uint16_t encoder   = 0;

                temperature =   temperature | frame_recv[i].Data[1];
                current     = ((    current | frame_recv[i].Data[3]) << 8) | frame_recv[i].Data[2];
                speed       = ((      speed | frame_recv[i].Data[5]) << 8) | frame_recv[i].Data[4];
                encoder     = ((    encoder | frame_recv[i].Data[7]) << 8) | frame_recv[i].Data[6];

                if(frame_recv[i].Data[0] == MOTOR_SPEED_CONTROL)
                    ROS_DEBUG("[motor_ack_parse] slave_ID: 0x%04x, 0xA2: MOTOR_SPEED_CONTROL.", frame_recv[i].ID);
                else if(frame_recv[i].Data[0] == MOTOR_MULT_POS_LIM)
                    ROS_DEBUG("[motor_ack_parse] slave_ID: 0x%04x, 0xA4: MOTOR_MULT_POS_LIM.", frame_recv[i].ID);
                else
                    ROS_DEBUG("[motor_ack_parse] slave_ID: 0x%04x, 0xA8: MOTOR_RELA_POS_LIM.", frame_recv[i].ID);

                if(temperature > 40){
                    ROS_WARN("[motor_ack_parse] slave_ID: 0x%04x, temperature = %d dC", frame_recv[i].ID, (int)(temperature));
                }
                // ROS_DEBUG("\ttemperature:%ddC, current:%+2.03fA, speed:%+5ddps, encoder:%5d/16383.",temperature, current/62.06, speed, encoder);
                break;
            }

            case MOTOR_CLOSE:{
                ROS_DEBUG("[motor_ack_parse] slave_ID: 0x%04x, 0x80: MOTOR_CLOSE.", frame_recv[i].ID);
                break;
            }

            case MOTOR_STOP:{
                ROS_DEBUG("[motor_ack_parse] slave_ID: 0x%04x, 0x81: MOTOR_STOP.", frame_recv[i].ID);
                break;
            }
                
            case MOTOR_WORK:{
                ROS_DEBUG("[motor_ack_parse] slave_ID: 0x%04x, 0x88: MOTOR_WORK.", frame_recv[i].ID);
                Motor_Online_Flag(frame_recv[i].ID-MOTOR_BASE_ADDRESS) = true; // 该电机可通讯
                break;
            }

            case MOTOR_READ_MULT_ANG:{
                int64_t motorAngle = 0; // 单位:0.01degree

                *((uint8_t *)(&motorAngle)  ) = frame_recv[i].Data[1];
                *((uint8_t *)(&motorAngle)+1) = frame_recv[i].Data[2];
                *((uint8_t *)(&motorAngle)+2) = frame_recv[i].Data[3];
                *((uint8_t *)(&motorAngle)+3) = frame_recv[i].Data[4];
                *((uint8_t *)(&motorAngle)+4) = frame_recv[i].Data[5];
                *((uint8_t *)(&motorAngle)+5) = frame_recv[i].Data[6];
                *((uint8_t *)(&motorAngle)+6) = frame_recv[i].Data[7];
                *((uint8_t *)(&motorAngle)+7) = (frame_recv[i].Data[7] >= 0x80) ? 0xFF : 0x00; // 根据最高位补全该有符号数的补码

                // ROS_DEBUG("[motor_ack_parse] slave_ID: 0x%04x, 0x92: MOTOR_READ_MULT_ANG.", frame_recv[i].ID);
                // ROS_DEBUG("[motor_ack_parse]             motorAngle: %f deg.",motorAngle/100.0);
                Cur_MotorRota_Array(frame_recv[i].ID-MOTOR_BASE_ADDRESS) = motorAngle/100.0; // 储存多圈角度
                break;
            }

            case MOTOR_READ_SING_ANG:{
                uint16_t circleAngle = 0;
                int16_t  circleAngle_t = 0;

                *((uint8_t *)(&circleAngle)  ) = frame_recv[i].Data[6];
                *((uint8_t *)(&circleAngle)+1) = frame_recv[i].Data[7];

                circleAngle_t = (circleAngle >= 18000) ? (circleAngle - 36000) : circleAngle; // // 从[0,360)内转换到[-180，180)范围内
                // ROS_DEBUG("[motor_ack_parse] slave_ID: 0x%04x, 0x94: MOTOR_READ_SING_ANG.", frame_recv[i].ID);
                // ROS_DEBUG("\tcircleAngle:%fdegree.",circleAngle_t/100.0);
                Cur_MotorSingAng_Array(frame_recv[i].ID-MOTOR_BASE_ADDRESS) = circleAngle_t/100.0; // 储存单圈角度
                break;
            }

            default:{
                ROS_WARN("Not supported command.");
                break;
            }
        }
    }
    return;
}