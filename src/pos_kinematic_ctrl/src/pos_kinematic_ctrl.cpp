#include <pos_kinematic_ctrl.h>

pos_kinematic_ctrl::pos_kinematic_ctrl(ros::NodeHandle *nh){

    /**************/
    /* 获取配置参数 */
    /**************/

    _nh = nh;
    _motor_device = new motor_commu(_nh);
    _serial_device = new serial_commu(_nh);

    printf("\n");
    ROS_INFO(">>>pos_kinematic_ctrl config information");
    ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

    Origin_JointAngle   = parseVVF<Arrayf_2_Joint>(_nh->param<std::string>("/pos_kinematic/SnakeArm/Origin_JointAngle","[[0]]")) / 180.0 * M_PI;
    Max_StepLength      = _nh->param<float>("/pos_kinematic/SnakeArm/Max_StepLength",2.0) / 180.0 * M_PI;
    Line_Allowance      = _nh->param<float>("/pos_kinematic/SnakeArm/Line_Allowance",0.15);

    // 状态储存文件
    JointAngle_Savefile = _nh->param<std::string>("/pos_kinematic/SnakeArm/JointAngle_Savefile", "/home/ocici/ROS_CODE/3_snake_arm/src/pos_kinematic_ctrl/config/joint_angle_savefile.txt");
    MotorZero_savefile  = _nh->param<std::string>("/pos_kinematic/SnakeArm/MotorZero_savefile" , "/home/ocici/ROS_CODE/3_snake_arm/src/pos_kinematic_ctrl/config/motor_zero_savefile.txt" );

    ROS_INFO_STREAM("\nOrigin_JointAngle: 1 deg *\n" << Origin_JointAngle*180/M_PI);
    ROS_INFO("Max_StepLength: %f deg",Max_StepLength*180/M_PI);
    ROS_INFO("Line_Allowance: %f",Line_Allowance);
    
    /**********************/
    /* 计算辅助参数（查找表） */
    /**********************/

    /* LinePos_FindTab
     * 一共JOINT_NUM*3根驱动绳；每个绳子要计算一个坐标，储存在Eigen::Array3f中； */
    for (int i = 0; i < 3*JOINT_NUM; i++){
        LinePos_FindTab(i) << 0                                                                 ,
                              (          i==12 )?0:(-1 * ARRANGE_LINE_RADIUS * sin(i * M_PI_12)),
                              ((i==6) | (i==18))?0:(     ARRANGE_LINE_RADIUS * cos(i * M_PI_12)); // 6、12、18号驱动绳的坐标计算计算结果应该有0。但是由于是浮点数，实际得到的可能是一个微小值，为此进行了一个判断。
        ROS_DEBUG_STREAM("LinePos_FindTab(" << i << "): " << LinePos_FindTab(i).transpose());
    }
    printf("\n");

    /* ColiShaft_line_FindTab
     * 相加的三段分别为 面约束固定板-导线轴承（30mm）、缠绕导线轴承、导线轴承-卷线盘 的距离；
     * 13mm为缠绕导线轴承的半径、68mm为导线轴承轴线到卷线盘轴线的距离；
     * 实际大小会有一些差异，但是微小的误差对之后的运算结果影响不大； */
    ColiShaft_line_FindTab(0,0) = 30 + 13 * M_PI_2 + sqrt(68 * 68 - pow(COIL_SHAFT_DIAM_1 - 13, 2));
    ColiShaft_line_FindTab(0,1) = 30 + 13 * M_PI_2 + sqrt(68 * 68 - pow(COIL_SHAFT_DIAM_2 - 13, 2));
    ColiShaft_line_FindTab(0,2) = 30 + 13 * M_PI_2 + sqrt(68 * 68 - pow(COIL_SHAFT_DIAM_3 - 13, 2));
    ColiShaft_line_FindTab(0,3) = 30 + 13 * M_PI_2 + sqrt(68 * 68 - pow(COIL_SHAFT_DIAM_4 - 13, 2));
    ColiShaft_line_FindTab(0,4) = 30 + 13 * M_PI_2 + sqrt(68 * 68 - pow(COIL_SHAFT_DIAM_5 - 13, 2));
    ColiShaft_line_FindTab(0,5) = 30 + 13 * M_PI_2 + sqrt(68 * 68 - pow(COIL_SHAFT_DIAM_6 - 13, 2));
    ColiShaft_line_FindTab(0,6) = 30 + 13 * M_PI_2 + sqrt(68 * 68 - pow(COIL_SHAFT_DIAM_7 - 13, 2));
    ColiShaft_line_FindTab(0,7) = 30 + 13 * M_PI_2 + sqrt(68 * 68 - pow(COIL_SHAFT_DIAM_8 - 13, 2));
    ColiShaft_line_FindTab.row(1) = ColiShaft_line_FindTab.row(0).square(); // 求平方，以便之后的运算使用
    ROS_DEBUG_STREAM("ColiShaft_line_FindTab: \n" << ColiShaft_line_FindTab);printf("\n");

    /* Slope_Coil_FindTab
     * 为了用于之后的计算，这里就不开方了*/
    Slope_Coil_FindTab(0) = pow(COIL_SHAFT_PITHC,2) + pow(M_PI * (COIL_SHAFT_DIAM_1 + LINE_DIAM), 2);
    Slope_Coil_FindTab(1) = pow(COIL_SHAFT_PITHC,2) + pow(M_PI * (COIL_SHAFT_DIAM_2 + LINE_DIAM), 2);
    Slope_Coil_FindTab(2) = pow(COIL_SHAFT_PITHC,2) + pow(M_PI * (COIL_SHAFT_DIAM_3 + LINE_DIAM), 2);
    Slope_Coil_FindTab(3) = pow(COIL_SHAFT_PITHC,2) + pow(M_PI * (COIL_SHAFT_DIAM_4 + LINE_DIAM), 2);
    Slope_Coil_FindTab(4) = pow(COIL_SHAFT_PITHC,2) + pow(M_PI * (COIL_SHAFT_DIAM_5 + LINE_DIAM), 2);
    Slope_Coil_FindTab(5) = pow(COIL_SHAFT_PITHC,2) + pow(M_PI * (COIL_SHAFT_DIAM_6 + LINE_DIAM), 2);
    Slope_Coil_FindTab(6) = pow(COIL_SHAFT_PITHC,2) + pow(M_PI * (COIL_SHAFT_DIAM_7 + LINE_DIAM), 2);
    Slope_Coil_FindTab(7) = pow(COIL_SHAFT_PITHC,2) + pow(M_PI * (COIL_SHAFT_DIAM_8 + LINE_DIAM), 2);

    /* Motor_Effi_FindTab
     * 收放1mm电机实际转过的角度（度） = 360（度/圈）x 减速比 / 转动一圈收放的线长（mm） */
    Motor_Effi_FindTab(0) = 360 * MOTOR_REDUC_RATIO_1 / sqrt(Slope_Coil_FindTab(0));
    Motor_Effi_FindTab(1) = 360 * MOTOR_REDUC_RATIO_2 / sqrt(Slope_Coil_FindTab(1));
    Motor_Effi_FindTab(2) = 360 * MOTOR_REDUC_RATIO_3 / sqrt(Slope_Coil_FindTab(2));
    Motor_Effi_FindTab(3) = 360 * MOTOR_REDUC_RATIO_4 / sqrt(Slope_Coil_FindTab(3));
    Motor_Effi_FindTab(4) = 360 * MOTOR_REDUC_RATIO_5 / sqrt(Slope_Coil_FindTab(4));
    Motor_Effi_FindTab(5) = 360 * MOTOR_REDUC_RATIO_6 / sqrt(Slope_Coil_FindTab(5));
    Motor_Effi_FindTab(6) = 360 * MOTOR_REDUC_RATIO_7 / sqrt(Slope_Coil_FindTab(6));
    Motor_Effi_FindTab(7) = 360 * MOTOR_REDUC_RATIO_8 / sqrt(Slope_Coil_FindTab(7));
    ROS_DEBUG_STREAM("Motor_Effi_FindTab: \n" << Motor_Effi_FindTab);printf("\n");

    /*不开方的值除以36，用于之后的运算中*/
    Slope_Coil_FindTab = Slope_Coil_FindTab / 36.0;
    ROS_DEBUG_STREAM("Slope_Coil_FindTab: \n" << Slope_Coil_FindTab);printf("\n");

    /*****************/
    /* 机械臂状态初始化 */
    /*****************/

    SnakeArm_State = IDLE;
    sleep(3);
    if(!SnakeArm_Init()){
        SnakeArm_State = ERROR;
        ROS_ERROR("SnakeArm Init error!");
        return;
    }

    SnakeArm_State = WAIT;

    /**************/
    /* debug temp */
    /**************/

}

pos_kinematic_ctrl::~pos_kinematic_ctrl(){
    // 关闭电机

    // 关闭制动器

    // 记录当前位置（是否复位？）

    // 析构内部例化的对象
    _motor_device->~motor_commu();
    delete(_motor_device);
    _serial_device->~serial_commu();
    delete(_serial_device);
}

bool pos_kinematic_ctrl::SnakeArm_Init()
{
    // CAN设备的初始化，以及电机的可通讯情况检测
    if(_motor_device->can_device_init()!=true){
        _motor_device->motors_selfunlock(); // 电机掉电以避免电机过热
        SnakeArm_State = ERROR; 
        return false;
    }

    // 串口初始化及制动器控制测试
    _motor_device->motors_selflock(); // 电机上电自锁（上电是因为接下来要测试制动器，若不上电机械臂会在重力的作用下变形）
    if(_serial_device->Serial_init() != true){
        _motor_device->motors_selfunlock(); // 电机掉电以避免电机过热
        ROS_ERROR("[SnakeArm_Init] Fail to init the serial device and brakes!");
        SnakeArm_State = ERROR; return false;
    }
    _motor_device->motors_selfunlock(); // 电机掉电以避免电机过热

    // 机械臂姿态初始化校准
    //   读取上一次保存的机械臂姿态文件中的值，并由此更新各电机多圈绝对位置
    //   如果文件不存在，则通过手动零点标定函数来更新，并重新上电启动
    Arrayf_3_Joint Init_MotorRota_Array;
    if((file_to_array(JointAngle_Savefile, Last_JointAngle_Array     ) != true) |
       (file_to_array(MotorZero_savefile , MotorZero_Compensate_Array) != true) ) 
    {
        // 若不存在该文件则报错，需要重新执行零点标定
        ROS_ERROR_STREAM("[SnakeArm_Init] Fail to open JointAngle_Savefile or MotorZero_savefile.");
        ROS_ERROR_STREAM("[SnakeArm_Init] To solve this problem, function Manual_ZeroCalib will be run.");
        if(!Confirm_Interaction()){
            SnakeArm_State = ERROR; return false;
        }
        
        // 进行零点标定
        if(Manual_ZeroCalib() != true){
            ROS_ERROR("[SnakeArm_Init] Fail to calibration Zero manually.");
            SnakeArm_State = ERROR; return false;
        }
        ROS_INFO("[SnakeArm_Init] Manual_ZeroCalib completed.");
        ROS_WARN("[SnakeArm_Init] Please restart the arm, WITH THE POWER RESTARTED!!!");
        ROS_WARN("[SnakeArm_Init] Please restart the arm, WITH THE POWER RESTARTED!!!");
        ROS_WARN("[SnakeArm_Init] Please restart the arm, WITH THE POWER RESTARTED!!!");
        return false;
    }
    else{
        // 若成功读取文件，则根据Last_JointAngle_Array文件计算对应的电机绝对值，并存放到Next_MotorRota_Array（多圈角度）中;
        ROS_INFO_STREAM("[SnakeArm_Init] Last_JointAngle_Array read from file is: 1 degree *\n" << Last_JointAngle_Array);printf("\n");
        Last_JointAngle_Array = Last_JointAngle_Array / 180 * M_PI; //文件中储存的单位为角度，需要转化为弧度
        JointAngle2MotorAngle(&Last_JointAngle_Array, &Init_MotorRota_Array);
    }

    // 电机初始化，包括电机位置的校准和参数校准
    _motor_device->motors_selflock();
    _serial_device->Brake_Release();

    SnakeArm_State = CORRECT;
    int func_return = _motor_device->motor_commu_init(Init_MotorRota_Array);

    _serial_device->Brake_Hold();
    _motor_device->motors_selfunlock();

    // 对电机初始化函数的返回值进行处理
    if(func_return!=1){
        ROS_ERROR("[SnakeArm_Init] Fail to init the motor communition!");

        // 如果电机校准判停异常，可以通过Manual_MotorCtrl函数来进行修复
        if(func_return == -4){    
            ROS_INFO("[SnakeArm_Init] To solve this problem, function Manual_MotorCtrl maybe helpful. Run it?");
            if(!Confirm_Interaction()){
                SnakeArm_State = ERROR; return false;
            }

            if(Manual_MotorCtrl() != true){  // 进行手动电机控制
                ROS_ERROR("[SnakeArm_Init] Fail to Control motor manually.");
                return false;
            }
            _motor_device->Reset_CANanalyst(); // 重新运行init函数前需要先对CAN设备复位
        }
        else if(func_return == -6){
            ROS_WARN("[SnakeArm_Init] Please restart the arm, WITH THE POWER RESTARTED!!!");
            ROS_WARN("[SnakeArm_Init] Please restart the arm, WITH THE POWER RESTARTED!!!");
            ROS_WARN("[SnakeArm_Init] Please restart the arm, WITH THE POWER RESTARTED!!!");
            return false;
        }
        else{
            SnakeArm_State = ERROR; return false;
        }
    }

    // 初始化完成
    ROS_INFO("[SnakeArm_Init] SnakeArm Init successfully!");
    ROS_INFO("[SnakeArm_Init] Begin to wait for move request ...");
    return true;
}

bool pos_kinematic_ctrl::SnakeArm_Reboot()
{
    // close


    // reset
    SnakeArm_State = IDLE;
    if(_motor_device->Reset_CANanalyst() != 1)
        return false;
    if(_serial_device->Serial_Reset() != 1)
        return false;
    
    if(SnakeArm_Init() != 1)
        return false;

    ROS_INFO("SnakeArm reboot successfully!");
    return true;
}

bool pos_kinematic_ctrl::SnakeArm_Close(){
    
}

bool pos_kinematic_ctrl::move_to_posture(Arrayf_2_Joint Target_JointAngle_Array){
    SnakeArm_State = MOVE_AUTO;
    // ROS_DEBUG("==== move_to_posture begin ====");

    for (int i = 0; i < 2 * JOINT_NUM; i++){
        if (abs(Target_JointAngle_Array(i)) > MAX_JOINTANGLE){
            ROS_WARN("Input Target_JointAngle_Array out of range!!! Target_JointAngle_Array[%d] = %f.", i, Target_JointAngle_Array(i));
            ROS_WARN("It will be set as 30/-30degree.");
            Target_JointAngle_Array(i) = (Target_JointAngle_Array(i) > 0) ? MAX_JOINTANGLE : (-MAX_JOINTANGLE);
        }
    }

    /*******/
    /* 分步 */
    /*******/
    int Step_Num; // 步数
    std::vector<Arrayf_2_Joint ,Eigen::aligned_allocator<Arrayf_2_Joint>> Steps_JointAngle_Array; // 分步运动过程关节角（数组）

    float Delta_JointAngle_Max = (Target_JointAngle_Array - Last_JointAngle_Array).abs().maxCoeff(); // 由角度变化最大的关节决定步数
    if(Delta_JointAngle_Max < Max_StepLength){ // 只有1步时，一步到位
        Step_Num = 1;
        ROS_DEBUG("[move_to_posture] Step_Num = 1");

        Steps_JointAngle_Array.push_back(Target_JointAngle_Array);
        ROS_DEBUG_STREAM("[move_to_posture] Step 1: 1 degree * \n" << Target_JointAngle_Array * 180 / M_PI << "\n");
    }
    else{
        // 步数 与 步距
        Step_Num = ceil(Delta_JointAngle_Max / Max_StepLength);
        ROS_DEBUG("[move_to_posture] Step_Num = %d", Step_Num);
        Arrayf_2_Joint Delta_JointAngle_Array; 
        Delta_JointAngle_Array = (Target_JointAngle_Array - Last_JointAngle_Array) / Step_Num;

        // 第一步
        Steps_JointAngle_Array.push_back(Last_JointAngle_Array + Delta_JointAngle_Array);
        ROS_DEBUG_STREAM("[move_to_posture] Step 1: 1 degree * \n" << Steps_JointAngle_Array[0] * 180 / M_PI);

        // 中间的步
        for (int i = 1; i < Step_Num-1; i++){
            Arrayf_2_Joint Steps_OneMore = Steps_JointAngle_Array[i-1]+Delta_JointAngle_Array;
            Steps_JointAngle_Array.push_back(Steps_OneMore);
            ROS_DEBUG_STREAM("[move_to_posture] Step " << i+1 << " 1 degree * : \n" << Steps_JointAngle_Array[i] * 180 / M_PI);
        }

        // 最后一步到达目标位置
        Steps_JointAngle_Array.push_back(Target_JointAngle_Array);
        ROS_DEBUG_STREAM("[move_to_posture] Step " << Step_Num << " 1 degree * : \n" << Steps_JointAngle_Array[Step_Num-1] * 180 / M_PI << "\n");
    }
    // Confirm_Interaction();

    /* Line_Allowance_Ang */
    if(Step_Num > 1){
        Arrayf_3_Joint Target_MotorRota_Array, Last_MotorRota_Array;
        JointAngle2MotorAngle(&Last_JointAngle_Array, &Last_MotorRota_Array);
        JointAngle2MotorAngle(&Target_JointAngle_Array, &Target_MotorRota_Array);
        Line_Allowance_Ang = ((Target_MotorRota_Array - Last_MotorRota_Array).abs()) * (Line_Allowance/Step_Num);
        ROS_DEBUG_STREAM("Line_Allowance_Ang: \n" << Line_Allowance_Ang << "\n");
    }
    
    /***********/
    /* 按步运动 */
    /***********/
    _motor_device->motors_selflock();
    _serial_device->Brake_Release();

    Arrayf_3_Joint Next_MotorRota_Array;
    for (int step = 0; step < Step_Num; step++){
        // 计算每一步对应的电机多圈位置
        Next_JointAngle_Array = Steps_JointAngle_Array[step];
        JointAngle2MotorAngle(&Next_JointAngle_Array, &Next_MotorRota_Array);
        if(step != Step_Num-1)
            Next_MotorRota_Array = Next_MotorRota_Array - _motor_device->MotorRota_Compensate_Array + Line_Allowance_Ang; // 最后一步前，要释放一定的余量
        else
            Next_MotorRota_Array = Next_MotorRota_Array - _motor_device->MotorRota_Compensate_Array; // 最后一步到达目标点

        // 调用函数multmotors_mult_pos_lim驱动电机运动
        ROS_DEBUG_STREAM("Next_MotorRota_Array: 1 degree * \n" << Next_MotorRota_Array);
        if(_motor_device->multmotors_mult_pos_lim(Next_MotorRota_Array) != true){
            SnakeArm_State = ERROR; return false;
        }

        // 完成一步运动后，进行状态的更新
        Last_JointAngle_Array = Next_JointAngle_Array;
        pose_update_reg = 1;
        state_save();
        ROS_DEBUG("\tStep %d finished.", step+1);
        // Confirm_Interaction();
    }

    _serial_device->Brake_Hold();
    _motor_device->motors_selfunlock();
    ROS_DEBUG("     move_to_posture finish ====");

    // 运动完成，更新状态机
    SnakeArm_State = WAIT;
    return true;
}

bool pos_kinematic_ctrl::Manual_MotorCtrl()
{
    SnakeArm_State = MOVE_DEBUG;
    int motor_Num;   // 电机号。可取范围：1～24
    float angle;     // 角度。单位：度。

    Arrayf_3_Joint Next_MotorRota_Array = _motor_device->get_Cur_MotorMultAng();

    ROS_INFO("[Manual_MotorCtrl] Beginning ... ");
    ROS_INFO("[Manual_MotorCtrl] This function provides the ability to control the movement of single motor manually.");
    while(1){
        ROS_INFO("[Manual_MotorCtrl] Format of the control command is: ");
        ROS_INFO("[Manual_MotorCtrl]   [Motor_Num Angle], such as [1 -100.0] means [motor 1 move to relative angle -100degree.]");
        ROS_INFO("[Manual_MotorCtrl]   Motor_Num should in range 1~24, or 0 for all of the online motors.]");
        ROS_INFO("[Manual_MotorCtrl]   Input [-1] to finish the function.");
        
        printf("Command : ");
        std::cin >> motor_Num;
        if(motor_Num == -1){
            ROS_INFO("\t\tFinish Manual_MotorCtrl.");
            break;
        }
        else{
            std::cin >> angle;
            if(motor_Num>=0 & motor_Num<25){

                // 计算期望多圈角度
                if(motor_Num == 0) Next_MotorRota_Array = Next_MotorRota_Array + angle;
                else Next_MotorRota_Array(motor_Num-1) += angle;

                // 运动
                _motor_device->motors_selflock();
                _serial_device->Brake_Release();

                if(_motor_device->multmotors_mult_pos_lim(Next_MotorRota_Array, 2) != true){
                    SnakeArm_State = ERROR; return false;
                }
                Next_MotorRota_Array = _motor_device->get_Cur_MotorMultAng();

                _serial_device->Brake_Hold();
                _motor_device->motors_selfunlock();

                // 获取当前多圈位置并监视输出
                ROS_INFO_STREAM("Cur_MotorRota_Array : 1 degree *\n" << Next_MotorRota_Array); 
            }
            else
                ROS_WARN("Motor_Num out of range!!!");
        }
    } 

    SnakeArm_State = WAIT;
    return true;
}

bool pos_kinematic_ctrl::state_save()
{
    if(array_to_file(JointAngle_Savefile,Last_JointAngle_Array*180/M_PI) != true) return false;
    else return true;
}

bool pos_kinematic_ctrl::pose_update()
{
    if(pose_update_reg){
        pose_update_reg = 0;
    }
    else{
        return false;
    }
}

void pos_kinematic_ctrl::JointAngle2MotorAngle(Arrayf_2_Joint *JointAngle_Array, Arrayf_3_Joint *MotorRota_Array){

    // ROS_DEBUG_STREAM("[JointAngle2MotorAngle] JointAngle_Array : 1 radian * \n" << *JointAngle_Array << "\n"); // 关节角

    // 清零 & 累加
    Arrayf_3_Joint LineLength_Array = Arrayf_3_Joint::Zero(); // 相对零点位姿的长度变化
    for (uint8_t JointNum = 0; JointNum < JOINT_NUM; JointNum++){
        for (uint8_t UnitNum = 0; UnitNum <= JointNum; UnitNum++){
            LineLength_Array.col(JointNum) += Cal_LineUnitLength(JointAngle_Array,JointNum,UnitNum);
        }

        // 减去零点位置下的长度，获得相对零点位姿的长度变化（仅考虑机械臂部分）。单位：mm
        LineLength_Array.col(JointNum) -= ((JointNum+1) * ROT_SHAFT_PART_LENGTH);
    }

    // 补偿由于卷线过程带来端点的轴向移动引入轻微的偏差
    { // temp的作用域
        Eigen::Array<float,1,JOINT_NUM> temp;
        for (int i = 0; i < 3; i++){
            temp = LineLength_Array.row(i).square() / Slope_Coil_FindTab + ColiShaft_line_FindTab.row(1); // (绳长变化量 / 一圈线长 * 螺距)^2 + l0^2
            temp = LineLength_Array.row(i) + temp.sqrt() - ColiShaft_line_FindTab.row(0); // 绳长变化量 + （补偿后 - 补偿前)
            MotorRota_Array->row(i) = temp * Motor_Effi_FindTab; // 补偿后的绳长变化量 * （degree/mm）
        }
    }
    ROS_DEBUG_STREAM("[JointAngle2MotorAngle] MotorRota_Array: 1 degree * \n" << *MotorRota_Array);

    return;
}

Eigen::Array3f pos_kinematic_ctrl::Cal_LineUnitLength(Arrayf_2_Joint *JointAngle_Array, uint8_t JointNum, uint8_t UnitNum)
{
    Eigen::Array3f Lines_Length; // 最终计算结果
    // Lines_Length << -1,-1,-1; // 出现异常时的返回值

    // 验证输入参数是否在允许范围内
    // if(JointNum<0 | JointNum>=JOINT_NUM | UnitNum<0 | UnitNum>=JOINT_NUM | JointNum<UnitNum){
    //     ROS_WARN(">>>Cal_LineUnitLength gets parameters out of range! JointNum: %d, UnitNum: %d", JointNum, UnitNum);
    //     return Lines_Length;
    // }
    // ROS_DEBUG(">>>Cal_LineUnitLength get parameters: JointNum=%d, UnitNum=%d", JointNum, UnitNum); 

    // 计算相关的两个旋转矩阵
    Eigen::AngleAxisf rotation_vector_1( ((*JointAngle_Array)(0,UnitNum)), Eigen::Vector3f(0, 0, 1)); // 靠基端的关节对应的旋转矩阵
    Eigen::AngleAxisf rotation_vector_2(-((*JointAngle_Array)(1,UnitNum)), Eigen::Vector3f(0, 1, 0)); // 靠末端的关节对应的旋转矩阵
    // ROS_DEBUG_STREAM("rotation_vector_1: " << "from JointAngle_Array(0,UnitNum) = " << (*JointAngle_Array)(0,UnitNum) << "\n"<< rotation_vector_1.matrix());
    // ROS_DEBUG_STREAM("rotation_vector_2: " << "from JointAngle_Array(1,UnitNum) = " << (*JointAngle_Array)(1,UnitNum) << "\n"<< rotation_vector_2.matrix());

    // 获取三根绳子的起点、终点位置，并计算绳长
    Eigen::Vector3f Lines_Start[3],Lines_End[3];
    for (int i = 0; i < 3; i++){
        Lines_Start[i] = LinePos_FindTab(JointNum,i);
        Lines_End[i] = rotation_vector_2 * Lines_Start[i];  // 旋转
        Lines_End[i](0) += ROT_SHAFT_PART_LENGTH;           // 平动，仅x发生变化
        Lines_End[i] = rotation_vector_1 * Lines_End[i];    // 旋转

        Lines_Length(i) = (Lines_End[i]-Lines_Start[i]).norm(); // 求模
    }
    
    return Lines_Length;
}

bool pos_kinematic_ctrl::Manual_ZeroCalib()
{
    printf("\n");
    ROS_INFO("==== Please control motors manually to zero the pose of snakearm ==== ");
    if(Manual_MotorCtrl() != true ) return false; // 调用Manual_MotorCtrl，通过手动控制使机械臂姿态归零
    ROS_INFO("     Manual_ZeroCalib finished. Input 'y' to save data, 'n' to break.");
    if(!Confirm_Interaction()){
        SnakeArm_State = ERROR; return false;
    }

    // 获取当前各个电机的单圈位置，机械臂关节角归零
    MotorZero_Compensate_Array = _motor_device->get_Cur_MotorSingAng();
    Last_JointAngle_Array.setZero();
    if(array_to_file(MotorZero_savefile, MotorZero_Compensate_Array) != true) return false;
    if(array_to_file(JointAngle_Savefile,Last_JointAngle_Array*180/M_PI) != true) return false;   
    return true;
}
