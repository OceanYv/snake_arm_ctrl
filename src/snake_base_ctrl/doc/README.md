## input

### service
    - 名称：BasePoseReq
    - 类型：snake_arm_msg/DoubleReq
    - 描述：请求机械臂底座运动到指定位置
    - 单位：meter

## output

### tf_tree
    - `world -> base`


## 非仿真模式下（取决于launch文件中的simulation参数设置），使用前的硬件相关注意事项

1. 需要连接串口，并开启串口权限 `sudo chmod 777 /dev/ttyUSBN`
   其中，N为编号，通常为0，但是如果插了多个设备，则需要自行判断
   查看设备编号的方式为 `$ ls /dev/ttyUSB*`

2. 实验室的小车使用时，开机后需要释放急停、按绿色按钮使之开始工作；
   如未响应，可按下急停、红色按钮后重复前述操作，或尝试重新上电。