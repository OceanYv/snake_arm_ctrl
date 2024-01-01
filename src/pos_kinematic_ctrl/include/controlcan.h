#ifndef CONTROLCAN_H
#define CONTROLCAN_H

////文件版本：v2.02 20190609
//接口卡类型定义

#define VCI_USBCAN1 3
#define VCI_USBCAN2 4 // USBCAN-2A或USBCAN-2C或CANalyst-II
#define VCI_USBCAN2A 4

#define VCI_USBCAN_E_U 20
#define VCI_USBCAN_2E_U 21

//函数调用返回状态值
#define STATUS_OK 1
#define STATUS_ERR 0

#define USHORT unsigned short int
#define BYTE unsigned char
#define CHAR char
#define UCHAR unsigned char
#define UINT unsigned int
#define DWORD unsigned int
#define PVOID void *
#define ULONG unsigned int
#define INT int
#define UINT32 UINT
#define LPVOID void *
#define BOOL BYTE
#define TRUE 1
#define FALSE 0

// ZLGCAN系列分析仪参数
typedef struct _VCI_BOARD_INFO
{
	USHORT hw_Version;		 // 硬件版本号，4byte
	USHORT fw_Version;		 // 固件版本号，4byte
	USHORT dr_Version;		 // 驱动版本号，4byte
	USHORT in_Version;		 // 接口库版本号，4byte
	USHORT irq_Num;			 // reserve，4byte
	BYTE can_Num;			 // CAN通道路数，1byte
	CHAR str_Serial_Num[20]; // 分析仪序列号，20byte字符串
	CHAR str_hw_Type[40];	 // 硬件类型，40byte字符串
	USHORT Reserved[4];		 // reserve，4byte
} VCI_BOARD_INFO, *PVCI_BOARD_INFO;

// CAN帧结构体,即1个结构体表示一个帧的数据结构。收发相同
typedef struct _VCI_CAN_OBJ
{
	UINT ID;          // 帧ID。32位变量,数据格式为靠右对齐。
	UINT TimeStamp;	  // 设备接收到某一帧的时间标识。时间标示从CAN卡上电开始计时,计时单位为0.1ms。
	BYTE TimeFlag;	  // 为1时TimeStamp有效,TimeFlag和TimeStamp只在此帧为接收帧时有意义
	BYTE SendType;	  // 0-正常发送(失败会自动重发,4秒内没有发出则取消),1-单次发送(发送失败不会自动重发)。建议值为1，以提高效率；
	BYTE RemoteFlag;  // 0-数据帧,1-远程帧（数据段空）
	BYTE ExternFlag;  // 0-标准帧（11bit ID），1-扩展帧（29bit ID）
	BYTE DataLen;     // 数据长度 DLC (<=8),即CAN帧Data有几个字节，决定了Data[8]中的有效字节数；
	BYTE Data[8];     // CAN帧的数据。低字节优先有效
	BYTE Reserved[3]; // 
} VCI_CAN_OBJ, *PVCI_CAN_OBJ;

// 3.定义初始化CAN的数据类型
typedef struct _INIT_CONFIG
{
	DWORD AccCode;	// 验收码。对经过屏蔽码过滤为“有关位”进行匹配,全部匹配成功后,此帧可以被接收,否则不接收。
	DWORD AccMask;	// 屏蔽码。对接收的CAN帧ID进行过滤,对应位为0的是“有关位”,对应位为1的是“无关位”。推荐设置为0xFFFFFFFF,即全部接收。
	DWORD Reserved; //
	UCHAR Filter;	// 滤波方式,允许设置为0-3; 0/1-接收所有类型; 2-只接收标准帧; 3-只接收扩展帧;
	UCHAR Timing0;	// 波特率定时器0(BTR0);
	UCHAR Timing1;	// 波特率定时器1(BTR1);
	UCHAR Mode;		// 模式。0-正常模式; 1-只听模式; 2-自发自收模式;
} VCI_INIT_CONFIG, *PVCI_INIT_CONFIG;

///////// new add struct for filter /////////
typedef struct _VCI_FILTER_RECORD
{
	DWORD ExtFrame; //是否为扩展帧
	DWORD Start;
	DWORD End;
} VCI_FILTER_RECORD, *PVCI_FILTER_RECORD;

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C
#endif

/**
 * @brief 用以打开设备，注意一个设备只能打开一次
 *
 * @param DeviceType 设备类型，对应不同的产品型号，CANalyst-II对应4，或者取宏定义【VCI_USBCAN2】
 * @param DeviceInd 设备索引，从0开始分别对应计算机上挂载的各个分析仪
 * @param Reserved 保留参数,通常为 0
 * @return EXTERN_C 返回值=1,表示操作成功; =0表示操作失败; =-1表示USB-CAN设备不存在或USB掉线
 */
EXTERN_C DWORD VCI_OpenDevice(DWORD DeviceType, DWORD DeviceInd, DWORD Reserved);

/**
 * @brief 此函数用以关闭设备。
 * 
 * @param DeviceType 设备类型。对应不同的产品型号 详见:适配器设备类型定义。
 * @param DeviceInd 设备索引,比如当只有一个USB-CAN适配器时,索引号为0,
 * @return EXTERN_C 返回值=1,表示操作成功; =0表示操作失败; =-1表示USB-CAN设备不存在或USB掉线。
 */
EXTERN_C DWORD VCI_CloseDevice(DWORD DeviceType, DWORD DeviceInd);

/**
 * @brief 此函数用以初始化指定的CAN通道。有多个CAN通道时,需要多次调用
 * 
 * @param DeviceType 设备类型，对应不同的产品型号，CANalyst-II对应4，或者取宏定义【VCI_USBCAN2】
 * @param DeviceInd 设备索引，从0开始分别对应计算机上挂载的各个分析仪
 * @param CANInd CAN通道索引。第几路 CAN。即对应卡的CAN通道号,CAN1为0,CAN2为1
 * @param pInitConfig 是一个结构体指针，指向初始化参数结构体
 * @return EXTERN_C 
 */
EXTERN_C DWORD VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig);

/**
 * @brief 获取设备信息
 *
 * @param DeviceType 设备类型，对应不同的产品型号，CANalyst-II对应4，或者取宏定义【VCI_USBCAN2】
 * @param DeviceInd 设备索引，从0开始分别对应计算机上挂载的各个分析仪
 * @param pInfo 用来存储设备信息的VCI_BOARD_INFO结构指针
 * @return EXTERN_C 返回值=1,表示操作成功; =0表示操作失败; =-1表示USB-CAN设备不存在或USB掉线
 */
EXTERN_C DWORD VCI_ReadBoardInfo(DWORD DeviceType, DWORD DeviceInd, PVCI_BOARD_INFO pInfo);


EXTERN_C DWORD VCI_SetReference(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, DWORD RefType, PVOID pData);

/**
 * @brief 此函数用以获取指定CAN通道的接收缓冲区中,接收到但尚未被读取的帧数量。
 *        主要用途是配合VCI_Receive使用,即缓冲区有数据,再接收。
 * @param DeviceType 设备类型。对应不同的产品型号 详见:适配器设备类型定义。
 * @param DeviceInd 设备索引,比如当只有一个USB-CAN适配器时,索引号为0
 * @param CANInd CAN通道索引。第几路 CAN。即对应卡的CAN通道号,CAN1为0,CAN2为1。
 * @return EXTERN_C 返回尚未被读取的帧数,=-1表示USB-CAN设备不存在或USB掉线。
 */
EXTERN_C ULONG VCI_GetReceiveNum(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);

/**
 * @brief 此函数用以清空指定CAN通道的缓冲区。
 *        主要用于需要清除接收缓冲区数据的情况,同时发送缓冲区数据也会一并清除
 * @param DeviceType 设备类型。对应不同的产品型号 详见:适配器设备类型定义。
 * @param DeviceInd 设备索引,比如当只有一个USB-CAN适配器时,索引号为0
 * @param CANInd CAN通道索引。第几路 CAN。即对应卡的CAN通道号,CAN1为0,CAN2为1。
 * @return EXTERN_C 返回值=1,表示操作成功; =0表示操作失败; =-1表示USB-CAN设备不存在或USB掉线。
 */
EXTERN_C DWORD VCI_ClearBuffer(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);

/**
 * @brief 启动CAN卡的某一个CAN通道。有多个CAN通道时,需要多次调用。
 * 
 * @param DeviceType 设备类型。对应不同的产品型号 详见:适配器设备类型定义。
 * @param DeviceInd 设备索引,比如当只有一个USB-CAN适配器时,索引号为0
 * @param CANInd CAN通道索引。第几路 CAN。即对应卡的CAN通道号,CAN1为0,CAN2为1。
 * @return EXTERN_C 返回值=1,表示操作成功; =0表示操作失败; =-1表示USB-CAN设备不存在或USB掉线。
 */
EXTERN_C DWORD VCI_StartCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);

/**
 * @brief 此函数用以复位CAN。
 *        主要用与VCI_StartCAN配合使用,无需再初始化,即可恢复CAN卡的正常状态。
 * 
 * @param DeviceType 设备类型。对应不同的产品型号 详见:适配器设备类型定义。
 * @param DeviceInd 设备索引,比如当只有一个USB-CAN适配器时,索引号为0
 * @param CANInd CAN通道索引。第几路 CAN。即对应卡的CAN通道号,CAN1为0,CAN2为1。
 * @return EXTERN_C 返回值=1,表示操作成功; =0表示操作失败; =-1表示USB-CAN设备不存在或USB掉线。
 */
EXTERN_C DWORD VCI_ResetCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);

/**
 * @brief 发送函数
 * 
 * @param DeviceType 设备类型。
 * @param DeviceInd 设备索引,比如当只有一个USB-CAN适配器时,索引号为0.
 * @param CANInd CAN通道索引。第几路 CAN。即对应卡的CAN通道号,CAN1为0,CAN2为1。
 * @param pSend 指向定义了发送帧内容和类型的结构体，可以是VCI_CAN_OBJ类型结构体数组的数组名。
 * @param Len 要发送的帧结构体数组的长度(发送的帧数量)。最大为1000,建议设为1,以提高发送效率。
 * @return EXTERN_C 实际发送成功的帧数
 */
EXTERN_C ULONG VCI_Transmit(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pSend, UINT Len);

/**
 * @brief 接收函数
 *        	由于设备内有2000帧的缓存区，因此建议pReceive长度、Len在2500左右，以避免数据溢出。
 *          建议运行频率为30ms/次。应尽量降低使用频率，提高每次调用接收的帧数。
 * @param DeviceType 设备类型。
 * @param DeviceInd 设备索引,比如当只有一个USB-CAN适配器时,索引号为0.
 * @param CANInd CAN通道索引。第几路 CAN。即对应卡的CAN通道号,CAN1为0,CAN2为1。
 * @param pReceive 指向定义了接收帧内容和类型的结构体，可以是VCI_CAN_OBJ类型结构体数组的数组名。数组长度一定要大于Len。
 * @param Len 本次允许接收的帧结构体数组的最大长度(帧数量)。
 * @return EXTERN_C 返回实际读取的帧数,=-1表示USB-CAN设备不存在或USB掉线
 */
EXTERN_C ULONG VCI_Receive(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pReceive, UINT Len, INT WaitTime);

/**
 * @brief 复位USB-CAN适配器,复位后需要重新使用VCI_OpenDevice打开设备。等同于插拔一次USB设备。
 * 
 * @param DevType 设备类型。
 * @param DevIndex 设备索引,比如当只有一个USB-CAN适配器时,索引号为0.
 * @param Reserved 
 * @return EXTERN_C 返回值=1,表示操作成功;=0表示操作失败;=-1表示设备不存在。
 */
EXTERN_C DWORD VCI_UsbDeviceReset(DWORD DevType, DWORD DevIndex, DWORD Reserved);

/**
 * @brief 查找设备并获取设备基本参数
 *
 * @param pInfo VCI_BOARD_INFO类型数组的数组名
 * @return EXTERN_C 计算机中已插入的USB-CAN适配器的数量
 */
EXTERN_C DWORD VCI_FindUsbDevice2(PVCI_BOARD_INFO pInfo);

#endif
