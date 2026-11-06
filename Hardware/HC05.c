#include "stm32f10x.h"
#include "HC05.h"
#include <stdlib.h>

/* 遥控模块给出的目标值，主程序在速度环里直接读取 */
volatile float HC05_Speed;			// 目标平均速度，单位转/秒，正数前进，负数后退
volatile float HC05_Turn;			// 目标差速，单位转/秒，正数右转，负数左转

/* 调试计数，OLED 上显示出来，一眼就能定位通信故障在哪一环 */
volatile uint16_t HC05_RxCount;		// 收到的字节数，只加不减
volatile uint16_t HC05_CmdCount;	// 成功解析的指令数，只加不减

/* ==================== 可调参数 ==================== */

#define HC05_BAUDRATE		9600		// 波特率，必须和 HC-05 模块内部设置一致，出厂默认 9600
#define HC05_TIMEOUT_MS		5000		// 多久收不到指令就判定失联，失联后自动清零目标值
#define HC05_RX_IDLE_MS		30			// 收到字符后多久没有新字符就认为一条指令发完了
#define HC05_RX_SIZE		32			// 接收缓冲区大小，够放 "move,-100,-100" 这种长指令

#define HC05_VALUE_SCALE	25.0f		// 数值指令的缩放：指令 ±100 换算成 ±4 转/秒
#define HC05_FORWARD		1.0f		// 'F' 对应的前进速度
#define HC05_BACKWARD		-1.0f		// 'B' 对应的后退速度
#define HC05_TURN_STEP		0.6f		// 'L' 'R' 对应的转向差速

#define HC05_JOG_MS			500			// 点动时长：单字符指令只走这么多毫秒就自动停
										// 调大车走得更远，调小走得更短；重复发送会不断续期，相当于"按住"

#define HC05_SPEED_MAX		4.0f		// 速度安全上限，防止指令值过大让车冲出去
#define HC05_TURN_MAX		5.0f		// 转向安全上限

/* ==================== 内部状态 ==================== */

static char RxBuf[HC05_RX_SIZE];		// 接收缓冲区，串口中断往里塞字符
static volatile uint8_t RxLen;			// 缓冲区里已有的字符数
static volatile uint8_t RxReady;		// 收到结束符（']' 或回车换行）后置 1
static volatile uint16_t RxIdleTime;	// 距离上一个字符过去了多少毫秒
static uint16_t IdleCount;				// 距离上一条有效指令过去了多少毫秒
static uint8_t OnlineFlag;				// 链路是否还活着
static uint16_t JogCount;				// 点动剩余毫秒数，减到 0 就把目标速度清零

/* ==================== 内部函数声明 ==================== */

static uint8_t HC05_GetField(const char *Packet, uint8_t Index, char *Out, uint8_t Size);
static uint8_t HC05_IsCmd(const char *Tag, const char *Name);
static float HC05_Limit(float Value, float Max);
static void HC05_Execute(const char *Packet);

/**
  * 函    数：HC-05 初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：HC-05 接在 PA2（模块 RX）/ PA3（模块 TX）上，这两个脚就是 USART2。
  *           本函数完整地配置 USART2 的硬件：GPIO、波特率、接收中断、NVIC。
  *           中断优先级设得比 TIM1 低，保证 1ms 控制循环永远能打断串口，
  *           串口再忙也不会拖慢平衡控制。
  */
void HC05_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 开时钟：USART2 挂在 APB1，PA2/PA3 挂在 APB2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* PA2 是 USART2 的 TX，复用推挽输出 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* PA3 是 USART2 的 RX，上拉输入，悬空时不会乱收噪声 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 串口参数：8 位数据、无校验、1 位停止位，这是 HC-05 的出厂配置 */
	USART_InitStructure.USART_BaudRate = HC05_BAUDRATE;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStructure);

	/* 只开接收中断，发送用轮询即可 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* NVIC 分组 2：2 位抢占优先级 + 2 位响应优先级 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	// 比 TIM1（抢占 2）低，串口不会打断控制循环
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);

	/* 清空所有软件状态，上电时视为"未连接、目标速度为 0" */
	HC05_Speed = 0;
	HC05_Turn = 0;
	HC05_RxCount = 0;
	HC05_CmdCount = 0;
	RxLen = 0;
	RxReady = 0;
	RxIdleTime = 0;
	IdleCount = HC05_TIMEOUT_MS;
	OnlineFlag = 0;
	JogCount = 0;
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void HC05_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送的字符串首地址
  * 返 回 值：无
  * 注意事项：这个函数会阻塞等待发送完成，不要在 1ms 定时中断里调用，
  *           否则会拖慢平衡控制。只在主循环里用。
  */
void HC05_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		HC05_SendByte(String[i]);
	}
}

/**
  * 函    数：串口接收中断
  * 参    数：无
  * 返 回 值：无
  * 注意事项：这里只做"收字符、攒缓冲区"这一件事，解析工作放到 HC05_Tick 里做，
  *           中断里干活越少越好。
  *           为了兼容各种蓝牙串口助手，以下三种写法都能识别：
  *           1) 纯文本，直接发 F，靠 30ms 空闲自动断句
  *           2) 带方括号，发 [F]
  *           3) 带回车换行，发 F 后按发送
  */
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART2);

		USART_ClearITPendingBit(USART2, USART_IT_RXNE);

		HC05_RxCount++;						// 每收到一个字节就加 1，用来判断串口到底通没通
		RxIdleTime = 0;						// 有新字符进来，空闲计时归零

		if (RxData == '[')					// 包头，丢掉并从新开始
		{
			RxLen = 0;
			RxReady = 0;
			return;
		}

		if (RxData == ']' || RxData == '\r' || RxData == '\n')	// 包尾，一条指令收完
		{
			if (RxLen > 0)
			{
				RxBuf[RxLen] = '\0';
				RxReady = 1;
			}
			return;
		}

		if (RxLen < HC05_RX_SIZE - 1)		// 普通字符，存进缓冲区
		{
			if (RxReady)					// 上一条还没被取走，直接丢掉，避免两条指令粘连
			{
				RxLen = 0;
				RxReady = 0;
			}
			RxBuf[RxLen++] = (char)RxData;
		}
	}
}

/**
  * 函    数：HC-05 定时任务，由 1ms 定时中断调用
  * 参    数：无
  * 返 回 值：无
  * 注意事项：做两件事——把攒够的指令送去解析、检测链路是否失联。
  */
void HC05_Tick(void)
{
	/* ---- 指令解析 ---- */
	if (RxReady)							// 收到结束符，立即解析
	{
		RxReady = 0;
		HC05_Execute(RxBuf);
		RxLen = 0;
	}
	else if (RxLen > 0)						// 没有结束符，靠空闲时间断句
	{
		if (++RxIdleTime >= HC05_RX_IDLE_MS)
		{
			RxBuf[RxLen] = '\0';
			HC05_Execute(RxBuf);
			RxLen = 0;
			RxIdleTime = 0;
		}
	}

	/* ---- 点动计时：单字符指令只走一小段就自动停 ---- */
	/* 收到 F 时 JogCount 被置为 HC05_JOG_MS，这里每毫秒减 1，
	   减到 0 就把目标速度清零，车自己减速停住，实现"按一下动一下" */
	if (JogCount > 0)
	{
		if (--JogCount == 0)
		{
			HC05_Speed = 0;
			HC05_Turn = 0;
		}
	}

	/* ---- 失联检测 ---- */
	if (IdleCount < HC05_TIMEOUT_MS)
	{
		IdleCount++;						// 还没超时，继续计时
	}
	else if (OnlineFlag)					// 超时了，做一次性的停车动作
	{
		OnlineFlag = 0;
		HC05_Speed = 0;						// 目标速度清零，车会自己刹停
		HC05_Turn = 0;						// 转向目标清零，车会回正
		JogCount = 0;						// 顺便结束点动计时
	}
}

/**
  * 函    数：查询蓝牙链路是否在线
  * 参    数：无
  * 返 回 值：1 表示超时时间内收到过指令，0 表示链路已断开
  */
uint8_t HC05_IsOnline(void)
{
	return OnlineFlag;
}

/**
  * 函    数：取出逗号分隔数据包中的第 Index 个字段（内部使用）
  * 参    数：Packet 数据包首地址
  * 参    数：Index  字段序号，从 0 开始
  * 参    数：Out    取出的字段存放位置
  * 参    数：Size   Out 的容量
  * 返 回 值：1 表示成功取到该字段，0 表示字段不存在
  * 注意事项：只读不改，不会破坏原始数据包。
  */
static uint8_t HC05_GetField(const char *Packet, uint8_t Index, char *Out, uint8_t Size)
{
	uint8_t Field = 0;
	uint8_t n = 0;
	const char *p = Packet;

	while (*p == ' ') {p++;}				// 跳过开头的空格

	/* 跳到目标字段的起始位置 */
	while (Field < Index && *p != '\0')
	{
		if (*p == ',')
		{
			Field++;
			p++;
			while (*p == ' ') {p++;}		// 跳过逗号后的空格
			continue;
		}
		p++;
	}
	if (Field != Index)
	{
		return 0;							// 字段不够
	}

	/* 拷贝目标字段，遇到逗号或字符串结束就停 */
	while (*p != '\0' && *p != ',' && n < Size - 1)
	{
		Out[n++] = *p++;
	}
	while (n > 0 && Out[n - 1] == ' ') {n--;}	// 去掉尾部空格
	Out[n] = '\0';
	return 1;
}

/**
  * 函    数：不区分大小写地比较指令名（内部使用）
  */
static uint8_t HC05_IsCmd(const char *Tag, const char *Name)
{
	while (*Name != '\0')
	{
		char a = *Tag;
		char b = *Name;
		if (a >= 'A' && a <= 'Z') {a = a - 'A' + 'a';}
		if (b >= 'A' && b <= 'Z') {b = b - 'A' + 'a';}
		if (a != b) {return 0;}
		Tag++;
		Name++;
	}
	return (*Tag == '\0');
}

/**
  * 函    数：把数值限制在 ±Max 之内（内部使用）
  */
static float HC05_Limit(float Value, float Max)
{
	if (Value > Max) {return Max;}
	if (Value < -Max) {return -Max;}
	return Value;
}

/**
  * 函    数：解析并执行一条指令（内部使用）
  * 参    数：Packet 一条完整指令的文本，例如 "F"、"move,50,-20"
  * 返 回 值：无
  * 注意事项：支持两种指令，单字符最省事，逗号分隔能一次给速度和转向：
  *           F 前进 / B 后退 / L 左转 / R 右转 / S 停车
  *           move,速度,转向   speed,速度   turn,转向   stop
  *           单字符的 L R 只改转向、保留当前速度，所以可以边走边转。
  */
static void HC05_Execute(const char *Packet)
{
	char Tag[16];
	char Value1[16];
	char Value2[16];
	float Speed, Turn;
	uint8_t Valid = 0;

	if (HC05_GetField(Packet, 0, Tag, sizeof(Tag)) == 0)
	{
		return;							// 空指令，忽略
	}

	if (Tag[1] == '\0')					// 单字符指令
	{
		switch (Tag[0])
		{
			case 'F': case 'f':
				HC05_Speed = HC05_FORWARD;
				HC05_Turn = 0;
				JogCount = HC05_JOG_MS;	// 开始点动，走 HC05_JOG_MS 毫秒后自动停
				Valid = 1;
				break;
			case 'B': case 'b':
				HC05_Speed = HC05_BACKWARD;
				HC05_Turn = 0;
				JogCount = HC05_JOG_MS;	// 后退同样是点动
				Valid = 1;
				break;
			case 'L': case 'l':
				HC05_Turn = -HC05_TURN_STEP;
				JogCount = HC05_JOG_MS;	// 点一下转一点
				Valid = 1;
				break;
			case 'R': case 'r':
				HC05_Turn = HC05_TURN_STEP;
				JogCount = HC05_JOG_MS;
				Valid = 1;
				break;
			case 'S': case 's':
				HC05_Speed = 0;
				HC05_Turn = 0;
				JogCount = 0;			// 立即停车，取消点动
				Valid = 1;
				break;
			default:
				break;					// 不认识的单字符，丢弃
		}
	}
	else if (HC05_IsCmd(Tag, "move"))		// move,速度,转向
	{
		if (HC05_GetField(Packet, 1, Value1, sizeof(Value1)) == 0) {return;}
		if (HC05_GetField(Packet, 2, Value2, sizeof(Value2)) == 0) {return;}
		Speed = atof(Value1) / HC05_VALUE_SCALE;
		Turn = atof(Value2) / HC05_VALUE_SCALE;
		HC05_Speed = HC05_Limit(Speed, HC05_SPEED_MAX);
		HC05_Turn = HC05_Limit(Turn, HC05_TURN_MAX);
		JogCount = 0;						// 数值指令是连续控制，取消点动
		Valid = 1;
	}
	else if (HC05_IsCmd(Tag, "speed"))		// speed,速度，只改前后
	{
		if (HC05_GetField(Packet, 1, Value1, sizeof(Value1)) == 0) {return;}
		Speed = atof(Value1) / HC05_VALUE_SCALE;
		HC05_Speed = HC05_Limit(Speed, HC05_SPEED_MAX);
		JogCount = 0;						// 数值指令是连续控制，取消点动
		Valid = 1;
	}
	else if (HC05_IsCmd(Tag, "turn"))		// turn,转向，只改左右
	{
		if (HC05_GetField(Packet, 1, Value1, sizeof(Value1)) == 0) {return;}
		Turn = atof(Value1) / HC05_VALUE_SCALE;
		HC05_Turn = HC05_Limit(Turn, HC05_TURN_MAX);
		JogCount = 0;						// 数值指令是连续控制，取消点动
		Valid = 1;
	}
	else if (HC05_IsCmd(Tag, "stop"))		// stop，立即停车
	{
		HC05_Speed = 0;
		HC05_Turn = 0;
		JogCount = 0;						// 立即停车，取消点动
		Valid = 1;
	}
	else if (HC05_IsCmd(Tag, "joystick"))	// joystick,左水平,左垂直,右水平,右垂直
	{
		if (HC05_GetField(Packet, 2, Value1, sizeof(Value1)) == 0) {return;}	// 左摇杆垂直，控制前后
		if (HC05_GetField(Packet, 3, Value2, sizeof(Value2)) == 0) {return;}	// 右摇杆水平，控制左右
		Speed = atof(Value1) / HC05_VALUE_SCALE;
		Turn = atof(Value2) / HC05_VALUE_SCALE;
		HC05_Speed = HC05_Limit(Speed, HC05_SPEED_MAX);
		HC05_Turn = HC05_Limit(Turn, HC05_TURN_MAX);
		JogCount = 0;						// 摇杆是连续控制，取消点动
		Valid = 1;
	}

	if (Valid)								// 只有认得出的指令才喂狗，乱码不会让车继续跑
	{
		HC05_CmdCount++;					// 成功解析一条指令，计数加 1
		IdleCount = 0;
		OnlineFlag = 1;
	}
}