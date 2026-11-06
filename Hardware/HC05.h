#ifndef __HC05_H
#define __HC05_H

#include "stm32f10x.h"

/* 遥控模块给出的目标值，由主程序读取后写进速度环/转向环 */
extern volatile float HC05_Speed;		// 目标平均速度，单位转/秒，正数前进，负数后退
extern volatile float HC05_Turn;		// 目标差速，单位转/秒，正数右转，负数左转

/* 调试计数：用来区分"串口没收到"和"收到了但没解析出来" */
extern volatile uint16_t HC05_RxCount;	// 串口收到的字节总数，收到任何一个字符就加 1
extern volatile uint16_t HC05_CmdCount;	// 成功解析并执行的指令条数

void HC05_Init(void);					// 配置 USART2（PA2/PA3）并清空状态
void HC05_Tick(void);					// 1ms 调用一次：解析指令 + 失联检测
uint8_t HC05_IsOnline(void);			// 查询链路是否在线
void HC05_SendByte(uint8_t Byte);		// 发送一个字节
void HC05_SendString(char *String);		// 发送一个字符串，仅限主循环使用

#endif