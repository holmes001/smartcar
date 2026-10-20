/**
 * @file    bluetooth.c
 * @brief   JDY-33 蓝牙模块状态查询实现
 *
 * @note    本次修改
 *          原第 32 行写的是 `else if( JDY33_DISCONNECT )`，漏了比较对象。
 *          由于 JDY33_DISCONNECT 的枚举值是 3（非零），这个条件**恒为真**，
 *          结果是只要状态不是 SPP/BLE 就会打印"蓝牙已断开"。
 *          已修正为与 JDY33_ConnectState 比较。
 */
#include "bluetooth.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stdint.h"

/** @brief AT 指令回包解析（内部实现） */
static uint8_t ATCommandFeedBack_JDY33(uint8_t recv);

//蓝牙连接状态
static uint8_t JDY33_ConnectState = JDY33_NO_STATE;

//APP数据处理标志位
uint8_t app_event_GetParamFlag = 0;
uint8_t app_event_SaveFlashFlag = 0;
uint8_t app_event_SendAllParamFlag = 0;
uint8_t app_event_SetParamFlag = 0;
char channelNum = 0;

//与APP交互的数据
uint32_t recv_app_param[9]={100,200,300,400,500,600,700,800,900};

void show_bt_state(void)
{
	static uint8_t LastState = JDY33_NO_STATE;
	
	if( JDY33_ConnectState!=LastState )
	{
		if( JDY33_ConnectState == JDY33_SPP_CONNECT )
			printf("SPP信号已被连接.\r\n");
		else if( JDY33_ConnectState == JDY33_BLE_CONNECT )
			printf("BLE信号已被连接.\r\n");
		else if( JDY33_ConnectState == JDY33_DISCONNECT )
			printf("蓝牙已断开.\r\n");
	}
	LastState = JDY33_ConnectState;
}

//JDY-33蓝牙AT指令集过滤
static uint8_t ATCommandFeedBack_JDY33(uint8_t recv)
{
	#define DEBUG_JDY33Command 0
	
	uint8_t isFilter = 0;//是否允许该字符通过,1过滤,0允许
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;
	
	//以下是当JDY-33被连接或断开时,发送的AT指令提示符
	const char* JDY33_SPPConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\nCONNECTED\r\r\n";
	const char* JDY33_BLEConnect = "CONNECTED\r\r\n";
	const char* JDY33_DisConnect = "+DISC:SUCCESS\r\r\n\0"; //注意\0不会被统计字长,需要自行把字长+1
	enum{
		JDY33_NORMAL=  0,
		JDY33_SPPCONNECTSTART,
		JDY33_BLECONNECTSTART,
		JDY33_DISCONNECTSTART,
	};
	
	static uint8_t statemachine = JDY33_NORMAL;
	
	switch( statemachine )
	{
		case JDY33_NORMAL:
			if( recv=='C'&&lastrecv=='+' )
			{
				statemachine = JDY33_SPPCONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			else if( recv=='O'&&lastrecv=='C' )
			{
				statemachine = JDY33_BLECONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			else if( recv=='D'&&lastrecv=='+' )
			{
				statemachine = JDY33_DISCONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			else if( recv=='C'&&lastrecv!='C' )//进入状态的判断优先,再到歧义的判断
			{
				//有关C的歧义,"+C"、"CO"不能通过. "?C也不允许通过",才能保证连接时不存在控制命令,若需要控制小车,需要连续的C
				isFilter = 1;//禁止通过
			}
			break;
		case JDY33_SPPCONNECTSTART:
			if( JDY33_SPPConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else if( (filterIndex>=13&&filterIndex<=29) && \
       				  ((recv>='0'&&recv<='9')||(recv>='A'&&recv<='Z')) )//进入到MAC地址匹配阶段.该阶段匹配 0~9 、a~z字段
			{
				isFilter = 1;//MAC地址过滤
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				//都不满足,允许字符通过.并退出过滤模式
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP->No:get:%c,but:%c\r\n",recv,JDY33_SPPConnect[filterIndex]);
				#endif
			}
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(JDY33_SPPConnect) )
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP filter con done!\r\n");
				#endif
				isFilter = 2;//SPP已连接
			}
			break;
		case JDY33_BLECONNECTSTART:
			if( JDY33_BLEConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("BLE->No:get:%c,but:%c\r\n",recv,JDY33_BLEConnect[filterIndex]);
				#endif
			}				
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(JDY33_BLEConnect) ) 
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("ble filter dis done!\r\n");
				#endif
				isFilter = 3;//BLE已连接
			}
			break;
		case JDY33_DISCONNECTSTART:
			if( JDY33_DisConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("dis->No:get:%c,but:%c\r\n",recv,JDY33_DisConnect[filterIndex]);
				#endif
			}				
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(JDY33_DisConnect)+1 ) //+1为补充空字符'\0'
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("filter dis done!\r\n");
				#endif
				isFilter = 4;//蓝牙已断开连接
			}
			break;
	}
	lastrecv = recv;
	
	return isFilter;
}


