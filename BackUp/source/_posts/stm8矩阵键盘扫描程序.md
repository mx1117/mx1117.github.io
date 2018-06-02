---
title: "stm8矩阵键盘扫描程序"
date: 2017-07-13 00:49:25
toc: false
tags: [单片机,stm8]
---
按键通常有：IO口按键（BUTTON)，AD按键（通过AD采样电压），IR（遥控器）
按按键功能分：有短按键，长按键，连续按键。打个比方，遥控电视机，按一下音量键，音量增加1，这个就是短按键。按住音量键不放，音量连续加，这个就是连续按键。按住一个按键5s,系统会复位，这个是长按键。
<!-- more -->
IO口按键，就是我们比较常见的一个IO接一个按键，或者是一个矩阵键盘。很多新人的处理方法可能是采样延时的方法，当年我也是这样的，如下
```
   if(GETIO==low)
    { 
      delay_10ms()
      if(GETIO==low)
      {
        //得到按键值
      }
    }
```
这种方法虽然简单，但是有很大弊端。首先 Delay浪费很多时间，影响系统。第二，无法判断长短按键，连续按键。第三，如果这个按键是开关机按键系统在低功耗状态下，需要中断唤醒，这种方法比较容易出问题，如STM8S系列的
halt 模式。

所以我们一般在产品开发的过程中，采用扫描的方法，就是每隔10ms或者20ms 去检测IO的状态，看是否有按键，然后去抖动，判断按键功能。

下面是本人写的4*4矩阵键盘扫描程序代码，容易修改，可以根据自己的时间需要，进行长短按键，连续按键,还有组合按键的判断。

 
```
/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "delay.h"
/* Private defines -----------------------------------------------------------*/

#define BUTTON_FILTER_TIME 5
#define BUTTON_LONG_TIME 50 /* 持续1秒，认为长按事件 */

/**
******************************************************************************
宏定义设定行号和列号
按键序号排列

第一列 第二列 第三列 第四列

第一行 1 2 3 4

第二行 5 6 7 8

第三行 9 10 11 12

第四行 13 14 15 16

 

******************************************************************************
*/

#define ROW_1 GPIOC, GPIO_PIN_4 /* 按键第一行*/
#define ROW_2 GPIOC, GPIO_PIN_5 /* 按键第二行*/
#define ROW_3 GPIOC, GPIO_PIN_6 /* 按键第三行*/
#define ROW_4 GPIOC, GPIO_PIN_7 /* 按键第四行*/
#define COLUMN_1 GPIOA, GPIO_PIN_1 /* 按键第一列*/
#define COLUMN_2 GPIOA, GPIO_PIN_2 /* 按键第二列*/
#define COLUMN_3 GPIOA, GPIO_PIN_3 /* 按键第三列*/
#define COLUMN_4 GPIOD, GPIO_PIN_4 /* 按键第四列*/

 

 

u8 KeyDownCode[]= {0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f};
//按键按下的键值
u8 KeyUpCode[]={0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2a,0x2b,0x2c,0x2d,0x2e,0x2f};
//按键弹起的键值
u8 KeyLongCode[]={0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff};
//按键长按的键值

 

typedef struct
{
/* 下面是一个函数指针，指向判断按键手否按下的函数 */

	unsigned char (*IsKeyDownFunc)(void); /* 按键按下的判断函数,1表示按下 */
	unsigned char Count; /* 滤波器计数器 */
	unsigned char FilterTime; /* 滤波时间(最大255,表示2550ms) */
	unsigned short LongCount; /* 长按计数器 */
	unsigned short LongTime; /* 按键按下持续时间, 0表示不检测长按 */
	unsigned char State; /* 按键当前状态（按下还是弹起） */
	unsigned char KeyCodeUp; /* 按键弹起的键值代码, 0表示不检测按键弹起 */
	unsigned char KeyCodeDown; /* 按键按下的键值代码, 0表示不检测按键按下 */
	unsigned char KeyCodeLong; /* 按键长按的键值代码, 0表示不检测长按 */
	unsigned char RepeatSpeed; /* 连续按键周期 */
	unsigned char RepeatCount; /* 连续按键计数器 */
}BUTTON_T;

BUTTON_T Key[16];

 

 

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void UART_Config(void);

void Pannelkey_Polling(void);
void Button_Detect(BUTTON_T *_pBtn);

 

void Key_Init(void);
//矩阵键盘扫描按键检测按下函数
unsigned char Key_1_Down(void);
unsigned char Key_2_Down(void);
unsigned char Key_3_Down(void);
unsigned char Key_4_Down(void);
unsigned char Key_5_Down(void);
unsigned char Key_6_Down(void);
unsigned char Key_7_Down(void);
unsigned char Key_8_Down(void);
unsigned char Key_9_Down(void);
unsigned char Key_10_Down(void);
unsigned char Key_11_Down(void);
unsigned char Key_12_Down(void);
unsigned char Key_13_Down(void);
unsigned char Key_14_Down(void);
unsigned char Key_15_Down(void);
unsigned char Key_16_Down(void);
//矩阵键盘扫描按键检测按下函数
//
//u8 KeyValue,COLUMN_VALUE1,COLUMN_VALUE2,COLUMN_VALUE3,COLUMN_VALUE4,column_value;
//u8 ROW_VALUE1,ROW_VALUE2,ROW_VALUE3,ROW_VALUE4,row_value;

/* 按键滤波时间50ms, 单位10ms
*只有连续检测到50ms状态不变才认为有效，包括弹起和按下两种事件
*/

 

/*
每个按键对应1个全局的结构体变量。
其成员变量是实现滤波和多种按键状态所必须的
*/

void main(void)
{
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	CLK_HSICmd(ENABLE);

	delay_init(16);
	Key_Init();
	UART_Config();

	while(1){
	Pannelkey_Polling();

}

/* Infinite loop */

}

/*
*********************************************************************************************************
* 函 数 名: Key_1 Down
* 功能说明: key1按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/
unsigned char Key_1_Down(void)
{
	GPIO_Init(ROW_1, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 1 推挽输出
	GPIO_WriteLow(ROW_1);
	GPIO_Init(COLUMN_1, GPIO_MODE_IN_PU_NO_IT);//列线 1 上拉输入

	u8 column_temp=GPIO_ReadInputPin(COLUMN_1);
	GPIO_Init(COLUMN_1, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 1 推挽输出
	GPIO_WriteLow(COLUMN_1);
	GPIO_Init(ROW_1, GPIO_MODE_IN_PU_NO_IT);//行线 1 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_1);

	if(column_temp || row_temp)
		return 0;
	return 1;
}

 

/*
*********************************************************************************************************
* 函 数 名: Key_2_Down
* 功能说明: key2按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_2_Down(void)
{
	GPIO_Init(ROW_1, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 1 推挽输出
	GPIO_WriteLow(ROW_1);
	GPIO_Init(COLUMN_2, GPIO_MODE_IN_PU_NO_IT);//列线 2 上拉输入

	u8 column_temp=GPIO_ReadInputPin(COLUMN_2);
	GPIO_Init(COLUMN_2, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 2 推挽输出
	GPIO_WriteLow(COLUMN_2);
	GPIO_Init(ROW_1, GPIO_MODE_IN_PU_NO_IT);//行线 1 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_1);

	if(column_temp || row_temp)
		return 0;
	return 1;
}

 

 

/*
*********************************************************************************************************
* 函 数 名: Key_3_Down
* 功能说明: key3按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_3_Down(void)
{
	GPIO_Init(ROW_1, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 1 推挽输出
	GPIO_WriteLow(ROW_1);
	GPIO_Init(COLUMN_3, GPIO_MODE_IN_PU_NO_IT);//列线 3 上拉输入

	u8 column_temp=GPIO_ReadInputPin(COLUMN_3);
	GPIO_Init(COLUMN_3, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 3 推挽输出
	GPIO_WriteLow(COLUMN_3);
	GPIO_Init(ROW_1, GPIO_MODE_IN_PU_NO_IT);//行线 1 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_1);

	if(column_temp || row_temp)
		return 0;
	return 1;
}

/*
*********************************************************************************************************
* 函 数 名: Key_4_Down
* 功能说明: key4按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_4_Down(void)
{
	GPIO_Init(ROW_1, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 1 推挽输出
	GPIO_WriteLow(ROW_1);
	GPIO_Init(COLUMN_4, GPIO_MODE_IN_PU_NO_IT);//列线 4 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_4);
	GPIO_Init(COLUMN_4, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 4 推挽输出
	GPIO_WriteLow(COLUMN_4);
	GPIO_Init(ROW_1, GPIO_MODE_IN_PU_NO_IT);//行线 1 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_1);

	if(column_temp || row_temp)
		return 0;
	return 1;
}

/*
*********************************************************************************************************
* 函 数 名: Key_5_Down
* 功能说明: key5按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_5_Down(void)
{
	GPIO_Init(ROW_2, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 2 推挽输出
	GPIO_WriteLow(ROW_2);
	GPIO_Init(COLUMN_1, GPIO_MODE_IN_PU_NO_IT);//列线 1 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_1);
	GPIO_Init(COLUMN_1, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 1 推挽输出
	GPIO_WriteLow(COLUMN_1);
	GPIO_Init(ROW_2, GPIO_MODE_IN_PU_NO_IT);//行线 2 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_2);

	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_6_Down
* 功能说明: key6按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_6_Down(void)
{
	GPIO_Init(ROW_2, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 2 推挽输出
	GPIO_WriteLow(ROW_2);
	GPIO_Init(COLUMN_2, GPIO_MODE_IN_PU_NO_IT);//列线 2 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_2);
	GPIO_Init(COLUMN_2, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 2 推挽输出
	GPIO_WriteLow(COLUMN_2);
	GPIO_Init(ROW_2, GPIO_MODE_IN_PU_NO_IT);//行线 2 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_2);

	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_7_Down
* 功能说明: key7按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_7_Down(void)
{
	GPIO_Init(ROW_2, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 2 推挽输出
	GPIO_WriteLow(ROW_2);
	GPIO_Init(COLUMN_3, GPIO_MODE_IN_PU_NO_IT);//列线 3 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_3);
	GPIO_Init(COLUMN_3, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 3 推挽输出
	GPIO_WriteLow(COLUMN_3);
	GPIO_Init(ROW_2, GPIO_MODE_IN_PU_NO_IT);//行线 2 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_2);

	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_8_Down
* 功能说明: key8按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_8_Down(void)
{
	GPIO_Init(ROW_2, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 2 推挽输出
	GPIO_WriteLow(ROW_2);
	GPIO_Init(COLUMN_4, GPIO_MODE_IN_PU_NO_IT);//列线 4 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_4);
	GPIO_Init(COLUMN_4, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 4 推挽输出
	GPIO_WriteLow(COLUMN_4);
	GPIO_Init(ROW_2, GPIO_MODE_IN_PU_NO_IT);//行线 2 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_2);

	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_9_Down
* 功能说明: key9按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_9_Down(void)
{
	GPIO_Init(ROW_3, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 3 推挽输出
	GPIO_WriteLow(ROW_3);
	GPIO_Init(COLUMN_1, GPIO_MODE_IN_PU_NO_IT);//列线 1 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_1);
	GPIO_Init(COLUMN_1, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 1 推挽输出
	GPIO_WriteLow(COLUMN_1);
	GPIO_Init(ROW_3, GPIO_MODE_IN_PU_NO_IT);//行线 3 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_3);

	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_10_Down
* 功能说明: key10按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_10_Down(void)
{
	GPIO_Init(ROW_3, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 3 推挽输出
	GPIO_WriteLow(ROW_3);
	GPIO_Init(COLUMN_2, GPIO_MODE_IN_PU_NO_IT);//列线 2 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_2);
	GPIO_Init(COLUMN_2, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 2 推挽输出
	GPIO_WriteLow(COLUMN_2);
	GPIO_Init(ROW_3, GPIO_MODE_IN_PU_NO_IT);//行线 3 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_3);
	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_11_Down
* 功能说明: key11按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_11_Down(void)
{
	GPIO_Init(ROW_3, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 3 推挽输出
	GPIO_WriteLow(ROW_3);
	GPIO_Init(COLUMN_3, GPIO_MODE_IN_PU_NO_IT);//列线 3 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_3);
	GPIO_Init(COLUMN_3, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 3 推挽输出
	GPIO_WriteLow(COLUMN_3);
	GPIO_Init(ROW_3, GPIO_MODE_IN_PU_NO_IT);//行线 3 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_3);
	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_12_Down
* 功能说明: key12按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_12_Down(void)
{
	GPIO_Init(ROW_3, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 3 推挽输出
	GPIO_WriteLow(ROW_3);
	GPIO_Init(COLUMN_4, GPIO_MODE_IN_PU_NO_IT);//列线 4 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_4);
	GPIO_Init(COLUMN_4, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 4 推挽输出
	GPIO_WriteLow(COLUMN_4);
	GPIO_Init(ROW_3, GPIO_MODE_IN_PU_NO_IT);//行线 3 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_3);
	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_13_Down
* 功能说明: key13按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_13_Down(void)
{
	GPIO_Init(ROW_4, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 4 推挽输出
	GPIO_WriteLow(ROW_4);
	GPIO_Init(COLUMN_1, GPIO_MODE_IN_PU_NO_IT);//列线 1 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_1);
	GPIO_Init(COLUMN_1, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 1 推挽输出
	GPIO_WriteLow(COLUMN_1);
	GPIO_Init(ROW_4, GPIO_MODE_IN_PU_NO_IT);//行线 4 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_4);
	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_14_Down
* 功能说明: key14按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_14_Down(void)
{
	GPIO_Init(ROW_4, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 4 推挽输出
	GPIO_WriteLow(ROW_4);
	GPIO_Init(COLUMN_2, GPIO_MODE_IN_PU_NO_IT);//列线 2 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_2);
	GPIO_Init(COLUMN_2, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 2 推挽输出
	GPIO_WriteLow(COLUMN_2);
	GPIO_Init(ROW_4, GPIO_MODE_IN_PU_NO_IT);//行线 4 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_4);
	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_15_Down
* 功能说明: key15按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_15_Down(void)
{
	GPIO_Init(ROW_4, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 4 推挽输出
	GPIO_WriteLow(ROW_4);
	GPIO_Init(COLUMN_3, GPIO_MODE_IN_PU_NO_IT);//列线 3 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_3);
	GPIO_Init(COLUMN_3, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 3 推挽输出
	GPIO_WriteLow(COLUMN_3);
	GPIO_Init(ROW_4, GPIO_MODE_IN_PU_NO_IT);//行线 4 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_4);
	if(column_temp || row_temp)
		return 0;
	return 1;
}
/*
*********************************************************************************************************
* 函 数 名: Key_16_Down
* 功能说明: key16按键状态测试
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/

unsigned char Key_16_Down(void)
{
	GPIO_Init(ROW_4, GPIO_MODE_OUT_PP_HIGH_FAST);//行线 4 推挽输出
	GPIO_WriteLow(ROW_4);
	GPIO_Init(COLUMN_4, GPIO_MODE_IN_PU_NO_IT);//列线 4 上拉输入
	//delay_us(5);
	u8 column_temp=GPIO_ReadInputPin(COLUMN_4);
	GPIO_Init(COLUMN_4, GPIO_MODE_OUT_PP_HIGH_FAST);//列线 4 推挽输出
	GPIO_WriteLow(COLUMN_4);
	GPIO_Init(ROW_4, GPIO_MODE_IN_PU_NO_IT);//行线 4 上拉输入
	u8 row_temp=GPIO_ReadInputPin(ROW_4);
	if(column_temp || row_temp)
		return 0;
	return 1;
}

/*
*********************************************************************************************************
* 函 数 名: bsp_DetectButton
* 功能说明: 检测一个按键。非阻塞状态，必须被周期性的调用。
* 形 参：按键结构变量指针
* 返 回 值: 无
*********************************************************************************************************
*/
void Button_Detect(BUTTON_T *_pBtn)
{
	if (_pBtn->IsKeyDownFunc())
	{
		if (_pBtn->Count < _pBtn->FilterTime)
		{
		_pBtn->Count = _pBtn->FilterTime;
		}
	else if(_pBtn->Count < 2 * _pBtn->FilterTime)
	{
		_pBtn->Count++;
	}
	else
	{
		if (_pBtn->State == 0)
			{
			_pBtn->State = 1;

			/* 发送按钮按下的消息 */
			if (_pBtn->KeyCodeDown > 0)
				{
					/* 键值放入按键FIFO */
					UART1_SendData8(_pBtn->KeyCodeDown);// 记录按键按下标志，等待释放
					while( UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET );
				}
			}

		if (_pBtn->LongTime > 0)
			{
				if (_pBtn->LongCount < _pBtn->LongTime)
					{
						/* 发送按钮持续按下的消息 */
						if (++_pBtn->LongCount == _pBtn->LongTime)
							{
								/* 键值放入按键FIFO */
								UART1_SendData8(_pBtn->KeyCodeLong);
								while( UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET );
							}
					}
			else
				{
				if (_pBtn->RepeatSpeed > 0)
					{
					if (++_pBtn->RepeatCount >= _pBtn->RepeatSpeed)
						{
							_pBtn->RepeatCount = 0;
							/* 常按键后，每隔10ms发送1个按键 */
							UART1_SendData8(_pBtn->KeyCodeDown);
							while( UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET );
						}
					}
				}
			}
		}
	}
	else
	{
	if(_pBtn->Count > _pBtn->FilterTime)
	{
	_pBtn->Count = _pBtn->FilterTime;
	}
	else if(_pBtn->Count != 0)
	{
	_pBtn->Count--;
	}
	else
	{
	if (_pBtn->State == 1)
		{
			_pBtn->State = 0;

	/* 发送按钮弹起的消息 */
			if (_pBtn->KeyCodeUp > 0) /*按键释放*/
			{
			/* 键值放入按键FIFO */
			UART1_SendData8(_pBtn->KeyCodeUp);
			while( UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET );
			}
		}
	}

	_pBtn->LongCount = 0;
	_pBtn->RepeatCount = 0;
	}
}

//功能说明: 检测所有按键。10MS 调用一次
void Pannelkey_Polling(void)
{
	for(u8 i=0;i<16;i++)
	{
		Button_Detect(&Key[i]); /*Key_1 键 */
	}
	delay_ms(10);
}

 

static void UART_Config(void)
{
	/* Deinitializes the UART1 and UART3 peripheral */
	UART1_DeInit();
	// UART3_DeInit();
	/* UART1 and UART3 configuration -------------------------------------------------*/
	/* UART1 and UART3 configured as follow:
	- BaudRate = 9600 baud
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Receive and transmit enabled
	- UART1 Clock disabled
	*/
	/* Configure the UART1 */
	UART1_Init((u32)9600, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
	UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);

	/* Enable UART1 Transmit interrupt*/
	UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);

	enableInterrupts();
	UART1_Cmd(ENABLE);
	UART1_SendData8(0x00);
}

 

void Key_Init()
{
	for(u8 i=0;i<16;i++){
		/* 初始化USER按键变量，支持按下、弹起、长按 */
		// Key[i].IsKeyDownFunc = Key_Down; /* 判断按键按下的函数 */
		Key[i].FilterTime = BUTTON_FILTER_TIME; /* 按键滤波时间 */
		Key[i].LongTime = BUTTON_LONG_TIME; /* 长按时间 */
		Key[i].Count = Key[i].FilterTime / 2; /* 计数器设置为滤波时间的一半 */
		Key[i].State = 0; /* 按键缺省状态，0为未按下 */
		Key[i].KeyCodeDown = KeyDownCode[i]; /* 按键按下的键值代码 */
		Key[i].KeyCodeUp =KeyUpCode[i]; /* 按键弹起的键值代码 */
		Key[i].KeyCodeLong = KeyLongCode[i]; /* 按键被持续按下的键值代码 */
		Key[i].RepeatSpeed = 0; /* 按键连发的速度，0表示不支持连发 */
		Key[i].RepeatCount = 0; /* 连发计数器 */
		switch(i){
		case 0:
		Key[0].IsKeyDownFunc = Key_1_Down;
		break;
		case 1:
		Key[1].IsKeyDownFunc = Key_2_Down;
		break;
		case 2:
		Key[2].IsKeyDownFunc = Key_3_Down;
		break;
		case 3:
		Key[3].IsKeyDownFunc = Key_4_Down;
		break;
		case 4:
		Key[4].IsKeyDownFunc = Key_5_Down;
		break;
		case 5:
		Key[5].IsKeyDownFunc = Key_6_Down;
		break;
		case 6:
		Key[6].IsKeyDownFunc = Key_7_Down;
		break;
		case 7:
		Key[7].IsKeyDownFunc = Key_8_Down;
		break;
		case 8:
		Key[8].IsKeyDownFunc = Key_9_Down;
		break;
		case 9:
		Key[9].IsKeyDownFunc = Key_10_Down;
		break;
		case 10:
		Key[10].IsKeyDownFunc = Key_11_Down;
		break;
		case 11:
		Key[11].IsKeyDownFunc = Key_12_Down;
		break;
		case 12:
		Key[12].IsKeyDownFunc = Key_13_Down;
		break;
		case 13:
		Key[13].IsKeyDownFunc = Key_14_Down;
		break;
		case 14:
		Key[14].IsKeyDownFunc = Key_15_Down;
		break;
		case 15:
		Key[15].IsKeyDownFunc = Key_16_Down;
		break;
		}
	}
}
```