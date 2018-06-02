---
title: "NEC协议红外接收之stm8s103f3p6"
date: 2017-08-10 12:55:22
toc: false
tags: [单片机,stm8]
---
单片机采用stm8s103f3p6,引脚接至 PB4。
<!-- more -->
```
#include “stm8s.h”
#include “delay.h”
/* Private defines ———————————————————–*/
/* Private function prototypes ———————————————–*/
/* Private functions ———————————————————*/
/*
*********************************************************************************************************
*
* 模块名称 : 红外遥控解码
*
引导码 地址第一位 地址第二位 数据码 数据反码
字符”S”=0x53 字符”Z”=0x5A
\ 9+4.5ms \ c0-c07 \ c0-c07 \ c0-c07 \ c0-c07 \
*********************************************************************************************************
*/

#define REMOTE_ID 1     //红外遥控识别码(ID)

u8  Ir_Status=0;              //红外接收处理状态
u8  Ir_Receive_Count=0;       //红外接收数据位计数
u32 Ir_Receive_Data=0;        //32位的红外接收数据
u8  Ir_receive_ok=0;          //红外接收完成标志

/*
********************************************************************************
定时器初始化
********************************************************************************
*/

void TIM2_Config(void)
{
  TIM2_DeInit();
  TIM2_TimeBaseInit(TIM2_PRESCALER_16, 50000);//定时器设置1M的计数频率，1US的分辨率 ,计时50ms
  TIM2_ClearFlag(TIM2_FLAG_UPDATE);
  TIM2_ITConfig(TIM2_IT_UPDATE, DISABLE);//关闭更新中断，只调用查询函数。
  TIM2_Cmd(ENABLE);
}
/*
********************************************************************************
外部中断初始化
********************************************************************************
*/

void EXTIB_Config(void)
{
  disableInterrupts();
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_FALL_ONLY);//下降沿中断
  enableInterrupts();
}

/*
********************************************************************************
红外解码初始化函数
********************************************************************************
*/
void Ir_Init(void)
{
  GPIO_Init(RED_RECEIVE_PORT, GPIO_MODE_IN_PU_IT);   //红外端口PB4初始化
  EXTIB_Config();
  TIM2_Config();
}

/*
********************************************************************************
红外接收数据处理函数
********************************************************************************
*/

u8 Ir_Process(void)
{
  u8 Ir_num=0;                  //最终处理后的键值返回值
  u8 Address_H,Address_L;       //地址码,地址反码
  u8 Data_H,Data_L;             //数据码,数据反码
  
  if(Ir_receive_ok==1)          //接收完成
  {
    Address_H=Ir_Receive_Data>>24;                //得到地址码第一位
    Address_L=(Ir_Receive_Data>>16)&0xff;         //得到地址码第二位 
    if((Address_H==0x53)&&(Address_L==0x5A))
    //检验遥控识别码(地址一Address_H = 'S'即0x53,地址二Address_L = 'Z') 
    { 
      Data_H=Ir_Receive_Data>>8;              //得到数据码
      Data_L=Ir_Receive_Data;                 //得到数据反码
      if(Data_H==(u8)~Data_L)                 //接收数据码正确
      {
        Ir_num=Data_H;                      //正确键值
        
      }
    }    
  }
  return  Ir_num;      //返回键值
}
/*
********************************************************************************
红外接收中断处理函数
********************************************************************************
*/

void Ir_Receive_Handle(void)
{
  u16 Interval_tim=0;//两个下降沿间隔时间 
  switch(Ir_Status)
  {
  case 0://第一个下降沿，定时器开始计数                    
    Ir_Status=1;
 
    TIM2_SetCounter(0);                 //定时器计数值清零
    break;
    
  case 1://第二个下降沿，定时器关闭，读取定时器计数值                                              
    

    Interval_tim=TIM2_GetCounter();     //读取定时器计数值
    TIM2_SetCounter(0);                 //定时器计数值清零

  
    if( (Interval_tim>=12500)&&(Interval_tim<=14500) )//判断引导码是否正确9+4.5ms { Ir_Status=2; //进入下一状态 } else //引导码错误，从新接收 { Ir_Status=0; Ir_Receive_Count=0; } break; case 2://开始32位数据的接收 Interval_tim=TIM2_GetCounter(); if( (Interval_tim>=1000)&&(Interval_tim<=1300) ) //间隔1.12ms ->0
    {
      Ir_Receive_Data=Ir_Receive_Data<<1; Ir_Receive_Count++; } else if( (Interval_tim>=2000)&&(Interval_tim<=2600) ) //间隔2.25ms ->1
    {
      Ir_Receive_Data=Ir_Receive_Data<<1;
      Ir_Receive_Data=Ir_Receive_Data|0x0001;
      Ir_Receive_Count++;
      
    }
    else//不是0,1 接收错误，从新接收
    {
      Ir_Status=0;
      Ir_Receive_Data=0;
      Ir_Receive_Count=0;
    }
    
    //超出接收数据位数，接收下一个
    while(Ir_Receive_Count==32)
    {         
      Ir_receive_ok=1;//红外接收完成
      Ir_Status=0;
      Ir_Receive_Count=0;
      enableInterrupts();//接收完成，打开中断，避免在接收过程中受其他中断影响。
      break;
    } 
    
    
    TIM2_SetCounter(0);//从零开始计数
    
    break;
    
  default :
    TIM2_SetCounter(0);//从零开始计数
    break;
  }
}
/*
********************************************************************************
********************************************************************************
*/

//当红外接收到信号进入中断，运行定时器执行程序，红外未收到信号空闲时无中断定时器关闭

void main(void)
{ 
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
  CLK_HSICmd(ENABLE);
  
  uint8_t key_val;
  Ir_Init();
  
  u8 test_val=(0x53&0x4F&0x5A&0x4f&0x53);
  if(test_val){}
  
  while(1)  
  {
    if( Ir_receive_ok == 1 ) //一帧红外数据接收完成 
    {
      key_val = Ir_Process();
      Ir_receive_ok=0;
      
      //不同的遥控器面板对应不同的键值 0-9
      switch( key_val )
      {                                                
      }
      Ir_Receive_Data=0;//清空接收数据
    }
    
  } 
}

/*****************************/
INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4)
{
  /* In order to detect unexpected events during development,
  it is recommended to set a breakpoint on the following instruction.
  */
  
  disableInterrupts();
  Ir_Receive_Handle(); 
  
}
```