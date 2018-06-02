---
title: "stm8s之NEC协议红外发射"
date: 2017-07-15 12:49:25
toc: false
tags: [单片机,stm8]
---
单片机采用stm8s103f3p6，发送引脚为TIM2的oc3即PA3引脚。
<!-- more -->
```
//******************************************************************************                              
//introduce:        红外传感器发送驱动        
//******************************************************************************     
#include "stm8s.h"   
#include "delay.h"

/*********************外部变量************************/    
static void TIM1_Config(void);
int main()
{
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
  CLK_HSICmd(ENABLE);
  delay_init(16);
  u32 Send_Val=0x535A00FF;
  u8 i;  
  GPIO_Init(SZ_INFRARED_SEND_PORT, GPIO_MODE_OUT_PP_HIGH_FAST);
  TIM1_Config();
  while(1){
    
    
    
    TIM2_SetCompare3(421);
    
    delay_ms(9);
    TIM2_SetCompare3(0);
    delay_ms(4);
    delay_us(500);
    
    for(i=0;i<32;i++){
      if((0x80000000>>i)&Send_Val){
        TIM2_SetCompare3(130);
        delay_us(560);
        TIM2_SetCompare3(0);
        delay_us(1690);
      }else{
        TIM2_SetCompare3(130);
        delay_us(560);
        TIM2_SetCompare3(0);
        delay_us(560);
      }
    }
    TIM2_SetCompare3(130);
    delay_ms(1);
    TIM2_SetCompare3(0);
    delay_ms(1000);
  }
}
定时器配置PWM如下：

static void TIM1_Config(void)
{
  
  TIM2_DeInit();
  
  
  TIM2_TimeBaseInit(TIM2_PRESCALER_1, 421);
  

  
  TIM2_OC3Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_DISABLE, 0, TIM2_OCPOLARITY_HIGH);
  
  TIM2_CCxCmd(TIM2_CHANNEL_3, ENABLE);
  TIM2_Cmd(ENABLE);
}
```