  
#include "bsp_SysTick.h"
#include "core_cm3.h"
//#include "misc.h"

static __IO uint32_t TimingDelay;

/**************************************************************************
Function: SysTick_Init
Input   : none
Output  : none
函数功能：嘀嗒定时器初始化
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
//	if (SysTick_Config(SystemFrequency / 100000))	// ST3.0.0库版本
	if (SysTick_Config(SystemCoreClock / 100000))	// ST3.5.0库版本
	{ 
		/* Capture error */ 
		while (1);
	}
}


/**************************************************************************
Function: Delay_us
Input   : nTime
Output  : none
函数功能：us延时程序
入口参数: us延时程序，10us为一个单位 
返回  值：无
**************************************************************************/	 	
//Delay_us( 1 ) 则实现的延时为 1 * 10us = 10us
void Delay_us(__IO uint32_t nTime)
{ 
	TimingDelay = nTime;	

	// 使能滴答定时器  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay != 0);
}

/**************************************************************************
Function: TimingDelay_Decrement
Input   : none
Output  : none
函数功能：获取节拍程序
入口参数: 无 
返回  值：无
**************************************************************************/	 	

void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{ 
		TimingDelay--;
	}
}

#if 0
// 这个 固件库函数 在 core_cm3.h中
static __INLINE uint32_t SysTick_Config(uint32_t ticks)
{ 
  // reload 寄存器为24bit，最大值为2^24
	if (ticks > SysTick_LOAD_RELOAD_Msk)  return (1);
  
  // 配置 reload 寄存器的初始值	
  SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;
	
	// 配置中断优先级为 1<<4-1 = 15，优先级为最低
  NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1); 
	
	// 配置 counter 计数器的值
  SysTick->VAL   = 0;
	
	// 配置systick 的时钟为 72M
	// 使能中断
	// 使能systick
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | 
                   SysTick_CTRL_TICKINT_Msk   | 
                   SysTick_CTRL_ENABLE_Msk;                    
  return (0); 
}
#endif


/**************************************************************************
Function: SysTick_Delay_Us
Input   : us 
Output  : none
函数功能：us延时程序
入口参数: us延时时间 
返回  值：无
**************************************************************************/	 	
// couter 减1的时间 等于 1/systick_clk
// 当counter 从 reload 的值减小到0的时候，为一个循环，如果开启了中断则执行中断服务程序，
// 同时 CTRL 的 countflag 位会置1
// 这一个循环的时间为 reload * (1/systick_clk)

void SysTick_Delay_Us( __IO uint32_t us)
{
	uint32_t i;
	SysTick_Config(SystemCoreClock/1000000);
	
	for(i=0;i<us;i++)
	{
		// 当计数器的值减小到0的时候，CRTL寄存器的位16会置1	
		while( !((SysTick->CTRL)&(1<<16)) );
	}
	// 关闭SysTick定时器
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}


/**************************************************************************
Function: SysTick_Delay_Ms
Input   : ms 
Output  : none
函数功能：ms延时程序
入口参数: ms延时时间 
返回  值：无
**************************************************************************/	 	

void SysTick_Delay_Ms( __IO uint32_t ms)
{
	uint32_t i;	
	SysTick_Config(SystemCoreClock/1000);
	
	for(i=0;i<ms;i++)
	{
		// 当计数器的值减小到0的时候，CRTL寄存器的位16会置1
		// 当置1时，读取该位会清0
		while( !((SysTick->CTRL)&(1<<16)) );
	}
	// 关闭SysTick定时器
	SysTick->CTRL &=~ SysTick_CTRL_ENABLE_Msk;
}


/*********************************************END OF FILE**********************/
