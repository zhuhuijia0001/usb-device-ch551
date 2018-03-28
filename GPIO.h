
#ifndef _GPIO_H_
#define _GPIO_H_

//gpio mode
typedef enum
{
	GPIO_Mode_IN_Floating = 0,
	GPIO_Mode_PP_Out      = 1,
	GPIO_Mode_OD_Out      = 2,
	GPIO_Mode_IPU         = 3
} GPIO_Mode;

//#define GPIO_INTERRUPT   1
  
/*******************************************************************************
* Function Name  : Port1Cfg()
* Description    : 端口1配置
* Input          : UINT8 Pin	(0-7),	
				   Mode  0 = 浮空输入，无上拉
                         1 = 推挽输入输出
                         2 = 开漏输入输出，无上拉
                         3 = 类51模式，开漏输入输出，有上拉，内部电路可以加速由低到高的电平爬升											 
* Output         : None
* Return         : None
*******************************************************************************/
extern void Port1Cfg(UINT8 Pin, GPIO_Mode Mode);

extern void Port3Cfg(UINT8 Pin, GPIO_Mode Mode);

/*******************************************************************************
* Function Name  : GPIOInterruptCfg()
* Description    : GPIO中断配置
* Input          : None									 
* Output         : None
* Return         : None
*******************************************************************************/
void GPIOInterruptCfg();

#endif

