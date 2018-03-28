
/********************************** (C) COPYRIGHT *******************************
* File Name          : GPIO.C
* Author             : WCH
* Version            : V1.0
* Date               : 2017/01/20
* Description        : CH554 IO 设置接口函数和GPIO中断函数  
*******************************************************************************/

#include "Type.h"
#include "CH552.h" 
#include "GPIO.h"
#include "stdio.h"

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
void Port1Cfg(UINT8 Pin, GPIO_Mode Mode)
{
	switch(Mode)
	{
	case GPIO_Mode_IN_Floating:
		P1_MOD_OC = P1_MOD_OC & ~(1<<Pin);
		P1_DIR_PU = P1_DIR_PU &	~(1<<Pin);	
		break;
		
    case GPIO_Mode_PP_Out:
		P1_MOD_OC = P1_MOD_OC & ~(1<<Pin);
		P1_DIR_PU = P1_DIR_PU |	(1<<Pin);				
		break;
		
    case GPIO_Mode_OD_Out:
		P1_MOD_OC = P1_MOD_OC | (1<<Pin);
		P1_DIR_PU = P1_DIR_PU &	~(1<<Pin);				
		break;	
		
    case GPIO_Mode_IPU:
		P1_MOD_OC = P1_MOD_OC | (1<<Pin);
		P1_DIR_PU = P1_DIR_PU |	(1<<Pin);			
		break;
		
    default:
		break;			
	}
}

/*******************************************************************************
* Function Name  : Port3Cfg()
* Description    : 端口3配置
* Input          : UINT8 Pin	(0-7),
				   Mode  0 = 浮空输入，无上拉
                         1 = 推挽输入输出
                         2 = 开漏输入输出，无上拉
                         3 = 类51模式，开漏输入输出，有上拉，内部电路可以加速由低到高的电平爬升										 
* Output         : None
* Return         : None
*******************************************************************************/
void Port3Cfg(UINT8 Pin, GPIO_Mode Mode)
{
	switch(Mode)
	{
	case GPIO_Mode_IN_Floating:
		P3_MOD_OC = P3_MOD_OC & ~(1<<Pin);
		P3_DIR_PU = P3_DIR_PU & ~(1<<Pin);	
		break;
		
	case GPIO_Mode_PP_Out:
		P3_MOD_OC = P3_MOD_OC & ~(1<<Pin);
		P3_DIR_PU = P3_DIR_PU | (1<<Pin);				
		break;
		
	case GPIO_Mode_OD_Out:
		P3_MOD_OC = P3_MOD_OC | (1<<Pin);
		P3_DIR_PU = P3_DIR_PU & ~(1<<Pin);				
		break;	
		
	case GPIO_Mode_IPU:
		P3_MOD_OC = P3_MOD_OC | (1<<Pin);
		P3_DIR_PU = P3_DIR_PU | (1<<Pin);			
		break;
		
	default:
		break;			
	}
}


/*******************************************************************************
* Function Name  : GPIOInterruptCfg()
* Description    : GPIO中断配置
* Input          : None									 
* Output         : None
* Return         : None
*******************************************************************************/
void GPIOInterruptCfg()
{
   GPIO_IE &= ~bIE_IO_EDGE;                                                    //高/低电平触发
//    GPIO_IE |= bIE_IO_EDGE;                                                  //上升/下降触发  
//    GPIO_IE |= bIE_RXD1_LO;                                                     //RXD1低电平或下降沿触发
   GPIO_IE |= bIE_P1_5_LO | bIE_P1_4_LO | bIE_P1_3_LO | bIE_RST_HI;            
   //P15\P14\P13低电平触发；RST高电平触发
//    GPIO_IE |= bIE_P3_1_LO;                                                     //P31低电平或下降沿触发
//    GPIO_IE |= bIE_RXD0_LO;                                                     //RXD0低电平或下降沿触发	
   IE_GPIO  = 1;                                                               //GPIO中断开启
}

#ifdef GPIO_INTERRUPT
/*******************************************************************************
* Function Name  : GPIOInterrupt(void)
* Description    : GPIO 中断服务程序
*******************************************************************************/
void	GPIOInterrupt( void ) interrupt INT_NO_GPIO  using 1                      //GPIO中断服务程序,使用寄存器组1
{ 
#ifdef DE_PRINTF
      printf("GPIO_STATUS: %02x\n",(UINT16)(PIN_FUNC&bIO_INT_ACT));             
#endif
}
#endif

