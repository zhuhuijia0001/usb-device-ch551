
#include "Type.h"
#include "Ch552.h"
#include "System.h"
#include "Uart.h"

/*******************************************************************************
* Function Name  : CH554UART0Alter()
* Description    : CH554串口0引脚映射,串口映射到P0.2和P0.3
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH554UART0Alter()
{
    PIN_FUNC |= bUART0_PIN_X;                                                  //串口映射到P1.2和P1.3
}

/*******************************************************************************
* Function Name  : InitUART0()
* Description    : CH554串口0初始化,默认使用T1作UART0的波特率发生器,也可以使用T2
                   作为波特率发生器
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitUART0( )
{
    UINT32 x;
    UINT8 x2; 

	//串口0使用模式1
    SM0 = 0;
    SM1 = 1;
    SM2 = 0;                                                                   
    
    //使用Timer1作为波特率发生器                                                                          
    RCLK = 0;
    TCLK = 0;
    
    PCON |= SMOD;
    x = 10 * FREQ_SYS / UART0_BUAD / 16;                                       //如果更改主频，注意x的值不要溢出                            
    x2 = x % 10;
    x /= 10;
    if ( x2 >= 5 ) x ++;                                                       //四舍五入

    TMOD = TMOD & ~ bT1_GATE & ~ bT1_CT & ~ MASK_T1_MOD | bT1_M1;              //0X20，Timer1作为8位自动重载定时器
    T2MOD = T2MOD | bTMR_CLK | bT1_CLK;                                        //Timer1时钟选择
    TH1 = 0-x;                                                                 //12MHz晶振,buad/12为实际需设置波特率
    TR1 = 1;                                                                   //启动定时器1
    TI = 0;
    REN = 1;                                                                   //串口0接收使能

    ES = 1;
}

/*******************************************************************************
* Function Name  : CH554UART0RcvByte()
* Description    : CH554UART0接收一个字节
* Input          : None
* Output         : None
* Return         : SBUF
*******************************************************************************/
UINT8  CH554UART0RcvByte( )
{
    while(RI == 0);                                                            //查询接收，中断方式可不用
    RI = 0;
    return SBUF;
}

static BOOL volatile s_sent = FALSE;

void SetUart0Sent(void)
{
	s_sent = TRUE;
}

/*******************************************************************************
* Function Name  : CH554UART0SendByte(UINT8 SendDat)
* Description    : CH554UART0发送一个字节
* Input          : UINT8 SendDat；要发送的数据
* Output         : None
* Return         : None
*******************************************************************************/
void CH554UART0SendByte(UINT8 SendDat)
{
	SBUF = SendDat;
	s_sent = FALSE;
	while (!s_sent);
}

void CH554UART0SendData(const UINT8 *pData, UINT8 len)
{
	while (len-- > 0)
	{
		s_sent = FALSE;
		SBUF = *pData++;
		while (!s_sent);
	}
}

/*******************************************************************************
* Function Name  : UART1Setup()
* Description    : CH554串口1初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART1Setup( )
{
   U1SM0 = 0;                                                                   //UART1选择8位数据位
   U1SMOD = 1;                                                                  //快速模式
   U1REN = 1;                                                                   //使能接收
   SBAUD1 = 0 - FREQ_SYS/16/UART1_BUAD;
}

/*******************************************************************************
* Function Name  : CH554UART1RcvByte()
* Description    : CH554UART1接收一个字节
* Input          : None
* Output         : None
* Return         : SBUF
*******************************************************************************/
UINT8  CH554UART1RcvByte( )
{
    while(U1RI == 0);                                                           //查询接收，中断方式可不用
    U1RI = 0;
    return SBUF1;
}

/*******************************************************************************
* Function Name  : CH554UART1SendByte(UINT8 SendDat)
* Description    : CH554UART1发送一个字节
* Input          : UINT8 SendDat；要发送的数据
* Output         : None
* Return         : None
*******************************************************************************/
void CH554UART1SendByte(UINT8 SendDat)
{
	SBUF1 = SendDat;                                                             //查询发送，中断方式可不用下面2条语句,但发送前需TI=0
	while(U1TI ==0);
	U1TI = 0;
}

