/* 调试 */
/* 提供printf子程序和延时函数 */
/* CH554主频修改、延时函数定义
   串口0和串口1初始化
   串口0和串口1的收发子函数
   看门狗初始化	
*/

#ifndef	__DEBUG_H__
#define __DEBUG_H__

//定义函数返回值
#ifndef  SUCCESS
#define  SUCCESS  0
#endif
#ifndef  FAIL
#define  FAIL    0xFF
#endif

//定义定时器起始
#ifndef  START
#define  START  1
#endif
#ifndef  STOP
#define  STOP    0
#endif

#ifndef  DE_PRINTF
#define  DE_PRINTF     0
#endif
#define	 FREQ_SYS	     16000000ul	         //系统主频12MHz
#ifndef  UART0_BUAD
#define  UART0_BUAD    100000ul
#define  UART1_BUAD    57600ul
#endif

void CfgFsys( );                        //CH554时钟选择和配置
void mDelayuS( UINT16 n );              // 以uS为单位延时
void mDelaymS( UINT16 n );              // 以mS为单位延时

void CH554WDTModeSelect(UINT8 mode);     //CH554看门狗模式设置 
void CH554WDTFeed(UINT8 tim);            //CH554看门狗喂狗
#endif

