
#include "Type.h"
#include "Mcu.h"
#include "Protocol.h"
#include "RecvBuffer.h"
#include "Task.h"

#include "System.h"
#include "Uart.h"
#include "Timer.h"
#include "Usb.h"

#include "Gpio.h"

#include "Packet.h"

#define OUT_BUFFER_SIZE  8

static BOOL s_isSwitchedPort = TRUE;

//keyboard break code
static UINT8C s_keyboardBreakCode[KEYBOARD_LEN] = 
{
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
};

//mouse break code
static UINT8C s_mouseBreakCode[MOUSE_LEN] = 
{
	0x00, 0x00, 0x00, 0x00,
};

static void SendKeyboardToUsb(UINT8 *pData, UINT8 len)
{
	Enp1IntIn(pData, len);
}

static void SendMouseToUsb(UINT8 *pData, UINT8 len)
{
	Enp2IntIn(pData, len);
}

static void InitTimer2(UINT8 ms)
{
	UINT16 val = FREQ_SYS / 1000ul * ms;
	
    mTimer2ClkFsys();	           //T2定时器时钟设置
    mTimer_x_ModInit(2, 0);         //T2 定时器模式设置
    mTimer_x_SetData(2, val);	   //T2定时器赋值
    mTimer2RunCTL(1);              //T2定时器启动			

    ET2 = 1;
}

void InitSystem(void)
{
    CfgFsys();           //CH551时钟选择配置
    mDelaymS(5);         //修改主频等待内部晶振稳定,必加	

	InitRecvBuffer();
	
    InitUART0();         //串口0初始化
	
	InitTimer2(4);       //4ms 中断

#ifdef DEBUG
	Port1Cfg(6, 1);
	Port1Cfg(7, 1);
#endif

    USBDeviceInit();     //USB设备模式初始化

#ifndef DEBUG
	CH554WDTModeSelect(1);
#endif

    HAL_ENABLE_INTERRUPTS();    //允许单片机中断
}

void ProcessUartData(void)
{
#ifdef DEBUG
	if (CheckPCReady())
	{
		P1_6 = 1;
	}
	else
	{
		P1_6 = 0;
	}
#endif

	if (!IsRecvBufferEmpty())
	{
		UINT8 *packet = GetOutputBuffer();

		UINT8 id = packet[0];
		UINT8 *pData = &packet[1];   
		switch (id)
		{
		case ID_USB_KEYBOARD:			
		    if (!CheckPCReady())
		    {
                break;
		    }
		    
		    if (CheckPCSleeped())
		    {
                CH554USBDevWakeup();
		    }

			SendKeyboardToUsb(pData, KEYBOARD_LEN);
			
			break;

		case ID_USB_MOUSE:
		    if (!CheckPCReady())
		    {
                break;
		    }
		    
		    if (CheckPCSleeped())
		    {
                if (pData[0] != 0x00)
                {
                    //only mouse button wakeup pc
                    CH554USBDevWakeup();
                }
		    }
		    
			SendMouseToUsb(pData, MOUSE_LEN);
			
			break;

		case ID_QUERY_ONLINE:
			{
				UINT8 online;
				UINT8 len;

				
                UINT8 buffer[OUT_BUFFER_SIZE];
#ifdef DEBUG
				P1_6 = !P1_6;
#endif
				if (CheckPCReady())
				{
					online = STATUS_ONLINE;
				}
				else
				{
					online = STATUS_OFFLINE;
				}

				if (BuildOnlineStatusPacket(buffer, sizeof(buffer), &len, online))
				{
				    CH554UART0SendData(buffer, len);
				}	

				if (pData[0] == QUERY_CURRENT_PORT)
				{
                    UINT8 led = GetKeyboardLedStatus();
                                
                    if (BuildKeyboardLedPacket(buffer, sizeof(buffer), &len, led))
                    {
                        CH554UART0SendData(buffer, len);
                    }

                    s_isSwitchedPort = TRUE;
                }
                else 
                {
                    s_isSwitchedPort = FALSE;
                }
			}

			break;

		case ID_SWITCH:
			if (pData[0] == SWITCH_IN)
			{
			    UINT8 len;

                UINT8 buffer[OUT_BUFFER_SIZE];

                UINT8 led = GetKeyboardLedStatus();
                            
                if (BuildKeyboardLedPacket(buffer, sizeof(buffer), &len, led))
                {
                    CH554UART0SendData(buffer, len);
                }
			}
			else
			{
				//send break code
				if (CheckPCReady())
			    {
                    SendKeyboardToUsb(s_keyboardBreakCode, KEYBOARD_LEN);

				    SendMouseToUsb(s_mouseBreakCode, MOUSE_LEN);
			    }
			}
			
			break;

		default:
			break;
		}
	}
}

void ProcessKeyboardLed(void)
{	
    static UINT8 ledSave = 0x00;
    
	UINT8 led = GetKeyboardLedStatus();

	if (led != ledSave)
	{
		if (s_isSwitchedPort)
		{
			UINT8 len;
			UINT8 buffer[OUT_BUFFER_SIZE];
			
			if (BuildKeyboardLedPacket(buffer, sizeof(buffer), &len, led))
			{
				CH554UART0SendData(buffer, len);
			}
		}

        ledSave = led;
        
#ifdef DEBUG 

		if (led & 0x02)
		{
			P1_7 = 1;
		}		
		else
		{
			P1_7 = 0;
		}
#endif
	}
}

#ifndef DEBUG
void FeedWdt(void)
{
	CH554WDTFeed(0);
}

#else
void FeedWdt(void)
{
}

#endif

