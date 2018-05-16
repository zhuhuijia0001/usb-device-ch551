
#include "Type.h"
#include "Ch552.h"
#include "RecvBuffer.h"
#include "Uart.h"

void Uart0Isr(void) interrupt INT_NO_UART0 using 1
{	
	if (RI)
	{
		UINT8 ch;
		RI = 0;

		ch = SBUF;

		RecvBufferOneByte(ch);
	}

	if (TI)
	{
		TI = 0;

		SetUart0Sent();
	}
}

void Timer2Isr(void) interrupt INT_NO_TMR2 using 2
{
	if(TF2)
    { 	
        TF2 = 0; 

		RecvBufferTimerout();
	}
}

