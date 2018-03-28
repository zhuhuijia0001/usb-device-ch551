
#ifndef _UART_H_
#define _UART_H_

extern void CH554UART0Alter(void);

extern void InitUART0(void);

extern UINT8 CH554UART0RcvByte(void);

extern void SetUart0Sent(void);

extern void CH554UART0SendByte(UINT8 SendDat);

extern void CH554UART0SendData(const UINT8 *pData, UINT8 len);

extern void	UART1Setup(void);

extern UINT8 CH554UART1RcvByte(void);

extern void CH554UART1SendByte(UINT8 SendDat);

#endif

