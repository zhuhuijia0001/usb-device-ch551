
#ifndef _USB_H_
#define _USB_H_

extern void CH554SoftReset(void);

extern void CH554USBDevWakeup(void);

extern void USBDeviceInit(void);

extern UINT8 GetKeyboardLedStatus(void);

extern BOOL CheckPCReady(void);

extern void SetPCReady(BOOL ready);

extern BOOL CheckPCSleeped(void);

extern void SetPCSleeped(BOOL sleeped);

extern void Enp1IntIn(UINT8 *dat, UINT8 size);

extern void Enp2IntIn(UINT8 *dat, UINT8 size);

#endif

