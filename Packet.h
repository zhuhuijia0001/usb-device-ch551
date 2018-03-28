
#ifndef _PACKET_H_
#define _PACKET_H_

extern BOOL BuildKeyboardLedPacket(UINT8 *out, UINT8 maxLen, UINT8 *pLen, UINT8 led);

extern BOOL BuildOnlineStatusPacket(UINT8 *out, UINT8 maxLen, UINT8 *pLen, UINT8 online);

#endif


