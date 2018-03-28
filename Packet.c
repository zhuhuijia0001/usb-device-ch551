#include "Type.h"
#include "Protocol.h"
#include "Packet.h"

static BOOL BuildPacket(UINT8 *out, UINT8 maxLen, UINT8 *pLen, UINT8 id, const UINT8 *dat, UINT8 size)
{
	UINT8 index;
	UINT8 i;
	UINT8 sum;
	
	if (size + 2 > maxLen)
	{
		return FALSE;
	}

	index = 0;
	out[index++] = id;

	sum = id;
	for (i = 0; i < size; i++) 
	{
		out[index++] = *dat;

		sum ^= *dat;

		dat++;
	}
	out[index++] = sum;

	if (pLen != NULL)
	{
		*pLen = index;
	}
	
	return TRUE;
}

BOOL BuildKeyboardLedPacket(UINT8 *out, UINT8 maxLen, UINT8 *pLen, UINT8 led)
{
	return BuildPacket(out, maxLen, pLen, ID_LED_STATUS, &led, sizeof(led));
}

BOOL BuildOnlineStatusPacket(UINT8 *out, UINT8 maxLen, UINT8 *pLen, UINT8 online)
{
	return BuildPacket(out, maxLen, pLen, ID_QUERY_ONLINE, &online, sizeof(online));
}

