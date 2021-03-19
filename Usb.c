
#include <string.h>

#include "Type.h"
#include "Ch552.h"
#include "Mcu.h"

#include "PinDefine.h"

#include "System.h"
#include "UsbDef.h"
#include "Usb.h"
#include "UsbDescriptor.h"

#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE

static UINT8X  Ep0Buffer[8>(THIS_ENDP0_SIZE+2)?8:(THIS_ENDP0_SIZE+2)] _at_ 0x0000;    //¶Ëµã0 OUT&IN»º³åÇø£¬±ØÐëÊÇÅ¼µØÖ·
static UINT8X  Ep1Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x000a;  //¶Ëµã1 IN»º³åÇø,±ØÐëÊÇÅ¼µØÖ·
static UINT8X  Ep2Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x0050;  //¶Ëµã2 IN»º³åÇø,±ØÐëÊÇÅ¼µØÖ·
static UINT8   SetupReq,SetupLen,UsbConfig;

static BOOL    PCSleeped = FALSE;

static BOOL    Ready = FALSE;

static PUINT8  pDescr;    

//keyboard led status
static UINT8 volatile keyboardLed = 0x00;

#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)

/*******************************************************************************
* Function Name  : CH554SoftReset()
* Description    : CH554Èí¸´Î»
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH554SoftReset()
{
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;
    GLOBAL_CFG	|=bSW_RESET;
}

/*******************************************************************************
* Function Name  : CH554USBDevWakeup()
* Description    : CH554Éè±¸Ä£Ê½»½ÐÑÖ÷»ú£¬·¢ËÍKÐÅºÅ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH554USBDevWakeup()
{
  UDEV_CTRL |= bUD_LOW_SPEED;
  mDelaymS(2);
  UDEV_CTRL &= ~bUD_LOW_SPEED;	
}

static UINT8 HexToAscii(UINT8 hex)
{
	if (hex <= 9)
	{
		hex += '0';
	}
	else if (hex >= 10 && hex <= 15)
	{
		hex += 'A' - 10;
	}

	return hex;
}

//for unique serial no
static void InitSerialString()
{
	UINT8 i = 2;
	UINT8 dat;

	dat = *((PUINT8C)(0x3FFC));	
	StringSerial[i++] = HexToAscii(dat & 0x0f);
	StringSerial[i++] = 0;

	StringSerial[i++] = HexToAscii(dat >> 4);
	StringSerial[i++] = 0;

	dat = *((PUINT8C)(0x3FFD));
	StringSerial[i++] = HexToAscii(dat & 0x0f);
	StringSerial[i++] = 0;

	StringSerial[i++] = HexToAscii(dat >> 4);
	StringSerial[i++] = 0;

	dat = *((PUINT8C)(0x3FFE));
	StringSerial[i++] = HexToAscii(dat & 0x0f);
	StringSerial[i++] = 0;

	StringSerial[i++] = HexToAscii(dat >> 4);
	StringSerial[i++] = 0;

	dat = *((PUINT8C)(0x3FFF));
	StringSerial[i++] = HexToAscii(dat & 0x0f);
	StringSerial[i++] = 0;

	StringSerial[i++] = HexToAscii(dat >> 4);
	StringSerial[i++] = 0;
}

/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : USBÉè±¸Ä£Ê½ÅäÖÃ,Éè±¸Ä£Ê½Æô¶¯£¬ÊÕ·¢¶ËµãÅäÖÃ£¬ÖÐ¶Ï¿ªÆô
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
	InitSerialString();
	
	IE_USB = 0;
	USB_CTRL = 0x00;                                                           // ÏÈÉè¶¨USBÉè±¸Ä£Ê½

	USB_CTRL &= ~bUC_LOW_SPEED;
    UDEV_CTRL &= ~bUD_LOW_SPEED; 
    
	UEP0_DMA = (UINT16)Ep0Buffer;                                                      //¶Ëµã0Êý¾Ý´«ÊäµØÖ·
	UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                                //¶Ëµã0µ¥64×Ö½ÚÊÕ·¢»º³åÇø
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //OUTÊÂÎñ·µ»ØACK£¬INÊÂÎñ·µ»ØNAK

	UEP1_DMA = (UINT16)Ep1Buffer;                                                      //¶Ëµã1Êý¾Ý´«ÊäµØÖ·
	UEP4_1_MOD = UEP4_1_MOD & ~bUEP1_BUF_MOD | bUEP1_TX_EN;                    //¶Ëµã1·¢ËÍÊ¹ÄÜ 64×Ö½Ú»º³åÇø
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //¶Ëµã1×Ô¶¯·­×ªÍ¬²½±êÖ¾Î»£¬INÊÂÎñ·µ»ØNAK	

	UEP2_DMA = (UINT16)Ep2Buffer;                                                      //¶Ëµã2Êý¾Ý´«ÊäµØÖ·
	UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN;                    //¶Ëµã2·¢ËÍÊ¹ÄÜ 64×Ö½Ú»º³åÇø
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //¶Ëµã2×Ô¶¯·­×ªÍ¬²½±êÖ¾Î»£¬INÊÂÎñ·µ»ØNAK
		
	USB_DEV_AD = 0x00;
	UDEV_CTRL = bUD_PD_DIS;                                                    // ½ûÖ¹DP/DMÏÂÀ­µç×è
	USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // Æô¶¯USBÉè±¸¼°DMA£¬ÔÚÖÐ¶ÏÆÚ¼äÖÐ¶Ï±êÖ¾Î´Çå³ýÇ°×Ô¶¯·µ»ØNAK
	UDEV_CTRL |= bUD_PORT_EN;                                                  // ÔÊÐíUSB¶Ë¿Ú
	USB_INT_FG = 0xFF;                                                         // ÇåÖÐ¶Ï±êÖ¾
	USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;

	UEP1_T_LEN = 0;                                                       //Ô¤Ê¹ÓÃ·¢ËÍ³¤¶ÈÒ»¶¨ÒªÇå¿Õ
    UEP2_T_LEN = 0;                                                       //Ô¤Ê¹ÓÃ·¢ËÍ³¤¶ÈÒ»¶¨ÒªÇå¿Õ
    
	IE_USB = 1;
}

UINT8 GetKeyboardLedStatus()
{
	UINT8 led;

	HAL_CRITICAL_STATEMENT(led = keyboardLed);

	return led;
}

BOOL CheckPCReady()
{
    BOOL ready;

	HAL_CRITICAL_STATEMENT(ready = Ready);

	return ready;
}

void SetPCSleeped(BOOL sleeped)
{    
    HAL_CRITICAL_STATEMENT(PCSleeped = sleeped);
}

BOOL CheckPCSleeped()
{
    BOOL sleeped;

	HAL_CRITICAL_STATEMENT(sleeped = PCSleeped);

	return sleeped;
}

/*******************************************************************************
* Function Name  : Enp1IntIn()
* Description    : USBÉè±¸Ä£Ê½¶Ëµã1µÄÖÐ¶ÏÉÏ´«
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp1IntIn(UINT8 *dat, UINT8 size)
{
    memcpy(Ep1Buffer, dat, size);
    UEP1_T_LEN = size;
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //ÓÐÊý¾ÝÊ±ÉÏ´«Êý¾Ý²¢Ó¦´ðACK
}
/*******************************************************************************
* Function Name  : Enp2IntIn()
* Description    : USBÉè±¸Ä£Ê½¶Ëµã2µÄÖÐ¶ÏÉÏ´«
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2IntIn(UINT8 *dat, UINT8 size)
{
    memcpy(Ep2Buffer, dat, size);
    UEP2_T_LEN = size;
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //ÓÐÊý¾ÝÊ±ÉÏ´«Êý¾Ý²¢Ó¦´ðACK
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USBÖÐ¶Ï´¦Àíº¯Êý
*******************************************************************************/
void UsbIsr(void) interrupt INT_NO_USB using 1                      //USBÖÐ¶Ï·þÎñ³ÌÐò,Ê¹ÓÃ¼Ä´æÆ÷×é1
{
    UINT8 len = 0;
    if(UIF_TRANSFER)                                                            //USB´«ÊäÍê³É±êÖ¾
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# ÖÐ¶Ï¶ËµãÉÏ´«
            UEP2_T_LEN = 0;                                                     //Ô¤Ê¹ÓÃ·¢ËÍ³¤¶ÈÒ»¶¨ÒªÇå¿Õ
//          UEP1_CTRL ^= bUEP_T_TOG;                                          //Èç¹û²»ÉèÖÃ×Ô¶¯·­×ªÔòÐèÒªÊÖ¶¯·­×ª
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Ä¬ÈÏÓ¦´ðNAK
            break;
            
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# ÖÐ¶Ï¶ËµãÉÏ´«
            UEP1_T_LEN = 0;                                                     //Ô¤Ê¹ÓÃ·¢ËÍ³¤¶ÈÒ»¶¨ÒªÇå¿Õ
//            UEP2_CTRL ^= bUEP_T_TOG;                                          //Èç¹û²»ÉèÖÃ×Ô¶¯·­×ªÔòÐèÒªÊÖ¶¯·­×ª
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Ä¬ÈÏÓ¦´ðNAK
            break;
            
        case UIS_TOKEN_SETUP | 0:                                                //SETUPÊÂÎñ
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // ÏÞÖÆ×Ü³¤¶È
                }
                len = 0;                                                        // Ä¬ÈÏÎª³É¹¦²¢ÇÒÉÏ´«0³¤¶È
                SetupReq = UsbSetupBuf->bRequest;								
                if (( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD)/* HIDÀàÃüÁî */
                {
					switch( SetupReq ) 
					{
					case HID_GET_REPORT:
						break;
						
					case HID_GET_IDLE:
						break;	
						
					case HID_GET_PROTOCOL:
						break;				
						
					case HID_SET_REPORT:										
						break;
						
					case HID_SET_IDLE:
						break;	

						
					case HID_SET_PROTOCOL:
						break;
						
					default:
						len = 0xFF;  								 					            /*ÃüÁî²»Ö§³Ö*/					
						break;
					}	
                }
                else
                {//±ê×¼ÇëÇó
                    switch(SetupReq)                                        //ÇëÇóÂë
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case USB_DESCR_TYP_DEVICE:                          //Éè±¸ÃèÊö·û
                            pDescr = DevDesc.descr;                               //°ÑÉè±¸ÃèÊö·ûËÍµ½Òª·¢ËÍµÄ»º³åÇø
                            len = DevDesc.size;

							//restart enumeration
                            Ready = FALSE;
                            
                            break;
                            
                        case USB_DESCR_TYP_CONFIG:                          //ÅäÖÃÃèÊö·û
                            pDescr = CfgDesc.descr;                               //°ÑÉè±¸ÃèÊö·ûËÍµ½Òª·¢ËÍµÄ»º³åÇø
                            len = CfgDesc.size;
                            break;
                        	
                        case USB_DESCR_TYP_STRING:
							pDescr = StringDescriptors[UsbSetupBuf->wValueL].descr;
							len = StringDescriptors[UsbSetupBuf->wValueL].size;
                        	break;
                        	
                        case USB_DESCR_TYP_REPORT:                          //±¨±íÃèÊö·û
                            if(UsbSetupBuf->wIndexL == 0)                   //½Ó¿Ú0±¨±íÃèÊö·û
                            {
                                pDescr = KeyRepDesc.descr;                        //Êý¾Ý×¼±¸ÉÏ´«
                                len = KeyRepDesc.size;
                            }
                            else if(UsbSetupBuf->wIndexL == 1)              //½Ó¿Ú1±¨±íÃèÊö·û
                            {
                                pDescr = MouseRepDesc.descr;                      //Êý¾Ý×¼±¸ÉÏ´«
                                len = MouseRepDesc.size;
                                Ready = TRUE;                                  //Èç¹ûÓÐ¸ü¶à½Ó¿Ú£¬¸Ã±ê×¼Î»Ó¦¸ÃÔÚ×îºóÒ»¸ö½Ó¿ÚÅäÖÃÍê³ÉºóÓÐÐ§
                            }
                            else
                            {
                                len = 0xFF;                               //±¾³ÌÐòÖ»ÓÐ2¸ö½Ó¿Ú£¬Õâ¾ä»°Õý³£²»¿ÉÄÜÖ´ÐÐ
                            }
                            break;

						case USB_DESCR_TYP_QUALIF:
							pDescr = DeviceQualifierCfg.descr;
                            len = DeviceQualifierCfg.size;
							
							break;
							
                        default:
                            len = 0xFF;                                    //²»Ö§³ÖµÄÃüÁî»òÕß³ö´í
                            break;
                        }

                        if (len != 0xFF)
                        {
							if (SetupLen > len)
							{
								SetupLen = len;    //ÏÞÖÆ×Ü³¤¶È
							}
							
							len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;				   //±¾´Î´«Êä³¤¶È
							memcpy(Ep0Buffer, pDescr, len); 					 //¼ÓÔØÉÏ´«Êý¾Ý
							SetupLen -= len;
							pDescr += len;
                        }
                        
                        break;
                        
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                     //ÔÝ´æUSBÉè±¸µØÖ·
                        break;
                        
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if (SetupLen >= 1)
                        {
                            len = 1;
                        }
                        break;
                        
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                        
                    case 0x0A:
                        break;
                        
                    case USB_CLEAR_FEATURE:                                            //Clear Feature
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)// ¶Ëµã
                        {
                            switch( UsbSetupBuf->wIndexL )
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF;                                            // ²»Ö§³ÖµÄ¶Ëµã
                                break;
                            }
                        }
                        
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE )// è®¾å¤‡
                        {
                            len = 0xFF;
                            
							break;
                        }													
                        else
                        {
                            len = 0xFF;                                                // ä¸æ˜¯ç«¯ç‚¹ä¸æ”¯æŒ
                        }
                        
                        break;
                        
                    case USB_SET_FEATURE:                                              /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )             /* ÉèÖÃÉè±¸ */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( (CfgDesc.descr[ 7 ] & 0x20) && Ready )
                                {
                                    /* ÉèÖÃ»½ÐÑÊ¹ÄÜ±êÖ¾ */
                                    PCSleeped = TRUE;
                                }
                                else
                                {
                                    len = 0xFF;                                        /* ²Ù×÷Ê§°Ü */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                            /* ²Ù×÷Ê§°Ü */
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )        /* ÉèÖÃ¶Ëµã */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( UINT16 )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ÉèÖÃ¶Ëµã2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* ÉèÖÃ¶Ëµã2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ÉèÖÃ¶Ëµã1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF;                                //²Ù×÷Ê§°Ü
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                   //²Ù×÷Ê§°Ü
                            }
                        }
                        else
                        {
                            len = 0xFF;                                     //²Ù×÷Ê§°Ü
                        }
                        break;
                        
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if ( SetupLen >= 2 )
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                        
                    default:
                        len = 0xFF;                                          //²Ù×÷Ê§°Ü
                        break;
                    }
                }
			}
			else 
			{
				len = 0xff;                                                 //°ü³¤¶È´íÎó
			}

			if (len == 0xff)
			{
				SetupReq = 0xFF;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
			}
			else if (len <= DEFAULT_ENDP0_SIZE)												//ÉÏ´«Êý¾Ý»òÕß×´Ì¬½×¶Î·µ»Ø0³¤¶È°ü
			{
				UEP0_T_LEN = len;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ä¬ÈÏÊý¾Ý°üÊÇDATA1£¬·µ»ØÓ¦´ðACK
			}
			else
			{
				UEP0_T_LEN = 0;  //ËäÈ»ÉÐÎ´µ½×´Ì¬½×¶Î£¬µ«ÊÇÌáÇ°Ô¤ÖÃÉÏ´«0³¤¶ÈÊý¾Ý°üÒÔ·ÀÖ÷»úÌáÇ°½øÈë×´Ì¬½×¶Î
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ä¬ÈÏÊý¾Ý°üÊÇDATA1,·µ»ØÓ¦´ðACK
			}
            
            break;
            
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
			switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
            case HID_GET_REPORT:
                len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                          //±¾´Î´«Êä³¤¶È
                memcpy(Ep0Buffer, pDescr, len);                              //¼ÓÔØÉÏ´«Êý¾Ý
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //Í¬²½±êÖ¾Î»·­×ª
				if (len < DEFAULT_ENDP0_SIZE)
				{
					SetupReq = 0xFF;
				}
                break;
                
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
                
            default:
                UEP0_T_LEN = 0;                                              //×´Ì¬½×¶ÎÍê³ÉÖÐ¶Ï»òÕßÊÇÇ¿ÖÆÉÏ´«0³¤¶ÈÊý¾Ý°ü½áÊø¿ØÖÆ´«Êä
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            
            break;
        case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
            len = USB_RX_LEN;
            if (SetupReq == HID_SET_REPORT)  //set report
            {
            	keyboardLed = Ep0Buffer[0];
            }
            UEP0_CTRL ^= bUEP_R_TOG;                                     //Í¬²½±êÖ¾Î»·­×ª						
            break;
            
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 //Ð´0Çå¿ÕÖÐ¶Ï
    }
    
    if (UIF_BUS_RST)                                                       //Éè±¸Ä£Ê½USB×ÜÏß¸´Î»ÖÐ¶Ï
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //ÇåÖÐ¶Ï±êÖ¾
    }
    
    if (UIF_SUSPEND)                                                     //USB×ÜÏß¹ÒÆð/»½ÐÑÍê³É
    {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND )                                 //¹ÒÆð
        {
            //suspend
            //TRACE( "zz" );                                              //Ë¯Ãß×´Ì¬
//          while ( XBUS_AUX & bUART0_TX );                              //µÈ´ý·¢ËÍÍê³É
//          SAFE_MOD = 0x55;
//          SAFE_MOD = 0xAA;
//          WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                      //USB»òÕßRXD0ÓÐÐÅºÅÊ±¿É±»»½ÐÑ
//          PCON |= PD;                                                  //Ë¯Ãß
//          SAFE_MOD = 0x55;
//          SAFE_MOD = 0xAA;
//          WAKE_CTRL = 0x00;

            if (!GET_GPIO_BIT(PIN_USB_POWER))
            {
			    Ready = FALSE;
			}
        }
        else
        {
            //resume
            PCSleeped = FALSE;
        }
    }
    else 
    {                                                               //ÒâÍâµÄÖÐ¶Ï,²»¿ÉÄÜ·¢ÉúµÄÇé¿ö
        USB_INT_FG = 0xFF;                                               //ÇåÖÐ¶Ï±êÖ¾
//      TRACE("UnknownInt  N");
    }
}

