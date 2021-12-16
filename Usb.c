
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

static UINT8X  Ep0Buffer[8>(THIS_ENDP0_SIZE+2)?8:(THIS_ENDP0_SIZE+2)] _at_ 0x0000;    //�˵�0 OUT&IN��������������ż��ַ
static UINT8X  Ep1Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x000a;  //�˵�1 IN������,������ż��ַ
static UINT8X  Ep2Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x0050;  //�˵�2 IN������,������ż��ַ
static UINT8   SetupReq,SetupLen,UsbConfig;

static BOOL    PCSleeped = FALSE;

static BOOL    Ready = FALSE;

static PUINT8  pDescr;    

//keyboard led status
static UINT8 volatile keyboardLed = 0x00;

#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)

/*******************************************************************************
* Function Name  : CH554SoftReset()
* Description    : CH554����λ
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
* Description    : CH554�豸ģʽ��������������K�ź�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH554USBDevWakeup()
{
#if USB_DEVICE_TYPE == LOW_SPEED_DEVICE
	UDEV_CTRL &= ~bUD_LOW_SPEED;
	mDelaymS(2);
	UDEV_CTRL |= bUD_LOW_SPEED;

#elif USB_DEVICE_TYPE == FULL_SPEED_DEVICE
	UDEV_CTRL |= bUD_LOW_SPEED;
	mDelaymS(2);
	UDEV_CTRL &= ~bUD_LOW_SPEED;
#else
	#error "usb device type error"
#endif
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
* Description    : USB�豸ģʽ����,�豸ģʽ�������շ��˵����ã��жϿ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
	InitSerialString();
	
	IE_USB = 0;
	USB_CTRL = 0x00;                                                           // ���趨USB�豸ģʽ
	UDEV_CTRL = bUD_PD_DIS;

#if USB_DEVICE_TYPE == LOW_SPEED_DEVICE
	USB_CTRL |= bUC_LOW_SPEED;
    UDEV_CTRL |= bUD_LOW_SPEED; 
#elif USB_DEVICE_TYPE == FULL_SPEED_DEVICE
	USB_CTRL &= ~bUC_LOW_SPEED;
    UDEV_CTRL &= ~bUD_LOW_SPEED; 
#else
	#error "usb device type error"
#endif

	UEP0_DMA = (UINT16)Ep0Buffer;                                                      //�˵�0���ݴ����ַ
	UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                                //�˵�0��64�ֽ��շ�������
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //OUT���񷵻�ACK��IN���񷵻�NAK

	UEP1_DMA = (UINT16)Ep1Buffer;                                                      //�˵�1���ݴ����ַ
	UEP4_1_MOD = UEP4_1_MOD & ~bUEP1_BUF_MOD | bUEP1_TX_EN;                    //�˵�1����ʹ�� 64�ֽڻ�����
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //�˵�1�Զ���תͬ����־λ��IN���񷵻�NAK	

	UEP2_DMA = (UINT16)Ep2Buffer;                                                      //�˵�2���ݴ����ַ
	UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN;                    //�˵�2����ʹ�� 64�ֽڻ�����
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //�˵�2�Զ���תͬ����־λ��IN���񷵻�NAK
		
	USB_DEV_AD = 0x00;                                                   // ��ֹDP/DM��������
	USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // ����USB�豸��DMA�����ж��ڼ��жϱ�־δ���ǰ�Զ�����NAK
	UDEV_CTRL |= bUD_PORT_EN;                                                  // ����USB�˿�
	USB_INT_FG = 0xFF;                                                         // ���жϱ�־
	USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;

	UEP1_T_LEN = 0;                                                       //Ԥʹ�÷��ͳ���һ��Ҫ���
    UEP2_T_LEN = 0;                                                       //Ԥʹ�÷��ͳ���һ��Ҫ���
    
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

void SetPCReady(BOOL ready)
{
	HAL_CRITICAL_STATEMENT(Ready = ready);
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
* Description    : USB�豸ģʽ�˵�1���ж��ϴ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp1IntIn(UINT8 *dat, UINT8 size)
{
    memcpy(Ep1Buffer, dat, size);
    UEP1_T_LEN = size;
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //������ʱ�ϴ����ݲ�Ӧ��ACK
}
/*******************************************************************************
* Function Name  : Enp2IntIn()
* Description    : USB�豸ģʽ�˵�2���ж��ϴ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2IntIn(UINT8 *dat, UINT8 size)
{
    memcpy(Ep2Buffer, dat, size);
    UEP2_T_LEN = size;
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //������ʱ�ϴ����ݲ�Ӧ��ACK
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB�жϴ�������
*******************************************************************************/
void UsbIsr(void) interrupt INT_NO_USB using 1                      //USB�жϷ������,ʹ�üĴ�����1
{
    UINT8 len = 0;
    if(UIF_TRANSFER)                                                            //USB������ɱ�־
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# �ж϶˵��ϴ�
            UEP2_T_LEN = 0;                                                     //Ԥʹ�÷��ͳ���һ��Ҫ���
//          UEP1_CTRL ^= bUEP_T_TOG;                                          //����������Զ���ת����Ҫ�ֶ���ת
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Ĭ��Ӧ��NAK
            break;
            
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# �ж϶˵��ϴ�
            UEP1_T_LEN = 0;                                                     //Ԥʹ�÷��ͳ���һ��Ҫ���
//            UEP2_CTRL ^= bUEP_T_TOG;                                          //����������Զ���ת����Ҫ�ֶ���ת
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Ĭ��Ӧ��NAK
            break;
            
        case UIS_TOKEN_SETUP | 0:                                                //SETUP����
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // �����ܳ���
                }
                len = 0;                                                        // Ĭ��Ϊ�ɹ������ϴ�0����
                SetupReq = UsbSetupBuf->bRequest;								
                if (( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD)/* HID������ */
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
						len = 0xFF;  								 					            /*���֧��*/					
						break;
					}	
                }
                else
                {//��׼����
                    switch(SetupReq)                                        //������
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case USB_DESCR_TYP_DEVICE:                          //�豸������
                            pDescr = DevDesc.descr;                               //���豸�������͵�Ҫ���͵Ļ�����
                            len = DevDesc.size;

							//restart enumeration
                            Ready = FALSE;
                            
                            break;
                            
                        case USB_DESCR_TYP_CONFIG:                          //����������
                            pDescr = CfgDesc.descr;                               //���豸�������͵�Ҫ���͵Ļ�����
                            len = CfgDesc.size;
                            break;
                        	
                        case USB_DESCR_TYP_STRING:
							pDescr = StringDescriptors[UsbSetupBuf->wValueL].descr;
							len = StringDescriptors[UsbSetupBuf->wValueL].size;
                        	break;
                        	
                        case USB_DESCR_TYP_REPORT:                          //����������
                            if(UsbSetupBuf->wIndexL == 0)                   //�ӿ�0����������
                            {
                                pDescr = KeyRepDesc.descr;                        //����׼���ϴ�
                                len = KeyRepDesc.size;
                            }
                            else if(UsbSetupBuf->wIndexL == 1)              //�ӿ�1����������
                            {
                                pDescr = MouseRepDesc.descr;                      //����׼���ϴ�
                                len = MouseRepDesc.size;
                                Ready = TRUE;                                  //����и���ӿڣ��ñ�׼λӦ�������һ���ӿ�������ɺ���Ч
                            }
                            else
                            {
                                len = 0xFF;                               //������ֻ��2���ӿڣ���仰����������ִ��
                            }
                            break;

						case USB_DESCR_TYP_QUALIF:
							pDescr = DeviceQualifierCfg.descr;
                            len = DeviceQualifierCfg.size;
							
							break;
							
                        default:
                            len = 0xFF;                                    //��֧�ֵ�������߳���
                            break;
                        }

                        if (len != 0xFF)
                        {
							if (SetupLen > len)
							{
								SetupLen = len;    //�����ܳ���
							}
							
							len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;				   //���δ��䳤��
							memcpy(Ep0Buffer, pDescr, len); 					 //�����ϴ�����
							SetupLen -= len;
							pDescr += len;
                        }
                        
                        break;
                        
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                     //�ݴ�USB�豸��ַ
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
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)// �˵�
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
                                len = 0xFF;                                            // ��֧�ֵĶ˵�
                                break;
                            }
                        }
                        
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE )// 设备
                        {
                            len = 0xFF;
                            
							break;
                        }													
                        else
                        {
                            len = 0xFF;                                                // 不是端点不支�?
                        }
                        
                        break;
                        
                    case USB_SET_FEATURE:                                              /* Set Feature */
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)             /* �����豸 */
                        {
                            if (((( UINT16 )UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
                            {
                            	if ((CfgDesc.descr[7] & 0x20) && Ready)
                            	{
									PCSleeped = TRUE;
                            	}
                                else
                                {
                                    len = 0xFF;                                        /* ����ʧ�� */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                            /* ����ʧ�� */
                            }
                        }
                        else if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)        /* ���ö˵� */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
                            {
                                switch (((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ���ö˵�2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* ���ö˵�2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ���ö˵�1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF;                                //����ʧ��
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                   //����ʧ��
                            }
                        }
                        else
                        {
                            len = 0xFF;                                     //����ʧ��
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
                        len = 0xFF;                                          //����ʧ��
                        break;
                    }
                }
			}
			else 
			{
				len = 0xff;                                                 //�����ȴ���
			}

			if (len == 0xff)
			{
				SetupReq = 0xFF;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
			}
			else if (len <= DEFAULT_ENDP0_SIZE)												//�ϴ����ݻ���״̬�׶η���0���Ȱ�
			{
				UEP0_T_LEN = len;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ĭ�����ݰ���DATA1������Ӧ��ACK
			}
			else
			{
				UEP0_T_LEN = 0;  //��Ȼ��δ��״̬�׶Σ�������ǰԤ���ϴ�0�������ݰ��Է�������ǰ����״̬�׶�
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ĭ�����ݰ���DATA1,����Ӧ��ACK
			}
            
            break;
            
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
			switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
            case HID_GET_REPORT:
                len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                          //���δ��䳤��
                memcpy(Ep0Buffer, pDescr, len);                              //�����ϴ�����
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //ͬ����־λ��ת
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
                UEP0_T_LEN = 0;                                              //״̬�׶�����жϻ�����ǿ���ϴ�0�������ݰ��������ƴ���
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
            UEP0_CTRL ^= bUEP_R_TOG;                                     //ͬ����־λ��ת						
            break;
            
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 //д0����ж�
    }
    
    if (UIF_BUS_RST)                                                       //�豸ģʽUSB���߸�λ�ж�
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //���жϱ�־
    }
    
    if (UIF_SUSPEND)                                                     //USB���߹���/�������
    {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND )                                 //����
        {
            //suspend
            //TRACE( "zz" );                                              //˯��״̬
//          while ( XBUS_AUX & bUART0_TX );                              //�ȴ��������
//          SAFE_MOD = 0x55;
//          SAFE_MOD = 0xAA;
//          WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                      //USB����RXD0���ź�ʱ�ɱ�����
//          PCON |= PD;                                                  //˯��
//          SAFE_MOD = 0x55;
//          SAFE_MOD = 0xAA;
//          WAKE_CTRL = 0x00;

            /*if (!GET_GPIO_BIT(PIN_USB_POWER))
            {
			    Ready = FALSE;
			}*/

            if (Ready)
            {
                PCSleeped = TRUE;
            }
        }
        else
        {
            //resume
            PCSleeped = FALSE;
        }
    }
    else 
    {                                                               //������ж�,�����ܷ��������
        USB_INT_FG = 0xFF;                                               //���жϱ�־
//      TRACE("UnknownInt  N");
    }
}

