
#include <string.h>

#include "Type.h"
#include "Ch552.h"
#include "Mcu.h"

#include "System.h"
#include "UsbDef.h"
#include "Usb.h"
#include "UsbDescriptor.h"

#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE

static UINT8X  Ep0Buffer[8>(THIS_ENDP0_SIZE+2)?8:(THIS_ENDP0_SIZE+2)] _at_ 0x0000;    //端点0 OUT&IN缓冲区，必须是偶地址
static UINT8X  Ep1Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x000a;  //端点1 IN缓冲区,必须是偶地址
static UINT8X  Ep2Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x0050;  //端点2 IN缓冲区,必须是偶地址
static UINT8   SetupReq,SetupLen,UsbConfig;

static BOOL    PCSleeped = FALSE;

static BOOL    Ready = FALSE;

static PUINT8  pDescr;    

//keyboard led status
static UINT8 volatile keyboardLed = 0x00;

#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)

/*******************************************************************************
* Function Name  : CH554SoftReset()
* Description    : CH554软复位
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
* Description    : CH554设备模式唤醒主机，发送K信号
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
* Description    : USB设备模式配置,设备模式启动，收发端点配置，中断开启
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
	InitSerialString();
	
	IE_USB = 0;
	USB_CTRL = 0x00;                                                           // 先设定USB设备模式

	UEP0_DMA = (UINT16)Ep0Buffer;                                                      //端点0数据传输地址
	UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                                //端点0单64字节收发缓冲区
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //OUT事务返回ACK，IN事务返回NAK

	UEP1_DMA = (UINT16)Ep1Buffer;                                                      //端点1数据传输地址
	UEP4_1_MOD = UEP4_1_MOD & ~bUEP1_BUF_MOD | bUEP1_TX_EN;                    //端点1发送使能 64字节缓冲区
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //端点1自动翻转同步标志位，IN事务返回NAK	

	UEP2_DMA = (UINT16)Ep2Buffer;                                                      //端点2数据传输地址
	UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN;                    //端点2发送使能 64字节缓冲区
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //端点2自动翻转同步标志位，IN事务返回NAK
		
	USB_DEV_AD = 0x00;
	UDEV_CTRL = bUD_PD_DIS;                                                    // 禁止DP/DM下拉电阻
	USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
	UDEV_CTRL |= bUD_PORT_EN;                                                  // 允许USB端口
	USB_INT_FG = 0xFF;                                                         // 清中断标志
	USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;

	UEP1_T_LEN = 0;                                                       //预使用发送长度一定要清空
    UEP2_T_LEN = 0;                                                       //预使用发送长度一定要清空
    
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
* Description    : USB设备模式端点1的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp1IntIn(UINT8 *dat, UINT8 size)
{
    memcpy(Ep1Buffer, dat, size);
    UEP1_T_LEN = size;
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //有数据时上传数据并应答ACK
}
/*******************************************************************************
* Function Name  : Enp2IntIn()
* Description    : USB设备模式端点2的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2IntIn(UINT8 *dat, UINT8 size)
{
    memcpy(Ep2Buffer, dat, size);
    UEP2_T_LEN = size;
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //有数据时上传数据并应答ACK
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB中断处理函数
*******************************************************************************/
void UsbIsr(void) interrupt INT_NO_USB using 1                      //USB中断服务程序,使用寄存器组1
{
    UINT8 len = 0;
    if(UIF_TRANSFER)                                                            //USB传输完成标志
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# 中断端点上传
            UEP2_T_LEN = 0;                                                     //预使用发送长度一定要清空
//          UEP1_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            break;
            
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# 中断端点上传
            UEP1_T_LEN = 0;                                                     //预使用发送长度一定要清空
//            UEP2_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            break;
            
        case UIS_TOKEN_SETUP | 0:                                                //SETUP事务
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // 限制总长度
                }
                len = 0;                                                        // 默认为成功并且上传0长度
                SetupReq = UsbSetupBuf->bRequest;								
                if (( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD)/* HID类命令 */
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
						len = 0xFF;  								 					            /*命令不支持*/					
						break;
					}	
                }
                else
                {//标准请求
                    switch(SetupReq)                                        //请求码
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case USB_DESCR_TYP_DEVICE:                          //设备描述符
                            pDescr = DevDesc.descr;                               //把设备描述符送到要发送的缓冲区
                            len = DevDesc.size;
                            break;
                            
                        case USB_DESCR_TYP_CONFIG:                          //配置描述符
                            pDescr = CfgDesc.descr;                               //把设备描述符送到要发送的缓冲区
                            len = CfgDesc.size;
                            break;
                        	
                        case USB_DESCR_TYP_STRING:
							pDescr = StringDescriptors[UsbSetupBuf->wValueL].descr;
							len = StringDescriptors[UsbSetupBuf->wValueL].size;
                        	break;
                        	
                        case USB_DESCR_TYP_REPORT:                          //报表描述符
                            if(UsbSetupBuf->wIndexL == 0)                   //接口0报表描述符
                            {
                                pDescr = KeyRepDesc.descr;                        //数据准备上传
                                len = KeyRepDesc.size;
                            }
                            else if(UsbSetupBuf->wIndexL == 1)              //接口1报表描述符
                            {
                                pDescr = MouseRepDesc.descr;                      //数据准备上传
                                len = MouseRepDesc.size;
                                Ready = TRUE;                                  //如果有更多接口，该标准位应该在最后一个接口配置完成后有效
                            }
                            else
                            {
                                len = 0xFF;                               //本程序只有2个接口，这句话正常不可能执行
                            }
                            break;

                        default:
                            len = 0xFF;                                    //不支持的命令或者出错
                            break;
                        }

                        if (len != 0xFF)
                        {
							if (SetupLen > len)
							{
								SetupLen = len;    //限制总长度
							}
							
							len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;				   //本次传输长度
							memcpy(Ep0Buffer, pDescr, len); 					 //加载上传数据
							SetupLen -= len;
							pDescr += len;
                        }
                        
                        break;
                        
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                     //暂存USB设备地址
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
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)// 端点
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
                                len = 0xFF;                                            // 不支持的端点
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
                            len = 0xFF;                                                // 不是端点不支持
                        }
                        break;
                        
                    case USB_SET_FEATURE:                                              /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )             /* 设置设备 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc.descr[ 7 ] & 0x20 )
                                {
                                    /* 设置唤醒使能标志 */
                                    PCSleeped = TRUE;
                                }
                                else
                                {
                                    len = 0xFF;                                        /* 操作失败 */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                            /* 操作失败 */
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )        /* 设置端点 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( UINT16 )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF;                                //操作失败
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                   //操作失败
                            }
                        }
                        else
                        {
                            len = 0xFF;                                     //操作失败
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
                        len = 0xFF;                                          //操作失败
                        break;
                    }
                }
			}
			else 
			{
				len = 0xff;                                                 //包长度错误
			}

			if (len == 0xff)
			{
				SetupReq = 0xFF;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
			}
			else if (len <= DEFAULT_ENDP0_SIZE)												//上传数据或者状态阶段返回0长度包
			{
				UEP0_T_LEN = len;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
			}
			else
			{
				UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
			}
            
            break;
            
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
			switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
            case HID_GET_REPORT:
                len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                          //本次传输长度
                memcpy(Ep0Buffer, pDescr, len);                              //加载上传数据
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //同步标志位翻转
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
                UEP0_T_LEN = 0;                                              //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
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
            UEP0_CTRL ^= bUEP_R_TOG;                                     //同步标志位翻转						
            break;
            
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 //写0清空中断
    }
    
    if (UIF_BUS_RST)                                                       //设备模式USB总线复位中断
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //清中断标志
    }
    
    if (UIF_SUSPEND)                                                     //USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND )                                 //挂起
        {
            //suspend
            //TRACE( "zz" );                                              //睡眠状态
//          while ( XBUS_AUX & bUART0_TX );                              //等待发送完成
//          SAFE_MOD = 0x55;
//          SAFE_MOD = 0xAA;
//          WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                      //USB或者RXD0有信号时可被唤醒
//          PCON |= PD;                                                  //睡眠
//          SAFE_MOD = 0x55;
//          SAFE_MOD = 0xAA;
//          WAKE_CTRL = 0x00;

            if (!PCSleeped)
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
    {                                                               //意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF;                                               //清中断标志
//      TRACE("UnknownInt  N");
    }
}

