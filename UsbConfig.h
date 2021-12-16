#ifndef _USB_CONFIG_H_
#define _USB_CONFIG_H_

#define VID     0x413C
#define PID     0x2107

#define DEFAULT_ENDP0_SIZE      8       /* default maximum packet size for endpoint 0 */

#define MAX_PACKET_SIZE         8       /* maximum packet size */

//endpoints
#define HID_ENPOINT             0x01
#define KEYBOARD_ENDPOINT       0x02
#define MOUSE_ENDPOINT          0x03

#endif
