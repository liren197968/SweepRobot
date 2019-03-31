#ifndef __USB_H__
#define __USB_H__


#include "stm32f4xx_hal.h"
#include "usbh_core.h"
#include "usbh_def.h"
#include "usb_host.h"
#include "usbh_hid.h"
#include "usbh_hid_parser.h"


typedef enum {
  HID_DEMO_IDLE = 0,
  HID_DEMO_WAIT,
  HID_DEMO_START,
  HID_DEMO_MOUSE,
  HID_DEMO_KEYBOARD,
  HID_DEMO_REENUMERATE,
}HID_Demo_State;

typedef enum {
  HID_MOUSE_IDLE = 0,
  HID_MOUSE_WAIT,
  HID_MOUSE_START,
}HID_mouse_State;

typedef enum {
  HID_KEYBOARD_IDLE = 0,
  HID_KEYBOARD_WAIT,  
  HID_KEYBOARD_START,    
}HID_keyboard_State;

typedef struct _DemoStateMachine {
  __IO HID_Demo_State     state;
  __IO HID_mouse_State    mouse_state;
  __IO HID_keyboard_State keyboard_state;
  __IO uint8_t            select;
  __IO uint8_t            lock;
}HID_DEMO_StateMachine;


extern USBH_HandleTypeDef hUsbHostFS;
extern ApplicationTypeDef Appli_state;
extern HID_MOUSE_Info_TypeDef mouse_info;
extern uint8_t *DEMO_MOUSE_menu[];
extern HID_DEMO_StateMachine hid_demo;
extern uint8_t prev_select;


void HID_SelectItem(uint8_t **menu, uint8_t item);
void HID_MenuInit(void);
void HID_MenuProcess(void);
void HID_MouseMenuProcess(void);
void HID_Joysticky(void);
void HID_KeyboardMenuProcess(void);
void HID_MOUSE_ButtonReleased(uint8_t button_idx);
void HID_MOUSE_ButtonPressed(uint8_t button_idx);
void USR_MOUSE_ProcessData(HID_MOUSE_Info_TypeDef *data);
void USR_KEYBRD_ProcessData(uint8_t data);


#endif
