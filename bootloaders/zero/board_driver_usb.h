/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Adapted by ASH Wireless - remove all class-specific items, and handle class-specific
  requests by passing in a pointer to a handler function.
*/

#ifndef _BOARD_DRIVER_USB_H_
#define _BOARD_DRIVER_USB_H_

#include <stdint.h>
#include <stdbool.h>
#include "sam_ba_usb.h"  // for P_USB_t definition

// Function-pointer type for class-specific request handler function
typedef bool(*ClassReqHandlerFn_t)(P_USB_t pUsb, uint8_t *SetupPacket, void *pParams);

extern UsbDeviceDescriptor usb_endpoint_table[MAX_EP];
extern uint8_t udd_ep_out_cache_buffer[2][64]; // 1 for control endpoint, 1 for bulk-out
extern uint8_t udd_ep_in_cache_buffer[2][64];  // 1 for control endpoint, 1 for bulk-in

void USB_Open(P_USB_t pUsb);

void USB_Init(void);

bool USB_HandleRequest(P_USB_t pUsb, ClassReqHandlerFn_t pClassReqHandler, void *pExtraParams);

uint32_t USB_Write(P_USB_t pUsb, const char *pData, uint32_t length, uint8_t ep_num);
uint32_t USB_Read(P_USB_t pUsb, char *pData, uint32_t length);
uint32_t USB_Read_blocking(USB_t *pUsb, char *pData, uint32_t length);

uint8_t USB_IsConfigured(P_USB_t pUsb);

void USB_SendStall(P_USB_t pUsb, bool direction_in);
void USB_SendZLP(P_USB_t pUsb);

void USB_SetAddress(P_USB_t pUsb, uint16_t wValue);
void USB_Configure(P_USB_t pUsb);
uint8_t USB_GetCurrentConfig(void);

#endif // _BOARD_DRIVER_USB_H_
