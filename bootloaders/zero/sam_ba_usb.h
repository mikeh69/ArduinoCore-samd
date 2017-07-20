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
  

  Amended to include generic USB definitions ONLY - definitions specfic to 
  a CDC (Communications Data Class, i.e. serial modem) device have been moved
  to another header file.
*/

#ifndef _SAM_BA_USB_H_
#define _SAM_BA_USB_H_

#include <sam.h>
#include <stdbool.h>

#define USB_EP_CTRL             (0u)
#define USB_EP_OUT              (2u)
#define USB_EP_OUT_SIZE         (0x40u)
#define USB_EP_IN               (1u)
#define USB_EP_IN_SIZE          (0x40u)
#define USB_EP_COMM             (3u)
#define MAX_EP                  (4u)

/* USB standard request code */
#define USBREQ_GET_STATUS_ZERO            (0x0080u)
#define USBREQ_GET_STATUS_INTERFACE       (0x0081u)
#define USBREQ_GET_STATUS_ENDPOINT        (0x0082u)

#define USBREQ_CLEAR_FEATURE_ZERO         (0x0100u)
#define USBREQ_CLEAR_FEATURE_INTERFACE    (0x0101u)
#define USBREQ_CLEAR_FEATURE_ENDPOINT     (0x0102u)

#define USBREQ_SET_FEATURE_ZERO           (0x0300u)
#define USBREQ_SET_FEATURE_INTERFACE      (0x0301u)
#define USBREQ_SET_FEATURE_ENDPOINT       (0x0302u)

#define USBREQ_SET_ADDRESS                (0x0500u)
#define USBREQ_GET_DESCRIPTOR             (0x0680u)
#define USBREQ_SET_DESCRIPTOR             (0x0700u)
#define USBREQ_GET_CONFIGURATION          (0x0880u)
#define USBREQ_SET_CONFIGURATION          (0x0900u)
#define USBREQ_GET_INTERFACE              (0x0A81u)
#define USBREQ_SET_INTERFACE              (0x0B01u)
#define USBREQ_SYNCH_FRAME                (0x0C82u)

#define DESCRIPTOR_DEVICE                          (1u)
#define DESCRIPTOR_CONFIGURATION                   (2u)
#define DESCRIPTOR_STRING                          (3u)
#define DESCRIPTOR_INTERFACE                       (4u)
#define DESCRIPTOR_ENDPOINT                        (5u)
#define DESCRIPTOR_DEVICE_QUALIFIER                (6u)
#define DESCRIPTOR_OTHER_SPEED_CONFIGURATION       (7u)
#define DESCRIPTOR_INTERFACE_POWER1                (8u)

#define FEATURE_ENDPOINT_HALT          (0u)
#define FEATURE_DEVICE_REMOTE_WAKEUP   (1u)
#define FEATURE_TEST_MODE              (2u)

#define STRING_INDEX_LANGUAGES         (0x00u)
#define STRING_INDEX_MANUFACTURER      (0x01u)
#define STRING_INDEX_PRODUCT           (0x02u)

#define minval(a, b) (((a) < (b)) ? (a) : (b))

typedef Usb USB_t, *P_USB_t;  // (define consistent names for the Atmel CMSIS type) 

uint32_t USB_SendString(P_USB_t pUsb, const char* ascii_string, uint8_t maxLength);


#endif // _SAM_BA_USB_H_
