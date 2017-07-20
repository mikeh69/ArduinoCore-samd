/*
Copyright (c) 2017 ASH Wireless Ltd.

Implements the Mass-Storage Class (USB flash-drive) functionality
*/

#include <stdint.h>
#include <string.h>
#include "board_definitions.h"
#include "sam_ba_usb.h"
#include "board_driver_usb.h"
#include "sam_ba_mass_stor.h"

/* This data array will be copied into SRAM as its length is inferior to 64 bytes,
* and so can stay in flash.
*/
static __attribute__((__aligned__(4)))
const char devDescriptor[] =
{
    /* Device descriptor */
    0x12,   // bLength
    0x01,   // bDescriptorType
    0x00,   // bcdUSB L
    0x02,   // bcdUSB H
    0x00,   // bDeviceClass:    mass-storage class code
    0x00,   // bDeviceSubclass
    0x00,   // bDeviceProtocol
    0x40,   // bMaxPacketSize0
    USB_VID_LOW,   // idVendor L
    USB_VID_HIGH,   // idVendor H
    USB_PID_LOW,   // idProduct L
    USB_PID_HIGH,  // idProduct H
    0x00,   // bcdDevice L, here matching SAM-BA version
    0x02,   // bcdDevice H
    STRING_INDEX_MANUFACTURER,   // iManufacturer
    STRING_INDEX_PRODUCT,        // iProduct
    0x00,   // SerialNumber, should be based on product unique ID
    0x01    // bNumConfigs
};

/* This data array will be consumed directly by USB_Write() and must be in SRAM.
* We cannot send data from product internal flash.
*/
static __attribute__((__aligned__(4)))
char cfgDescriptor[] =
{
    /* ============== CONFIGURATION 1 =========== */
    /* Configuration 1 descriptor */
    0x09,   // CbLength
    0x02,   // CbDescriptorType
    0x43,   // CwTotalLength 2 EP + Control
    0x00,
    0x02,   // CbNumInterfaces
    0x01,   // CbConfigurationValue
    0x00,   // CiConfiguration
    0x80,   // CbmAttributes Bus powered without remote wakeup: 0x80, Self powered without remote wakeup: 0xc0
    0x32,   // CMaxPower, report using 100mA, enough for a bootloader

    /* Mass Storage Interface Descriptor */
    0x09, // bLength
    0x04, // bDescriptorType
    0x00, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x02, // bNumEndpoints
    0x08, // bInterfaceClass = Mass-Storage
    0x06, // bInterfaceSubclass = SCSI
    0x50, // bInterfaceProtocol = Bulk-Only
    0x00, // iInterface

    /* Bulk In Endpoint */
    7,     /* bLength */
    DESCRIPTOR_ENDPOINT,      /* bDescriptorType */
    0x02,                     /* bEndpointAddress */
    0x02,                     /* bmAttributes  = USB_ENDPOINT_TYPE_BULK*/
    0x40, 0x00,               /* wMaxPacketSize */
    0,                        /* bInterval */
    /* Bulk Out Endpoint */
    7,     /* bLength */
    DESCRIPTOR_ENDPOINT,      /* bDescriptorType */
    0x82,                     /* bEndpointAddress */
    0x02,                     /* bmAttributes  = USB_ENDPOINT_TYPE_BULK*/
    0x40, 0x00,               /* wMaxPacketSize */
    0,                        /* bInterval */
    /* Terminator */
    0                                  /* bLength */
};

#ifndef STRING_MANUFACTURER
#  define STRING_MANUFACTURER "ASH Wireless Ltd."
#endif

#ifndef STRING_PRODUCT
#  define STRING_PRODUCT "SAMD21 mass-storage device"
#endif

USB_MSD_t sam_ba_msd;

/*----------------------------------------------------------------------------
* \brief This function is a callback invoked when a SETUP packet is received
*/
void sam_ba_usb_mass_stor_enumerate(P_USB_MSD_t pMSD)
{
    P_USB_t pUsb = pMSD->pUsb;
    static volatile uint8_t bmRequestType, bRequest, dir;
    static volatile uint16_t wValue, wIndex, wLength, wStatus;

    /* Clear the Received Setup flag */
    pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;

    /* Read the USB request parameters */
    uint8_t *SetupPacket = udd_ep_out_cache_buffer[0];
    bmRequestType = SetupPacket[0];
    bRequest      = SetupPacket[1];
    uint8_t bDesc = SetupPacket[2];
    wValue = (SetupPacket[3] << 8) | bDesc;
    wIndex   = (SetupPacket[4] & 0xFF);
    wIndex  |= (SetupPacket[5] << 8);
    wLength  = (SetupPacket[6] & 0xFF);
    wLength |= (SetupPacket[7] << 8);

    /* Clear the Bank 0 ready flag on Control OUT */
    pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

    /* Handle supported standard device request Cf Table 9-3 in USB specification Rev 1.1 */
    switch ((bRequest << 8) | bmRequestType)
    {
        case USBREQ_GET_DESCRIPTOR:
            if ( bDesc == DESCRIPTOR_DEVICE)
            {
                /* Return Device Descriptor */
                USB_Write(pUsb, devDescriptor, minval(sizeof(devDescriptor), wLength), USB_EP_CTRL);
            }
            else if ( bDesc == DESCRIPTOR_CONFIGURATION)
            {
                /* Return Configuration Descriptor */
                USB_Write(pUsb, cfgDescriptor, minval(sizeof(cfgDescriptor), wLength), USB_EP_CTRL);
            }
            else if ( bDesc == DESCRIPTOR_STRING)
            {
                switch ( wValue & 0xff )
                {
                    case STRING_INDEX_LANGUAGES: 
                        {
                            uint16_t STRING_LANGUAGE[2] = { (DESCRIPTOR_STRING << 8) | 4, 0x0409 };

                            USB_Write(pUsb, (const char*)STRING_LANGUAGE, minval(sizeof(STRING_LANGUAGE), wLength), USB_EP_CTRL);
                        }
                        break;

                    case STRING_INDEX_MANUFACTURER:
                        USB_SendString(pUsb, STRING_MANUFACTURER, wLength );
                        break;

                    case STRING_INDEX_PRODUCT:
                        USB_SendString(pUsb, STRING_PRODUCT, wLength );
                        break;
                    default:
                        /* Stall the request */
                        USB_SendStall(pUsb, true);
                        break;
                }
            }
            else
            {
                /* Stall the request */
                USB_SendStall(pUsb, true);
            }
            break;

        case USBREQ_SET_ADDRESS:
            USB_SendZLP(pUsb);
            /* Set device address to the newly received address from host */
            USB_SetAddress(pUsb, wValue);
            break;

        case USBREQ_SET_CONFIGURATION:
            /* Store configuration */
            pMSD->currentConfiguration = (uint8_t)wValue;
            USB_SendZLP(pUsb);
            /* Configure the 3 needed endpoints */
            USB_Configure(pUsb);
            break;

        case USBREQ_GET_CONFIGURATION:
            /* Return current configuration value */
            USB_Write(pUsb, (char *) &(pMSD->currentConfiguration), sizeof(pMSD->currentConfiguration), USB_EP_CTRL);
            break;

        case USBREQ_GET_STATUS_ZERO:
            wStatus = 0;
            USB_Write(pUsb, (char *) &wStatus, sizeof(wStatus), USB_EP_CTRL);
            break;

        case USBREQ_GET_STATUS_INTERFACE:
            wStatus = 0;
            USB_Write(pUsb, (char *) &wStatus, sizeof(wStatus), USB_EP_CTRL);
            break;

        case USBREQ_GET_STATUS_ENDPOINT:
            wStatus = 0;
            dir = wIndex & 80;
            wIndex &= 0x0F;
            if (wIndex <= 3)
            {
                if (dir)
                {
                    wStatus = (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ1) ? 1 : 0;
                }
                else
                {
                    wStatus = (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ0) ? 1 : 0;
                }
                /* Return current status of endpoint */
                USB_Write(pUsb, (char *) &wStatus, sizeof(wStatus), USB_EP_CTRL);
            }
            else
            {
                /* Stall the request */
                USB_SendStall(pUsb, true);
            }
            break;

        case USBREQ_SET_FEATURE_ZERO:
            /* Stall the request */
            USB_SendStall(pUsb, true);
            break;

        case USBREQ_SET_FEATURE_INTERFACE:
            USB_SendZLP(pUsb);
            break;

        case USBREQ_SET_FEATURE_ENDPOINT:
            dir = wIndex & 0x80;
            wIndex &= 0x0F;
            if ((wValue == 0) && wIndex && (wIndex <= 3))
            {
                /* Set STALL request for the endpoint */
                if (dir)
                {
                    pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
                }
                else
                {
                    pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
                }
                USB_SendZLP(pUsb);
            }
            else
            {
                /* Stall the request */
                USB_SendStall(pUsb, true);
            }
            break;

        case USBREQ_SET_INTERFACE:
        case USBREQ_CLEAR_FEATURE_ZERO:
            /* Stall the request */
            USB_SendStall(pUsb, true);
            break;

        case USBREQ_CLEAR_FEATURE_INTERFACE:
            USB_SendZLP(pUsb);
            break;

        case USBREQ_CLEAR_FEATURE_ENDPOINT:
            dir = wIndex & 0x80;
            wIndex &= 0x0F;

            if ((wValue == 0) && wIndex && (wIndex <= 3))
            {
                if (dir)
                {
                    if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.bit.STALLRQ1)
                    {
                        // Remove stall request
                        pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ1;
                        if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.bit.STALL1)
                        {
                            pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL1;
                            // The Stall has occurred, then reset data toggle
                            pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLIN;
                        }
                    }
                }
                else
                {
                    if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.bit.STALLRQ0)
                    {
                        // Remove stall request
                        pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ0;
                        if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.bit.STALL0)
                        {
                            pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL0;
                            // The Stall has occurred, then reset data toggle
                            pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLOUT;
                        }
                    }
                }
                USB_SendZLP(pUsb);
            }
            else
            {
                USB_SendStall(pUsb, true);
            }
            break;

        // handle Mass-Storage class requests
        // Any to add?

        default:
            USB_SendStall(pUsb, true);
            break;
    }
}

/*----------------------------------------------------------------------------
* \brief
*/
P_USB_MSD_t usb_msd_init(void)
{
    /* Initialize USB */
    USB_Init();
    /* Get the default MSD structure settings */
    USB_Open(&sam_ba_msd, USB);

    return &sam_ba_msd;
}

/*----------------------------------------------------------------------------
* \brief Send a USB descriptor string.
*
* The input string is plain ASCII but is sent out as UTF-16 with the correct 2-byte prefix.
*/
uint32_t USB_SendString(P_USB_t pUsb, const char* ascii_string, uint8_t maxLength)
{
    uint8_t string_descriptor[255]; // Max USB-allowed string length

    string_descriptor[0] = (strlen(ascii_string) + 1) * 2;
    string_descriptor[1] = DESCRIPTOR_STRING;

    uint16_t* unicode_string=(uint16_t*)(string_descriptor + 2); // point on 3 bytes of descriptor
    int resulting_length;
    for ( resulting_length = 1 ; *ascii_string && (resulting_length < (maxLength/2)) ; resulting_length++ )
    {
        *unicode_string++ = (uint16_t)(*ascii_string++);
    }

    return USB_Write(pUsb, (const char*)string_descriptor, (resulting_length * 2), USB_EP_CTRL);
}
