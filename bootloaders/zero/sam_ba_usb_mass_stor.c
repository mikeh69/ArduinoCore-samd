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
    STRING_INDEX_SERIAL_NUMBER,  // SerialNumber, should be based on product unique ID
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

USB_MSD_t sam_ba_msd;

const char *DeviceDescriptor = devDescriptor;
const char *ConfigDescriptor[] = {cfgDescriptor};
const char* ManufacturerString =  "ASH Wireless Ltd.";
const char* ProductString = "SAMD21 mass-storage device";
const char* SerialNumberString = "0123456789";  // TODO: derive the serial-number from the device unique-ID

/*----------------------------------------------------------------------------
* \brief This function is a callback invoked when a SETUP packet is received
*/
bool sam_ba_usb_mass_stor_handle_req(P_USB_t pUsb, uint8_t *pCurrentConfig)
{
    static volatile uint8_t bmRequestType, bRequest, dir;
    static volatile uint16_t wValue, wIndex, wLength, wStatus;
    bool retval = true;

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

    // handle Mass-Storage class requests
    switch ((bRequest << 8) | bmRequestType)
    {
        case MASS_STORAGE_RESET:
            // anything to be done?
            USB_SendZLP(pUsb);
            break;
            
        case GET_MAX_LUN:
            {
                uint8_t max_lun = 0;  // as observed using USBPCAP on a Kingston 8Gb flash stick
                USB_Write(pUsb, (char *) &max_lun, 1, USB_EP_CTRL);
            }            
            break;
            
        default:
            USB_SendStall(pUsb, true);
            retval = false;
            break;
    }
    return retval;
}

/*----------------------------------------------------------------------------
* \brief
*/
P_USB_MSD_t usb_msd_init(void)
{
    /* Initialize USB */
    USB_Init();
    /* Get the default MSD structure settings */
//    USB_Open(&sam_ba_msd, USB);
    P_USB_MSD_t pMSD = &sam_ba_msd;
    pMSD->pUsb = USB;
    pMSD->currentConfiguration = 0;
    pMSD->IsConfigured = USB_IsConfigured;
    pMSD->Write        = USB_Write;
    pMSD->Read         = USB_Read;
    USB_Open(USB);

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
