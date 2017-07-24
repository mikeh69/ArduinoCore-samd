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

#define lesser_of(a, b) (a < b ? a : b)

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

static char InquiryResponse0[] = {  0,    // peripheral device-type and qualifier
                                    0x80, // EVPD Page Code: Unit Serial-Number Page
                                    0x04, // conforms to SCSI Primary Commands v2
                                    0x02, // SPC-2/SPC-3 response format
                                    31,   // additional length 31
                                    0,    // flags5
                                    0,    // flags6
                                    0,    // flags7
                                    'A', 'S', 'H', ' ', 'W', 'l', 's', 's',  // vendor ident.
                                    'L', 'o', 'g', 'g', 'e', 'r', ' ', 'P', 'l', 'a', 't', 'f', 'o', 'r', 'm', ' ',
                                    'v', '1', '.', '0'  // product revision level
                                    };
                                    
static char InquiryResponse1[] = {  0,    // peripheral device-type and qualifier
                                    0x80, // EVPD PAge Code = Unit Serial-Number Page
                                    0, 16,   // page length 16
                                    '0', '1', '2', '3', '4', '5', '6', '7',  // serial-number
                                    '0', '1', '2', '3', '4', '5', '6', '7',  // more serial-number
};

static char CapacityResponse[] = { 0x00, 0xE8, 0x8D, 0x7F,  // flash-card capacity in sectors
                                    0x00, 0x00, 0x02, 0x00  // sector-size 512 bytes
                                    };

static char SenseResponse[] = { 0x70,  // sense-code Current Error
                                0,     // (obsolete)
                                0x06,  // sense-key Unit Attention
                                0, 0, 0, 0,  // sense information
                                10,  // additional sense length
                                0, 0, 0, 0,  // command-specific information
                                0x28, 0x00,  // additional sense code + qualifier
                                0,  // Field-Replaceable Unit code
                                0, 0, 0  // sense key specific info
                                };

USB_MSD_t sam_ba_msd;
__attribute__((__aligned__(4))) uint8_t msd_bulk_buffer[8192]; // shared buffer for bulk in and out

const char *DeviceDescriptor = devDescriptor;
const char *ConfigDescriptor[] = {cfgDescriptor};
const char* ManufacturerString =  "ASH Wireless Ltd.";
const char* ProductString = "SAMD21 mass-storage device";
const char* SerialNumberString = "0123456789";  // TODO: derive the serial-number from the device unique-ID

// Utility functions to read little-endian and big-endian integers from a buffer:
static uint16_t INT16BE(char *pData)
{
    return ( ((uint16_t)pData[0]) << 8) | pData[1]; 
}

static uint16_t INT16LE(char *pData)
{
    return *(uint16_t*)pData;
}

static uint32_t INT32BE(char *pData)
{
    return ( ((uint32_t)pData[0]) << 24) | ( ((uint32_t)pData[1]) << 16) | ( ((uint32_t)pData[2]) << 8) | pData[3];
}

static uint32_t INT32LE(char *pData)
{
    return *(uint32_t*)pData;
}

/*----------------------------------------------------------------------------
* \brief This function is a callback invoked when a class-specific request is recognised
*/
bool sam_ba_usb_mass_stor_handle_req(P_USB_t pUsb, uint8_t *SetupPacket, void *pParams)
{
    static volatile uint8_t bmRequestType, bRequest, dir;
    static volatile uint16_t wValue, wIndex, wLength, wStatus;
    bool retval = true;

    /* Clear the Received Setup flag */
    pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;

    /* Read the USB request parameters */
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
    P_USB_MSD_t pMSD = &sam_ba_msd;
    pMSD->pUsb = USB;
    pMSD->currentConfiguration = 0;
    pMSD->IsConfigured = USB_IsConfigured;
    pMSD->Write        = USB_Write;
    pMSD->Read         = USB_Read;
    USB_Open(USB);

    return &sam_ba_msd;
}

bool sam_ba_usb_mass_stor_handle_scsi(uint8_t *pData, uint8_t ScsiLength, uint32_t tag)
{
    uint32_t SCSI_Response[4];
    SCSI_Response[0] = 0x53425355;  // 'USBS' little-endian
    SCSI_Response[1] = tag;
    SCSI_Response[2] = 0;  // "DataResidue"
    SCSI_Response[3] = 0;
    bool isOK = true;
    
    switch(pData[0]) 
    {
    case UFI_CMD_INQUIRY:
        if (pData[1] == 0)  // 
        {
            USB_Write(USB, InquiryResponse0, sizeof(InquiryResponse0), USB_EP_IN);
        }
        else if (pData[1] == 1)  // Unit Serial-Number Page
        {
            USB_Write(USB, InquiryResponse1, sizeof(InquiryResponse0), USB_EP_IN);
        }
        else
        {
            isOK = false;
        }
        break;

    case UFI_CMD_READ_FORMAT_CAPACITIES:
        isOK = false;  // Kingston SE9 does not support this, why should we?
        break;
        
    case UFI_CMD_REQUEST_SENSE:
        USB_Write(USB, SenseResponse, sizeof(SenseResponse), USB_EP_IN);
        break;
    
    case UFI_CMD_READ_CAPACITY:
        USB_Write(USB, CapacityResponse, sizeof(CapacityResponse), USB_EP_IN);
        break;
        
    case UFI_CMD_TEST_UNIT_READY:
        break;  // never not ready!
        
    case UFI_CMD_READ10:
    {
        uint32_t LBA = INT32BE(pData + 2);
        uint16_t TfrLengthSectors = INT16BE(pData + 7);
        uint32_t length = lesser_of(TfrLengthSectors * 512, sizeof(msd_bulk_buffer));
        // TODO: read sectors from SD card into msd_bulk_buffer
        USB_Write_Raw(USB, (uint32_t)msd_bulk_buffer, length, USB_EP_IN);
    }        
        break;
        
    case UFI_CMD_MODE_SENSE6:
        
        break;

    case UFI_CMD_MEDIA_REMOVAL:
        
        break;

    case UFI_CMD_FORMAT_UNIT:
    case UFI_CMD_MODE_SELECT6:
    case UFI_CMD_START_STOP_UNIT:
    case UFI_CMD_WRITE10:
    case UFI_CMD_SEEK10:
    case UFI_CMD_VERIFY10:
    case UFI_CMD_MODE_SELECT10:
    case UFI_CMD_MODE_SENSE10:
    case UFI_CMD_READ12:
    case UFI_CMD_WRITE12:
        return true;
    
    default:
        isOK = false;
    }    

    if ( !isOK && SCSI_Response[3] == 0)
    {
        SCSI_Response[2] = 0xFC;  // TODO: check the meaning of this
        SCSI_Response[3] = 0x01;  // "Command Failed"
    }

    USB_Write(USB, SCSI_Response, 13, USB_EP_IN);
    return isOK;
}