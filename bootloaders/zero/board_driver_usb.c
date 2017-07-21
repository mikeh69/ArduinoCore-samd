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

This module is essentially a Hardware Abstraction Layer - it encapsulates 
the SAMD21-specific code and presents a "generic USB" API.
*/

#include <string.h>
#include "board_driver_usb.h"

#define NVM_USB_PAD_TRANSN_POS            (45)
#define NVM_USB_PAD_TRANSN_SIZE           (5)
#define NVM_USB_PAD_TRANSP_POS            (50)
#define NVM_USB_PAD_TRANSP_SIZE           (5)
#define NVM_USB_PAD_TRIM_POS              (55)
#define NVM_USB_PAD_TRIM_SIZE             (3)

extern const char *DeviceDescriptor;
extern const char *ConfigDescriptor[];
extern const char* ManufacturerString;
extern const char* ProductString;
extern const char* SerialNumberString;

__attribute__((__aligned__(4))) UsbDeviceDescriptor usb_endpoint_table[MAX_EP]; // Initialized to zero in USB_Init
__attribute__((__aligned__(4))) uint8_t udd_ep_out_cache_buffer[2][64]; //1 for CTRL, 1 for BULK
__attribute__((__aligned__(4))) uint8_t udd_ep_in_cache_buffer[2][64]; //1 for CTRL, 1 for BULK

static volatile bool read_job = false;
static uint8_t bCurrentConfig = 0;

/*----------------------------------------------------------------------------
* \brief Enable the USB interface at hardware level
*/
void USB_Open(P_USB_t pUsb)
{
    pUsb->HOST.CTRLA.bit.ENABLE = true;
}

/*----------------------------------------------------------------------------
* \brief Initializes USB
*/
void USB_Init(void)
{
    uint32_t pad_transn, pad_transp, pad_trim;

    /* Enable USB clock */
    PM->APBBMASK.reg |= PM_APBBMASK_USB;

    /* Set up the USB DP/DN pins */
    PORT->Group[0].PINCFG[PIN_PA24G_USB_DM].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg &= ~(0xF << (4 * (PIN_PA24G_USB_DM & 0x01u)));
    PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg |= MUX_PA24G_USB_DM << (4 * (PIN_PA24G_USB_DM & 0x01u));
    PORT->Group[0].PINCFG[PIN_PA25G_USB_DP].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg &= ~(0xF << (4 * (PIN_PA25G_USB_DP & 0x01u)));
    PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg |= MUX_PA25G_USB_DP << (4 * (PIN_PA25G_USB_DP & 0x01u));

    /* ----------------------------------------------------------------------------------------------
    * Put Generic Clock Generator 0 as source for Generic Clock Multiplexer 6 (USB reference)
    */
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( 6 ) | // Generic Clock Multiplexer 6
    GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
    GCLK_CLKCTRL_CLKEN ;

    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
    {
        /* Wait for synchronization */
    }

    /* Reset */
    USB->DEVICE.CTRLA.bit.SWRST = 1;
    while (USB->DEVICE.SYNCBUSY.bit.SWRST)
    {
        /* Sync wait */
    }

    /* Load Pad Calibration */
    pad_transn =( *((uint32_t *)(NVMCTRL_OTP4)
    + (NVM_USB_PAD_TRANSN_POS / 32))
    >> (NVM_USB_PAD_TRANSN_POS % 32))
    & ((1 << NVM_USB_PAD_TRANSN_SIZE) - 1);

    if (pad_transn == 0x1F)
    {
        pad_transn = 5;
    }

    USB->HOST.PADCAL.bit.TRANSN = pad_transn;

    pad_transp =( *((uint32_t *)(NVMCTRL_OTP4)
    + (NVM_USB_PAD_TRANSP_POS / 32))
    >> (NVM_USB_PAD_TRANSP_POS % 32))
    & ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);

    if (pad_transp == 0x1F)
    {
        pad_transp = 29;
    }

    USB->HOST.PADCAL.bit.TRANSP = pad_transp;
    pad_trim =( *((uint32_t *)(NVMCTRL_OTP4)
    + (NVM_USB_PAD_TRIM_POS / 32))
    >> (NVM_USB_PAD_TRIM_POS % 32))
    & ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);

    if (pad_trim == 0x7)
    {
        pad_trim = 3;
    }

    USB->HOST.PADCAL.bit.TRIM = pad_trim;

    /* Set the configuration */
    /* Set mode to Device mode */
    USB->HOST.CTRLA.bit.MODE = 0;
    /* Enable Run in Standby */
    USB->HOST.CTRLA.bit.RUNSTDBY = true;
    /* Set the descriptor address */
    USB->HOST.DESCADD.reg = (uint32_t)(&usb_endpoint_table[0]);
    /* Set speed configuration to Full speed */
    USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
    /* Attach to the USB host */
    USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_DETACH;

    /* Initialize endpoint table RAM location to a known value 0 */
    memset((uint8_t *)(&usb_endpoint_table[0]), 0, sizeof(usb_endpoint_table));
}

uint32_t USB_Write(P_USB_t pUsb, const char *pData, uint32_t length, uint8_t ep_num)
{
    uint32_t data_address;
    uint8_t buf_index;

    /* Set buffer index */
    buf_index = (ep_num == 0) ? 0 : 1;

    /* Check for requirement for multi-packet or auto zlp */
    if (length >= (1 << (usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.SIZE + 3)))
    {
        /* Update the EP data address */
        data_address = (uint32_t) pData;
        /* Enable auto zlp */
        usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.AUTO_ZLP = true;
    }
    else
    {
        /* Copy to local buffer */
        memcpy(udd_ep_in_cache_buffer[buf_index], pData, length);
        /* Update the EP data address */
        data_address = (uint32_t) &udd_ep_in_cache_buffer[buf_index];
    }

    /* Set the buffer address for ep data */
    usb_endpoint_table[ep_num].DeviceDescBank[1].ADDR.reg = data_address;
    /* Set the byte count as zero */
    usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = length;
    /* Set the multi packet size as zero for multi-packet transfers where length > ep size */
    usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    /* Clear the transfer complete flag  */
    pUsb->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
    /* Set the bank as ready */
    pUsb->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.bit.BK1RDY = true;

    /* Wait for transfer to complete */
    while ( (pUsb->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.bit.TRCPT1) == 0 );

    return length;
}

/*----------------------------------------------------------------------------
* \brief Read available data from Endpoint OUT
*/
uint32_t USB_Read(P_USB_t pUsb, char *pData, uint32_t length)
{
    uint32_t packetSize = 0;

    if (!read_job)
    {
        /* Set the buffer address for ep data */
        usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[USB_EP_OUT-1];
        /* Set the byte count as zero */
        usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
        /* Set the byte count as zero */
        usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
        /* Start the reception by clearing the bank 0 ready bit */
        pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSCLR.bit.BK0RDY = true;
        /* set the user flag */
        read_job = true;
    }

    /* Check for Transfer Complete 0 flag */
    if ( pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.bit.TRCPT0 )
    {
        /* Set packet size */
        packetSize = minval(usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT, length);
        /* Copy read data to user buffer */
        memcpy(pData, udd_ep_out_cache_buffer[USB_EP_OUT-1], packetSize);
        /* Clear the Transfer Complete 0 flag */
        pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
        /* Clear the user flag */
        read_job = false;
    }

    return packetSize;
}

uint32_t USB_Read_blocking(P_USB_t pUsb, char *pData, uint32_t length)
{
    if (read_job)
    {
        /* Stop the reception by setting the bank 0 ready bit */
        pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSSET.bit.BK0RDY = true;
        /* Clear the user flag */
        read_job = false;
    }

    /* Set the buffer address for ep data */
    usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].ADDR.reg = ((uint32_t)pData);
    /* Set the byte count as zero */
    usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
    /* Set the multi packet size as zero for multi-packet transfers where length > ep size */
    usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = length;
    /* Clear the bank 0 ready flag */
    pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSCLR.bit.BK0RDY = true;
    /* Wait for transfer to complete */
    while (!( pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.bit.TRCPT0 ));
    /* Clear Transfer complete 0 flag */
    pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;

    return length;
}

/*----------------------------------------------------------------------------
* \brief Test if the device is configured and handle requests
*/
uint8_t USB_IsConfigured(P_USB_t pUsb)
{
    uint8_t currentConfiguration = 0;

    /* Check for End of Reset flag */
    if (pUsb->DEVICE.INTFLAG.reg & USB_DEVICE_INTFLAG_EORST)
    {
        /* Clear the flag */
        pUsb->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
        /* Set Device address as 0 */
        pUsb->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | 0;
        /* Configure endpoint 0 */
        /* Configure Endpoint 0 for Control IN and Control OUT */
        pUsb->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1) | USB_DEVICE_EPCFG_EPTYPE1(1);
        pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
        pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
        /* Configure control OUT Packet size to 64 bytes */
        usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.SIZE = 3;
        /* Configure control IN Packet size to 64 bytes */
        usb_endpoint_table[0].DeviceDescBank[1].PCKSIZE.bit.SIZE = 3;
        /* Configure the data buffer address for control OUT */
        usb_endpoint_table[0].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[0];
        /* Configure the data buffer address for control IN */
        usb_endpoint_table[0].DeviceDescBank[1].ADDR.reg = (uint32_t)&udd_ep_in_cache_buffer[0];
        /* Set Multipacket size to 8 for control OUT and byte count to 0*/
        usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 8;
        usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
        pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

        // Reset current configuration value to 0
        currentConfiguration = 0;
    }
    else
    {
        if (pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_RXSTP)
        {
            USB_HandleRequest(pUsb, 0, 0);  // TODO - pass pointers for class-specific request handling
        }
    }

    return currentConfiguration;
}

/*----------------------------------------------------------------------------
* \brief Check whether a USB Request has been received from the Host
*/
bool USB_IsRequestPending(P_USB_t pUsb)
{
    if (pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_RXSTP)
    {
        return true;
    }
    return false;
}

/*----------------------------------------------------------------------------
* \brief Deal with the Request, and return whether it was handled successfully or NACKed
*/
bool USB_HandleRequest(P_USB_t pUsb, ClassReqHandlerFn_t pClassReqHandler, void *pExtraParams)
{
    static volatile uint8_t bmRequestType, bRequest, dir;
    static volatile uint16_t wValue, wIndex, wLength, wStatus;
    bool retval = true;

    /* Clear the Received Setup flag */
    pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;

    /* Read the USB request parameters */
    uint8_t *SetupPacket = udd_ep_out_cache_buffer[0];
    bmRequestType = SetupPacket[0];
    if ((bmRequestType & 0x60) != 0)  // if this is a class-specific or vendor-specific request,
    {
        return pClassReqHandler(pUsb, SetupPacket, pExtraParams);  // call the class-specific request handler function
    }
    
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
                uint8_t desc_length = DeviceDescriptor[0];  // first byte of descriptor gives its length
                USB_Write(pUsb, (const char*)DeviceDescriptor, minval(desc_length, wLength), USB_EP_CTRL);
            }
            else if ( bDesc == DESCRIPTOR_CONFIGURATION)
            {
                /* Return Configuration Descriptor */
                uint8_t desc_length = ConfigDescriptor[bCurrentConfig][0];  // first byte of descriptor gives its length
                USB_Write(pUsb, (const char*)ConfigDescriptor[bCurrentConfig], minval(desc_length, wLength), USB_EP_CTRL);
            }
            else if ( bDesc == DESCRIPTOR_STRING)
            {
                switch ( wValue & 0xff )
                {
                    case STRING_INDEX_LANGUAGES:
                    {
                        uint16_t STRING_LANGUAGE[2] = { (DESCRIPTOR_STRING << 8) | 4, 0x0409 };  // 0409 is US English
                        USB_Write(pUsb, (const char*)STRING_LANGUAGE, minval(sizeof(STRING_LANGUAGE), wLength), USB_EP_CTRL);
                    }
                    break;

                    case STRING_INDEX_MANUFACTURER:
                    USB_SendString(pUsb, ManufacturerString, wLength );
                    break;

                    case STRING_INDEX_PRODUCT:
                    USB_SendString(pUsb, ProductString, wLength );
                    break;

                    case STRING_INDEX_SERIAL_NUMBER:
                    USB_SendString(pUsb, SerialNumberString, wLength );  // TODO: create serial-number string from device unique-ID
                    break;

                    default:
                    USB_SendStall(pUsb, true);
                    retval = false;
                    break;
                }
            }
            else
            {
                USB_SendStall(pUsb, true);
                retval = false;
            }
            break;

        case USBREQ_SET_ADDRESS:
            USB_SendZLP(pUsb);
            /* Set device address to the newly received address from host */
            USB_SetAddress(pUsb, wValue);
            break;

        case USBREQ_SET_CONFIGURATION:
            /* Store configuration */
            bCurrentConfig = (uint8_t)wValue;
            USB_SendZLP(pUsb);
            /* Configure the 3 needed endpoints */
            USB_Configure(pUsb);
            break;

        case USBREQ_GET_CONFIGURATION:
            /* Return current configuration value */
            USB_Write(pUsb, (char*) &bCurrentConfig, 1, USB_EP_CTRL);
            break;

        case USBREQ_GET_STATUS_ZERO:
            wStatus = 0;
            USB_Write(pUsb, (char*) &wStatus, sizeof(wStatus), USB_EP_CTRL);
            break;

        case USBREQ_GET_STATUS_INTERFACE:
            wStatus = 0;
            USB_Write(pUsb, (char*) &wStatus, sizeof(wStatus), USB_EP_CTRL);
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
                USB_SendStall(pUsb, true);
                retval = false;
            }
            break;

        case USBREQ_SET_FEATURE_ZERO:
            USB_SendStall(pUsb, true);
            retval = false;
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
                USB_SendStall(pUsb, true);
                retval = false;
            }
            break;

        case USBREQ_SET_INTERFACE:
        case USBREQ_CLEAR_FEATURE_ZERO:
            USB_SendStall(pUsb, true);
            retval = false;
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
                retval = false;
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
* \brief Stall the control endpoint
*/
void USB_SendStall(P_USB_t pUsb, bool direction_in)
{
    /* Check the direction */
    if (direction_in)
    {
        /* Set STALL request on IN direction */
        pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
    }
    else
    {
        /* Set STALL request on OUT direction */
        pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ0 = 1;
    }
}

/*----------------------------------------------------------------------------
* \brief Send zero-length packet through the control endpoint
*/
void USB_SendZLP(P_USB_t pUsb)
{
    /* Set the byte count as zero */
    usb_endpoint_table[0].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
    /* Clear the transfer complete flag  */
    pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
    /* Set the bank as ready */
    pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK1RDY = true;
    /* Wait for transfer to complete */
    while (!( pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1 ));
}

/*----------------------------------------------------------------------------
* \brief Set USB device address obtained from host
*/
void USB_SetAddress(P_USB_t pUsb, uint16_t wValue)
{
    pUsb->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | wValue;
}

/*----------------------------------------------------------------------------
* \brief Configure USB device
*/
void USB_Configure(P_USB_t pUsb)
{
    /* Configure BULK OUT endpoint for CDC Data interface*/
    pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(3);
    /* Set maximum packet size as 64 bytes */
    usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.SIZE = 3;
    pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
    /* Configure the data buffer */
    usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[1];

    /* Configure BULK IN endpoint for CDC Data interface */
    pUsb->DEVICE.DeviceEndpoint[USB_EP_IN].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE1(3);
    /* Set maximum packet size as 64 bytes */
    usb_endpoint_table[USB_EP_IN].DeviceDescBank[1].PCKSIZE.bit.SIZE = 3;
    pUsb->DEVICE.DeviceEndpoint[USB_EP_IN].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
    /* Configure the data buffer */
    usb_endpoint_table[USB_EP_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&udd_ep_in_cache_buffer[1];

    /* Configure INTERRUPT IN endpoint for CDC COMM interface*/
    pUsb->DEVICE.DeviceEndpoint[USB_EP_COMM].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE1(4);
    /* Set maximum packet size as 64 bytes */
    usb_endpoint_table[USB_EP_COMM].DeviceDescBank[1].PCKSIZE.bit.SIZE = 0;
    pUsb->DEVICE.DeviceEndpoint[USB_EP_COMM].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
}

uint8_t USB_GetCurrentConfig(void)
{
    return bCurrentConfig;
}