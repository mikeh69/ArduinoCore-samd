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
*/

#include <stdio.h>
#include <sam.h>

#define SAM_BA_INTERFACE    9999     // nothing at all!

#define BUFFER_SIZE_BYTES 512  // enough for one flash-sector 

#include "sam_ba_monitor.h"
#include "sam_ba_serial.h"
#include "board_definitions.h"
#include "board_driver_led.h"
// USB headers: generic and class-specific
#include "board_driver_usb.h"
#include "sam_ba_mass_stor.h"
#include "sam_ba_cdc.h"

extern uint32_t __sketch_vectors_ptr; // Exported value from linker script
extern void board_init(void);

USB_CDC_t sam_ba_cdc;  // not really used, but expected by sam_ba_monitor.c

#if (defined DEBUG) && (DEBUG == 1)
volatile uint32_t* pulSketch_Start_Address;
#endif

static volatile bool main_b_cdc_enable = false;

/**
* \brief Check the application startup condition
*
*/
static void check_start_application(void)
{
    //  LED_init();
    //  LED_off();

    #if (!defined DEBUG) || ((defined DEBUG) && (DEBUG == 0))
    uint32_t* pulSketch_Start_Address;
    #endif

    /*
    * Test sketch stack pointer @ &__sketch_vectors_ptr
    * Stay in SAM-BA if value @ (&__sketch_vectors_ptr) == 0xFFFFFFFF (Erased flash cell value)
    */
    if (__sketch_vectors_ptr == 0xFFFFFFFF)
    {
        /* Stay in bootloader */
        return;
    }

    /*
    * Load the sketch Reset Handler address
    * __sketch_vectors_ptr is exported from linker script and point on first 32b word of sketch vector table
    * First 32b word is sketch stack
    * Second 32b word is sketch entry point: Reset_Handler()
    */
    pulSketch_Start_Address = &__sketch_vectors_ptr ;
    pulSketch_Start_Address++ ;

    /*
    * Test vector table address of sketch @ &__sketch_vectors_ptr
    * Stay in SAM-BA if this function is not aligned enough, ie not valid
    */
    if ( ((uint32_t)(&__sketch_vectors_ptr) & ~SCB_VTOR_TBLOFF_Msk) != 0x00)
    {
        /* Stay in bootloader */
        return;
    }

    #if defined(BOOT_DOUBLE_TAP_ADDRESS)
    #define DOUBLE_TAP_MAGIC 0x07738135
    if (PM->RCAUSE.bit.POR)
    {
        /* On power-on initialize double-tap */
        BOOT_DOUBLE_TAP_DATA = 0;
    }
    else
    {
        if (BOOT_DOUBLE_TAP_DATA == DOUBLE_TAP_MAGIC)
        {
            /* Second tap, stay in bootloader */
            BOOT_DOUBLE_TAP_DATA = 0;
            return;
        }

        /* First tap */
        BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;

        /* Wait 0.5sec to see if the user tap reset again.
        * The loop value is based on SAMD21 default 1MHz clock @ reset.
        */
        for (uint32_t i=0; i<125000; i++) /* 500ms */
        /* force compiler to not optimize this... */
        __asm__ __volatile__("");

        /* Timeout happened, continue boot... */
        BOOT_DOUBLE_TAP_DATA = 0;
    }
    #endif

    /*
    #if defined(BOOT_LOAD_PIN)
    volatile PortGroup *boot_port = (volatile PortGroup *)(&(PORT->Group[BOOT_LOAD_PIN / 32]));
    volatile bool boot_en;

    // Enable the input mode in Boot GPIO Pin
    boot_port->DIRCLR.reg = BOOT_PIN_MASK;
    boot_port->PINCFG[BOOT_LOAD_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
    boot_port->OUTSET.reg = BOOT_PIN_MASK;
    // Read the BOOT_LOAD_PIN status
    boot_en = (boot_port->IN.reg) & BOOT_PIN_MASK;

    // Check the bootloader enable condition
    if (!boot_en)
    {
    // Stay in bootloader
    return;
    }
    #endif
    */

    //  LED_on();

    /* Rebase the Stack Pointer */
    __set_MSP( (uint32_t)(__sketch_vectors_ptr) );

    /* Rebase the vector table base address */
    SCB->VTOR = ((uint32_t)(&__sketch_vectors_ptr) & SCB_VTOR_TBLOFF_Msk);

    /* Jump to application Reset Handler in the application */
    asm("bx %0"::"r"(*pulSketch_Start_Address));
}

#if DEBUG_ENABLE
#	define DEBUG_PIN_HIGH 	port_pin_set_output_level(BOOT_LED, 1)
#	define DEBUG_PIN_LOW 	port_pin_set_output_level(BOOT_LED, 0)
#else
#	define DEBUG_PIN_HIGH 	do{}while(0)
#	define DEBUG_PIN_LOW 	do{}while(0)
#endif

/**
*  \brief SAMD21 SAM-BA Main loop.
*  \return Unused (ANSI-C compatibility).
*/
int main(void)
{
    union {
        char Bytes[BUFFER_SIZE_BYTES];
        uint32_t DWORDS[BUFFER_SIZE_BYTES/4];
    } Buffer;        

    /* Jump in application if condition is satisfied */
    check_start_application();

    /* We have determined we should stay in the monitor. */
    /* System initialization */
    board_init();
    __enable_irq();

    #if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
    /* UART is enabled in all cases */
    serial_open();
    #endif

    #if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
    pCdc = usb_init();  // start the serial-port device
    #else
    P_USB_MSD_t pMSD = usb_msd_init();  // start the mass-storage device
    #endif

    DEBUG_PIN_LOW;

    /* Initialize LEDs */
    LED_init();
    LEDRX_init();
    LEDRX_off();
    LEDTX_init();
    LEDTX_off();

    /* Start the sys tick (1 ms) */
    SysTick_Config(1000);

    
    uint8_t isConfigured = USB_IsConfigured(USB);
    /* Wait for a USB request or a '#' char on serial line */
    while (1)
    {
        // Check for Setup packet (USB Request)
        if (USB_IsRequestPending(USB))
        {
            bool result = USB_HandleRequest(USB, sam_ba_usb_mass_stor_handle_req, 0);
        }
        // Check for Bulk-out packet (Mass-Storage/SCSI transaction)
        if (USB_IsBulkDataAvailable(USB))
        {
            bool result = USB_Read(USB, Buffer.Bytes, BUFFER_SIZE_BYTES);
            if (result)
            {
                if (Buffer.DWORDS[0] == 0x43425355) // 'USBC'
                {
                    uint32_t Tag = Buffer.DWORDS[1];
                    uint32_t TransferLength = Buffer.DWORDS[2];
                    uint8_t CdbLength = Buffer.Bytes[14];
                    result = sam_ba_usb_mass_stor_handle_scsi(&(Buffer.Bytes[15]), CdbLength, Tag);
                }
            }
        }

        #if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
        /* Check if a '#' has been received */
        if (!main_b_cdc_enable && serial_sharp_received())
        {
            sam_ba_monitor_init(SAM_BA_INTERFACE_USART);
            /* SAM-BA on Serial loop */
            while(1)
            {
                sam_ba_monitor_run();
            }
        }
        #endif
    }
}

void SysTick_Handler(void)
{
    LED_pulse();

    sam_ba_monitor_sys_tick();
}
