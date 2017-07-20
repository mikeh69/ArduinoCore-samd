/*
  Copyright (c) ASH Wireless Ltd. 2017
  
  See USB.org Mass Storage Class documents - Overview, UFI Command Specification, Bulk-Only Transport.
*/

#ifndef _SAM_BA_MASS_STOR_H_
#define _SAM_BA_MASS_STOR_H_

#include <stdint.h>
#include <stdbool.h>

/* MSC Subclass Codes */
#define MSC_SUBCLASS_RBC               0x01
#define MSC_SUBCLASS_SFF8020I_MMC2     0x02
#define MSC_SUBCLASS_QIC157            0x03
#define MSC_SUBCLASS_UFI               0x04
#define MSC_SUBCLASS_SFF8070I          0x05
#define MSC_SUBCLASS_SCSI              0x06

/* MSC Protocol Codes */
#define MSC_PROTOCOL_CBI_INT           0x00
#define MSC_PROTOCOL_CBI_NOINT         0x01
#define MSC_PROTOCOL_BULK_ONLY         0x50

/* MSC Request Codes */
#define MSC_REQUEST_RESET              0xFF
#define MSC_REQUEST_GET_MAX_LUN        0xFE

/* MSC Bulk-only Stage */
#define MSC_BS_CBW                     0       /* Command Block Wrapper */
#define MSC_BS_DATA_OUT                1       /* Data Out Phase */
#define MSC_BS_DATA_IN                 2       /* Data In Phase */
#define MSC_BS_DATA_IN_LAST            3       /* Data In Last Phase */
#define MSC_BS_DATA_IN_LAST_STALL      4       /* Data In Last Phase with Stall */
#define MSC_BS_CSW                     5       /* Command Status Wrapper */
#define MSC_BS_ERROR                   6       /* Error */

/* Bulk-only Command Block Wrapper */
typedef struct {
    uint32_t dSignature;
    uint32_t dTag;
    uint32_t dDataLength;
    uint8_t  bmFlags;
    uint8_t  bLUN;
    uint8_t  bCBLength;
    uint8_t  CB[16];
} __attribute__((packed)) MSC_CBW_t;

/* Bulk-only Command Status Wrapper */
typedef struct {
    uint32_t dSignature;
    uint32_t dTag;
    uint32_t dDataResidue;
    uint8_t  bStatus;
} __attribute__((packed)) MSC_CSW_t;

#define MSC_CBW_Signature              0x43425355  // "USBC"
#define MSC_CSW_Signature              0x53425355  // "USBS"

/* CSW Status Definitions */
#define CSW_CMD_PASSED                 0x00
#define CSW_CMD_FAILED                 0x01
#define CSW_PHASE_ERROR                0x02

/* Required UFI (SCSI) commands - Table 1 of UFI Command Spec. */
#define UFI_CMD_TEST_UNIT_READY        0x00
#define UFI_CMD_REQUEST_SENSE          0x03
#define UFI_CMD_FORMAT_UNIT            0x04
#define UFI_CMD_INQUIRY                0x12
#define UFI_CMD_MODE_SELECT6           0x15
#define UFI_CMD_MODE_SENSE6            0x1A
#define UFI_CMD_START_STOP_UNIT        0x1B
#define UFI_CMD_MEDIA_REMOVAL          0x1E
#define UFI_CMD_READ_FORMAT_CAPACITIES 0x23
#define UFI_CMD_READ_CAPACITY          0x25
#define UFI_CMD_READ10                 0x28
#define UFI_CMD_WRITE10                0x2A
#define UFI_CMD_SEEK10                 0x2B
#define UFI_CMD_VERIFY10               0x2F
#define UFI_CMD_MODE_SELECT10          0x55
#define UFI_CMD_MODE_SENSE10           0x5A
#define UFI_CMD_READ12                 0xA8
#define UFI_CMD_WRITE12                0xAA

#endif // _SAM_BA_MASS_STOR_H_
