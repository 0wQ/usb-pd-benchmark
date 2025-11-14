#pragma once

#include <stddef.h>
#include "config.h"

// 用于解决和 POWER-Z KM003C 的 CC 线序相反的问题
#ifndef CC_SWAP
    #define PD_PIN_CC1       GPIO_Pin_14
    #define PD_PIN_CC2       GPIO_Pin_15
    #define PORT_CC1_REG     (*((volatile uint16_t *)0x4002700C))
    #define PORT_CC2_REG     (*((volatile uint16_t *)0x4002700E))
    #define SET_CC1_ACTIVE() (USBPD->CONFIG &= ~CC_SEL)
    #define SET_CC2_ACTIVE() (USBPD->CONFIG |= CC_SEL)
    #define IS_CC1_ACTIVE()  ((USBPD->CONFIG & CC_SEL) == CC_SEL)
#else
    #define PD_PIN_CC1       GPIO_Pin_15
    #define PD_PIN_CC2       GPIO_Pin_14
    #define PORT_CC1_REG     (*((volatile uint16_t *)0x4002700E))
    #define PORT_CC2_REG     (*((volatile uint16_t *)0x4002700C))
    #define SET_CC1_ACTIVE() (USBPD->CONFIG |= CC_SEL)
    #define SET_CC2_ACTIVE() (USBPD->CONFIG &= ~CC_SEL)
    #define IS_CC1_ACTIVE()  ((USBPD->CONFIG & CC_SEL) != CC_SEL)
#endif

// 默认的 CC 电压比较器，设置 CC_CMP_66 对比 KM003C 某些特殊情况会漏包，CC_CMP_45 CC_CMP_55 没问题
#define CC_CMP_DEFAULT CC_CMP_45

/* CC 通道枚举 */
typedef enum {
    PD_CC_NONE = 0,
    PD_CC1 = 1,
    PD_CC2 = 2,
} usbpd_cc_channel_t;

// /* PD 连接状态枚举 */
// typedef enum {
//     PD_STATE_DISCONNECTED = 0,
//     PD_STATE_CC1_CONNECTED = 1,
//     PD_STATE_CC2_CONNECTED = 2
// } usbpd_state_t;

/* PD 数据包最大长度 */
#define USBPD_DATA_MAX_LEN 34  // 2(Header) + (7*4)DataBlock + 4(CRC32)

/* PD 消息类型描述结构体 */
typedef struct {
    const uint8_t type;  // 消息类型编号
    const char *name;    // 消息类型名称
} usbpd_msg_type_desc_t;

/* Message Header */
typedef union {
    uint16_t d16;
    struct {
        uint16_t MessageType             : 5u;
        uint16_t PortDataRole            : 1u;  // 0b->UFP, 1b->DFP (SOP Only)
        uint16_t SpecificationRevision   : 2u;  // 00b->v1.0, 01b->v2.0, 10b->v3.0
        uint16_t PortPowerRole_CablePlug : 1u;  // PortPowerRole: 0b->sink, 1b->source (SOP Only)
                                                // CablePlug: 0b->DFP/UFP, 1b->CablePlug/VPD (SOP’/SOP’’)
                                                // The Cable Plug field Shall only apply to SOP’ Packet and SOP’’ Packet types
        uint16_t MessageID           : 3u;
        uint16_t NumberOfDataObjects : 3u;
        uint16_t Extended            : 1u;
    } MessageHeader;
} USBPD_MessageHeader_t;

/* PD 控制消息类型描述表 */
static const usbpd_msg_type_desc_t ctrl_msg_desc[] = {
    {0b00001, "GoodCRC"},
    {0b00010, "GotoMin"},
    {0b00011, "Accept"},
    {0b00100, "Reject"},
    {0b00101, "Ping"},
    {0b00110, "PSRDY"},
    {0b00111, "GetSourceCap"},
    {0b01000, "GetSinkCap"},
    {0b01001, "DRSwap"},
    {0b01010, "PRSwap"},
    {0b01011, "VconnSwap"},
    {0b01100, "Wait"},
    {0b01101, "SoftReset"},
    {0b01110, "DataReset"},
    {0b01111, "DataResetComplete"},
    {0b10000, "NotSupported"},
    {0b10001, "GetSourceCapExt"},
    {0b10010, "GetStatus"},
    {0b10011, "FRSwap"},
    {0b10100, "GetPPSStatus"},
    {0b10101, "GetCountryCodes"},
    {0b10110, "GetSinkCapExt"},
    {0b10111, "GetSourceInfo"},
    {0b11000, "GetRevision"},
    {0, NULL},
};

/* PD 数据消息类型描述表 */
static const usbpd_msg_type_desc_t data_msg_desc[] = {
    {0b00001, "SourceCap"},
    {0b00010, "Request"},
    {0b00011, "BIST"},
    {0b00100, "SinkCap"},
    {0b00101, "BatteryStatus"},
    {0b00110, "Alert"},
    {0b00111, "GetCountryInfo"},
    {0b01000, "EnterUSB"},
    {0b01001, "EPRRequest"},
    {0b01010, "EPRMode"},
    {0b01011, "SourceInfo"},
    {0b01100, "Revision"},
    {0b01111, "VendorDefined"},
    {0, NULL},
};

/* PD 扩展消息类型描述表 */
static const usbpd_msg_type_desc_t ext_msg_desc[] = {
    {0x01, "SourceCapExt"},
    {0x02, "Status"},
    {0x03, "GetBatteryCap"},
    {0x04, "GetBatteryStatus"},
    {0x05, "BatteryCap"},
    {0x06, "GetMfrInfo"},
    {0x07, "MfrInfo"},
    {0x08, "SecurityReq"},
    {0x09, "SecurityResp"},
    {0x0A, "FWUpdateReq"},
    {0x0B, "FWUpdateResp"},
    {0x0C, "PPSStatus"},
    {0x0D, "CountryInfo"},
    {0x0E, "CountryCodes"},
    {0x0F, "SinkCapExt"},
    {0x10, "ExtControl"},
    {0x11, "EPRSourceCap"},
    {0x12, "EPRSinkCap"},
    {0x1F, "VendorDefinedExt"},
    {0, NULL},
};

/* Extended Control Message Types */
static const usbpd_msg_type_desc_t ext_ctrl_msg_desc[] = {
    {1, "EPRGetSourceCap"},
    {2, "EPRGetSinkCap"},
    {3, "EPRKeepAlive "},
    {4, "EPRKeepAliveAck "},
    {0, NULL},
};

// Table 6.5 Control Message Types
typedef enum {
    USBPD_CONTROL_MSG_GOODCRC = 0x01u,
    USBPD_CONTROL_MSG_GOTOMIN = 0x02u,  // Deprecated
    USBPD_CONTROL_MSG_ACCEPT = 0x03u,
    USBPD_CONTROL_MSG_REJECT = 0x04u,
    USBPD_CONTROL_MSG_PING = 0x05u,  // Deprecated
    USBPD_CONTROL_MSG_PS_RDY = 0x06u,
    USBPD_CONTROL_MSG_GET_SRC_CAP = 0x07u,
    USBPD_CONTROL_MSG_GET_SNK_CAP = 0x08u,
    USBPD_CONTROL_MSG_DR_SWAP = 0x09u,
    USBPD_CONTROL_MSG_PR_SWAP = 0x0Au,
    USBPD_CONTROL_MSG_VCONN_SWAP = 0x0Bu,
    USBPD_CONTROL_MSG_WAIT = 0x0Cu,
    USBPD_CONTROL_MSG_SOFT_RESET = 0x0Du,
    USBPD_CONTROL_MSG_DATA_RESET = 0x0Eu,
    USBPD_CONTROL_MSG_DATA_RESET_COMPLETE = 0x0Fu,
    USBPD_CONTROL_MSG_NOT_SUPPORTED = 0x10u,
    USBPD_CONTROL_MSG_GET_SRC_CAP_EXTENDED = 0x11u,
    USBPD_CONTROL_MSG_GET_STATUS = 0x12u,
    USBPD_CONTROL_MSG_FR_SWAP = 0x13u,
    USBPD_CONTROL_MSG_GET_PPS_STATUS = 0x14u,
    USBPD_CONTROL_MSG_GET_COUNTRY_CODES = 0x15u,
    USBPD_CONTROL_MSG_GET_SNK_CAP_EXTENDED = 0x16u,
    USBPD_CONTROL_MSG_GET_SRC_INFO = 0x17u,
    USBPD_CONTROL_MSG_GET_REVISION = 0x18u,
} USBPD_ControlMessageType_t;

// Table 6.6 Data Message Types
typedef enum {
    USBPD_DATA_MSG_SRC_CAP = 0x01u,
    USBPD_DATA_MSG_REQUEST = 0x02u,
    USBPD_DATA_MSG_BIST = 0x03u,
    USBPD_DATA_MSG_SNK_CAP = 0x04u,
    USBPD_DATA_MSG_BATTERY_STATUS = 0x05u,
    USBPD_DATA_MSG_ALERT = 0x06u,
    USBPD_DATA_MSG_GET_COUNTRY_INFO = 0x07u,
    USBPD_DATA_MSG_ENTER_USB = 0x08u,
    USBPD_DATA_MSG_EPR_REQUEST = 0x09u,
    USBPD_DATA_MSG_EPR_MODE = 0x0Au,
    USBPD_DATA_MSG_SRC_INFO = 0x0Bu,
    USBPD_DATA_MSG_REVISION = 0x0Cu,
    USBPD_DATA_MSG_VENDOR_DEFINED = 0x0Fu,
} USBPD_DataMessageType_t;

// Table 6.7 Power Data Object (B31-B30)
typedef enum {
    PDO_TYPE_FIXED_SUPPLY = 0b00,     // Fixed Supply (Vmin = Vmax)
    PDO_TYPE_BATTERY = 0b01,          // Battery
    PDO_TYPE_VARIABLE_SUPPLY = 0b10,  // Variable Supply (non-Battery)
    PDO_TYPE_APDO = 0b11              // Augmented Power Data Object (APDO)
} USBPD_PDO_Type_t;

// Table 6.8 Augmented Power Data Object (B29-B28, only valid when B31-B30 = 0b11)
typedef enum {
    APDO_TYPE_SPR_PPS = 0b00,  // SPR Programmable Power Supply
    APDO_TYPE_EPR_AVS = 0b01,  // EPR Adjustable Voltage Supply
    APDO_TYPE_SPR_AVS = 0b10,  // SPR Adjustable Voltage Supply
    APDO_TYPE_RESERVED = 0b11  // Reserved
} USBPD_APDO_Subtype_t;

// 6.4.1.2 Source Power Data Objects
typedef union {
    uint32_t d32;

    // General structure for parsing PDO type
    struct {
        uint32_t SpecificFields : 28u;  // Specific fields depending on the PDO type
        uint32_t APDO_SubType   : 2u;   // B29-B28: APDO Subtype(only valid when B31-B30 = 0b11)
        uint32_t PDO_Type       : 2u;   // B31-B30: PDO Type
    } General;

    // Table 6.9 Fixed Supply PDO – Source
    struct {
        uint32_t MaxCurrentIn10mAunits             : 10u;  // Maximum current in 10mA units
        uint32_t VoltageIn50mVunits                : 10u;  // Voltage in 50mV units
        uint32_t PeakCurrent                       : 2u;   // Peak Current value
        uint32_t Reserved_22                       : 1u;   // Reserved – Shall be set to zero
        uint32_t EPRCapable                        : 1u;   // Set to 1 if EPR Capable.
        uint32_t UnchunkedExtendedMessageSupported : 1u;   // Set to 1 if Unchunked Extended Messages are supported.
        uint32_t DualRoleData                      : 1u;   // Set to 1 for a Dual-Role Data device.
        uint32_t USBCommunicationsCapable          : 1u;   // Set to 1 if capable of USB Communications capable
        uint32_t UnconstrainedPower                : 1u;   // Set to 1 if unconstrained power is available.
        uint32_t USBSuspendSupported               : 1u;   // Set to 1 if USB suspend is supported.
        uint32_t DualRolePower                     : 1u;   // Set to 1 for Dual-Role Power device.
        uint32_t FixedSupply                       : 2u;   // 00b - Fixed Supply PDO
    } Fixed;

    // Table 6.11 Variable Supply (non-Battery) PDO – Source
    struct {
        uint32_t MaxCurrentIn10mAunits : 10u;  // Maximum current in 10mA units
        uint32_t MinVoltageIn50mVunits : 10u;  // Minimum voltage in 50mV units
        uint32_t MaxVoltageIn50mVunits : 10u;  // Maximum voltage in 50mV units
        uint32_t VariableSupply        : 2u;   // 01b - Variable Supply (non-Battery) PDO
    } Variable;

    // Table 6.12 Battery Supply PDO – Source
    struct {
        uint32_t MaxAllowablePowerIn250mWunits : 10u;  // Maximum allowable power in 250mW units
        uint32_t MinVoltageIn50mVunits         : 10u;  // Minimum voltage in 50mV units
        uint32_t MaxVoltageIn50mVunits         : 10u;  // Maximum voltage in 50mV units
        uint32_t BatterySupply                 : 2u;   // 10b - Battery Supply PDO
    } Battery;

    // Table 6.13 SPR Programmable Power Supply APDO – Source
    struct {
        uint32_t MaxCurrentIn50mAunits  : 7u;  // Maximum current in 50mA units
        uint32_t Reserved_7             : 1u;  // Reserved – Shall be set to zero
        uint32_t MinVoltageIn100mVunits : 8u;  // Minimum voltage in 100mV units
        uint32_t Reserved_16            : 1u;  // Reserved – Shall be set to zero
        uint32_t MaxVoltageIn100mVunits : 8u;  // Maximum voltage in 100mV units
        uint32_t Reserved_25_26         : 2u;  // Reserved – Shall be set to zero
        uint32_t PPSpowerLimited        : 1u;  // Set to 1 when PPS Power Limited
        uint32_t SPRPPS                 : 2u;  // 00b – SPR PPS
        uint32_t APDO                   : 2u;  // 11b – Augmented Power Data Object (APDO)
    } SPR_PPS;

    // Table 6.14 SPR Adjustable Voltage Supply APDO – Source
    struct {
        uint32_t MaxCurrentFor15V20VIn10mAunits : 10u;  // For 15V – 20V range: Maximum current in 10mA units equal to the Maximum Current field of the 20V Fixed Supply PDO, set to 0 if the maximum voltage in the SPR AVS range is 15V
        uint32_t MaxCurrentFor9V15VIn10mAunits  : 10u;  // For  9V – 15V range: Maximum current in 10mA units equal to the Maximum Current field of the 15V Fixed Supply PDO
        uint32_t Reserved_20_25                 : 6u;   // Reserved – Shall be set to zero
        uint32_t PeakCurrent                    : 2u;   // Peak Current (see Table 6.10, "Fixed Power Source Peak Current Capability"))
        uint32_t SPRAVS                         : 2u;   // 10b – SPR AVS
        uint32_t APDO                           : 2u;   // 11b – Augmented Power Data Object (APDO)
    } SPR_AVS;

    // Table 6.15 EPR Adjustable Voltage Supply APDO – Source
    struct {
        uint32_t PDPIn1Wunits           : 8u;  // PDP in 1W units
        uint32_t MinVoltageIn100mVunits : 8u;  // Minimum voltage in 100mV units
        uint32_t Reserved_16            : 1u;  // Reserved – Shall be set to zero
        uint32_t MaxVoltageIn100mVunits : 9u;  // Maximum voltage in 100mV units
        uint32_t PeakCurrent            : 2u;  // Peak Current (see Table 6.10, "Fixed Power Source Peak Current Capability"))
        uint32_t EPRAVS                 : 2u;  // 01b – EPR AVS
        uint32_t APDO                   : 2u;  // 11b – Augmented Power Data Object (APDO)
    } EPR_AVS;
} USBPD_SourcePDO_t;
