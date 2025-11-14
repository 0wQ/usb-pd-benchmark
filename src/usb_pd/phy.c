#include "phy.h"

#include <ch32x035.h>
#include "delay.h"

static uint8_t usbpd_tx_buffer[USBPD_DATA_MAX_LEN] __attribute__((aligned(4)));

void usbpd_phy_init(void) {
    // 时钟配置
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);

    // CC 引脚
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 控制 CC_EN、RD_EN、RA_EN 相关引脚
    usbpd_phy_cc_enable(false);
    usbpd_phy_cc_rd_enable(PD_CC1, false);
    usbpd_phy_cc_rd_enable(PD_CC2, false);
    usbpd_phy_cc_ra_enable(PD_CC1, false);
    usbpd_phy_cc_ra_enable(PD_CC2, false);

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;

    // 设置默认的 CC 电压比较器
    PORT_CC1_REG |= CC_CMP_DEFAULT;
    PORT_CC1_REG |= CC_PU_180;
    PORT_CC2_REG |= CC_CMP_DEFAULT;
    PORT_CC2_REG |= CC_PU_180;

    // 设置 CC 以正常 VDD 电压驱动输出
    PORT_CC1_REG &= ~CC_LVE;
    PORT_CC2_REG &= ~CC_LVE;

    // 清除全部状态
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;

    // 清除所有中断标志位
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;

    // DMA 使能
    USBPD->CONFIG |= PD_DMA_EN;

    NVIC_SetPriority(USBPD_IRQn, 0);
    NVIC_EnableIRQ(USBPD_IRQn);
}

void usbpd_phy_cc_enable(bool en) {
#if (CONFIG_HW_VERSION == 2)
    // CC_EN: PB0
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOB, GPIO_Pin_0, en ? Bit_SET : Bit_RESET);
#endif

#if (CONFIG_HW_VERSION == 3 || CONFIG_HW_VERSION == 255)
    // CC_EN: PA4
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, en ? Bit_SET : Bit_RESET);
#endif
}

void usbpd_phy_cc_rd_enable(usbpd_cc_channel_t cc_channel, bool en) {
#if (CONFIG_HW_VERSION == 2)
    // CC_RD_EN: PB12
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOB, GPIO_Pin_12, en ? Bit_SET : Bit_RESET);
#endif

#if (CONFIG_HW_VERSION == 3 || CONFIG_HW_VERSION == 255)
    // CC_RD1_EN: PB1, CC_RD2_EN: PB12
    register uint32_t pin;
    switch (cc_channel) {
        case PD_CC1:
            pin = GPIO_Pin_1;
            break;
        case PD_CC2:
            pin = GPIO_Pin_12;
            break;
        default:
            return;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    if (en) {
        // Push-Pull Output
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        GPIO_WriteBit(GPIOB, pin, Bit_RESET);
    } else {
        // Floating Input
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
#endif
}

void usbpd_phy_cc_ra_enable(usbpd_cc_channel_t cc_channel, bool en) {
#if (CONFIG_HW_VERSION == 3 || CONFIG_HW_VERSION == 255)
    // CC_RA1_EN: PB3, CC_RA2_EN: PB11
    register uint32_t pin;
    if (cc_channel == PD_CC1) {
        pin = GPIO_Pin_3;
    }
    if (cc_channel == PD_CC2) {
        pin = GPIO_Pin_11;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    if (en) {
        // Push-Pull Output
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        GPIO_WriteBit(GPIOB, pin, Bit_RESET);
    } else {
        // Floating Input
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
#endif
}

void usbpd_phy_set_active_cc(usbpd_cc_channel_t cc_channel) {
    switch (cc_channel) {
        case PD_CC1:
            SET_CC1_ACTIVE();
            break;
        case PD_CC2:
            SET_CC2_ACTIVE();
            break;
        case PD_CC_NONE:
        default:
            break;
    }
}

void usbpd_phy_tx(uint8_t data_len, uint8_t sop) __attribute__((section(".highcode")));
void usbpd_phy_tx(uint8_t data_len, uint8_t sop) {
    if (data_len > USBPD_DATA_MAX_LEN) {
        data_len = USBPD_DATA_MAX_LEN;
    }

    // 设置 CC 低电压驱动输出
    if (IS_CC1_ACTIVE()) {
        PORT_CC1_REG |= CC_LVE;
    } else {
        PORT_CC2_REG |= CC_LVE;
    }

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;
    USBPD->TX_SEL = sop;
    USBPD->DMA = (uint32_t)usbpd_tx_buffer;
    USBPD->BMC_TX_SZ = data_len;

    USBPD->STATUS |= IF_TX_END;   // 发送完成中断标志
    USBPD->CONTROL |= PD_TX_EN;   // PD 发送使能
    USBPD->CONTROL |= BMC_START;  // BMC 发送开始信号

    // wait tx end
    while ((USBPD->STATUS & IF_TX_END) == 0);
}

void usbpd_phy_send_src_cap(void) {
    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageID = 0;
    header.MessageHeader.MessageType = USBPD_DATA_MSG_SRC_CAP;
    header.MessageHeader.SpecificationRevision = 2;
    header.MessageHeader.PortDataRole = 1;
    header.MessageHeader.PortPowerRole_CablePlug = 1;
    header.MessageHeader.NumberOfDataObjects = 7;

    USBPD_SourcePDO_t src_pdo_pos1 = {0};
    src_pdo_pos1.General.PDO_Type = PDO_TYPE_FIXED_SUPPLY;
    src_pdo_pos1.Fixed.MaxCurrentIn10mAunits = 500;
    src_pdo_pos1.Fixed.VoltageIn50mVunits = 5000 / 50;
    USBPD_SourcePDO_t src_pdo_pos2 = {0};
    src_pdo_pos2.General.PDO_Type = PDO_TYPE_FIXED_SUPPLY;
    src_pdo_pos2.Fixed.MaxCurrentIn10mAunits = 500;
    src_pdo_pos2.Fixed.VoltageIn50mVunits = 9000 / 50;
    USBPD_SourcePDO_t src_pdo_pos3 = {0};
    src_pdo_pos3.General.PDO_Type = PDO_TYPE_FIXED_SUPPLY;
    src_pdo_pos3.Fixed.MaxCurrentIn10mAunits = 500;
    src_pdo_pos3.Fixed.VoltageIn50mVunits = 12000 / 50;
    USBPD_SourcePDO_t src_pdo_pos4 = {0};
    src_pdo_pos4.General.PDO_Type = PDO_TYPE_FIXED_SUPPLY;
    src_pdo_pos4.Fixed.MaxCurrentIn10mAunits = 500;
    src_pdo_pos4.Fixed.VoltageIn50mVunits = 15000 / 50;
    USBPD_SourcePDO_t src_pdo_pos5 = {0};
    src_pdo_pos5.General.PDO_Type = PDO_TYPE_FIXED_SUPPLY;
    src_pdo_pos5.Fixed.MaxCurrentIn10mAunits = 500;
    src_pdo_pos5.Fixed.VoltageIn50mVunits = 20000 / 50;
    USBPD_SourcePDO_t src_pdo_pos6 = {0};
    src_pdo_pos6.General.PDO_Type = PDO_TYPE_APDO;
    src_pdo_pos6.General.APDO_SubType = APDO_TYPE_SPR_PPS;
    src_pdo_pos6.SPR_PPS.MaxCurrentIn50mAunits = 5000 / 50;
    src_pdo_pos6.SPR_PPS.MinVoltageIn100mVunits = 3300 / 100;
    src_pdo_pos6.SPR_PPS.MaxVoltageIn100mVunits = 21000 / 100;
    USBPD_SourcePDO_t src_pdo_pos7 = {0};
    src_pdo_pos7.General.PDO_Type = PDO_TYPE_APDO;
    src_pdo_pos7.General.APDO_SubType = APDO_TYPE_SPR_AVS;
    src_pdo_pos7.SPR_AVS.MaxCurrentFor15V20VIn10mAunits = 500;
    src_pdo_pos7.SPR_AVS.MaxCurrentFor9V15VIn10mAunits = 500;

    *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
    *(uint32_t *)&usbpd_tx_buffer[2] = src_pdo_pos1.d32;
    *(uint32_t *)&usbpd_tx_buffer[6] = src_pdo_pos2.d32;
    *(uint32_t *)&usbpd_tx_buffer[10] = src_pdo_pos3.d32;
    *(uint32_t *)&usbpd_tx_buffer[14] = src_pdo_pos4.d32;
    *(uint32_t *)&usbpd_tx_buffer[18] = src_pdo_pos5.d32;
    *(uint32_t *)&usbpd_tx_buffer[22] = src_pdo_pos6.d32;
    *(uint32_t *)&usbpd_tx_buffer[26] = src_pdo_pos7.d32;

    usbpd_phy_tx((2 + 4 * 7), UPD_SOP0);
}

void usbpd_phy_send_ping(void) {
    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageType = USBPD_CONTROL_MSG_PING;
    *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
    usbpd_phy_tx(2, UPD_SOP0);
}

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((section(".highcode")));
void USBPD_IRQHandler(void) {
    // 接收完成中断标志
    if (USBPD->STATUS & IF_RX_ACT) {
        USBPD->STATUS |= IF_RX_ACT;

        // 清除所有中断标志位
        USBPD->CONFIG |= PD_ALL_CLR;
        USBPD->CONFIG &= ~PD_ALL_CLR;
    }

    // 发送完成中断标志
    if (USBPD->STATUS & IF_TX_END) {
        USBPD->STATUS |= IF_TX_END;

        // 清除所有中断标志位
        USBPD->CONFIG |= PD_ALL_CLR;
        USBPD->CONFIG &= ~PD_ALL_CLR;
    }

    // 接收复位中断标志
    if (USBPD->STATUS & IF_RX_RESET) {
        USBPD->STATUS |= IF_RX_RESET;

        // 清除所有中断标志位
        USBPD->CONFIG |= PD_ALL_CLR;
        USBPD->CONFIG &= ~PD_ALL_CLR;
    }
}