#include <stdio.h>
#include <ch32x035.h>

#include "config.h"
#include "delay.h"
#include "uid.h"

#include "usb_pd/phy.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();

    delay_init();

    // USB PD
    usbpd_phy_init();
    usbpd_phy_cc_enable(1);
    usbpd_phy_cc_rd_enable(PD_CC1, 1);
    usbpd_phy_cc_rd_enable(PD_CC2, 1);

    while (1) {
        delay_ms(5000);

        for (int i = 0; i < 100; i++) {
            usbpd_phy_send_ping();
            delay_us(500 - 7);  // 500 - 7 大概是 1ms 间隔，注释掉后大概 504us 间隔
        }
    }
}

void NMI_Handler(void) {}
void HardFault_Handler(void) {
    while (1);
}
