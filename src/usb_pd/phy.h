#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "types.h"

void usbpd_phy_init(void);

void usbpd_phy_cc_enable(bool en);
void usbpd_phy_cc_rd_enable(usbpd_cc_channel_t cc_channel, bool en);
void usbpd_phy_cc_ra_enable(usbpd_cc_channel_t cc_channel, bool en);

void usbpd_phy_set_active_cc(usbpd_cc_channel_t cc_channel);

void usbpd_phy_send_src_cap(void) __attribute__((section(".highcode")));
void usbpd_phy_send_ping(void) __attribute__((section(".highcode")));
