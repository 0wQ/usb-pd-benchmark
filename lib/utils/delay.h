#pragma once

#include <stdint.h>

// SysTick Control / Status Register Definitions
#define SysTick_CTLR_STRE_Msk  (1UL << 3U)  // SysTick CTRL: COUNTFLAG Mask
#define SysTick_CTLR_STCLK_Msk (1UL << 2U)  // SysTick CTRL: CLKSOURCE Mask(1:SYS、0:SYS/8)
#define SysTick_CTLR_STIE_Msk  (1UL << 1U)  // SysTick CTRL: TICKINT Mask
#define SysTick_CTLR_STE_Msk   (1UL << 0U)  // SysTick CTRL: ENABLE Mask

void delay_init(void);

void delay_us(uint32_t us) __attribute__((section(".highcode")));
void delay_ms(uint32_t ms) __attribute__((section(".highcode")));

uint32_t micros(void) __attribute__((section(".highcode")));
uint32_t millis(void) __attribute__((section(".highcode")));

char *format_time_ms(uint32_t milliseconds, char *buffer);
char *format_time_us(uint64_t microseconds, char *buffer);
