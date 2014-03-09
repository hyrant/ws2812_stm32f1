/*
 * Copyright (C) 2014 Derek Hageman <Derek.C.Hageman@gmail.com> 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>

#include "ws2812.h"
#include "ws2812_config.h"

#define N_LEDS      16

#if 1
static const uint8_t outputColors[8*3] = {
    /* G     R     B */
    0x00, 0xFF, 0x00,
    0xFF, 0xFF, 0x00,
    0xFF, 0x00, 0x00,
    0xFF, 0x00, 0xFF,
    0x00, 0x00, 0xFF,
    0x00, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF,
    0x00, 0x00, 0x00,
};
#else
static const uint8_t outputColors[8*3] = {
    /* G     R     B */
    0x00, 0x00, 0x00,
    0x00, 0x20, 0x00,
    0x00, 0x40, 0x00,
    0x00, 0x80, 0x00,
    0x00, 0xA0, 0x00,
    0x00, 0xC0, 0x00,
    0x00, 0xE0, 0x00,
    0x00, 0xFF, 0x00,
};
#endif


volatile uint32_t systick;
void sys_tick_handler(void) {
    systick++;
}

static uint32_t wrapU32(uint32_t start, uint32_t now)
{
    if (now < start)
        return 0xFFFFFFFF - start + now + 1;
    return now - start;
}

static void outputPercent(uint32_t n, int decimals)
{
    uint32_t div = 100;
    for (int i=0; i<decimals; i++) {
        div *= 10;
    }
    for (int i=0; i<3; i++, div /= 10) {
        usart_send_blocking(USART1, '0' + ((n/div) % 10));
    }
    if (decimals > 0) {
        usart_send_blocking(USART1, '.');
        for (int i=0; i<decimals; i++, div /= 10) {
            usart_send_blocking(USART1, '0' + ((n/div) % 10));
        }
    }
}
static void outputString(const char *str)
{
    for (; *str; ++str) {
        usart_send_blocking(USART1, *str);
    }
}

int main( void ) {
    rcc_clock_setup_in_hsi_out_24mhz();
    
    systick = 0;
    STK_LOAD = (uint32_t)(24E6 / 1000 / 8);
    STK_CTRL = STK_CTRL_CLKSOURCE_AHB_DIV8|STK_CTRL_TICKINT;
    STK_CTRL |= STK_CTRL_ENABLE;
    
    nvic_set_priority(NVIC_SYSTICK_IRQ, 128);
    
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    
    gpio_set_mode(GPIO_BANK_USART1_TX, GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_enable(USART1);
    
    #if defined(PWM_MODE)
        #if !defined(OUTPUT_INVERT)
            /* PA8 */
            gpio_set_mode(GPIO_BANK_TIM1_CH1, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM1_CH1);
        #else
            /* PA7 */
            gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ, 
                AFIO_MAPR_TIM1_REMAP_PARTIAL_REMAP);
            gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7);
        #endif
    #elif defined(GPIO_MODE)
        gpio_set_mode(PORT, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, 1<<OUTPUT);
    #elif defined(MULTI_MODE)
        gpio_set_mode(PORT, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, 0xFFFF);
    #endif
    
    int offset = 0;
    #if !defined(MULTI_MODE)
        uint8_t buffer[N_LEDS*3];
    #else
        #if !defined(MUTLI_WIDE)
            uint8_t buffer[N_LEDS*8*3];
        #else
            uint16_t buffer[N_LEDS*8*3];
        #endif
    #endif
    while (1) {
        #if !defined(MULTI_MODE)
            uint8_t *target = buffer;
            for (int i=0; i<N_LEDS; i++) {
                const uint8_t *source = &outputColors[((i+offset) % 
                    (sizeof(outputColors)/3)) * 3];
                for (int j=0; j<3; j++) {
                    *target = *source;
                    ++target; ++source;
                }
            }
        #else
            #if !defined(MUTLI_WIDE)
                uint8_t *target = buffer;
            #else
                uint16_t *target = buffer;
            #endif
            for (int i=0; i<N_LEDS; i++) {
                const uint8_t *source = &outputColors[((i+offset) % 
                    (sizeof(outputColors)/3)) * 3];
                for (int j=0; j<3; j++) {
                    uint8_t bits = *source; ++source;
                    for (int k=0; k<8; bits <<= 1, k++) {
                        if (bits & 0x80) {
                            #if !defined(MUTLI_WIDE)
                                *target = 0xFF;
                            #else
                                *target = 0xFFFF;
                            #endif
                        } else {
                            *target = 0;
                        }
                        ++target;
                    }
                }
            }
        #endif
        
        offset = (offset+1)%8;
        
        WS2821_send(buffer, sizeof(buffer));
        
        uint32_t availableCyles = 0;
        while (WS2812_transmitting()) {
            availableCyles++;
        }
        /* Based on the asm my GCC generated, the above is is:
         *      (1+P) + 1 + (1+P) + 1 + 2 + 2 + 1 + 1 + 1 + (1+P)
         * where P=pipeline refill (1-3 cycles, we just assume 1) */
        availableCyles *= 15; 
        
        uint32_t elapsedCycles = (uint32_t)((24E6 / ((CLOCK) * 1.0)) * 
            (N_LEDS*8*3));
        if (availableCyles > elapsedCycles)
            availableCyles = elapsedCycles;
        elapsedCycles /= 1000;
        availableCyles /= elapsedCycles;
        
        outputString("Estimated available CPU: ");
        outputPercent(availableCyles, 1);
        outputString(" %\r\n");
    
        
        uint32_t begin = systick;
        while (wrapU32(begin, systick) < 1000) { }
    }

    return 0;
}
