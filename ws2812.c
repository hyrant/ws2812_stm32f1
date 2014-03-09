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

#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/gpio.h>

#include "ws2812.h"
#include "ws2812_config.h"

/*
 * PWM Mode:
 * This mode works by transfering a new value to the output compare register
 * for the selected output at every update event.  The value transferred comes
 * from a buffer of outputs that define the duty cycle for the bits it covers.
 * So each time the timer completes a cycle the DMA clocks in the next duty
 * cycle (the shadow register means this is actually one behind the one that's
 * currently being evaluated).
 * 
 * GPIO Modes:
 * These modes work by setting up three DMA transfers.  The first sets
 * the output(s) to high defining the rising edge of the clock.  The second
 * sets the output(s) low if the bit is zero at T0H.  The third sets the 
 * output(s) low regardless at T1H (no effect if they're already low).  A single
 * timer is used for all threee DMA transfers.  The first DMA (set high) occurs
 * at the update event if in use or a CCR set equal to the maximum otherwise.
 * The other two CCRs are set to the fractions of the clock cycle to give T0H
 * and T1H.  For single GPIO mode two intermediate buffers are used that
 * contain the values to be sent to the RSR for the GPIO, the other two are
 * from static 32-bit locations setting the bits in BSRR.
 * 
 * In PWM mode and single GPIO mode, the DMA buffer is split in half refilled
 * by an interrupt once that half completes.  The transfer itself is a circular
 * one with interrupts at the half and complete for the refilling.  For 
 * GPIO multiple mode, only a single long DMA transfer is needed.
 */

#ifndef CLOCK
	#define CLOCK   800000
#endif
#ifndef T0H
	#define T0H     0.32
#endif
#ifndef T1H
	#define T1H     0.64
#endif
#ifndef TRESET
    #define TRESET  50E-6
#endif

#if defined(PWM_MODE)
    #if defined(GPIO_MODE) || defined(MULTI_MODE)
        #error Only one mode can be selected
    #endif
    #if !defined(BUFFER_SIZE) || BUFFER_SIZE <= 0 || BUFFER_SIZE % 8 != 0
        #error Buffer size must be a multiple of 8
    #endif
    #if !defined(OUTPUT) || OUTPUT < 0 || OUTPUT > 4
        #error Invalid timer output
    #endif
#elif defined(GPIO_MODE)
    #if defined(MULTI_MODE)
        #error Only one mode can be selected
    #endif
    #if !defined(BUFFER_SIZE) || BUFFER_SIZE <= 0 || BUFFER_SIZE % 8 != 0
        #error Buffer size must be a multiple of 8
    #endif
#elif defined(MULTI_MODE)

#else
    #error Must select an output mode
#endif

static volatile enum {
    Uninitialized = 0,
    ResetBlank,
    Idle,
    SendingData,
} state = Uninitialized;

extern u32 rcc_ppre1_frequency;
extern u32 rcc_ppre2_frequency;

static uint8_t timerPeriod(void);
static uint8_t timerZeroHigh(void);
static uint8_t timerOneHigh(void);
static uint16_t timerResetPeriod(void);
static uint32_t timerInputFrequency(void);
static void setOutputsLow(void);
#if (TIMER) == 1
    #define USE_TIMER_DIER  TIM_DIER(TIM1)    
    static uint32_t timerInputFrequency(void)
    {
        return rcc_ppre2_frequency;
    }    
    static void timerInitialize(void)
    {
        rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);
        rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_TIM1RST);
		rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_TIM1RST);
        TIM_CR1(TIM1) = 0;
        TIM_SR(TIM1) = 0;
        
        nvic_clear_pending_irq(NVIC_TIM1_UP_IRQ);
        nvic_enable_irq(NVIC_TIM1_UP_IRQ);
        nvic_set_priority(NVIC_TIM1_UP_IRQ, 0);
        
        TIM_CR2(TIM1) = 0;
        TIM_RCR(TIM1) = 0;
        TIM_BDTR(TIM1) = TIM_BDTR_MOE;
        TIM_PSC(TIM1) = 0;        
        USE_TIMER_DIER = 0;
    }
    static void timerStartMain(void)
    {        
        TIM_CR1(TIM1) = 0;
        TIM_ARR(TIM1) = (uint16_t)timerPeriod() - 1;
        TIM_CNT(TIM1) = 0;
        TIM_SR(TIM1) = 0;
        TIM_CR1(TIM1) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    static void timerStartReset(void)
    {
        state = ResetBlank;
        TIM_CR1(TIM1) = 0;
        TIM_ARR(TIM1) = (uint16_t)timerResetPeriod();        
        TIM_CNT(TIM1) = 0;
        setOutputsLow();
        TIM_EGR(TIM1) = TIM_EGR_UG;
        TIM_SR(TIM1) = 0;
        USE_TIMER_DIER = TIM_DIER_UIE;
        TIM_CR1(TIM1) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    void tim1_up_isr(void) 
    {
        TIM_CR1(TIM1) = 0;
        USE_TIMER_DIER = 0;
        TIM_SR(TIM1) = 0;
        state = Idle;
    }
#elif (TIMER) == 2
    #define USE_TIMER_DIER  TIM_DIER(TIM2)    
    static uint32_t timerInputFrequency(void)
    {
        return rcc_ppre1_frequency;
    }    
    static void timerInitialize(void)
    {
        rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
        rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_TIM2RST);
		rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_TIM2RST);
        TIM_CR1(TIM2) = 0;
        TIM_SR(TIM2) = 0;
        
        nvic_clear_pending_irq(NVIC_TIM2_IRQ);
        nvic_enable_irq(NVIC_TIM2_IRQ);
        nvic_set_priority(NVIC_TIM2_IRQ, 0);
        
        TIM_CR2(TIM2) = 0;
        TIM_PSC(TIM2) = 0;
    }
    static void timerStartMain(void)
    {
        TIM_CR1(TIM2) = 0;
        TIM_ARR(TIM2) = (uint16_t)timerPeriod() - 1;
        TIM_CNT(TIM2) = 0;
        TIM_SR(TIM2) = 0;
        TIM_CR1(TIM2) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    static void timerStartReset(void)
    {        
        state = ResetBlank;
        TIM_CR1(TIM2) = 0;
        TIM_ARR(TIM2) = (uint16_t)timerResetPeriod();
        TIM_CNT(TIM2) = 0;
        setOutputsLow();
        TIM_EGR(TIM2) = TIM_EGR_UG;
        TIM_SR(TIM2) = 0;
        USE_TIMER_DIER = TIM_DIER_UIE;
        TIM_CR1(TIM2) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    void tim2_isr(void) 
    {
        TIM_CR1(TIM2) = 0;
        USE_TIMER_DIER = 0;
        TIM_SR(TIM2) = 0;
        state = Idle;
    }
#elif (TIMER) == 3
    #define USE_TIMER_DIER  TIM_DIER(TIM3)    
    static uint32_t timerInputFrequency(void)
    {
        return rcc_ppre1_frequency;
    }    
    static void timerInitialize(void)
    {
        rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);
        rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_TIM3RST);
		rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_TIM3RST);
        TIM_CR1(TIM3) = 0;
        TIM_SR(TIM3) = 0;
        
        nvic_clear_pending_irq(NVIC_TIM3_IRQ);
        nvic_enable_irq(NVIC_TIM3_IRQ);
        nvic_set_priority(NVIC_TIM3_IRQ, 0);
        
        TIM_CR2(TIM3) = 0;
        TIM_PSC(TIM3) = 0;
    }
    static void timerStartMain(void)
    {
        TIM_CR1(TIM3) = 0;
        TIM_ARR(TIM3) = (uint16_t)timerPeriod() - 1;
        TIM_CNT(TIM3) = 0;
        TIM_SR(TIM3) = 0;
        TIM_CR1(TIM3) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    static void timerStartReset(void)
    {
        state = ResetBlank;
        TIM_CR1(TIM3) = 0;
        TIM_ARR(TIM3) = (uint16_t)timerResetPeriod();
        TIM_CNT(TIM3) = 0;        
        setOutputsLow();
        TIM_EGR(TIM3) = TIM_EGR_UG;
        TIM_SR(TIM3) = 0;
        USE_TIMER_DIER = TIM_DIER_UIE;
        TIM_CR1(TIM3) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    void tim3_isr(void) 
    {
        TIM_CR1(TIM3) = 0;
        USE_TIMER_DIER = 0;
        TIM_SR(TIM3) = 0;
        state = Idle;
    }
#elif (TIMER) == 4
    #define USE_TIMER_DIER  TIM_DIER(TIM4)    
    static uint32_t timerInputFrequency(void)
    {
        return rcc_ppre1_frequency;
    }    
    static void timerInitialize(void)
    {
        rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);
        rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_TIM4RST);
		rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_TIM4RST);
        TIM_CR1(TIM4) = 0;
        TIM_SR(TIM4) = 0;
        
        nvic_clear_pending_irq(NVIC_TIM4_IRQ);
        nvic_enable_irq(NVIC_TIM4_IRQ);
        nvic_set_priority(NVIC_TIM4_IRQ, 0);
        
        TIM_CR2(TIM4) = 0;
        TIM_PSC(TIM4) = 0;
    }
    static void timerStartMain(void)
    {        
        TIM_CR1(TIM4) = 0;
        TIM_ARR(TIM4) = (uint16_t)timerPeriod() - 1;
        TIM_CNT(TIM4) = 0;
        TIM_SR(TIM4) = 0;
        TIM_CR1(TIM4) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    static void timerStartReset(void)
    {
        TIM_CR1(TIM4) = 0;
        TIM_ARR(TIM4) = (uint16_t)timerResetPeriod();
        TIM_CNT(TIM4) = 0;        
        setOutputsLow();
        TIM_EGR(TIM4) = TIM_EGR_UG;
        TIM_SR(TIM4) = 0;
        USE_TIMER_DIER = TIM_DIER_UIE;
        TIM_CR1(TIM4) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    void tim4_isr(void) 
    {
        TIM_CR1(TIM4) = 0;
        USE_TIMER_DIER = 0;
        TIM_SR(TIM4) = 0;
        dmaDisable();
        state = Idle;
    }
#elif (TIMER) == 5
    #define USE_TIMER_DIER  TIM_DIER(TIM5)    
    static uint32_t timerInputFrequency(void)
    {
        return rcc_ppre1_frequency;
    }    
    static void timerInitialize(void)
    {
        rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM5EN);
        rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_TIM5RST);
		rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_TIM5RST);
        TIM_CR1(TIM5) = 0;
        TIM_SR(TIM5) = 0;
        
        nvic_clear_pending_irq(NVIC_TIM5_IRQ);
        nvic_enable_irq(NVIC_TIM5_IRQ);
        nvic_set_priority(NVIC_TIM5_IRQ, 0);
        
        TIM_CR2(TIM5) = 0;
        TIM_PSC(TIM5) = 0;
    }
    static void timerStartMain(void)
    {        
        TIM_CR1(TIM5) = 0;
        TIM_ARR(TIM5) = (uint16_t)timerPeriod() - 1;
        TIM_CNT(TIM5) = 0;
        TIM_SR(TIM5) = 0;
        TIM_CR1(TIM5) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    static void timerStartReset(void)
    {
        state = ResetBlank;
        TIM_CR1(TIM5) = 0;
        TIM_ARR(TIM5) = (uint16_t)timerResetPeriod();
        TIM_CNT(TIM5) = 0;
        setOutputsLow();
        TIM_EGR(TIM5) = TIM_EGR_UG;
        TIM_SR(TIM5) = 0;
        USE_TIMER_DIER = TIM_DIER_UIE;
        TIM_CR1(TIM5) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    void tim5_isr(void) 
    {
        TIM_CR1(TIM5) = 0;
        USE_TIMER_DIER = 0;
        TIM_SR(TIM5) = 0;
        state = Idle;
    }
#elif (TIMER) == 8
    #define USE_TIMER_DIER  TIM_DIER(TIM8)    
    static uint32_t timerInputFrequency(void)
    {
        return rcc_ppre2_frequency;
    }    
    static void timerInitialize(void)
    {
        rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM8EN);
        rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_TIM8RST);
		rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_TIM8RST);
        TIM_CR1(TIM8) = 0;
        TIM_SR(TIM8) = 0;
        
        nvic_clear_pending_irq(NVIC_TIM8_UP_IRQ);
        nvic_enable_irq(NVIC_TIM8_UP_IRQ);
        nvic_set_priority(NVIC_TIM8_UP_IRQ, 0);
        
        TIM_CR2(TIM8) = 0;
        TIM_RCR(TIM8) = 0;
        TIM_BDTR(TIM8) = TIM_BDTR_MOE;
        TIM_PSC(TIM8) = 0;
    }
    static void timerStartMain(void)
    {        
        TIM_CR1(TIM8) = 0;
        TIM_ARR(TIM8) = (uint16_t)timerPeriod() - 1;
        TIM_CNT(TIM8) = 0;
        TIM_SR(TIM8) = 0;
        TIM_CR1(TIM8) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    static void timerStartReset(void)
    {
        state = ResetBlank;
        TIM_CR1(TIM8) = 0;
        TIM_ARR(TIM8) = (uint16_t)timerResetPeriod();
        TIM_CNT(TIM8) = 0;        
        setOutputsLow();
        TIM_EGR(TIM8) = TIM_EGR_UG;
        TIM_SR(TIM8) = 0;
        USE_TIMER_DIER = TIM_DIER_UIE;
        TIM_CR1(TIM8) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | 
            TIM_CR1_DIR_UP | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
    }
    void tim8_up_isr(void) 
    {
        TIM_CR1(TIM8) = 0;
        USE_TIMER_DIER = 0;
        TIM_SR(TIM8) = 0;
        state = Idle;
    }
#else
    #error Unsupported timer
#endif
static uint8_t timerPeriod(void)
{
    return (uint16_t)((timerInputFrequency() + (uint32_t)((CLOCK)/2)) / 
        (uint32_t)(CLOCK));
}
static uint8_t timerZeroHigh(void)
{
    uint32_t high = (uint32_t)((T0H) * (CLOCK) + 0.5);
    return (uint8_t)((high * (uint32_t)timerPeriod() + (uint32_t)((CLOCK)/2)) / 
        (uint32_t)(CLOCK));
}
static uint8_t timerOneHigh(void)
{
    uint32_t high = (uint32_t)((T1H) * (CLOCK) + 0.5);
    return (uint8_t)((high * (uint32_t)timerPeriod() + 
        (uint32_t)((CLOCK)/2)) / (uint32_t)(CLOCK));
}
static uint16_t timerResetPeriod(void)
{
    return (uint16_t)(timerInputFrequency() / 
        (uint32_t)(1.0/(TRESET) + 0.5)) + 2;
}

#if defined(PWM_MODE) || defined(GPIO_MODE)
    static bool bufferHalf;
    static const uint8_t *sendBegin;
    static const uint8_t *sendEnd;
    
    #if defined(PWM_MODE) || !defined(GPIOANY_STRICT)
        static uint8_t dmaBuffer[(BUFFER_SIZE)];
        static void fillNextBuffer(uint8_t high, uint8_t low)
        {
            uint8_t *target;
            if (bufferHalf) {
                target = &dmaBuffer[(BUFFER_SIZE)/2];
            } else {
                target = &dmaBuffer[0];
            }
            bufferHalf = !bufferHalf;
            for (int total = 0; total < (BUFFER_SIZE)/(8*2) &&
                    sendBegin != sendEnd; ++sendBegin, ++total) {
                uint8_t source = *sendBegin;
                for (int bit=0; bit<8; bit++, source <<= 1, ++target) {
                    if (source & 0x80) {
                        *target = high;
                    } else {
                        *target = low;
                    }
                }
            }
        }
    #else
        static uint16_t dmaBuffer[(BUFFER_SIZE)];
        static void fillNextBuffer(void)
        {
            uint16_t *target;
            if (bufferHalf) {
                target = &dmaBuffer[(BUFFER_SIZE)/2];
            } else {
                target = &dmaBuffer[0];
            }
            bufferHalf = !bufferHalf;
            for (int total = 0; total < (BUFFER_SIZE)/(8*2) &&
                    sendBegin != sendEnd; ++sendBegin, ++total) {
                uint8_t source = *sendBegin;
                for (int bit=0; bit<8; bit++, source <<= 1, ++target) {
                    if (source & 0x80) {
                        *target = 0;
                    } else {
                        *target = 1 << (OUTPUT);
                    }
                }
            }
        }
    #endif
    
    #if defined(GPIO_MODE)
        #if !defined(GPIOANY_STRICT)
            static const uint32_t outputHigh = (1 << (OUTPUT)) | 
                (1 << (((OUTPUT)+8)%16));
            static const uint32_t outputLow = (1 << ((OUTPUT)+16)) | 
                (1 << (((OUTPUT)+8)%16+16));
        #else
            static const uint32_t outputHigh = 1 << (OUTPUT);
            static const uint32_t outputLow = 1 << ((OUTPUT) + 16);
        #endif
    #endif
#endif


#if defined(PWM_MODE)
    /* This is circular, memory incrementing transfer to peripheral (8->16 bit
     * pads the high order bits with zeros, which we want), with interrupts
     * at the half and end of the cycle (we refill the half that just 
     * completed). */
    #define DMA_CCR_WRITE_UP    (DMA_CCR_PL_HIGH | DMA_CCR_MSIZE_8BIT | \
            DMA_CCR_PSIZE_16BIT | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR | \
            DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_EN)
            
    #if (OUTPUT) == 1
        #define USE_TIMER_CCMRV (TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE)
        #if defined(OUTPUT_INVERT) && ((TIMER) == 1 || (TIMER) == 8)
            #define USE_TIMER_CCERV (TIM_CCER_CC1NE)
        #else
            #define USE_TIMER_CCERV (TIM_CCER_CC1E)
        #endif
    #elif (OUTPUT) == 2
        #define USE_TIMER_CCMRV (TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE)
        #if defined(OUTPUT_INVERT) && ((TIMER) == 1 || (TIMER) == 8)
            #define USE_TIMER_CCERV (TIM_CCER_CC1NE)
        #else
            #define USE_TIMER_CCERV (TIM_CCER_CC2E)            
        #endif
    #elif (OUTPUT) == 3
        #define USE_TIMER_CCMRV (TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE)
        #if defined(OUTPUT_INVERT) && ((TIMER) == 1 || (TIMER) == 8)
            #define USE_TIMER_CCERV (TIM_CCER_CC3NE)
        #else
            #define USE_TIMER_CCERV (TIM_CCER_CC3E)            
        #endif
    #elif (OUTPUT) == 4
        #define USE_TIMER_CCMRV (TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE)
        #if defined(OUTPUT_INVERT) && ((TIMER) == 1 || (TIMER) == 8)
            #define USE_TIMER_CCERV (TIM_CCER_CC3NE)
        #else            
            #define USE_TIMER_CCERV (TIM_CCER_CC4E)
        #endif
    #else
        #error Invalid output
    #endif
            
    #if (TIMER) == 1
        #define USE_DMA_IFCR    DMA_IFCR(DMA1)
        #define USE_DMA_CCR     DMA_CCR(DMA1, DMA_CHANNEL5)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(DMA_CHANNEL5)
        #define DMA_IRQ_NAME    dma1_channel5_isr
        
        #if OUTPUT == 1
            #define USE_TIMER_CCR   TIM_CCR1(TIM1)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM1)
        #elif OUTPUT == 2
            #define USE_TIMER_CCR   TIM_CCR2(TIM1)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM1)
        #elif OUTPUT == 3
            #define USE_TIMER_CCR   TIM_CCR3(TIM1)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM1)
        #elif OUTPUT == 4
            #define USE_TIMER_CCR   TIM_CCR4(TIM1)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM1)
        #endif
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
            
            TIM_CCER(TIM1) = USE_TIMER_CCERV;
            USE_TIMER_CCMR = USE_TIMER_CCMRV;
            
            USE_DMA_CCR = 0;
            DMA_CNDTR(DMA1, DMA_CHANNEL5) = BUFFER_SIZE;
            DMA_CPAR(DMA1, DMA_CHANNEL5) = (uint32_t)(&USE_TIMER_CCR);
            DMA_CMAR(DMA1, DMA_CHANNEL5) = (uint32_t)(&dmaBuffer[0]);
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            nvic_clear_pending_irq(NVIC_DMA1_CHANNEL5_IRQ);
            nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);
            nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, 0);
        }                 
    #elif (TIMER) == 2
        #define USE_DMA_IFCR    DMA_IFCR(DMA1)
        #define USE_DMA_CCR     DMA_CCR(DMA1, DMA_CHANNEL2)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(DMA_CHANNEL2)
        #define DMA_IRQ_NAME    dma1_channel2_isr
        
        #if OUTPUT == 1
            #define USE_TIMER_CCR   TIM_CCR1(TIM2)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM2)
        #elif OUTPUT == 2
            #define USE_TIMER_CCR   TIM_CCR2(TIM2)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM2)
        #elif OUTPUT == 3
            #define USE_TIMER_CCR   TIM_CCR3(TIM2)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM2)
        #elif OUTPUT == 4
            #define USE_TIMER_CCR   TIM_CCR4(TIM2)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM2)
        #endif
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
            
            TIM_CCER(TIM2) = USE_TIMER_CCERV;
            USE_TIMER_CCMR = USE_TIMER_CCMRV;
            
            USE_DMA_CCR = 0;
            DMA_CNDTR(DMA1, DMA_CHANNEL2) = BUFFER_SIZE;
            DMA_CPAR(DMA1, DMA_CHANNEL2) = (uint32_t)(&USE_TIMER_CCR);
            DMA_CMAR(DMA1, DMA_CHANNEL2) = (uint32_t)(&dmaBuffer[0]);
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            nvic_clear_pending_irq(NVIC_DMA1_CHANNEL2_IRQ);
            nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ);
            nvic_set_priority(NVIC_DMA1_CHANNEL2_IRQ, 0);
        } 
    #elif (TIMER) == 3
        #define USE_DMA_IFCR    DMA_IFCR(DMA1)
        #define USE_DMA_CCR     DMA_CCR(DMA1, DMA_CHANNEL3)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(DMA_CHANNEL3)
        #define DMA_IRQ_NAME    dma1_channel3_isr
        
        #if OUTPUT == 1
            #define USE_TIMER_CCR   TIM_CCR1(TIM3)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM3)
        #elif OUTPUT == 2
            #define USE_TIMER_CCR   TIM_CCR2(TIM3)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM3)
        #elif OUTPUT == 3
            #define USE_TIMER_CCR   TIM_CCR3(TIM3)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM3)
        #elif OUTPUT == 4
            #define USE_TIMER_CCR   TIM_CCR4(TIM3)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM3)
        #endif
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
            
            TIM_CCER(TIM3) = USE_TIMER_CCERV;
            USE_TIMER_CCMR = USE_TIMER_CCMRV;
            
            USE_DMA_CCR = 0;
            DMA_CNDTR(DMA1, DMA_CHANNEL3) = BUFFER_SIZE;
            DMA_CPAR(DMA1, DMA_CHANNEL3) = (uint32_t)(&USE_TIMER_CCR);
            DMA_CMAR(DMA1, DMA_CHANNEL3) = (uint32_t)(&dmaBuffer[0]);
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            nvic_clear_pending_irq(NVIC_DMA1_CHANNEL3_IRQ);
            nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
            nvic_set_priority(NVIC_DMA1_CHANNEL3_IRQ, 0);
        } 
    #elif (TIMER) == 4
        #define USE_DMA_IFCR    DMA_IFCR(DMA1)
        #define USE_DMA_CCR     DMA_CCR(DMA1, DMA_CHANNEL7)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(DMA_CHANNEL7)
        #define DMA_IRQ_NAME    dma1_channel7_isr
        
        #if OUTPUT == 1
            #define USE_TIMER_CCR   TIM_CCR1(TIM4)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM4)
        #elif OUTPUT == 2
            #define USE_TIMER_CCR   TIM_CCR2(TIM4)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM4)
        #elif OUTPUT == 3
            #define USE_TIMER_CCR   TIM_CCR3(TIM4)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM4)
        #elif OUTPUT == 4
            #define USE_TIMER_CCR   TIM_CCR4(TIM4)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM4)
        #endif
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
            
            TIM_CCER(TIM4) = USE_TIMER_CCERV;
            USE_TIMER_CCMR = USE_TIMER_CCMRV;
            
            USE_DMA_CCR = 0;
            DMA_CNDTR(DMA1, DMA_CHANNEL7) = BUFFER_SIZE;
            DMA_CPAR(DMA1, DMA_CHANNEL7) = (uint32_t)(&USE_TIMER_CCR);
            DMA_CMAR(DMA1, DMA_CHANNEL7) = (uint32_t)(&dmaBuffer[0]);
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            nvic_clear_pending_irq(NVIC_DMA1_CHANNEL7_IRQ);
            nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);
            nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 0);
        } 
    #elif (TIMER) == 5
        #define USE_DMA_IFCR    DMA_IFCR(DMA2)
        #define USE_DMA_CCR     DMA_CCR(DMA2, DMA_CHANNEL2)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(DMA_CHANNEL2)
        #define DMA_IRQ_NAME    dma2_channel2_isr
        
        #if OUTPUT == 1
            #define USE_TIMER_CCR   TIM_CCR1(TIM5)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM5)
        #elif OUTPUT == 2
            #define USE_TIMER_CCR   TIM_CCR2(TIM5)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM5)
        #elif OUTPUT == 3
            #define USE_TIMER_CCR   TIM_CCR3(TIM5)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM5)
        #elif OUTPUT == 4
            #define USE_TIMER_CCR   TIM_CCR4(TIM5)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM5)
        #endif
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA2EN);
            
            TIM_CCER(TIM5) = USE_TIMER_CCERV;
            USE_TIMER_CCMR = USE_TIMER_CCMRV;
            
            USE_DMA_CCR = 0;
            DMA_CNDTR(DMA2, DMA_CHANNEL2) = BUFFER_SIZE;
            DMA_CPAR(DMA2, DMA_CHANNEL2) = (uint32_t)(&USE_TIMER_CCR);
            DMA_CMAR(DMA2, DMA_CHANNEL2) = (uint32_t)(&dmaBuffer[0]);
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            nvic_clear_pending_irq(NVIC_DMA2_CHANNEL2_IRQ);
            nvic_enable_irq(NVIC_DMA2_CHANNEL2_IRQ);
            nvic_set_priority(NVIC_DMA2_CHANNEL2_IRQ, 0);
        } 
    #elif (TIMER) == 8
        #define USE_DMA_IFCR    DMA_IFCR(DMA2)
        #define USE_DMA_CCR     DMA_CCR(DMA2, DMA_CHANNEL1)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(DMA_CHANNEL1)
        #define DMA_IRQ_NAME    dma2_channel1_isr
        
        #if OUTPUT == 1
            #define USE_TIMER_CCR   TIM_CCR1(TIM8)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM8)
        #elif OUTPUT == 2
            #define USE_TIMER_CCR   TIM_CCR2(TIM8)
            #define USE_TIMER_CCMR  TIM_CCMR1(TIM8)
        #elif OUTPUT == 3
            #define USE_TIMER_CCR   TIM_CCR3(TIM8)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM8)
        #elif OUTPUT == 4
            #define USE_TIMER_CCR   TIM_CCR4(TIM8)
            #define USE_TIMER_CCMR  TIM_CCMR2(TIM8)
        #endif
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA2EN);
            
            TIM_CCER(TIM8) = USE_TIMER_CCERV;
            USE_TIMER_CCMR = USE_TIMER_CCMRV;
            
            USE_DMA_CCR = 0;
            DMA_CNDTR(DMA2, DMA_CHANNEL1) = BUFFER_SIZE;
            DMA_CPAR(DMA2, DMA_CHANNEL1) = (uint32_t)(&USE_TIMER_CCR);
            DMA_CMAR(DMA2, DMA_CHANNEL1) = (uint32_t)(&dmaBuffer[0]);
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            nvic_clear_pending_irq(NVIC_DMA2_CHANNEL1_IRQ);
            nvic_enable_irq(NVIC_DMA2_CHANNEL1_IRQ);
            nvic_set_priority(NVIC_DMA2_CHANNEL1_IRQ, 0);
        } 
    #else
        #error Unsupported timer
    #endif
    
    static void setOutputsLow(void)
    {
        USE_TIMER_CCR = 0;
    }
    static void dmaDisable(void)
    {
        USE_DMA_CCR = 0;
        setOutputsLow();
    }    
    static void modeInitialize(const void *data, uint16_t length)
    {    
        /* Initialize both buffers and start a send (the request won't
         * actually come in until the timer is started). */
        sendBegin = (const uint8_t *)data;
        sendEnd = sendBegin + length;        
        bufferHalf = false;
        fillNextBuffer(timerOneHigh(), timerZeroHigh());
        fillNextBuffer(timerOneHigh(), timerZeroHigh());
        
        /* Enable the DMA request at the update event */
        USE_DMA_CCR = DMA_CCR_WRITE_UP;
        USE_TIMER_DIER = TIM_DIER_UDE;
        
        /* This means that output won't be updated until the second
         * clock cycle (since the DMA request won't come in until the
         * update at the end of the first, and the register itself won't
         * take the shadow value until the one after that).  This is fine,
         * however, as we just hold it low and this gives us plenty of
         * time to finish up handling. */
        USE_TIMER_CCR = 0;
    }
#endif

#ifdef MULTI_MODE
    static const uint32_t outputHigh = 0x000FFFF;
    static const uint32_t outputLow = 0xFFFF0000;
#endif

#if defined(GPIO_MODE) || defined(MULTI_MODE)
    /* This is circular in-place contentious transfer to a peripheral with
     * the full word write.  That is, this just writes the memory word every
     * DMA request and does nothing else. */
    #define DMA_CCR_WRITE_FIXED (DMA_CCR_PL_HIGH | DMA_CCR_MSIZE_32BIT | \
            DMA_CCR_PSIZE_32BIT | DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_EN)
            
    /* We rely on the fact that if both set and reset are set in
     * BSRR then set takes priority.  So the fact that the DMA
     * will duplicate the data lines (BSRR is word access only) still
     * works.  For non-strict DMA mode, the same is true for BRR, we just
     * write zeros when we're on a high bit (so no action taken). */
    #if defined(GPIO_MODE)
        #if !defined(GPIO_STRICT)
            #define DMA_WRITE_TL_MODE   (DMA_CCR_MSIZE_8BIT | \
                DMA_CCR_PSIZE_8BIT | DMA_CCR_HTIE)
        #else
            #define DMA_WRITE_TL_MODE   (DMA_CCR_MSIZE_16BIT | \
                DMA_CCR_PSIZE_16BIT | DMA_CCR_HTIE)
        #endif
    #elif defined(MULTI_MODE)
        #if !defined(MUTLI_WIDE)
            #define DMA_WRITE_TL_MODE   (DMA_CCR_MSIZE_8BIT | \
                DMA_CCR_PSIZE_8BIT)
        #else
            #define DMA_WRITE_TL_MODE   (DMA_CCR_MSIZE_16BIT | \
                DMA_CCR_PSIZE_16BIT)
        #endif
    #endif
    
    /* This is circular, memory incrementing transfer to peripheral.  For
     * GPIO mode there are interrupts at the half and end of the transfer that
     * we use to refill the half that just completed.  For multiple out mode
     * there's only an interrupt at the end, which just starts our reset
     * time. */
    #define DMA_CCR_WRITE_TL    (DMA_CCR_PL_HIGH | DMA_WRITE_TL_MODE | \
            DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_TCIE | \
            DMA_CCR_EN)
            
            
    #if (TIMER) == 1
        #if (DMAC1) == (DMAC2) || (DMAC2) == (DMAC3) || (DMAC3) == (DMAC1)
            #error Three different DMA channels must be selected
        #endif
        #if (DMAC1) != 2 && (DMAC1) != 3 && (DMAC1) != 4 && (DMAC1) != 5 && \
                (DMAC1) != 6
            #error DMA selection 1 does not belong to timer 1
        #endif
        #if (DMAC2) != 2 && (DMAC2) != 3 && (DMAC2) != 4 && (DMAC2) != 5 && \
                (DMAC2) != 6
            #error DMA selection 2 does not belong to timer 1
        #endif
        #if (DMAC3) != 2 && (DMAC3) != 3 && (DMAC3) != 4 && (DMAC3) != 5 && \
                (DMAC3) != 6
            #error DMA selection 3 does not belong to timer 1
        #endif
        
        #if (DMAC1) == 5
            #define SET_DMA_CHANNEL DMA_CHANNEL5
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC2
            #define DMACCLR DMAC3
        #elif (DMAC2) == 5
            #define SET_DMA_CHANNEL DMA_CHANNEL5
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC1
            #define DMACCLR DMAC3
        #elif (DMAC3) == 5
            #define SET_DMA_CHANNEL DMA_CHANNEL5
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC1
            #define DMACCLR DMAC2
        #else
            #define DMACSET DMAC1
            #define DMACUSE DMAC2
            #define DMACCLR DMAC3
        #endif
        
        #if defined(DMACSET)
            #if (DMACSET) == 2
                #define SET_DMA_CHANNEL DMA_CHANNEL2
                #define SET_TIMER_CCR   TIM_CCR1(TIM1)
                #define SET_TIMER_DMA   TIM_DIER_CC1DE
            #elif (DMACSET) == 3
                #define SET_DMA_CHANNEL DMA_CHANNEL3
                #define SET_TIMER_CCR   TIM_CCR2(TIM1)
                #define SET_TIMER_DMA   TIM_DIER_CC2DE
            #elif (DMACSET) == 4
                #define SET_DMA_CHANNEL DMA_CHANNEL4
                #define SET_TIMER_CCR   TIM_CCR4(TIM1)
                #define SET_TIMER_DMA   TIM_DIER_CC4DE
            #elif (DMACSET) == 6
                #define SET_DMA_CHANNEL DMA_CHANNEL6
                #define SET_TIMER_CCR   TIM_CCR3(TIM1)
                #define SET_TIMER_DMA   TIM_DIER_CC3DE
            #else
                #error Invalid set DMA channel
            #endif
        #endif
        #if (DMACCLR) == 2
            #define CLR_DMA_CHANNEL DMA_CHANNEL2
            #define CLR_TIMER_CCR   TIM_CCR1(TIM1)
            #define CLR_TIMER_DMA   TIM_DIER_CC1DE
        #elif (DMACCLR) == 3
            #define CLR_DMA_CHANNEL DMA_CHANNEL3
            #define CLR_TIMER_CCR   TIM_CCR2(TIM1)
            #define CLR_TIMER_DMA   TIM_DIER_CC2DE
        #elif (DMACCLR) == 4
            #define CLR_DMA_CHANNEL DMA_CHANNEL4
            #define CLR_TIMER_CCR   TIM_CCR4(TIM1)
            #define CLR_TIMER_DMA   TIM_DIER_CC4DE
        #elif (DMACCLR) == 6
            #define CLR_DMA_CHANNEL DMA_CHANNEL6
            #define CLR_TIMER_CCR   TIM_CCR3(TIM1)
            #define CLR_TIMER_DMA   TIM_DIER_CC3DE
        #else
            #error Invalid clear DMA channel
        #endif
        #if (DMACUSE) == 2
            #define DMA_IRQ_NAME    dma1_channel2_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL2
            #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL2_IRQ
            #define USE_TIMER_CCR   TIM_CCR1(TIM1)
            #define USE_TIMER_DMA   TIM_DIER_CC1DE
        #elif (DMACUSE) == 3
            #define DMA_IRQ_NAME    dma1_channel3_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL3
            #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL3_IRQ
            #define USE_TIMER_CCR   TIM_CCR2(TIM1)
            #define USE_TIMER_DMA   TIM_DIER_CC2DE
        #elif (DMACUSE) == 4
            #define DMA_IRQ_NAME    dma1_channel4_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL4
            #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL4_IRQ
            #define USE_TIMER_CCR   TIM_CCR4(TIM1)
            #define USE_TIMER_DMA   TIM_DIER_CC4DE
        #elif (DMACUSE) == 6
            #define DMA_IRQ_NAME    dma1_channel6_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL6
            #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL6_IRQ
            #define USE_TIMER_CCR   TIM_CCR3(TIM1)
            #define USE_TIMER_DMA   TIM_DIER_CC3DE
        #else
            #error Invalid use DMA channel
        #endif
        
        #define USE_DMA_IFCR    DMA_IFCR(DMA1)
        #define USE_DMA_CCR     DMA_CCR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CNDTR   DMA_CNDTR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CMAR    DMA_CMAR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(USE_DMA_CHANNEL)
        #define SET_DMA_CCR     DMA_CCR(DMA1, SET_DMA_CHANNEL)
        #define CLR_DMA_CCR     DMA_CCR(DMA1, CLR_DMA_CHANNEL)
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
            
            USE_DMA_CCR = 0;
            #if defined(GPIO_MODE)
                USE_DMA_CNDTR = BUFFER_SIZE;
                USE_DMA_CMAR = (uint32_t)(&dmaBuffer[0]);
                DMA_CPAR(DMA1, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_BRR(PORT));
            #else            
                DMA_CPAR(DMA1, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_ODR(PORT));
            #endif
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            DMA_CNDTR(DMA1, SET_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA1, SET_DMA_CHANNEL) = (uint32_t)(&outputHigh);
            DMA_CPAR(DMA1, SET_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            SET_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            DMA_CNDTR(DMA1, CLR_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA1, CLR_DMA_CHANNEL) = (uint32_t)(&outputLow);
            DMA_CPAR(DMA1, CLR_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            CLR_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            nvic_clear_pending_irq(USE_DMA_IRQ);
            nvic_enable_irq(USE_DMA_IRQ);
            nvic_set_priority(USE_DMA_IRQ, 0);
        }
    #elif (TIMER) == 2
        #if (DMAC1) == (DMAC2) || (DMAC2) == (DMAC3) || (DMAC3) == (DMAC1)
            #error Three different DMA channels must be selected
        #endif
        #if (DMAC1) != 1 && (DMAC1) != 2 && (DMAC1) != 5 && (DMAC1) != 7
            #error DMA selection 1 does not belong to timer 2
        #endif
        #if (DMAC2) != 1 && (DMAC2) != 2 && (DMAC2) != 5 && (DMAC2) != 7
            #error DMA selection 2 does not belong to timer 2
        #endif
        #if (DMAC3) != 1 && (DMAC3) != 2 && (DMAC3) != 5 && (DMAC3) != 7
            #error DMA selection 3 does not belong to timer 2
        #endif
        
        #if (DMAC1) == 2
            #define SET_DMA_CHANNEL DMA_CHANNEL2
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC2
            #define DMACCLR DMAC3
        #elif (DMAC2) == 2
            #define SET_DMA_CHANNEL DMA_CHANNEL2
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC1
            #define DMACCLR DMAC3
        #elif (DMAC3) == 2
            #define SET_DMA_CHANNEL DMA_CHANNEL2
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC1
            #define DMACCLR DMAC2
        #else
            #define DMACSET DMAC1
            #define DMACUSE DMAC2
            #define DMACCLR DMAC3
        #endif
        
        #if defined(DMACSET)
            #if (DMACSET) == 1
                #define SET_DMA_CHANNEL DMA_CHANNEL1
                #define SET_TIMER_CCR   TIM_CCR3(TIM2)
                #define SET_TIMER_DMA   TIM_DIER_CC3DE
            #elif (DMACSET) == 5
                #define SET_DMA_CHANNEL DMA_CHANNEL5
                #define SET_TIMER_CCR   TIM_CCR1(TIM2)
                #define SET_TIMER_DMA   TIM_DIER_CC1DE
            #elif (DMACSET) == 7
                #define SET_DMA_CHANNEL DMA_CHANNEL7
                #define SET_TIMER_CCR   TIM_CCR2(TIM2)
                #define SET_TIMER_DMA   TIM_DIER_CC2DE
            #else
                #error Invalid set DMA channel
            #endif
        #endif
        #if (DMACCLR) == 1
            #define CLR_DMA_CHANNEL DMA_CHANNEL1
            #define CLR_TIMER_CCR   TIM_CCR3(TIM2)
            #define CLR_TIMER_DMA   TIM_DIER_CC3DE
        #elif (DMACCLR) == 5
            #define CLR_DMA_CHANNEL DMA_CHANNEL5
            #define CLR_TIMER_CCR   TIM_CCR1(TIM2)
            #define CLR_TIMER_DMA   TIM_DIER_CC1DE
        #elif (DMACCLR) == 7
            #define CLR_DMA_CHANNEL DMA_CHANNEL7
            #define CLR_TIMER_CCR   TIM_CCR2(TIM2)
            #define CLR_TIMER_DMA   TIM_DIER_CC2DE
        #else
            #error Invalid clear DMA channel
        #endif
        #if (DMACUSE) == 1
            #define DMA_IRQ_NAME    dma1_channel1_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL1
            #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL1_IRQ
            #define USE_TIMER_CCR   TIM_CCR3(TIM2)
            #define USE_TIMER_DMA   TIM_DIER_CC3DE
        #elif (DMACUSE) == 5
            #define DMA_IRQ_NAME    dma1_channel5_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL5
            #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL5_IRQ
            #define USE_TIMER_CCR   TIM_CCR1(TIM2)
            #define USE_TIMER_DMA   TIM_DIER_CC1DE
        #elif (DMACUSE) == 7
            #define DMA_IRQ_NAME    dma1_channel7_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL7
            #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL7_IRQ
            #define USE_TIMER_CCR   TIM_CCR2(TIM2)
            #define USE_TIMER_DMA   TIM_DIER_CC2DE
        #else
            #error Invalid use DMA channel
        #endif
        
        #define USE_DMA_IFCR    DMA_IFCR(DMA1)
        #define USE_DMA_CCR     DMA_CCR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CNDTR   DMA_CNDTR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CMAR    DMA_CMAR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(USE_DMA_CHANNEL)
        #define SET_DMA_CCR     DMA_CCR(DMA1, SET_DMA_CHANNEL)
        #define CLR_DMA_CCR     DMA_CCR(DMA1, CLR_DMA_CHANNEL)
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
            
            USE_DMA_CCR = 0;
            #if defined(GPIO_MODE)
                USE_DMA_CNDTR = BUFFER_SIZE;
                USE_DMA_CMAR = (uint32_t)(&dmaBuffer[0]);
                DMA_CPAR(DMA1, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_BRR(PORT));
            #else            
                DMA_CPAR(DMA1, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_ODR(PORT));
            #endif
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            DMA_CNDTR(DMA1, SET_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA1, SET_DMA_CHANNEL) = (uint32_t)(&outputHigh);
            DMA_CPAR(DMA1, SET_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            SET_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            DMA_CNDTR(DMA1, CLR_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA1, CLR_DMA_CHANNEL) = (uint32_t)(&outputLow);
            DMA_CPAR(DMA1, CLR_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            CLR_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            nvic_clear_pending_irq(USE_DMA_IRQ);
            nvic_enable_irq(USE_DMA_IRQ);
            nvic_set_priority(USE_DMA_IRQ, 0);
        }
    #elif (TIMER) == 3
        #define SET_DMA_CHANNEL DMA_CHANNEL3
        #define SET_TIMER_DMA   TIM_DIER_UDE
        #undef SET_TIMER_CCR
        
        #define CLR_DMA_CHANNEL DMA_CHANNEL6
        #define CLR_TIMER_CCR   TIM_CCR1(TIM3)
        #define CLR_TIMER_DMA   TIM_DIER_CC1DE

        #define DMA_IRQ_NAME    dma1_channel2_isr
        #define USE_DMA_CHANNEL DMA_CHANNEL2
        #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL2_IRQ
        #define USE_TIMER_CCR   TIM_CCR3(TIM3)
        #define USE_TIMER_DMA   TIM_DIER_CC3DE
        
        #define USE_DMA_IFCR    DMA_IFCR(DMA1)
        #define USE_DMA_CCR     DMA_CCR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CNDTR   DMA_CNDTR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CMAR    DMA_CMAR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(USE_DMA_CHANNEL)
        #define SET_DMA_CCR     DMA_CCR(DMA1, SET_DMA_CHANNEL)
        #define CLR_DMA_CCR     DMA_CCR(DMA1, CLR_DMA_CHANNEL)
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
            
            USE_DMA_CCR = 0;
            #if defined(GPIO_MODE)
                USE_DMA_CNDTR = BUFFER_SIZE;
                USE_DMA_CMAR = (uint32_t)(&dmaBuffer[0]);
                DMA_CPAR(DMA1, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_BRR(PORT));
            #else            
                DMA_CPAR(DMA1, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_ODR(PORT));
            #endif
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            DMA_CNDTR(DMA1, SET_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA1, SET_DMA_CHANNEL) = (uint32_t)(&outputHigh);
            DMA_CPAR(DMA1, SET_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            SET_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            DMA_CNDTR(DMA1, CLR_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA1, CLR_DMA_CHANNEL) = (uint32_t)(&outputLow);
            DMA_CPAR(DMA1, CLR_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            CLR_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            nvic_clear_pending_irq(USE_DMA_IRQ);
            nvic_enable_irq(USE_DMA_IRQ);
            nvic_set_priority(USE_DMA_IRQ, 0);
        }
    #elif (TIMER) == 4
        #if (DMAC1) == (DMAC2) || (DMAC2) == (DMAC3) || (DMAC3) == (DMAC1)
            #error Three different DMA channels must be selected
        #endif
        #if (DMAC1) != 1 && (DMAC1) != 4 && (DMAC1) != 5 && (DMAC1) != 7
            #error DMA selection 1 does not belong to timer 4
        #endif
        #if (DMAC2) != 1 && (DMAC2) != 4 && (DMAC2) != 5 && (DMAC2) != 7
            #error DMA selection 2 does not belong to timer 4
        #endif
        #if (DMAC3) != 1 && (DMAC3) != 4 && (DMAC3) != 5 && (DMAC3) != 7
            #error DMA selection 3 does not belong to timer 4
        #endif
        
        #if (DMAC1) == 7
            #define SET_DMA_CHANNEL DMA_CHANNEL7
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC2
            #define DMACCLR DMAC3
        #elif (DMAC2) == 7
            #define SET_DMA_CHANNEL DMA_CHANNEL7
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC1
            #define DMACCLR DMAC3
        #elif (DMAC3) == 7
            #define SET_DMA_CHANNEL DMA_CHANNEL7
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC1
            #define DMACCLR DMAC2
        #else
            #define DMACSET DMAC1
            #define DMACUSE DMAC2
            #define DMACCLR DMAC3
        #endif
        
        #if defined(DMACSET)
            #if (DMACSET) == 1
                #define SET_DMA_CHANNEL DMA_CHANNEL1
                #define SET_TIMER_CCR   TIM_CCR1(TIM4)
                #define SET_TIMER_DMA   TIM_DIER_CC1DE
            #elif (DMACSET) == 4
                #define SET_DMA_CHANNEL DMA_CHANNEL4
                #define SET_TIMER_CCR   TIM_CCR2(TIM4)
                #define SET_TIMER_DMA   TIM_DIER_CC2DE
            #elif (DMACSET) == 5
                #define SET_DMA_CHANNEL DMA_CHANNEL5
                #define SET_TIMER_CCR   TIM_CCR3(TIM4)
                #define SET_TIMER_DMA   TIM_DIER_CC3DE
            #else
                #error Invalid set DMA channel
            #endif
        #endif
        #if (DMACCLR) == 1
            #define CLR_DMA_CHANNEL DMA_CHANNEL1
            #define CLR_TIMER_CCR   TIM_CCR1(TIM4)
            #define CLR_TIMER_DMA   TIM_DIER_CC1DE
        #elif (DMACCLR) == 4
            #define CLR_DMA_CHANNEL DMA_CHANNEL4
            #define CLR_TIMER_CCR   TIM_CCR2(TIM4)
            #define CLR_TIMER_DMA   TIM_DIER_CC2DE
        #elif (DMACCLR) == 5
            #define CLR_DMA_CHANNEL DMA_CHANNEL5
            #define CLR_TIMER_CCR   TIM_CCR3(TIM4)
            #define CLR_TIMER_DMA   TIM_DIER_CC3DE
        #else
            #error Invalid clear DMA channel
        #endif
        #if (DMACUSE) == 1
            #define DMA_IRQ_NAME    dma1_channel1_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL1
            #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL1_IRQ
            #define USE_TIMER_CCR   TIM_CCR1(TIM4)
            #define USE_TIMER_DMA   TIM_DIER_CC1DE
        #elif (DMACUSE) == 4
            #define DMA_IRQ_NAME    dma1_channel4_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL4
            #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL4_IRQ
            #define USE_TIMER_CCR   TIM_CCR2(TIM4)
            #define USE_TIMER_DMA   TIM_DIER_CC2DE
        #elif (DMACUSE) == 5
            #define DMA_IRQ_NAME    dma1_channel5_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL5
            #define USE_DMA_IRQ     NVIC_DMA1_CHANNEL5_IRQ
            #define USE_TIMER_CCR   TIM_CCR3(TIM4)
            #define USE_TIMER_DMA   TIM_DIER_CC3DE
        #else
            #error Invalid use DMA channel
        #endif
        
        #define USE_DMA_IFCR    DMA_IFCR(DMA1)
        #define USE_DMA_CCR     DMA_CCR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CNDTR   DMA_CNDTR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CMAR    DMA_CMAR(DMA1, USE_DMA_CHANNEL)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(USE_DMA_CHANNEL)
        #define SET_DMA_CCR     DMA_CCR(DMA1, SET_DMA_CHANNEL)
        #define CLR_DMA_CCR     DMA_CCR(DMA1, CLR_DMA_CHANNEL)
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
            
            USE_DMA_CCR = 0;
            #if defined(GPIO_MODE)
                USE_DMA_CNDTR = BUFFER_SIZE;
                USE_DMA_CMAR = (uint32_t)(&dmaBuffer[0]);
                DMA_CPAR(DMA1, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_BRR(PORT));
            #else            
                DMA_CPAR(DMA1, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_ODR(PORT));
            #endif
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            DMA_CNDTR(DMA1, SET_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA1, SET_DMA_CHANNEL) = (uint32_t)(&outputHigh);
            DMA_CPAR(DMA1, SET_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            SET_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            DMA_CNDTR(DMA1, CLR_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA1, CLR_DMA_CHANNEL) = (uint32_t)(&outputLow);
            DMA_CPAR(DMA1, CLR_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            CLR_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            nvic_clear_pending_irq(USE_DMA_IRQ);
            nvic_enable_irq(USE_DMA_IRQ);
            nvic_set_priority(USE_DMA_IRQ, 0);
        }
    #elif (TIMER) == 5
        #if (DMAC1) == (DMAC2) || (DMAC2) == (DMAC3) || (DMAC3) == (DMAC1)
            #error Three different DMA channels must be selected
        #endif
        #if (DMAC1) != 1 && (DMAC1) != 2 && (DMAC1) != 4 && (DMAC1) != 5
            #error DMA selection 1 does not belong to timer 5
        #endif
        #if (DMAC2) != 1 && (DMAC2) != 2 && (DMAC2) != 4 && (DMAC2) != 5
            #error DMA selection 2 does not belong to timer 5
        #endif
        #if (DMAC3) != 1 && (DMAC3) != 2 && (DMAC3) != 4 && (DMAC3) != 5
            #error DMA selection 3 does not belong to timer 5
        #endif
        
        #if (DMAC1) == 2
            #define SET_DMA_CHANNEL DMA_CHANNEL2
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC2
            #define DMACCLR DMAC3
        #elif (DMAC2) == 2
            #define SET_DMA_CHANNEL DMA_CHANNEL2
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC1
            #define DMACCLR DMAC3
        #elif (DMAC3) == 2
            #define SET_DMA_CHANNEL DMA_CHANNEL2
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC1
            #define DMACCLR DMAC2
        #else
            #define DMACSET DMAC1
            #define DMACUSE DMAC2
            #define DMACCLR DMAC3
        #endif
        
        #if defined(DMACSET)
            #if (DMACSET) == 1
                #define SET_DMA_CHANNEL DMA_CHANNEL1
                #define SET_TIMER_CCR   TIM_CCR4(TIM5)
                #define SET_TIMER_DMA   TIM_DIER_CC4DE
            #elif (DMACSET) == 4
                #define SET_DMA_CHANNEL DMA_CHANNEL4
                #define SET_TIMER_CCR   TIM_CCR2(TIM5)
                #define SET_TIMER_DMA   TIM_DIER_CC2DE
            #elif (DMACSET) == 5
                #define SET_DMA_CHANNEL DMA_CHANNEL5
                #define SET_TIMER_CCR   TIM_CCR1(TIM5)
                #define SET_TIMER_DMA   TIM_DIER_CC1DE
            #else
                #error Invalid set DMA channel
            #endif
        #endif
        #if (DMACCLR) == 1
            #define CLR_DMA_CHANNEL DMA_CHANNEL1
            #define CLR_TIMER_CCR   TIM_CCR4(TIM5)
            #define CLR_TIMER_DMA   TIM_DIER_CC4DE
        #elif (DMACCLR) == 4
            #define CLR_DMA_CHANNEL DMA_CHANNEL4
            #define CLR_TIMER_CCR   TIM_CCR2(TIM5)
            #define CLR_TIMER_DMA   TIM_DIER_CC2DE
        #elif (DMACCLR) == 5
            #define CLR_DMA_CHANNEL DMA_CHANNEL5
            #define CLR_TIMER_CCR   TIM_CCR1(TIM5)
            #define CLR_TIMER_DMA   TIM_DIER_CC1DE
        #else
            #error Invalid clear DMA channel
        #endif
        #if (DMACUSE) == 1
            #define DMA_IRQ_NAME    dma1_channel1_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL1
            #define USE_DMA_IRQ     NVIC_DMA2_CHANNEL1_IRQ
            #define USE_TIMER_CCR   TIM_CCR4(TIM5)
            #define USE_TIMER_DMA   TIM_DIER_CC4DE
        #elif (DMACUSE) == 4
            #define DMA_IRQ_NAME    dma1_channel4_5_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL4
            #define USE_DMA_IRQ     NVIC_DMA2_CHANNEL4_5_IRQ
            #define USE_TIMER_CCR   TIM_CCR2(TIM5)
            #define USE_TIMER_DMA   TIM_DIER_CC2DE
        #elif (DMACUSE) == 5
            #define DMA_IRQ_NAME    dma1_channel4_5_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL5
            #define USE_DMA_IRQ     NVIC_DMA2_CHANNEL4_5_IRQ
            #define USE_TIMER_CCR   TIM_CCR1(TIM5)
            #define USE_TIMER_DMA   TIM_DIER_CC1DE
        #else
            #error Invalid use DMA channel
        #endif
        
        #define USE_DMA_IFCR    DMA_IFCR(DMA2)
        #define USE_DMA_CCR     DMA_CCR(DMA2, USE_DMA_CHANNEL)
        #define USE_DMA_CNDTR   DMA_CNDTR(DMA2, USE_DMA_CHANNEL)
        #define USE_DMA_CMAR    DMA_CMAR(DMA2, USE_DMA_CHANNEL)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(USE_DMA_CHANNEL)
        #define SET_DMA_CCR     DMA_CCR(DMA2, SET_DMA_CHANNEL)
        #define CLR_DMA_CCR     DMA_CCR(DMA2, CLR_DMA_CHANNEL)
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA2EN);
            
            USE_DMA_CCR = 0;
            #if defined(GPIO_MODE)
                USE_DMA_CNDTR = BUFFER_SIZE;
                USE_DMA_CMAR = (uint32_t)(&dmaBuffer[0]);
                DMA_CPAR(DMA2, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_BRR(PORT));
            #else            
                DMA_CPAR(DMA2, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_ODR(PORT));
            #endif
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            DMA_CNDTR(DMA2, SET_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA2, SET_DMA_CHANNEL) = (uint32_t)(&outputHigh);
            DMA_CPAR(DMA2, SET_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            SET_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            DMA_CNDTR(DMA2, CLR_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA2, CLR_DMA_CHANNEL) = (uint32_t)(&outputLow);
            DMA_CPAR(DMA2, CLR_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            CLR_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            nvic_clear_pending_irq(USE_DMA_IRQ);
            nvic_enable_irq(USE_DMA_IRQ);
            nvic_set_priority(USE_DMA_IRQ, 0);
        }
    #elif (TIMER) == 8
        #if (DMAC1) == (DMAC2) || (DMAC2) == (DMAC3) || (DMAC3) == (DMAC1)
            #error Three different DMA channels must be selected
        #endif
        #if (DMAC1) != 1 && (DMAC1) != 2 && (DMAC1) != 3 && (DMAC1) != 5
            #error DMA selection 1 does not belong to timer 5
        #endif
        #if (DMAC2) != 1 && (DMAC2) != 2 && (DMAC2) != 3 && (DMAC2) != 5
            #error DMA selection 2 does not belong to timer 5
        #endif
        #if (DMAC3) != 1 && (DMAC3) != 2 && (DMAC3) != 3 && (DMAC3) != 5
            #error DMA selection 3 does not belong to timer 5
        #endif
        
        #if (DMAC1) == 1
            #define SET_DMA_CHANNEL DMA_CHANNEL1
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC2
            #define DMACCLR DMAC3
        #elif (DMAC2) == 1
            #define SET_DMA_CHANNEL DMA_CHANNEL1
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC1
            #define DMACCLR DMAC3
        #elif (DMAC3) == 1
            #define SET_DMA_CHANNEL DMA_CHANNEL1
            #define SET_TIMER_DMA   TIM_DIER_UDE
            #undef SET_TIMER_CCR
            #undef DMACSET
            #define DMACUSE DMAC1
            #define DMACCLR DMAC2
        #else
            #define DMACSET DMAC1
            #define DMACUSE DMAC2
            #define DMACCLR DMAC3
        #endif
        
        #if defined(DMACSET)
            #if (DMACSET) == 2
                #define SET_DMA_CHANNEL DMA_CHANNEL2
                #define SET_TIMER_CCR   TIM_CCR4(TIM8)
                #define SET_TIMER_DMA   TIM_DIER_CC4DE
            #elif (DMACSET) == 3
                #define SET_DMA_CHANNEL DMA_CHANNEL3
                #define SET_TIMER_CCR   TIM_CCR1(TIM8)
                #define SET_TIMER_DMA   TIM_DIER_CC1DE
            #elif (DMACSET) == 5
                #define SET_DMA_CHANNEL DMA_CHANNEL5
                #define SET_TIMER_CCR   TIM_CCR2(TIM8)
                #define SET_TIMER_DMA   TIM_DIER_CC2DE
            #else
                #error Invalid set DMA channel
            #endif
        #endif
        #if (DMACCLR) == 2
            #define CLR_DMA_CHANNEL DMA_CHANNEL2
            #define CLR_TIMER_CCR   TIM_CCR4(TIM8)
            #define CLR_TIMER_DMA   TIM_DIER_CC4DE
        #elif (DMACCLR) == 3
            #define CLR_DMA_CHANNEL DMA_CHANNEL3
            #define CLR_TIMER_CCR   TIM_CCR1(TIM8)
            #define CLR_TIMER_DMA   TIM_DIER_CC1DE
        #elif (DMACCLR) == 5
            #define CLR_DMA_CHANNEL DMA_CHANNEL5
            #define CLR_TIMER_CCR   TIM_CCR2(TIM8)
            #define CLR_TIMER_DMA   TIM_DIER_CC2DE
        #else
            #error Invalid clear DMA channel
        #endif
        #if (DMACUSE) == 2
            #define DMA_IRQ_NAME    dma1_channel2_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL2
            #define USE_DMA_IRQ     NVIC_DMA2_CHANNEL2_IRQ
            #define USE_TIMER_CCR   TIM_CCR4(TIM8)
            #define USE_TIMER_DMA   TIM_DIER_CC4DE
        #elif (DMACUSE) == 3
            #define DMA_IRQ_NAME    dma1_channel3_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL3
            #define USE_DMA_IRQ     NVIC_DMA2_CHANNEL3_IRQ
            #define USE_TIMER_CCR   TIM_CCR1(TIM8)
            #define USE_TIMER_DMA   TIM_DIER_CC1DE
        #elif (DMACUSE) == 5
            #define DMA_IRQ_NAME    dma1_channel4_5_isr
            #define USE_DMA_CHANNEL DMA_CHANNEL5
            #define USE_DMA_IRQ     NVIC_DMA2_CHANNEL5_IRQ
            #define USE_TIMER_CCR   TIM_CCR2(TIM8)
            #define USE_TIMER_DMA   TIM_DIER_CC2DE
        #else
            #error Invalid use DMA channel
        #endif
        
        #define USE_DMA_IFCR    DMA_IFCR(DMA2)
        #define USE_DMA_CCR     DMA_CCR(DMA2, USE_DMA_CHANNEL)
        #define USE_DMA_CNDTR   DMA_CNDTR(DMA2, USE_DMA_CHANNEL)
        #define USE_DMA_CMAR    DMA_CMAR(DMA2, USE_DMA_CHANNEL)
        #define USE_DMA_CGIF    DMA_IFCR_CGIF(USE_DMA_CHANNEL)
        #define SET_DMA_CCR     DMA_CCR(DMA2, SET_DMA_CHANNEL)
        #define CLR_DMA_CCR     DMA_CCR(DMA2, CLR_DMA_CHANNEL)
        
        static void dmaInitialize(void)
        {
            rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA2EN);
            
            USE_DMA_CCR = 0;
            #if defined(GPIO_MODE)
                USE_DMA_CNDTR = BUFFER_SIZE;
                USE_DMA_CMAR = (uint32_t)(&dmaBuffer[0]);
                DMA_CPAR(DMA2, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_BRR(PORT));
            #else            
                DMA_CPAR(DMA2, USE_DMA_CHANNEL) = (uint32_t)(&GPIO_ODR(PORT));
            #endif
            USE_DMA_IFCR = USE_DMA_CGIF;
            
            DMA_CNDTR(DMA2, SET_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA2, SET_DMA_CHANNEL) = (uint32_t)(&outputHigh);
            DMA_CPAR(DMA2, SET_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            SET_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            DMA_CNDTR(DMA2, CLR_DMA_CHANNEL) = 1;
            DMA_CMAR(DMA2, CLR_DMA_CHANNEL) = (uint32_t)(&outputLow);
            DMA_CPAR(DMA2, CLR_DMA_CHANNEL) = (uint32_t)(&GPIO_BSRR(PORT));
            CLR_DMA_CCR = DMA_CCR_WRITE_FIXED;
            
            nvic_clear_pending_irq(USE_DMA_IRQ);
            nvic_enable_irq(USE_DMA_IRQ);
            nvic_set_priority(USE_DMA_IRQ, 0);
        }
    #else
        #error Unsupported timer
    #endif
    
    static void setOutputsLow(void)
    {
        GPIO_BSRR(PORT) = outputLow;
    } 
    static void dmaDisable(void)
    {
        USE_DMA_CCR = 0;
        SET_DMA_CCR = 0;
        CLR_DMA_CCR = 0;
        setOutputsLow();
    }    
    static void modeInitialize(const void *data, uint16_t length)
    {
        #if !defined(MULTI_MODE)
            /* Initialize both buffers and start a send (the request won't
             * actually come in until the timer is started). */
            sendBegin = (const uint8_t *)data;
            sendEnd = sendBegin + length;        
            bufferHalf = false;
            #if !defined(GPIOANY_STRICT)
                fillNextBuffer(0, (1 << ((OUTPUT) % 8)));
                fillNextBuffer(0, (1 << ((OUTPUT) % 8)));
            #else
                fillNextBuffer();
                fillNextBuffer();
            #endif
        #else
            /* Transfer one extra bit (from a random memory location, basically)
             * so that we don't start the reset too soon. */
            #if !defined(MULTI_WIDE)
                USE_DMA_CNDTR = length+1;
            #else
                USE_DMA_CNDTR = (length/2)+1;
            #endif
            USE_DMA_CMAR = (uint32_t)(data);
        #endif
        
        #if defined(SET_TIMER_CCR)
            SET_TIMER_CCR = 0;
        #endif
        USE_TIMER_CCR = timerZeroHigh();
        CLR_TIMER_CCR = timerOneHigh();
        
        setOutputsLow();        
        USE_DMA_CCR = DMA_CCR_WRITE_TL;
        USE_TIMER_DIER = SET_TIMER_DMA | USE_TIMER_DMA | CLR_TIMER_DMA;
    }
#endif

#if defined(PWM_MODE) || defined(GPIO_MODE)
    void DMA_IRQ_NAME (void)
    {
        USE_DMA_IFCR = USE_DMA_CGIF;
        
        /* Not done yet, so fill the next buffer and wait for the send to
         * complete. */
        if (sendBegin != sendEnd) {
            #if defined(PWM_MODE)
                fillNextBuffer(timerOneHigh(), timerZeroHigh());
            #elif !defined(GPIOANY_STRICT)
                fillNextBuffer(0, (1 << ((OUTPUT) % 8)));
            #else
                fillNextBuffer();
            #endif
            return;
        }

        /* Just clock out one extra buffer to make sure everything is
         * completely out before we start the reset sequence. */
        if (sendBegin != 0) {
            sendBegin = 0;
            sendEnd = 0;
            return;
        }
        
        /* May already have clocked something out, but we don't care at this
         * point since it's past the end of the chain. */
        dmaDisable();
        timerStartReset();
    }
#else
    void DMA_IRQ_NAME (void)
    {
        USE_DMA_IFCR = USE_DMA_CGIF;
        dmaDisable();
        timerStartReset();
    }
#endif

void WS2821_initialize(void)
{
    asm volatile ("CPSID i");
    timerInitialize();
    dmaInitialize();
    timerStartReset();
    asm volatile ("CPSIE i");
}

void WS2821_send(const void *data, uint16_t length)
{
    if (state == Uninitialized)
        WS2821_initialize();
    while (WS2812_busy()) { }
    if (length == 0)
        return;
    
    asm volatile ("CPSID i");
    state = SendingData;    
    timerInitialize();
    dmaInitialize();    
    modeInitialize(data, length);
    timerStartMain();
    asm volatile ("CPSIE i");
}

bool WS2812_busy(void)
{
    return state != Idle;
}
bool WS2812_transmitting(void)
{
    return state == SendingData;
}
