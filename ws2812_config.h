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

#ifndef WS2812CONFIG_H
#define WS2812CONFIG_H

/* Use a timer as the output.  This can output to any of the timer compare 
 * outputs.  It requires a single DMA channel determined by the timer in
 * use and some buffer space. */
#define PWM_MODE

/* Use general DMA for any pin.  This can output to any pin.  Unless strict mode
 * is enabled then the half-word mirror of the GPIO is also controlled (e.x.
 * GPIO1 controls both GPIO1 and GPIO9).  It uses a timer, three DMA channels 
 * determined by the timer selected, and some buffer space. */
//#define GPIO_MODE
/* Only change the selected output.  This causes the buffer size to grow by
 * a factor of two */
//#define GPIO_STRICT

/* Use general DMA to output to any port.  This can output to all channels of
 * any port.  Unless wide mode is selected then the high and low bytes are
 * set equal.  In wide mode all channels are set independently.  It uses a
 * timer and three DMA channels determined by the timer selected.  Data is
 * output in MSB first, so the first byte/half-word is the MSB of the green
 * channel of the first drivers. */
//#define MULTI_MODE
/* Change all outputs on a port independently.  The size given is still the
 * size in bytes, but outputs are done in two byte pairs to set the whole
 * port. */
//#define MUTLI_WIDE

#if defined(PWM_MODE)
    /*
     * Timer 1: DMA1/5
     * Timer 2: DMA1/2
     * Timer 3: DMA1/3
     * Timer 4: DMA1/7
     * Timer 5: DMA2/2
     * Timer 8: DMA2/1
     */
    #define TIMER   1

    /* Must be configured as an alternate function output */
    #define OUTPUT  1
    
    /* Use the inverted output (timers 1 or 8) instead of the primary one */
    //#define OUTPUT_INVERT 1

    /* Total space used is this plus 12.  Must be a multiple of 8
     * and should generally be a multiple of 24 (24 bits per LED). */
    #define BUFFER_SIZE 24*4
#endif

#if defined(GPIO_MODE) || defined(MULTI_MODE)
    /* 
     * Timer 1: DMA1/2,3,4,5,6
     * Timer 2: DMA1/1,2,5,7
     * Timer 3: DMA1/2,3,6
     * Timer 4: DMA1/1,4,5,7
     * Timer 5: DMA2/1,2,4,5
     * Timer 8: DMA2/1,2,3,5
     * Any three of the above DMAs can be selected
     */
    #define TIMER   1
    
    #if TIMER == 1
        #define DMAC1   4
        #define DMAC2   5
        #define DMAC3   6
    #elif TIMER == 2
        #define DMAC1   1
        #define DMAC2   5
        #define DMAC3   7
    #elif TIMER == 4
        #define DMAC1   1
        #define DMAC2   4
        #define DMAC3   5
    #elif TIMER == 5
        #define DMAC1   1
        #define DMAC2   2
        #define DMAC3   5
    #elif TIMER == 8
        #define DMAC1   1
        #define DMAC2   2
        #define DMAC3   5
    #endif
    
    /* Must be configured as a normal output */
    #define PORT    GPIOA
#endif

#if defined(GPIO_MODE)
    /* Must be configured as a normal output */
    #define OUTPUT  8
    
    /* Total space is this (times two if in strict mode) plus 12.  Must be a 
     * multiple of 8 and should generally be a multiple 
     * of 24 (24 bits per LED). */
    #define BUFFER_SIZE 24*6
#endif

/* The clock rate in Hz */
#define CLOCK   800000
/* The 0-bit high duty cycle */
#define T0H     0.32
/* The 1-bit high duty cycle */
#define T1H     0.64
/* The reset/latch time */
#define TRESET  50E-6

#endif
