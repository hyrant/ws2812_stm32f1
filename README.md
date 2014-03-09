A simple library to write data to chains of WS2812 based RGB LEDs from a 
STM32F1 series microcontroller.

Output to any pin through variable methods depending on which pin is desired.
All methods make use of DMA and interrupts to preserve timing while consuming
(relatively) few processor cycles.

Requirements
============

A working arm-none-eabi gcc.  Use https://github.com/esden/summon-arm-toolchain
to make it easy.

In general there should be no other interrupts run at less than priority 16
while the output is in progress so that the timing requirements are maintained.
All methods require a dedicated (while operating) timer.

Output Methods
=============

There are three available output methods.  The first and the lightest on
system resources (both DMA channels and cycle usage) can output to any timer
CC output included the inverted ones.  The second can output to any single
GPIO pin.  The third can output to a whole or half of a GPIO port controlling
8 or 16 chains in parallel.

Timer PWM Mode
--------------

This mode can output to any timer CC output.  It functions by doing continuous
DMA to the timer's CCR register updating the half buffer at the transfer 
complete and half transfer interrupts.  This allows it to maintain the timing
with only a single DMA channel, determined by the timer in use.  The buffer
space is configurable with more buffer space freeing more processor cycles.

Single GPIO Mode
----------------

This mode can output to any single pin.  In normal mode both the high and low
bytes of the port are controlled but this can be set to only change the desired
pin at the cost of doubling the required buffer space.  This mode uses three
DMA channels (to set, lower on 0 bits and lower on 1 bits) determined by the
timer selected.  Most timers have multiple options for the DMA channels but
three unique ones must be chosen.  Because of the larger DMA requirements this
mode uses more cycles than the PWM mode.  The buffer space is configurable 
with more buffer space freeing more processor cycles.

Multiple Parallel GPIO Mode
---------------------------

This mode can output to all pins of a GPIO port.  In normal mode th high
and low halves of the port are set to the same while in wide mode all 16 are
controlled independently.  Each byte or half-word controls one bit of the
output chain.  This mode uses three  DMA channels (to set, lower on 0 bits 
and lower on 1 bits) determined by the timer selected.  Most timers have 
multiple options for the DMA channels but three unique ones must be chosen. 
No buffer space is required.
