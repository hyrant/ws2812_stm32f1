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

#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the WS2812 communications.  This starts the initial reset 
 * blank time.
 */
void WS2821_initialize(void);

/**
 * Send data to the WS2821 chain.  This will block if called before the previous
 * send is complete.  This will also initialize the chain if needed and may
 * block for that on the first call.
 * 
 * @param data      the data to send (must remain available until the transmission is complete)
 * @param length    the length in bytes to send
 */
void WS2821_send(const void *data, uint16_t length);

/**
 * Test if the send is still active.  This includes the reset period
 * after the send.
 * 
 * @return true if the WS2812 send is still in progress
 */
bool WS2812_busy(void);

/**
 * Test if the transmission is still complete.  The reset period after
 * transmission may still be in progress.  The data passed to the the send
 * must not become unavailable or change until this returns false.
 * 
 * @return true if the WS2812 transmission is still in progress
 */
bool WS2812_transmitting(void);

#endif
