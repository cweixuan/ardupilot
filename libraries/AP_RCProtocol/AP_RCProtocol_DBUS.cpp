/*
  DBUS decoder, based on src/modules/px4iofirmware/dbus.c from PX4Firmware
  modified for use in AP_HAL_* by Andrew Tridgell
 */
/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "AP_RCProtocol_DBUS.h"

#define DBUS_FRAME_SIZE		18
#define DBUS_INPUT_CHANNELS	7
#define DBUS_FLAGS_BYTE		23
#define DBUS_FAILSAFE_BIT	3
#define DBUS_FRAMELOST_BIT	2

/* define range mapping here, -+100% -> 1000..2000 */
#define DBUS_RANGE_MIN 200
#define DBUS_RANGE_MAX 1800
#define DBUS_RANGE_RANGE (DBUS_RANGE_MAX - DBUS_RANGE_MIN)

#define DBUS_TARGET_MIN 1000
#define DBUS_TARGET_MAX 2000
#define DBUS_TARGET_RANGE (DBUS_TARGET_MAX - DBUS_TARGET_MIN)

// this is 875
#define DBUS_SCALE_OFFSET (DBUS_TARGET_MIN - ((DBUS_TARGET_RANGE * DBUS_RANGE_MIN / DBUS_RANGE_RANGE)))

#ifndef HAL_DBUS_FRAME_GAP
#define HAL_DBUS_FRAME_GAP 100U
#endif

// constructor
AP_RCProtocol_DBUS::AP_RCProtocol_DBUS(AP_RCProtocol &_frontend, bool _inverted) :
    AP_RCProtocol_Backend(_frontend),
    inverted(_inverted),
    ss{100000, SoftSerial::SERIAL_CONFIG_8E1I}
{}

// decode a full DBUS frame
bool AP_RCProtocol_DBUS::dbus_decode(const uint8_t frame[18], uint16_t *values, uint16_t *num_values,
                                     bool *dbus_failsafe, bool *dbus_frame_drop, uint16_t max_values)
{
    /* check frame boundary markers to avoid out-of-sync cases */
    uint16_t chancount = DBUS_INPUT_CHANNELS;

	values[0] = ((int16_t)frame[0] | ((int16_t)frame[1] << 8)) & 0x07FF;
	values[0] = uint16_t(0.758 * double(values[0]) + 624.242);
	values[1] = (((int16_t)frame[1] >> 3) | ((int16_t)frame[2] << 5)) & 0x07FF;
	values[1] = uint16_t(0.758 * double(values[1]) + 624.242);
	values[2] = (((int16_t)frame[2] >> 6) | ((int16_t)frame[3] << 2) |
		((int16_t)frame[4] << 10)) & 0x07FF;
	values[2] = uint16_t(0.758 * double(values[2]) + 624.242);
	values[3] = (((int16_t)frame[4] >> 1) | ((int16_t)frame[5] << 7)) & 0x07FF;
	values[3] = uint16_t(0.758 * double(values[3]) + 624.242);



	values[4] = ((frame[5] >> 4) & 0x000C) >> 2;
	values[4] =  500 * values[4] + 400;
	values[5] = ((frame[5] >> 4) & 0x0003);
	values[5] =  500 * values[5] + 400;	
    values[6] = ((int16_t)frame[16]) | ((int16_t)frame[17] << 8);
	values[6] = uint16_t(0.758 * double(values[6]) + 624.242);
    *num_values = chancount;


    return true;
}


/*
  process a DBUS input pulse of the given width
 */
void AP_RCProtocol_DBUS::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint32_t w0 = width_s0;
    uint32_t w1 = width_s1;
    if (inverted) {
        w0 = saved_width;
        w1 = width_s0;
        saved_width = width_s1;
    }
    uint8_t b;
    if (ss.process_pulse(w0, w1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

// support byte input
void AP_RCProtocol_DBUS::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    const bool have_frame_gap = (timestamp_us - byte_input.last_byte_us >= HAL_DBUS_FRAME_GAP);
    byte_input.last_byte_us = timestamp_us;
    if (have_frame_gap) {
        // if we have a frame gap then this must be the start of a new
        // frame
        byte_input.ofs = 0;
    }
   if (byte_input.ofs == 0 && !have_frame_gap) {
        // must have a frame gap before the start of a new DBUS frame
        return;
    }

    byte_input.buf[byte_input.ofs++] = b;

    if (byte_input.ofs == sizeof(byte_input.buf)) {
        log_data(AP_RCProtocol::DBUS, timestamp_us, byte_input.buf, byte_input.ofs);
        uint16_t values[DBUS_INPUT_CHANNELS];
        uint16_t num_values=0;
        bool dbus_failsafe = false;
        bool dbus_frame_drop = false;
        if (dbus_decode(byte_input.buf, values, &num_values,
                        &dbus_failsafe, &dbus_frame_drop, DBUS_INPUT_CHANNELS)) 
                        {
            add_input(num_values, values, dbus_failsafe);
        }
    }
}

// support byte input
void AP_RCProtocol_DBUS::process_byte(uint8_t b, uint32_t baudrate)
{
    if (baudrate != 100000) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}
