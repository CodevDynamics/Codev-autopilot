/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

/**
 * @file led.cpp
 */


#include "led_flash.h"

int LedFlashController::update(LedFlashControlData &control_data)
{
	// handle state updates
	hrt_abstime now = hrt_absolute_time();

	led_control_s led_control = {};

	uint8_t led_mask_mark = 0;

	_led_control_sub.updated();

	if (_led_control_sub.copy(&led_control)) {

		if (_last_update_call == 0) {
			_last_update_call = now;
			return 0;
		}

		for (int i = 0; i < BOARD_MAX_LEDS; ++i) {
			led_mask_mark = (led_control.led_mask & (BOARD_FRONT_LED_MASK));

			if (led_mask_mark) {
				if ((led_mask_mark & (1 << i))) {
					control_data.leds[i].color = led_control_s::COLOR_WHITE;
				}
			}

			// tranlate led status
			switch (led_on_off) {
			case led_control_s::MODE_OFF:
				if (led_control.led_mask & (1 << i)) {
					led_mask_mark = (led_control.led_mask & (BOARD_REAR_LED_MASK));

					if (led_mask_mark & (1 << i)) {
						control_data.leds[i].color = led_control_s::COLOR_OFF;
						control_data.leds[i].brightness = 0;
					}

				}

				if (hrt_elapsed_time(&_last_update_call) > LED_OFF_TIME_MS) {
					led_on_off = led_control_s::MODE_ON;
					_last_update_call = now;
				}

				break;

			case led_control_s::MODE_ON:
				if (led_control.led_mask & (1 << i)) {
					led_mask_mark = (led_control.led_mask & (BOARD_REAR_LED_MASK));

					if (led_mask_mark) {
						// fix the condition led_mask = 0xff
						if (led_mask_mark & (1 << i)) {
							control_data.leds[i].color = led_control.color;
							control_data.leds[i].brightness = 255;
						}

					}
				}

				if (hrt_elapsed_time(&_last_update_call) > LED_ON_TIME_MS) {
					led_on_off = led_control_s::MODE_OFF;
					_last_update_call = now;
				}

				break;

			default:
				led_on_off = led_control_s::MODE_ON;
			}

			// fix the long time do not update led status
			if (hrt_elapsed_time(&_last_update_call) > (LED_OFF_TIME_MS + LED_ON_TIME_MS)) {
				control_data.leds[i].color = led_control.color;
				control_data.leds[i].brightness = 255;
				led_on_off = led_control_s::MODE_ON;
			}
		}
	}


	return 1;
}
