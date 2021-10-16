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
 * @file led_flash.h
 *
 * Led controller helper class, used by Led drivers
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/led_control.h>

struct LedFlashControlDataSingle {
	uint8_t color; ///< one of led_control_s::COLOR_*
	uint8_t brightness; ///< brightness in [0, 255]
};
struct LedFlashControlData {
	LedFlashControlDataSingle leds[BOARD_MAX_LEDS];
};

#ifndef BOARD_FRONT_LED_MASK
#define BOARD_FRONT_LED_MASK (0)
#endif

#ifndef BOARD_REAR_LED_MASK
#define BOARD_REAR_LED_MASK  (0)
#endif

#ifndef BOARD_LEFT_LED_MASK
#define BOARD_LEFT_LED_MASK  (0)
#endif

#ifndef BOARD_RIGHT_LED_MASK
#define BOARD_RIGHT_LED_MASK (0)
#endif


/**
 ** class LedFlashController
 * Handles the led_control topic: blinking, priorities and state updates.
 */
class LedFlashController
{
public:
	LedFlashController() = default;
	~LedFlashController() = default;

	/**
	 * Update and retrieve the Led state. It will do the orb_copy() and needs to be called at least every
	 * maximum_update_interval(). In addition a caller might poll on the led_control_sub
	 * @param control_data output structure (will always be set)
	 * @return 1 if control_data set (state changed), 0 if control_data not changed (state did not change), <0 error otherwise
	 */
	int update(LedFlashControlData &control_data);

#define LED_ON_TIME_MS  100 * 1000
#define LED_OFF_SHORT_TIME_MS  100 * 1000
#define LED_OFF_LONG_TIME_MS 500 * 1000

private:

	/** set control_data based on current Led states */
	struct PerPriorityData {
		uint8_t color = 0; ///< one of led_control_s::COLOR_*
		uint8_t mode = led_control_s::MODE_DISABLED; ///< one of led_control_s::MODE_*
		uint8_t blink_times_left = 0; /**< how many times left to blink (MSB bit is used for infinite case).
									This limits the number of complete blink cycles to 64 (if not infinite) */
	};


	uORB::Subscription _led_control_sub{ORB_ID(led_control)}; ///< uorb subscription
	hrt_abstime _last_update_call{0};
	bool _force_update{true}; ///< force an orb_copy in the beginning
	bool _breathe_enabled{false}; ///< true if at least one of the led's is currently in breathe mode
	uint8_t led_on_off_state = 0;

	uint16_t current_blink_duration = 1000;
};
