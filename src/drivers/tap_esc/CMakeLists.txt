############################################################################
#
#   Copyright (c) 2016 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
px4_add_module(
	MODULE drivers__tap_esc
	MAIN tap_esc
	COMPILE_FLAGS
		-Wno-missing-field-initializers
	SRCS
		tap_esc.cpp
		tap_esc_common.cpp
	DEPENDS
		mixer
		led_flash
	)

# include the codev ESC binary
if(DEFINED config_tap_esc_fw)
	set(fw_file_name "${config_tap_esc_fw}_ESC.bin")
	message(STATUS "ESC firmware: ${fw_file_name}")

	set(fw_esc_bin "${PX4_SOURCE_DIR}/ROMFS/codev/esc/${fw_file_name}")
	set(fw_esc_target_dir "${PX4_BINARY_DIR}/romfs_extras")
	set(renamed_fw_esc_bin "${fw_esc_target_dir}/tap_esc.bin")

	add_custom_command(OUTPUT ${renamed_fw_esc_bin}
		COMMAND ${CMAKE_COMMAND} -E make_directory ${fw_esc_target_dir}
		COMMAND cp -f ${fw_esc_bin} ${renamed_fw_esc_bin}
		DEPENDS ${fw_esc_bin}
		COMMENT "Copying ${fw_esc_bin} to ${renamed_fw_esc_bin}"
	)
	add_custom_target(copy_tap_esc_fw DEPENDS ${renamed_fw_esc_bin})

else()
	message(WARNING "ESC firmware: Not defined in target cmake. Cannot copy binaries!")
endif()
