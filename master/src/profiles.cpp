/*
 * Copyright (c) 2016, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "profiles.h"
#include "logger.h"

namespace kaco {

	const std::map<uint16_t,std::map<std::string, const Profiles::Operation>> Profiles::operations {
		{
			(uint16_t) 402,
			{
				{
					"enable_operation",
					[](Device& device,const Value&) -> Value {
						device.set_entry("controlword", (uint16_t) 0x0006); // shutdown
						device.set_entry("controlword", (uint16_t) 0x0007); // switch on
						device.set_entry("controlword", (uint16_t) 0x000F); // enable operation
						return Value(); // invalid value (return value not needed)
					}
				},
				{
					"set_controlword_flag",
					[](Device& device,const Value& flag_name) -> Value {
						DEBUG_LOG("Set controlword flag "<<flag_name);
						const uint16_t cw = device.get_entry("controlword");
						const uint16_t flag = device.get_constant(flag_name);
						device.set_entry("controlword", static_cast<uint16_t>(cw | flag));
						return Value();
					}
				},
				{
					"unset_controlword_flag",
					[](Device& device,const Value& flag_name) -> Value {
						DEBUG_LOG("Unset controlword flag "<<flag_name);
						const uint16_t cw = device.get_entry("controlword");
						const uint16_t flag = device.get_constant(flag_name);
						device.set_entry("controlword", static_cast<uint16_t>(cw & ~flag));
						return Value();
					}
				},
				{
					"set_target_position",
					[](Device& device,const Value& target_position) -> Value {
						DEBUG_LOG("Set target pos to "<<target_position);
						device.set_entry("Target position", target_position);
						device.execute("set_controlword_flag","controlword_pp_new_set_point");
						device.execute("unset_controlword_flag","controlword_pp_new_set_point");
						return Value();
					}
				},
				{
					"set_target_velocity",
					[](Device& device,const Value& target_velocity) -> Value {
						DEBUG_LOG("Set target vel to "<<target_velocity);
						device.set_entry("Target velocity", target_velocity);
						return Value();
					}
				},
				{
					"set_target_torque",
					[](Device& device,const Value& target_torque) -> Value {
						DEBUG_LOG("Set target tor to "<<target_torque);
						device.set_entry("Target torque", target_torque);
						return Value();
					}
				}
			}
		}
	};

	const std::map<uint16_t,std::map<std::string, const Value>> Profiles::constants {
		{
			(uint16_t) 402,
			{
				// profiles
				{ "profile_position_mode",		static_cast<int8_t>(1) },
				{ "velocity_mode",				static_cast<int8_t>(2) },
				{ "profile_velocity_mode",		static_cast<int8_t>(3) },
				{ "torque_profile_mode",		static_cast<int8_t>(4) },
				{ "homing_mode",				static_cast<int8_t>(6) },
				{ "interpolated_position_mode",	static_cast<int8_t>(7) },

				// control word flags general
				{ "controlword_switch_on",			static_cast<uint16_t>(1<<0) },
				{ "controlword_enable_voltage",		static_cast<uint16_t>(1<<1) },
				{ "controlword_quick_stop",			static_cast<uint16_t>(1<<2) },
				{ "controlword_enable_operation",	static_cast<uint16_t>(1<<3) },
				{ "controlword_fault_reset",		static_cast<uint16_t>(1<<7) },
				{ "controlword_halt",				static_cast<uint16_t>(1<<8) },

				// control word flags manufacturer specific
				{ "controlword_manufacturer_11",	static_cast<uint16_t>(1<<11) },
				{ "controlword_manufacturer_12",	static_cast<uint16_t>(1<<12) },
				{ "controlword_manufacturer_13",	static_cast<uint16_t>(1<<13) },
				{ "controlword_manufacturer_14",	static_cast<uint16_t>(1<<14) },
				{ "controlword_manufacturer_15",	static_cast<uint16_t>(1<<15) },

				// control word flags velocity mode specific
				{ "controlword_vl_rfg_enable",	static_cast<uint16_t>(1<<4) },
				{ "controlword_vl_rfg_unlock",	static_cast<uint16_t>(1<<5) },
				{ "controlword_vl_use_ref",		static_cast<uint16_t>(1<<6) },

				// control word flags profile position mode specific
				{ "controlword_pp_new_set_point",				static_cast<uint16_t>(1<<4) },
				{ "controlword_pp_change_set_immediately",		static_cast<uint16_t>(1<<5) },
				{ "controlword_pp_abs_rel",						static_cast<uint16_t>(1<<6) },

				// control word flags homing mode specific
				{ "controlword_hm_operation_start",	static_cast<uint16_t>(1<<4) },

				// control word flags interpolated position mode specific
				{ "controlword_ip_enable_ip_mode",	static_cast<uint16_t>(1<<4) },

				// status word flags general
				{ "statusword_ready_to_switch_on",		static_cast<uint16_t>(1<<0) },
				{ "statusword_switched_on",				static_cast<uint16_t>(1<<1) },
				{ "statusword_operation_enabled",		static_cast<uint16_t>(1<<2) },
				{ "statusword_fault",					static_cast<uint16_t>(1<<3) },
				{ "statusword_voltage_enabled",			static_cast<uint16_t>(1<<4) },
				{ "statusword_quick_stop",				static_cast<uint16_t>(1<<5) },
				{ "statusword_switch_on_disabled",		static_cast<uint16_t>(1<<6) },
				{ "statusword_warning",					static_cast<uint16_t>(1<<7) },
				{ "statusword_remote",					static_cast<uint16_t>(1<<9) },
				{ "statusword_target_reached",			static_cast<uint16_t>(1<<10) },
				{ "statusword_internal_limit_active",	static_cast<uint16_t>(1<<11) },

				// status word flags manufacturer specific
				{ "statusword_manufacturer_8",	static_cast<uint16_t>(1<<8) },
				{ "statusword_manufacturer_14",	static_cast<uint16_t>(1<<14) },
				{ "statusword_manufacturer_15",	static_cast<uint16_t>(1<<15) },

				// status word flags profile position mode specific
				{ "statusword_pp_set_point_acknowledge",	static_cast<uint16_t>(1<<12) },
				{ "statusword_pp_following_error",			static_cast<uint16_t>(1<<13) },

				// status word flags profile velocity mode specific
				{ "statusword_pv_speed",				static_cast<uint16_t>(1<<12) },
				{ "statusword_pv_max_slippage_error",	static_cast<uint16_t>(1<<13) },

				// status word flags homing mode specific
				{ "statusword_hm_homing_attained",	static_cast<uint16_t>(1<<12) },
				{ "statusword_hm_homing_error",		static_cast<uint16_t>(1<<13) },

				// status word flags interpolated position mode specific
				{ "statusword_ip_ip_mode_active",	static_cast<uint16_t>(1<<12) }
			}
		}
	};

} // end namespace kaco