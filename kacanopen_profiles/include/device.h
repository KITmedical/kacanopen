/*
 * Copyright (c) 2015, Thomas Keh
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
 
#pragma once

#include "core.h"
#include "entry.h"
#include "type.h"
#include "pdo_mapping.h"
#include "access_method.h"

#include <vector>
#include <map>
#include <string>

namespace kaco {

	class Device {

	public:

		Device(Core& core, uint8_t node_id);
		~Device();

		void start();

		//! Gets the value of a dictionary entry by index via SDO
		//! It does not change the corresponding internal value and therefore the new value
		//! cannot be used by Transmit PDOs.
		Value get_entry_via_sdo(uint32_t index, uint8_t subindex, Type type);

		//! Gets the value of a dictionary entry by name internally.
		//! If there is no cached value or the entry is configured to send an SDO on request, the new value is fetched from the device via SDO.
		//! Otherwise it returns the cached value. This makes sense, if a Reveive PDO is configured on the corresponding entry.
		const Value& get_entry(std::string name, uint8_t array_index=0, AccessMethod access_method = AccessMethod::use_default);

		//! Sets the value of a dictionary entry by index via SDO
		//! It does not change the corresponding internal value and therefore the new value
		//! cannot be used by Transmit PDOs.
		void set_entry_via_sdo(uint32_t index, uint8_t subindex, const Value& value);

		//! Sets the value of a dictionary entry by name internally.
		//! If the entry is configured to send an SDO on update, the new value is also sent to the device via SDO.
		//! If a PDO is configured on the corresponding entry, it will from now on use the new value stored internally.
		void set_entry(std::string name, const Value& value, uint8_t array_index=0, AccessMethod access_method = AccessMethod::use_default);

		void add_pdo_mapping(uint16_t cob_id, std::string entry_name, uint8_t first_byte, uint8_t last_byte, uint8_t array_index=0);

	private:

		void pdo_received_callback(const PDOMapping& mapping, std::vector<uint8_t> data);

		static const bool debug = true;

		Core& m_core;
		uint8_t m_node_id;

		std::map<std::string, Entry> m_dictionary;
		std::vector<PDOMapping> m_pdo_mappings;
		const Value m_dummy_value;

		const std::vector<Entry> profile301 {

			Entry(0x1000, 0, "device_type", Type::uint32, AccessType::read_only),
			Entry(0x1001, 0, "error_register", Type::uint8, AccessType::read_only),
			
			// TODO remove. This is CiA401!
			Entry(0x6000, "read_digital_input", Type::uint8, AccessType::read_only),
			Entry(0x6200, "write_output", Type::uint8, AccessType::write_only)
			
		};

	};

} // end namespace kaco