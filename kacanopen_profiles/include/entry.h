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

#include <cstdint>
#include <string>
#include <vector>

#include "type.h"
#include "value.h"
#include "access_method.h"

namespace kaco {

	enum AccessType {
		read_only,
		write_only,
		read_write,
		constant
	};

	struct Entry {

		Entry();

		// standard constructor
		Entry(uint32_t _index, uint8_t _subindex, std::string _name, Type _type, AccessType _access);

		// array constructor
		Entry(uint32_t _index, std::string _name, Type _type, AccessType _access);

		std::string name;
		uint16_t index;
		
		uint8_t subindex; // only used if is_array==false
		bool is_array;

		AccessType access;

		Type type;
		
		Value value;
		bool valid;

		std::vector<Value> array;
		std::vector<bool> array_entry_valid;

		bool sdo_on_read;
		bool sdo_on_write;

		bool is_slice;
		uint8_t slice_first_bit;
		uint8_t slice_last_bit;

		AccessMethod access_method;

		std::string description;

		void set_value(const Value& value, uint8_t array_index=0);
		const Value& get_value(uint8_t array_index=0) const;

	};

} // end namespace kaco