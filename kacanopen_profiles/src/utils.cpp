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

#include "utils.h"
#include "type.h"
#include "logger.h"

#include <string>
#include <cstdint>

namespace kaco {

std::string Utils::type_to_string(Type type) {
	switch(type) {
		case Type::uint8:
			return "uint8";
		case Type::uint16:
			return "uint16";
		case Type::uint32:
			return "uint32";
		case Type::int8:
			return "int8";
		case Type::int16:
			return "int16";
		case Type::int32:
			return "int32";
		case Type::string:
			return "string";
		default:
			return "unknown type";
	}
}

uint8_t Utils::get_type_size(Type type) {
	switch(type) {
		case Type::uint8:
		case Type::int8:
			return 1;
		case Type::uint16:
		case Type::int16:
			return 2;
		case Type::uint32:
		case Type::int32:
			return 4;
		case Type::string:
		default:
			DEBUG("[Utils::get_type_size] Unknown type or type with variable size.");
			return 0;
	}
}

} // end namespace kaco