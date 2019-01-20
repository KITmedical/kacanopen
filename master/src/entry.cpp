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
 
#include "entry.h"
#include "canopen_error.h"

#include <cassert>
#include <future>
#include <iomanip>
#include <memory>
#include <iostream>

namespace kaco {

Entry::Entry()
	: type(Type::invalid),
    disabled(false),
    m_value_changed_callbacks_mutex(new std::mutex),
    m_read_write_mutex(new std::recursive_mutex)
	{ }

// standard constructor
Entry::Entry(const uint16_t _index, const uint8_t _subindex, const std::string& _name, const Type _type, const AccessType _access_type)
	: index(_index),
		subindex(_subindex),
		name(_name),
		type(_type),
		access_type(_access_type),
		disabled(false),
		is_generic(false),
		m_valid(false),
		m_value_changed_callbacks_mutex(new std::mutex),
		m_read_write_mutex(new std::recursive_mutex)
	{ }

void Entry::set_value(const Value& value) {

	if (value.type != type) {
		throw canopen_error("[Entry::set_value] You passed a value of wrong type: "+Utils::type_to_string(value.type)+" != "+Utils::type_to_string(type)+".");
	}

	bool value_changed = false;

	{
		std::lock_guard<std::recursive_mutex> lock(*m_read_write_mutex);
		
		if (m_value.type != type || m_value != value) {
			value_changed = true;
		}

		m_value = value;
		m_valid = true;

	}

	if (value_changed) {
		std::lock_guard<std::mutex> lock(*m_value_changed_callbacks_mutex);
		for (auto& callback : m_value_changed_callbacks) {
			// TODO: currently callbacks are only internal and it's ok to call them synchonously.
			//std::async(std::launch::async, callback, value);
			callback(value);
		}
	}

}

const Value& Entry::get_value() const {
	std::lock_guard<std::recursive_mutex> lock(*m_read_write_mutex);
	if (!valid()) {
		throw canopen_error("[Entry::get_value] Value is not valid.");
	}
	return m_value;
}

bool Entry::valid() const {
	return m_valid;
}

Type Entry::get_type() const {
	return type;
}

void Entry::add_value_changed_callback(ValueChangedCallback callback) {
	std::lock_guard<std::mutex> lock(*m_value_changed_callbacks_mutex);
	m_value_changed_callbacks.push_back(std::move(callback));
}

void Entry::print() const {

	std::cout << "0x"<<std::hex<<index;
	std::cout << "/";
	std::cout << std::dec<<(unsigned)subindex;
	std::cout << "\t";
	std::cout << Utils::access_type_to_string(access_type);
	std::cout << "\t";
	std::cout << std::setw(75) << std::left << name;
	std::cout << " ";

	if (valid()) {
		std::cout << std::setw(10) << get_value();
	} else {
		std::cout << std::setw(10) << "empty";
	}

	std::cout << std::endl;

}

bool Entry::operator<(const Entry& other) const {
	return (index<other.index) || (index==other.index && subindex<other.subindex);
}

} // end namespace kaco
