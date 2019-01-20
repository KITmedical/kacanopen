/*
 * Copyright (c) 2015, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:M
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

#include "logger.h"
#include "eds_reader.h"
#include "entry.h"
#include "utils.h"
#include "global_config.h"

#include <regex>
#include <string>
#include <cassert>
#include <utility>

#include <boost/property_tree/ptree.hpp> // property_tree
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string/predicate.hpp> // string::starts_with()
#include  <boost/algorithm/string/trim.hpp> // trim()

namespace kaco {

EDSReader::EDSReader(std::unordered_map<Address, Entry>& dictionary, std::unordered_map<std::string, Address>& name_to_address)
	: m_dictionary(dictionary), m_name_to_address(name_to_address)
	{ }

bool EDSReader::load_file(std::string filename) {

	DEBUG_LOG_EXHAUSTIVE("Trying to read EDS file " << filename);
	try {
		boost::property_tree::ini_parser::read_ini(filename, m_ini);
		return true;
	} catch (const std::exception& e) {
		ERROR("[EDSReader::load_file] Could not open file: "<<e.what());
		return false;
	}

}

bool EDSReader::import_entries() {

	bool success = true;

	for (const auto& section_node : m_ini) {
		const std::string& section_name = section_node.first;
		//const boost::property_tree::ptree& section = section_node.second;

		try {

			std::smatch matches;

			if (std::regex_match(section_name, std::regex("[[:xdigit:]]{1,4}"))) {

				uint16_t index = (uint16_t) Utils::hexstr_to_uint(section_name);
				DEBUG_LOG_EXHAUSTIVE("Section "<<section_name<<" corresponds to index "<<index<<".");
				success = parse_index(section_name, index) && success; // mind order!

			}
			/*
			else if (std::regex_match(section_name, matches, std::regex("([[:xdigit:]]{1,4})sub([[:xdigit:]]{1,2})"))) {

				// Ignoring subindex entries here!

			} else {

				// Ignoring metadata entries here!
				DEBUG_LOG_EXHAUSTIVE("Section "<<section_name<<" contains meta data. Doing nothing.");

			}
			*/

		} catch (std::regex_error& e) {
			ERROR("[EDSReader::import_entries] " << parse_regex_error(e.code(), section_name));
			success = false;
		}

	}

	return success;

}

bool EDSReader::parse_index(const std::string& section, uint16_t index) {

	std::string str_object_type = trim(m_ini.get(section+".ObjectType", ""));
	
	uint8_t object_code = (uint8_t) ObjectType::VAR;
	if (str_object_type.empty()) {
		DEBUG_LOG("Field ObjectType missing. Assuming ObjectType::VAR (according to DS 306 V1.3 page 16).");
	} else {
		object_code = (uint8_t) Utils::hexstr_to_uint(str_object_type);
	}

	
	if (object_code == (uint8_t) ObjectType::VAR) {
		return parse_var(section, index, 0);
	} else if ((object_code == (uint8_t) ObjectType::RECORD) || (object_code == (uint8_t) ObjectType::ARRAY)) {
		return parse_array_or_record(section, index);
	}

	DEBUG_LOG("This is not a variable and no array. Ignoring.")
	return true;

}

bool EDSReader::parse_var(const std::string& section, uint16_t index, uint8_t subindex, const std::string& name_prefix) {

	std::string var_name = Utils::escape(trim(m_ini.get(section+".ParameterName", "")));

	if (var_name.empty()) {
		ERROR("[EDSReader::parse_var] Field ParameterName missing");
		return false;
	}

	if (!name_prefix.empty()) {
		var_name = name_prefix + "/" + var_name;
	}

	DEBUG_LOG("[EDSReader::parse_var] Parsing variable "<<section<<": "<<var_name);

	std::string str_sub_number = trim(m_ini.get(section+".SubNumber", ""));
	std::string str_object_type = trim(m_ini.get(section+".ObjectType", ""));
	std::string str_data_type = trim(m_ini.get(section+".DataType", ""));
	std::string str_low_limit = trim(m_ini.get(section+".LowLimit", ""));
	std::string str_high_limit = trim(m_ini.get(section+".HighLimit", ""));
	std::string str_access_type = trim(m_ini.get(section+".AccessType", ""));
	std::string str_default_value = trim(m_ini.get(section+".DefaultValue", ""));
	std::string str_pdo_mapping = trim(m_ini.get(section+".PDOMapping", ""));
	std::string str_obj_flags = trim(m_ini.get(section+".ObjFlags", ""));

//	entry.name = var_name;
//	entry.type = Utils::type_code_to_type((uint16_t) Utils::hexstr_to_uint(str_data_type));
//	entry.index = index;
//	entry.subindex = subindex;
  Entry entry(index, subindex, var_name,
              Utils::type_code_to_type((uint16_t) Utils::hexstr_to_uint(str_data_type)),
              Utils::string_to_access_type(str_access_type));

	if (Config::eds_reader_mark_entries_as_generic) {
		entry.is_generic = true;
	}

	if (entry.type == Type::invalid) {
		ERROR("[EDSReader::parse_var] "<<entry.name<<": Ignoring entry due to unsupported data type.");
		return true;
		// TODO: return false; ? At the moment, unsupported entries are not considered as error.
	}

	const Address address = Address{entry.index,entry.subindex};

	// --- insert entry --- //

	if (!Config::eds_reader_just_add_mappings) {

		// Resolve name conflics...

		while (m_name_to_address.count(var_name)>0) {

			WARN("[EDSReader::parse_var] Entry "<<var_name<<" already exists. Adding or increasing counter.");

			try {
				std::smatch matches;
				if (std::regex_match(var_name, matches, std::regex("^(.+)_([[:xdigit:]]{1,3})$"))) {
					assert(matches.size()>2);
					uint8_t count = Utils::decstr_to_uint(matches[2]);
					++count;
					var_name = std::string(matches[1])+"_"+std::to_string(count);
				} else {
					var_name = var_name+"_1";
				}
			} catch (std::regex_error& e) {
				WARN("[EDSReader::parse_var] "<<parse_regex_error(e.code(), var_name));
				return false;
			}

			DEBUG_LOG("[EDSReader::parse_var] New entry name: "<<var_name);
			entry.name = var_name;

		}

		DEBUG_LOG("[EDSReader::parse_var] Inserting entry "<<var_name<<".");

		// The following line isn't possible because STL would require copy constructor:
		// m_dictionary[Address{entry.index,entry.subindex}] = std::move(entry);

		m_dictionary.insert(std::make_pair(address, std::move(entry)));
		m_name_to_address.insert(std::make_pair(var_name,address));

	} else {

		// Entering this path means that a generic EDS file is loaded on top of
		// a manufacturer-specific dictionary. Only additional generic
		// entry names shall be added.

		if (m_dictionary.count(address)>0) {
			// entry exists.
			if (m_dictionary[address].name != var_name) {
				DEBUG_LOG("[EDSReader::parse_var] Manufacturer-specific entry name \""<<m_dictionary[address].name<<"\" differs from CiA standard \""<<var_name<<"\".");
				if (m_name_to_address.count(var_name)>0) {
					WARN("[EDSReader::parse_var] Conflict with existing mapping \""<<var_name<<"\"->0x"
						<<std::hex<<m_name_to_address[var_name].index<<"sub"<<std::dec<<m_name_to_address[var_name].subindex<<".");
					DUMP_HEX(index);
					DUMP_HEX(subindex);
				} else {
					m_name_to_address.insert(std::make_pair(var_name,address));
					DEBUG_LOG("[EDSReader::parse_var] Added additional name-to-address mapping.");
				}
			} else {
				DEBUG_LOG_EXHAUSTIVE("[EDSReader::parse_var] Standard conformal entry: "<<var_name);
			}
		} else {
			DEBUG_LOG_EXHAUSTIVE("[EDSReader::parse_var] Standard entry not available in manufacturer-specific EDS: "<<var_name);
		}
	}

	// new method:
	/*const Address address = Address{entry.index,entry.subindex}:
	if (m_dictionary.count(address)>0) {
		// replace entry
		// TODO: keep generic name?

		std::string existing_name = m_dictionary[address].name;
		if (existing_name != var_name) {
			// insert additional name
			if (m_name_to_address.count(var_name)>0) {
				const Entry& conflicting_entry = m_dictionary.at(m_name_to_address[var_name]);
				WARNING("Conflict! Entry name \""<<var_name<<"\" already exists for Entry 0x"<<std::hex<<conflicting_entry.index<<"sub"
					<<std::dec<<conflicting_entry.subindex<<". Erasing old mapping...");
				m_name_to_address.erase(var_name);
			}
		}
		
		DEBUG_LOG("[EDSReader::parse_var] Entry already exists "<<var_name<<".");

		// C++17: insert_or_assign...
		m_dictionary.erase(address);
	}

	m_dictionary.insert(std::make_pair(address, std::move(entry)));
	m_name_to_address.insert(std::make_pair(var_name,address));*/

	return true;

}

bool EDSReader::parse_array_or_record(const std::string& section, uint16_t index) {

	std::string array_name = Utils::escape(trim(m_ini.get(section+".ParameterName", "")));

	if (array_name.empty()) {
		ERROR("[EDSReader::parse_array_or_record] Field ParameterName missing");
		return false;
	}

	DEBUG_LOG_EXHAUSTIVE("[EDSReader::parse_array_or_record] Parsing array/record "<<section<<": "<<array_name);

	for (const auto& section_node : m_ini) {
		const std::string& section_name = section_node.first;
		//const boost::property_tree::ptree& parameters = section_node.second;

		if (boost::starts_with(section_name, section)) {

			DEBUG_LOG_EXHAUSTIVE("[EDSReader::parse_array_or_record] Found record/array entry: "<<section_name);

			try {

				std::smatch matches;

				if (std::regex_match(section_name, matches, std::regex("([[:xdigit:]]{1,4})sub([[:xdigit:]]{1,2})"))) {

					assert(matches.size()>2);
					assert(Utils::hexstr_to_uint(matches[1])==index);
					uint8_t subindex = Utils::hexstr_to_uint(matches[2]);
					bool success = parse_var(section_name, index, subindex, array_name);
					if (!success) {
						ERROR("[EDSReader::parse_array_or_record] Malformed variable entry: "<<section_name);
						return false;
					}

				} else if (section_name == section) {
					// ignore own entry
					continue;
				} else {
					ERROR("[EDSReader::parse_array_or_record] Malformed array entry: "<<section_name);
					return false;
				}

			} catch (std::regex_error& e) {
				ERROR("[EDSReader::parse_array_or_record] "<<parse_regex_error(e.code(), section_name));
				return false;
			}


		}

	}

	return true;

}

std::string EDSReader::parse_regex_error(const std::regex_constants::error_type& etype, const std::string element_name) const {
	std::string result;
	switch (etype) {
		case std::regex_constants::error_collate:
			result = "error_collate: invalid collating element request";
			break;
		case std::regex_constants::error_ctype:
			result = "error_ctype: invalid character class";
			break;
		case std::regex_constants::error_escape:
			result = "error_escape: invalid escape character or trailing escape";
			break;
		case std::regex_constants::error_backref:
			result = "error_backref: invalid back reference";
			break;
		case std::regex_constants::error_brack:
			result = "error_brack: mismatched bracket([ or ])";
			break;
		case std::regex_constants::error_paren:
			result = "error_paren: mismatched parentheses(( or ))";
			break;
		case std::regex_constants::error_brace:
			result = "error_brace: mismatched brace({ or })";
			break;
		case std::regex_constants::error_badbrace:
			result = "error_badbrace: invalid range inside a { }";
			break;
		case std::regex_constants::error_range:
			result = "erro_range: invalid character range(e.g., [z-a])";
			break;
		case std::regex_constants::error_space:
			result = "error_space: insufficient memory to handle this regular expression";
			break;
		case std::regex_constants::error_badrepeat:
			result = "error_badrepeat: a repetition character (*, ?, +, or {) was not preceded by a valid regular expression";
			break;
		case std::regex_constants::error_complexity:
			result = "error_complexity: the requested match is too complex";
			break;
		case std::regex_constants::error_stack:
			result = "error_stack: insufficient memory to evaluate a match";
			break;
		default:
			result = "";
			break;
	}
	result += " in element " + element_name;
	return result;
}

std::string EDSReader::trim(const std::string& str) {
	std::string result = str;
	size_t found = result.find("#");
	if (found!=std::string::npos) {
		result = result.substr(0,found);
	}
	boost::algorithm::trim(result);
	return result;
}

} // end namespace kaco
