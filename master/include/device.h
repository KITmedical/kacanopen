/*
 * Copyright (c) 2015-2016, Thomas Keh
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
#include "types.h"
#include "receive_pdo_mapping.h"
#include "transmit_pdo_mapping.h"
#include "access_method.h"
#include "eds_library.h"
#include "eds_reader.h"

#include <vector>
#include <unordered_map>
#include <string>
#include <chrono>
#include <functional>
#include <mutex>

namespace kaco {

	/// This class represents a CanOpen slave device in the network.
	///
	/// It provides easy access to it's object dictionary entries via
	/// addesses or names. There is a locally cached version of the
	/// device's object dictionary. This allows for dynamically mapping
	/// entries to transmit and receive PDOs. This abstracts away the
	/// underlaying communication method when manipulating the dictionary.
	/// The device object must know the structure of the slave's dictionary
	/// for these features, however, it can be fetched automatically from
	/// an EDS file or from KaCanOpen's EDS library.
	///
	/// Call print_dictionary() if you want to know which entries are
	/// accessible by name.
	///
	/// ### Remarks on thread-safety:
	///
	///   There are three classes of methods:
	///
	///     (1) Methods which manipulate the dictionary structure
	///
	///     (2) Methods which access dictionary entries
	///
	///     (3) Methods which only access Core (private)
	///
	///    Methods in (3) as well as get_node_id() are always thread-safe.
	///
	///    Methods in (2) are thread-safe if they are not intermixed with
	///    methods in (1).
	///
	///    Methods in (1) should be run in sequence before accessing
	///    dictionary entries (group 2).
	class Device {

	public:

		/// Type of a operation. See Profiles::Operation in profiles.h.
		using Operation = std::function<Value(Device&,const Value&)>;

		/// Constructor.
		/// \param core Reference of a Core instance
		/// \param node_id ID of the represented device
		Device(Core& core, uint8_t node_id);

		/// Copy constructor deleted because of callbacks with self-references.
		Device(const Device&) = delete;

		/// Move constructor deleted because of callbacks with self-references.
		Device(Device&&) = delete;

		/// Destructor
		/// \todo Something to shut down? NMT?
		~Device();

		/// Returns the node ID of the device.
		/// \remark thread-safe
		uint8_t get_node_id() const;

		/// \name (1) Methods which manipulate the dictionary structure
		///@{

		/// Starts the node via NMT protocol and loads
		/// mandatory entries, operations, and constants.
		/// \todo Add m_started member?
		void start();

		/// Tries to load the most specific EDS file available in KaCanOpen's internal EDS library.
		/// This is either device specific, CiA profile specific, or mandatory CiA 301.
		/// \returns Path to the loaded EDS file.
		/// \throws canopen_error if mandatory CiA 301 dictionary entries cannot be loaded.
		std::string load_dictionary_from_library();

		/// Loads the dictionary from a custom EDS file.
		/// \param path A filesystem path where the EDS library can be found.
		/// \throws canopen_error if the EDS file cannot be loaded.
		void load_dictionary_from_eds(const std::string& path);

		/// Loads convenience operations associated with the device profile.
		/// \returns true, if successful
		bool load_operations();

		/// Adds a convenience operation.
		void add_operation(const std::string& coperation_name, const Operation& operation);

		/// Loads constants associated with the device profile.
		/// \returns true, if successful
		bool load_constants();

		/// Adds a constant.
		void add_constant(const std::string& constant_name, const Value& constant);

		///@}

		/// \name (2) Methods which access dictionary entries
		///@{

		/// Returns true if the entry is contained in the device dictionary.
		/// \param entry_name Name of the dictionary entry
		/// \return True if entry_name is contained in the device dictionary.
		/// \todo Overload with index+subindex.
		bool has_entry(const std::string& entry_name);

		/// Returns true if the entry is contained in the device dictionary.
		/// \param index Index of the dictionary entry.
		/// \param subindex Sub-index of the dictionary entry. Default is zero.
		/// \return True if entry_name is contained in the device dictionary.
		bool has_entry(const uint16_t index, const uint8_t subindex = 0);

		/// Returns the type of a dictionary entry identified by name as it is defined in the local dictionary.
		/// \param entry_name Name of the dictionary entry
		/// \throws dictionary_error if there is no entry with the given name
		/// \todo Overload with index+subindex.
		Type get_entry_type(const std::string& entry_name);

		/// Returns the type of a dictionary entry identified by name as it is defined in the local dictionary.
		/// \param index Index of the dictionary entry.
		/// \param subindex Sub-index of the dictionary entry. Default is zero.
		/// \throws dictionary_error if there is no entry with the given name
		Type get_entry_type(const uint16_t index, const uint8_t subindex = 0);

		/// Gets the value of a dictionary entry by name internally.
		/// If there is no cached value or the entry is configured to send an SDO on request, the new value is fetched from the device via SDO.
		/// Otherwise it returns the cached value. This makes sense, if a Reveive PDO is configured on the corresponding entry.
		/// \param entry_name Name of the dictionary entry
		/// \param access_method Method of value retrival
		/// \throws dictionary_error if there is no entry with the given name.
		/// \throws sdo_error
		/// \todo check access_type from dictionary
		const Value& get_entry(const std::string& entry_name, const ReadAccessMethod access_method = ReadAccessMethod::use_default);

		/// Gets the value of a dictionary entry by name internally.
		/// If there is no cached value or the entry is configured to send an SDO on request, the new value is fetched from the device via SDO.
		/// Otherwise it returns the cached value. This makes sense, if a Reveive PDO is configured on the corresponding entry.
		/// \param index Index of the dictionary entry.
		/// \param subindex Sub-index of the dictionary entry. Default is zero.
		/// \param access_method Method of value retrival
		/// \throws dictionary_error if there is no entry with the given name.
		/// \throws sdo_error
		/// \todo check access_type from dictionary
		const Value& get_entry(const uint16_t index, const uint8_t subindex = 0, const ReadAccessMethod access_method = ReadAccessMethod::use_default);

		/// Sets the value of a dictionary entry by name internally.
		/// If the entry is configured to send an SDO on update, the new value is also sent to the device via SDO.
		/// If a PDO is configured on the corresponding entry, it will from now on use the new value stored internally.
		/// \param entry_name Name of the dictionary entry
		/// \param value The value to write, wrapped in a Value object. The Value class has implicit cast constructors for all supported data types.
		/// \param access_method How, where and when to write the value.
		/// \throws dictionary_error if there is no entry with the given name.
		/// \throws sdo_error
		/// \todo check access_type from dictionary
		void set_entry(const std::string& entry_name, const Value& value, const WriteAccessMethod access_method = WriteAccessMethod::use_default);

		/// Sets the value of a dictionary entry by name internally.
		/// If the entry is configured to send an SDO on update, the new value is also sent to the device via SDO.
		/// If a PDO is configured on the corresponding entry, it will from now on use the new value stored internally.
		/// \param index Index of the dictionary entry.
		/// \param subindex Sub-index of the dictionary entry.
		/// \param value The value to write, wrapped in a Value object. The Value class has implicit cast constructors for all supported data types.
		/// \param access_method How, where and when to write the value.
		/// \throws dictionary_error if there is no entry with the given name.
		/// \throws sdo_error
		/// \todo check access_type from dictionary
		void set_entry(const uint16_t index, const uint8_t subindex, const Value& value, const WriteAccessMethod access_method = WriteAccessMethod::use_default);

		/// Adds an entry to the dictionary. You have to take care that exactly this entry exists on the device for yourself!
		/// \param index Index
		/// \param subindex Sub-index
		/// \param name Name
		/// \param type Data type
		/// \param access_type Access rights
		/// \throws canopen_error if entry with this name or index already exists.
		void add_entry(const uint16_t index, const uint8_t subindex, const std::string& name, const Type type, const AccessType access_type);

		/// Returns the CiA profile number
		/// \throws sdo_error
		/// \todo Make this an operation?
		uint16_t get_device_profile_number();

		/// Executes a convenience operation. It must exist due to a previous
		/// load_operations() or add_operation() call.
		/// \param operation_name Name of the operation.
		/// \param argument Optional argument to be passed to the operation.
		/// \returns The result value of the operation. Invalid value in case there is no result.
		/// \throws dictionary_error if operation is not available
		Value execute(const std::string& operation_name, const Value& argument = m_dummy_value);

		/// Returns a constant. It must exist due to a previous
		/// load_constants() or add_constant() call.
		/// \throws dictionary_error if constant is not available
		const Value& get_constant(const std::string& constant_name) const;

		/// Adds a receive PDO mapping. This means values sent by the device via PDO are saved into the dictionary cache.
		/// \param cob_id COB-ID of the PDO
		/// \param entry_name Name of the dictionary entry
		/// \param offset index of the first mapped byte in the PDO message
		/// \throws dictionary_error if there is no entry with the given name.
		/// \todo Add index/subindex overload?
		void add_receive_pdo_mapping(uint16_t cob_id, const std::string& entry_name, uint8_t offset);


    void add_receive_pdo_mapping(uint16_t cob_id, const std::string& entry_name, uint8_t offset,
                                 std::function<void(const ReceivePDOMapping&, std::vector<uint8_t>)> funtion);

		/// Adds a transmit PDO mapping. This means values from the dictionary cache are sent to the device.
		///
		/// Example:
		/// 
		/// 	The following command maps the "Controlword" entry (2 bytes, see CiA 402)
		///		to the first two bytes of the PDO channel with cob_id 0x206 (RPDO1 of CANOpen device 6),
		///		and the "Target Position" entry (4 bytes, see CiA 402) to bytes 2-5 of this PDO channel.
		///		The PDO is sent whenever one of the values is changed via set_entry("Controlword", ...)
		///		or set_entry("Target Position", ...),
		///
		/// 	device.add_transmit_pdo_mapping(0x206, {{"Controlword", 0, 0},{"Target Position", 2, 0}});
		///
		/// \param cob_id The cob_id of the PDO to transmit
		/// \param mappings A vector of mappings. A mapping maps a dictionary entry (by name) to a part of a PDO (by first and last byte index)
		/// \param transmission_type Send PDO "ON_CHANGE" or "PERIODIC"
		/// \param repeat_time If transmission_type==TransmissionType::PERIODIC, PDO is sent periodically according to repeat_time.
		/// \throws dictionary_error
		void add_transmit_pdo_mapping(uint16_t cob_id, const std::vector<Mapping>& mappings, TransmissionType transmission_type=TransmissionType::ON_CHANGE, std::chrono::milliseconds repeat_time=std::chrono::milliseconds(0));

		/// Prints the dictionary together with currently cached values to command line.
		void print_dictionary() const;

		/// Fetches all dictionary entries from the device.
		/// Afterwards, all values exist in cache and can for
		/// example be printed via print_dictionary().
		void read_complete_dictionary();

		///@}

	private:

		/// \name (3) Methods which only access Core
		///@{

		/// Gets the value of a dictionary entry by index via SDO
		/// It does not change the corresponding internal value and therefore the new value
		/// cannot be used by Transmit PDOs.
		/// \param index Dictionary index of the entry
		/// \param subindex Subindex of the entry
		/// \param type Data type of the entry
		/// \throws sdo_error
		/// \remark thread-safe
		Value get_entry_via_sdo(uint32_t index, uint8_t subindex, Type type);

		/// Sets the value of a dictionary entry by index via SDO
		/// It does not change the corresponding internal value and therefore the new value
		/// cannot be used by Transmit PDOs.
		/// \param index Dictionary index of the entry
		/// \param subindex Subindex of the entry
		/// \param value The value to write, wrapped in a Value object. The Value class has implicit cast constructors for all supported data types.
		/// \throws sdo_error
		/// \remark thread-safe
		void set_entry_via_sdo(uint32_t index, uint8_t subindex, const Value& value);

		///@}

		/// Loads most specific CiA standard profile.
		void load_cia_dictionary();

		void pdo_received_callback(const ReceivePDOMapping& mapping, std::vector<uint8_t> data);

		static const bool debug = false;

		Core& m_core;
		uint8_t m_node_id;

		std::unordered_map<Address, Entry> m_dictionary;
		std::unordered_map<std::string, Address> m_name_to_address;

		std::unordered_map<std::string, Operation> m_operations;
		std::unordered_map<std::string, Value> m_constants;
		std::forward_list<ReceivePDOMapping> m_receive_pdo_mappings;
		std::mutex m_receive_pdo_mappings_mutex;
		std::forward_list<TransmitPDOMapping> m_transmit_pdo_mappings;
		std::mutex m_transmit_pdo_mappings_mutex;
		static const Value m_dummy_value;
		EDSLibrary m_eds_library;

    std::vector<uint16_t> cob_ids_;

	};

} // end namespace kaco
