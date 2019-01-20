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

#include "message.h"

#include <vector>
#include <functional>
#include <future>
#include <forward_list>
#include <mutex>
#include <unordered_map>
#include <thread>

namespace kaco {
	
	// forward declaration
	class Core;

	/// \class NMT
	///
	/// This class implements the CanOpen NMT protocol
	class NMT {

	public:
		
		/// Type of a device alive callback function
		/// Important: Never call register_device_alive_callback()
		///   from within (-> deadlock)!
		using DeviceAliveCallback = std::function< void(const uint8_t node_id) >;
		
		/// Type of a new device callback function
		/// \deprecated
		using NewDeviceCallback = DeviceAliveCallback;

		/// NMT commands
		enum class Command : uint8_t {
			start_node = 0x01,
			stop_node = 0x02,
			enter_preoperational = 0x80,
			reset_node = 0x81,
			reset_communication = 0x82
		};

		/// Constructor.
		/// \param core Reference to the Core
		NMT(Core& core);

		/// Copy constructor deleted because of mutexes.
		NMT(const NMT&) = delete;

    ~NMT();

		/// Process incoming NMT message.
		/// \param message The received CanOpen message.
		/// \remark thread-safe
		void process_incoming_message(const Message& message);

		/// Sends a NMT message to a given device
		/// \param node_id Node id of the device.
		/// \param cmd The NMT command.
		/// \remark thread-safe
		void send_nmt_message(uint8_t node_id, Command cmd);

		/// Sends a broadcast NMT message
		/// \param cmd The NMT command.
		/// \remark thread-safe
		void broadcast_nmt_message(Command cmd);

		/// Resets all nodes in the network.
		/// \remark thread-safe
		void reset_all_nodes();

		/// Discovers nodes in the network via node guard protocol.
		/// \remark thread-safe
		void discover_nodes();

		/// Registers a callback which will be called when a slave sends
		/// it's state via NMT and the state indicates that the device
		/// is alive. This can be uses as a "new device" callback.
		/// \remark thread-safe
		void register_device_alive_callback(const DeviceAliveCallback& callback);

		/// Registers a callback which will be called when a new slave device is discovered.
		/// \remark thread-safe
		/// \deprecated
		void register_new_device_callback(const NewDeviceCallback& callback);

    void check_alive_devices();
    void register_device_dead_callback(const DeviceAliveCallback& callback);

	private:

		static const bool debug = false;
		Core& m_core;

		/// \todo rename to device_alive_callback
		std::vector<NewDeviceCallback> m_device_alive_callbacks;
    std::vector<NewDeviceCallback> m_device_dead_callbacks;
		mutable std::mutex m_device_alive_callbacks_mutex;

		static const bool m_cleanup_futures = true;
		std::forward_list<std::future<void>> m_callback_futures; // forward_list because of remove_if
		mutable std::mutex m_callback_futures_mutex;

    std::unordered_map<size_t, bool> alive_devices_;
    bool thread_alive_;
    std::thread alive_devices_thread_;
	};

} // end namespace kaco
