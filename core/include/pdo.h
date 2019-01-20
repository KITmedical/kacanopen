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

#include <vector>
#include <cstdint>
#include <functional>
#include <mutex>

#include "message.h"

namespace kaco {

	// forward declaration
	class Core;

	/// \class PDO
	///
	/// This class implements the CanOpen PDO protocol
	///
	/// All methods are thread-safe.
	class PDO {

	public:

		/// A PDO message receiver function
		/// together with it's COB-ID
		/// Important: Never call add_pdo_received_callback or
		///   process_incoming_message from within (-> deadlock)!
		struct PDOReceivedCallback {

			/// Type of the callback
			using Callback = std::function< void(std::vector<uint8_t>) >;

			/// The COB-ID of the PDO to receive
			uint16_t cob_id;

			/// The callback
			Callback callback;

		};

		/// Constructor
		/// \param core Reference to the Core
		PDO(Core& core);

		/// Copy constructor deleted because of mutexes.
		PDO(const PDO&) = delete;

		/// Handler for an incoming PDO message
		/// \param message The message from the network
		/// \todo Rename this to process_incoming_tpdo() and add process_incoming_rpdo()
		/// \remark thread-safe
		void process_incoming_message(const Message& message) const;

		/// Sends a PDO message
		/// \param cob_id COB-ID of the message to send
		/// \param data A vector containing the data bytes to send. PDOs can have most 8 bytes!
		/// \remark thread-safe
		void send(uint16_t cob_id, const std::vector<uint8_t>& data);

		/// Adds a callback which will be called when a PDO has been received with the given COB-ID.
		/// \param cob_id COB-ID to listen for
		/// \param callback Callback function, which takes a const Message reference as argument.
		/// \todo Rename this to add_tpdo_received_callback() and add add_rpdo_received_callback()
		/// \remark thread-safe
		void add_pdo_received_callback(uint16_t cob_id, PDOReceivedCallback::Callback callback);

    void remove_pdo_received_callback(uint16_t cob_id);

	private:

		static const bool debug = false;
		Core& m_core;

		std::vector<PDOReceivedCallback> m_receive_callbacks;
		mutable std::mutex m_receive_callbacks_mutex;

	};

} // end namespace kaco
