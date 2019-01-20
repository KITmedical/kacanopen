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
 
#include "nmt.h"
#include "core.h"
#include "logger.h"

#include <iostream>
#include <cstdint>
#include <future>
#include <chrono>

namespace kaco {

NMT::NMT(Core& core) 
  : m_core(core),
    thread_alive_(true),
  alive_devices_thread_(&NMT::check_alive_devices, this)
{
}

NMT::~NMT(){
  thread_alive_ = false;
  alive_devices_thread_.join();
}

void NMT::send_nmt_message(uint8_t node_id, Command cmd) {
	DEBUG_LOG("Set NMT state of "<<(unsigned)node_id<<" to "<<static_cast<uint32_t>(cmd));
	const Message message = { 0x0000, false, 2, {static_cast<uint8_t>(cmd),node_id,0,0,0,0,0,0} };
	m_core.send(message);
}

void NMT::broadcast_nmt_message(Command cmd) {
	send_nmt_message(0, cmd);
}

void NMT::reset_all_nodes() {
	//broadcast_nmt_message(Command::reset_node);
	// TODO check node_id range
	const auto pause = std::chrono::milliseconds(CONSECUTIVE_SEND_PAUSE_MS);
	for (size_t node_id = 1; node_id < 239; ++node_id) {
		send_nmt_message(node_id, Command::reset_node);
		std::this_thread::sleep_for(pause);
	}
}

void NMT::discover_nodes() {
	// TODO check node_id range
	const auto pause = std::chrono::milliseconds(CONSECUTIVE_SEND_PAUSE_MS);
	for (size_t node_id = 1; node_id < 239; ++node_id) {
		// Protocol node guarding. See CiA 301. All devices will answer with their state via NMT.
		uint16_t cob_id = 0x700+node_id;
		const Message message = { cob_id, true, 0, {0,0,0,0,0,0,0,0} };
		m_core.send(message);
		std::this_thread::sleep_for(pause);
	}
}

void NMT::process_incoming_message(const Message& message) {

	DEBUG_LOG("NMT Error Control message from node "
		<<(unsigned)message.get_node_id()<<".");
	
	uint8_t data = message.data[0];
	uint8_t state = data&0x7F;

	if (message.rtr) {
		DEBUG_LOG("NMT: Ignoring remote transmission request.");
		DEBUG_DUMP(message.cob_id);
		return;
	}

	//bool toggle_bit = data>>7;
	//DEBUG_DUMP(toggle_bit);

	switch (state) {
		
		case 0:
		case 2:
		case 3:
    case 5:
      // If the device is in our map, we set it to true
      if (alive_devices_.find(message.get_node_id()) != alive_devices_.end())
        alive_devices_[message.get_node_id()] = true;

		case 127: {
			// device is alive
			// cleaning up old futures
			if (m_cleanup_futures) {
				std::lock_guard<std::mutex> scoped_lock(m_callback_futures_mutex);
				m_callback_futures.remove_if([](const std::future<void>& f) {
					// return true if callback has finished it's computation.
					return (f.wait_for(std::chrono::steady_clock::duration::zero())==std::future_status::ready);
				});
			}
			// TODO: this should be device_alive callback
			{
				std::lock_guard<std::mutex> scoped_lock(m_device_alive_callbacks_mutex);
				for (const auto& callback : m_device_alive_callbacks) {
					DEBUG_LOG("Calling new device callback (async)");
					// The future returned by std::async has to be stored,
					// otherwise the immediately called future destructor
					// blocks until callback has finished.
					std::lock_guard<std::mutex> scoped_lock(m_callback_futures_mutex);
					m_callback_futures.push_front(
						std::async(std::launch::async, callback, message.get_node_id())
					);
				}
        // Register into our alive device vector
        alive_devices_.insert({message.get_node_id(), true});
			}
			break;
		}

		default: {
			// TODO disconnect device
		}

	}

	switch (state) {
		
		case 0: {
			DEBUG_LOG("New state is Initialising");
			break;
		}
		
		case 1: {
			DEBUG_LOG("New state is Disconnected");
			break;
		}
		
		case 2: {
			DEBUG_LOG("New state is Connecting");
			break;
		}
		
		case 3: {
			DEBUG_LOG("New state is Preparing");
			break;
		}
		
		case 4: {
			DEBUG_LOG("New state is Stopped");
			break;
		}
		
		case 5: {
			DEBUG_LOG("New state is Operational");
			break;
		}
		
		case 127: {
			DEBUG_LOG("New state is Pre-operational");
			break;
		}
		
		default: {
			DEBUG_LOG("New state is unknown: "<<(unsigned)state);
			break;
		}

	}

}

void NMT::register_device_alive_callback(const DeviceAliveCallback& callback) {
	std::lock_guard<std::mutex> scoped_lock(m_device_alive_callbacks_mutex);
	m_device_alive_callbacks.push_back(callback);
}

void NMT::register_new_device_callback(const NewDeviceCallback& callback) {
  register_device_alive_callback(callback);
}

void NMT::check_alive_devices()
{
  while (thread_alive_){
    // Set all devices to dead
    for (auto& device: alive_devices_)
      device.second = false;

    // Sleep for some time
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Check if they are alive
    for (auto& device: alive_devices_){
      if (!device.second){
        // Remove it from the map
        alive_devices_.erase(device.first);
        // Call dead callbacks
        std::lock_guard<std::mutex> scoped_lock(m_device_alive_callbacks_mutex);
        for (const auto& callback : m_device_dead_callbacks) {
          DEBUG_LOG("Calling dead device callback (async)");
          // The future returned by std::async has to be stored,
          // otherwise the immediately called future destructor
          // blocks until callback has finished.
          std::lock_guard<std::mutex> scoped_lock(m_callback_futures_mutex);
          m_callback_futures.push_front(
            std::async(std::launch::async, callback, device.first)
          );
        }
      }
    }
  }
}

void NMT::register_device_dead_callback(const DeviceAliveCallback& callback) {
  std::lock_guard<std::mutex> scoped_lock(m_device_alive_callbacks_mutex);
  m_device_dead_callbacks.push_back(callback);
}

} // end namespace kaco
