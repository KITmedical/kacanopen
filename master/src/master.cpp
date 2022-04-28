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
 
#include "master.h"
#include "core.h"
#include "logger.h"

#include <memory>

namespace kaco {

Master::Master() {
	m_device_alive_callback_functional = std::bind(&Master::device_alive_callback, this, std::placeholders::_1);
	core.nmt.register_device_alive_callback(m_device_alive_callback_functional);
}

Master::~Master() {
	if (m_running) {
		stop();
	}
}

bool Master::start(const std::string busname, const std::string& baudrate) {
	bool success = core.start(busname, baudrate);
	if (!success) {
		return false;
	}
	m_running = true;
	//core.nmt.reset_all_nodes();
	// TODO: let user do this explicitly?
	core.nmt.discover_nodes();
	return true;
}

bool Master::start(const std::string busname, const unsigned baudrate) {
	bool success = core.start(busname, baudrate);
	if (!success) {
		return false;
	}
	m_running = true;
	//core.nmt.reset_all_nodes();
	// TODO: let user do this explicitly?
	core.nmt.discover_nodes();
	return true;
}

void Master::stop() {
	m_running = false;
	core.stop();
}

size_t Master::num_devices() const {
	std::lock_guard<std::mutex> scoped_lock(m_devices_mutex);
	return m_devices.size();
}

Device& Master::get_device(size_t index) const {
	std::lock_guard<std::mutex> scoped_lock(m_devices_mutex);
	assert(m_devices.size()>index);
	return *(m_devices.at(index).get());
}

void Master::device_alive_callback(const uint8_t node_id) {
	std::lock_guard<std::mutex> scoped_lock(m_devices_mutex);
	if (!m_device_alive.test(node_id)) {
		m_device_alive.set(node_id);
		m_devices.emplace_back(new Device(core, node_id));
	} else {
		WARN("Device with node ID "<<node_id<<" already exists. Ignoring...");
	}
	
}


} // end namespace kaco
