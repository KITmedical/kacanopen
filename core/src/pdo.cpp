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

#include "pdo.h"
#include "core.h"
#include "logger.h"

#include <cassert>
#include <iostream>

namespace kaco {

PDO::PDO(Core& core) : m_core(core) {}

void PDO::process_incoming_message(const Message& message) const {
  uint16_t cob_id = message.cob_id;
  uint8_t node_id = message.get_node_id();
  std::vector<uint8_t> data;

  DEBUG_LOG("Received transmit PDO with cob_id 0x"
            << std::hex << cob_id << " (usually from node 0x" << node_id
            << ") length = " << message.len);

  for (unsigned i = 0; i < message.len; ++i) {
    data.push_back(message.data[i]);
  }

  // call registered callbacks
  bool found_callback = false;
  {
    std::lock_guard<std::mutex> scoped_lock(m_receive_callbacks_mutex);
    for (const PDOReceivedCallback& callback : m_receive_callbacks) {
      if (callback.cob_id == cob_id) {
        found_callback = true;
        // This is not async because callbacks are only registered internally.
        // Copy data vector because there can be multiple callbacks for this
        // PDO.
        callback.callback(data);
      }
    }
  }

  DEBUG(if (!found_callback) {
    PRINT("PDO is unassigned. Here is the data (LSB):");
    for (unsigned i = 0; i < data.size(); ++i) {
      std::cout << std::hex << (unsigned)data[i] << " ";
    }
    std::cout << std::endl;
  })
}

void PDO::send(uint16_t cob_id, const std::vector<uint8_t>& data) {
  assert(data.size() <= 8 &&
         "[PDO::send] A PDO message can have at most 8 data bytes.");

  Message message;
  message.cob_id = cob_id;
  message.rtr = false;
  message.len = data.size();
  for (uint8_t i = 0; i < data.size(); ++i) {
    message.data[i] = data[i];
  }

  DEBUG_LOG_EXHAUSTIVE("Sending the following PDO:");
  DEBUG_EXHAUSTIVE(message.print();)

  m_core.send(message);
}

void PDO::add_pdo_received_callback(uint16_t cob_id,
                                    PDOReceivedCallback::Callback callback) {
  std::lock_guard<std::mutex> scoped_lock(m_receive_callbacks_mutex);
  m_receive_callbacks.push_back({cob_id, callback});
}

void PDO::remove_pdo_received_callback(uint16_t cob_id) {
  std::lock_guard<std::mutex> scoped_lock(m_receive_callbacks_mutex);
  for (auto i = m_receive_callbacks.begin(); i != m_receive_callbacks.end();
       i++)
    if ((*i).cob_id == cob_id) {
      m_receive_callbacks.erase(i);
      break;
    }
}

}  // end namespace kaco
