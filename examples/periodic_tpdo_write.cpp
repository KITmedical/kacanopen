/*
 * Copyright (c) 2018-2019, Musarraf Hossain
 * All rights reservd.
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

#include <signal.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include "../tools/include/parse_sdo.h"
#include "../tools/src/parse_sdo.cpp"
#include "core.h"
#include "device.h"
#include "logger.h"
#include "master.h"

static volatile int keepRunning = 1;

void intHandler(int dummy) {
  (void)dummy;
  keepRunning = 0;
}

int main() {
  // Signal handleing
  signal(SIGINT, intHandler);

  // ----------- //
  // Preferences //
  // ----------- //

  // A Roboteq motor driver was used to test this program //

  // The node ID of the slave we want to communicate with.
  const uint8_t node_id = 4;

  // Set the name of your CAN bus. "slcan0" is a common bus name
  // for the first SocketCAN device on a Linux system.
  const std::string busname = "can0";

  // Set the baudrate of your CAN bus. Most drivers support the values
  // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
  const std::string baudrate = "500K";

  // -------------- //
  // Initialization //
  // -------------- //
  // Create core.
  kaco::Core core;
  std::cout << "Starting Core (connect to the driver and start the receiver "
               "thread)..."
            << std::endl;
  if (!core.start(busname, baudrate)) {
    std::cout << "Starting core failed." << std::endl;
    return EXIT_FAILURE;
  }
  // Create device pointer
  std::shared_ptr<kaco::Device> device;
  // This will be set to true by the callback below.
  bool found_node = false;
  bool node_initialized = false;
  int channel1_speed_ref = 0;
  bool max = false;

  std::cout << "Registering a callback which is called when a device is "
               "detected via NMT..."
            << std::endl;
  core.nmt.register_device_alive_callback([&](
      const uint8_t new_node_id) mutable {
    // Check if this is the node we are looking for.
    if (new_node_id == node_id) {
      if (!found_node) {
        found_node = true;

        device.reset(new kaco::Device(core, node_id));
        device->load_dictionary_from_eds(
            "/home/mhs/Desktop/EDS/roboteq_motor_controllers_v80beta.eds");

        device->set_entry("cmd_cango/cmd_cango_1", channel1_speed_ref,
                          kaco::WriteAccessMethod::pdo);
        device->add_transmit_pdo_mapping(
            0x200 + node_id, {{"cmd_cango/cmd_cango_1", 0}},
            kaco::TransmissionType::PERIODIC, std::chrono::milliseconds(250));

        node_initialized = true;
      }
    }
  });

  while (keepRunning) {
    if (node_initialized) {
      // std::cout << " I am running." << std::endl;
      // Prepare the commands; master side tpdo1 and tpdo2
      if (3000 > channel1_speed_ref && max == false) {
        // channel1_speed_ref++;
        channel1_speed_ref = channel1_speed_ref + 100;
        if (3000 == channel1_speed_ref) {
          max = true;
        }
      }
      if (-3000 < channel1_speed_ref && max == true) {
        // channel1_speed_ref--;
        channel1_speed_ref = channel1_speed_ref - 100;
        if (-3000 == channel1_speed_ref) {
          max = false;
        }
      }
      std::cout << "Channel 1 speed command = " << std::dec
                << channel1_speed_ref << std::endl;

      device->set_entry("cmd_cango/cmd_cango_1", (channel1_speed_ref),
                        kaco::WriteAccessMethod::pdo);
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
}
