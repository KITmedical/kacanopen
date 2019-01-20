/*
 * Copyright (c) 2018-2019, Musarraf Hossain
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

#include <signal.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include "canopen_error.h"
#include "core.h"
#include "device.h"
#include "device_tpdo.h"
#include "logger.h"
#include "master.h"
#include "parse_sdo.h"
#include "receive_pdo_mapping.h"

static volatile int keepRunning = 1;

void intHandler(int dummy) {
  (void)dummy;
  keepRunning = 0;
}
// void qry_abspeed_channel_1_callback(const kaco::ReceivePDOMapping& mapping,
//                                    std::vector<uint8_t> data) {
//  std::cout << "hola" << std::endl;
//}

int main() {
  // Signal handleing
  signal(SIGINT, intHandler);

  // ----------- //
  // Preferences //
  // ----------- //

  // A simulated CANopenSocket CiA-401 Slave was used to test this program //

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
  std::cout << "Registering a callback which is called when a device is "
               "detected via NMT..."
            << std::endl;
   // core.pdo.send(0x204, ch1_speed); // raw pdo message
  core.nmt.register_device_alive_callback(
      [&](const uint8_t new_node_id) mutable {
        // Check if this is the node we are looking for.
        if (new_node_id == node_id) {
          if (!found_node) {
            found_node = true;
            device.reset(new kaco::Device(core, node_id));
            device->load_dictionary_from_eds(
                "/home/mhs/bor/test/CANopenSocket/canopend/objDict/"
                "roboteq_motor_controllers_v80beta.eds");
            device->add_transmit_pdo_mapping(0x200 + node_id, {{"Cmd_DOUT", 0}},
                                             kaco::TransmissionType::ON_CHANGE,
                                             std::chrono::milliseconds(250));
            device->add_receive_pdo_mapping(
                0x180 + node_id, "qry_abspeed/channel_1", 0);  // offset 1,
            device->add_receive_pdo_mapping(
                0x180 + node_id, "qry_abspeed/channel_2", 2);  // offset 2,
            device->add_receive_pdo_mapping(
                0x180 + node_id, "qry_batamps/channel_1", 4);  // offset 4,
            device->add_receive_pdo_mapping(
                0x180 + node_id, "qry_batamps/channel_2", 6);  // offset 6,
            // 0x21030110, 0x21030210, 0x210C0110, 0x21000110
            // 0x210C0110,0x210C0210,0x210C0110,0x210C0210
            const std::vector<uint32_t> entries_to_be_mapped{
                0x210C0110, 0x210C0210, 0x210C0110, 0x210C0210};
            // map_tpdo_in_device(tpdo4, entries_to_be_mapped, 255, 100, 500,
            // device,node_id);
            map_tpdo_in_device(tpdo3, entries_to_be_mapped, 255, 100, 500,
                               device);
            map_tpdo_in_device(tpdo2, entries_to_be_mapped, 255, 100, 500,
                               device);
            map_tpdo_in_device(tpdo1, entries_to_be_mapped, 255, 100, 500,
                               device);
            node_initialized = true;
          }
        }
      });
  uint8_t digtal_out_write = 0x0;
  // core.nmt.send_nmt_message(04,kaco::NMT::Command::reset_communication);
  while (keepRunning) {
    if (node_initialized) {
      DUMP_HEX(device->get_entry(0x1A03, 0x01, kaco::ReadAccessMethod::sdo));
      try {
        DUMP_HEX(
            device->get_entry("qry_abspeed/channel_1",
                              kaco::ReadAccessMethod::pdo_request_and_wait));
        DUMP_HEX(
            device->get_entry("qry_abspeed/channel_2",
                              kaco::ReadAccessMethod::pdo_request_and_wait));
        DUMP_HEX(
            device->get_entry("qry_batamps/channel_1",
                              kaco::ReadAccessMethod::pdo_request_and_wait));
        DUMP_HEX(
            device->get_entry("qry_batamps/channel_2",
                              kaco::ReadAccessMethod::pdo_request_and_wait));
      } catch (kaco::canopen_error exception) {
      }
      digtal_out_write++;
      device->set_entry("Cmd_DOUT", digtal_out_write,
                        kaco::WriteAccessMethod::pdo);
      if (0xFF == digtal_out_write) {
        digtal_out_write = 0x0;
      }
    }
  }
  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
}
