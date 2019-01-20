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
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>
#include "canopen_error.h"
#include "core.h"
#include "device.h"
#include "device_rpdo.h"
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

bool printDeviceInfo(std::shared_ptr<kaco::Device> device_) {
  try {
    auto device_type =
        device_->get_entry(0x1000, 0x0, kaco::ReadAccessMethod::sdo);
    auto device_name =
        device_->get_entry(0x1008, 0x0, kaco::ReadAccessMethod::sdo);
    auto hardware_version =
        device_->get_entry(0x1009, 0x0, kaco::ReadAccessMethod::sdo);
    auto firmware_version =
        device_->get_entry(0x100A, 0x0, kaco::ReadAccessMethod::sdo);
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "* Device Name found as '" << device_name << "'" << std::endl;
    std::cout << "* Device Type found as CiA-" << device_type << "'"
              << std::endl;
    std::cout << "* Hardware version=" << hardware_version << std::endl;
    std::cout << "* Firmware version=" << firmware_version << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    return true;
  } catch (...) {
    std::cout << "Exception occured while acquiring device information"
              << std::endl;
    return false;
  }
}

void initializeDevice(std::shared_ptr<kaco::Device> device,
                      uint16_t heartbeat_interval, uint8_t node_id) {
  // Load eds
  device->load_dictionary_from_eds(
      "/home/mhs/bor/test/CANopenSocket/canopend/objDict/"
      "roboteq_motor_controllers_v80beta.eds");

  // set the our desired heartbeat_interval time
  device->set_entry(0x1017, 0x0, heartbeat_interval,
                    kaco::WriteAccessMethod::sdo);

  device->print_dictionary();

  // Master side rpdo1 mapping
  device->add_receive_pdo_mapping(0x180 + node_id, "qry_abspeed/channel_1", 0);
  device->add_receive_pdo_mapping(0x180 + node_id, "qry_abspeed/channel_2", 2);
  device->add_receive_pdo_mapping(0x180 + node_id, "qry_batamps/channel_1", 4);
  device->add_receive_pdo_mapping(0x180 + node_id, "qry_batamps/channel_2", 6);
  // Master side rpdo2 mapping
  device->add_receive_pdo_mapping(0x280 + node_id, "qry_volts/v_int", 0);
  device->add_receive_pdo_mapping(0x280 + node_id, "qry_volts/v_bat", 2);
  device->add_receive_pdo_mapping(0x280 + node_id, "qry_volts/v_5vout", 4);
  device->add_receive_pdo_mapping(0x280 + node_id, "qry_digout", 6);
  // Master side tpdo1 mapping
  device->add_transmit_pdo_mapping(
      0x200 + node_id, {{"cmd_cango/cmd_cango_1", 0}},
      kaco::TransmissionType::ON_CHANGE, std::chrono::milliseconds(250));

  // Master side tpdo2 mapping
  device->add_transmit_pdo_mapping(
      0x300 + node_id, {{"cmd_cango/cmd_cango_2", 0}},
      kaco::TransmissionType::ON_CHANGE, std::chrono::milliseconds(250));

  // Device side tpdo1 mapping entries and mapping
  const std::vector<uint32_t> tpdo1_entries_to_be_mapped{
      0x21030110, 0x21030210, 0x210C0110, 0x210C0210};
  map_tpdo_in_device(tpdo1, tpdo1_entries_to_be_mapped, 255, 100, 250, device);

  // Device side tpdo2 mapping entries and mapping
  const std::vector<uint32_t> tpdo2_entries_to_be_mapped{
      0x210D0110, 0x210D0210, 0x210D0310,
      0x21130010};  // {0x210D0110, 0x210D0210, 0x210D0310, 0x21030110}
  map_tpdo_in_device(tpdo2, tpdo2_entries_to_be_mapped, 255, 100, 250, device);
  // Device side rpdo1 mapping entries and mapping
  const std::vector<uint32_t> rpdo1_entries_to_be_mapped{0x20000120};
  map_rpdo_in_device(rpdo1, rpdo1_entries_to_be_mapped, 255, device);

  // Device side rpdo2 mapping entries and mapping
  const std::vector<uint32_t> rpdo2_entries_to_be_mapped{0x20000220};
  map_rpdo_in_device(rpdo2, rpdo2_entries_to_be_mapped, 255, device);
}

int main() {
  // Signal handleing
  signal(SIGINT, intHandler);

  // ----------- //
  // Preferences //
  // ----------- //

  // A Roboteq motor driver was used to test this program.//

  // The node ID of the slave we want to communicate with.
  uint8_t node_id[2] = {3, 4};

  // Set the name of your CAN bus. "slcan0" is a common bus name
  // for the first SocketCAN device on a Linux system.
  const std::string busname = "can0";

  // Set the baudrate of your CAN bus. Most drivers support the values
  // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
  const std::string baudrate = "500K";

  // Set the heartbeat interval for slave device. Most drivers support the
  // values can be "125", "250", "500" and "1000" millisecond.
  const uint16_t heartbeat_interval = 250;

  // Set the heartbeat time out, after which the system should detect slave
  // disconnection; values can be "250", "500", "1000" and "2000" millisecond.
  // Temporary disabled the timeout parameters; A gloabl 2 second time is now
  // used in device_alive and device_dead callback
  // const uint16_t heartbeat_timeout = heartbeat_interval * 3;

  // -------------- //
  // Initialization //
  // -------------- //

  // Create core.
  // kaco::Core core;

  kaco::Master master;

  std::mutex device_mutex;

  std::cout << "Starting master (connect to the driver and start the receiver "
               "thread)..."
            << std::endl;
  if (!master.start(busname, baudrate)) {
    std::cout << "Starting master failed." << std::endl;
    return EXIT_FAILURE;
  }
  const size_t number_of_devices = master.num_devices();
  std::shared_ptr<kaco::Device> device[2];
  bool found_node = false;
  // memset(found_node, false, sizeof found_node);
  // memset(found_node, 0, sizeof(found_node));
  bool device_connected[2] = {false, false};

  std::cout << "Total number of device found on the bus \"" << busname
            << "\" = " << number_of_devices << std::endl;

  if (!found_node) {
    std::cout << "Initial config loop" << std::endl;
    found_node = true;
    for (size_t i = 0; i < 2; i++) {
      uint8_t temp_node_id = master.get_device(i).get_node_id();
      if (temp_node_id == node_id[0] || temp_node_id == node_id[1]) {
        device[i].reset(new kaco::Device(master.core, temp_node_id));
        std::cout << "Found device[" << i << "]"
                  << "with node id= " << temp_node_id << std::endl;
        PRINT("Starting");
        device[i]->start();
        initializeDevice(device[i], heartbeat_interval, temp_node_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        device_connected[i] = true;
      }
    }
  }
  // Create device pointer
  // std::shared_ptr<kaco::Device> device;
  // This will be set to true by the callback below.

  //  std::cout << "Registering a callback which is called when a device is "
  //               "detected via NMT..."
  //            << std::endl;

  //  core.nmt.register_device_alive_callback([&](const uint8_t new_node_id) {
  //    // Check if this is the node we are looking for.
  //    if (new_node_id == node_id) {
  //      // lock
  //      if (!found_node) {
  //        found_node = true;
  //        // Lock device mutex
  //        std::lock_guard<std::mutex> lock(device_mutex);
  //        try {
  //          // Initialize the device
  //          device.reset(new kaco::Device(core, node_id));
  //          initializeDevice(device, heartbeat_interval, node_id);
  //          device_connected = true;
  //        } catch (...) {
  //          std::cout << "Exception in device alive!" << std::endl;
  //          found_node = false;
  //          device_connected = false;
  //        }
  //      }
  //    }
  //  });
  //  core.nmt.register_device_dead_callback(
  //      [&](const uint8_t current_node_id) mutable {
  //        if (device_connected && found_node) {
  //          // Lock device mutex
  //          std::lock_guard<std::mutex> lock(device_mutex);
  //          // Check if our node is disconnected.
  //          found_node = false;
  //          device_connected = false;
  //          device.reset();

  //          std::cout << "Device with Node ID=0x" << std::hex <<
  //          current_node_id
  //                    << " is disconnected...." << std::endl;
  //          // exit(1); // TO DO the
  //          // This callback idealy should reinitiate the connection with
  //          the
  //          // device
  //        }
  //      });

  int channel1_speed_ref = 0;
  int channel2_speed_ref = 0;
  bool max = false;
  uint8_t i = 0;
  while (keepRunning) {
    if (device_connected[i]) {
      if (device[i]->get_node_id() == 4) {
        // Lock device mutex
        // std::lock_guard<std::mutex> lock(device_mutex);
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "Controlling device with node id=4" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        try {
          // Early exception handling
          //      try {
          //        //        DUMP_HEX(
          //        //            device->get_entry("qry_abspeed/channel_1",
          //        // kaco::ReadAccessMethod::pdo_request_and_wait));
          //        //        DUMP_HEX(
          //        //            device->get_entry("qry_abspeed/channel_2",
          //        // kaco::ReadAccessMethod::pdo_request_and_wait));
          //        //        DUMP_HEX(
          //        //            device->get_entry("qry_batamps/channel_1",
          //        // kaco::ReadAccessMethod::pdo_request_and_wait));
          //        //        DUMP_HEX(
          //        //            device->get_entry("qry_batamps/channel_2",
          //        // kaco::ReadAccessMethod::pdo_request_and_wait));
          //      } catch (kaco::canopen_error exception) {
          //        // No specific action is decided
          //      }

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
          channel2_speed_ref = channel1_speed_ref;
          device[i]->set_entry("cmd_cango/cmd_cango_1",
                               static_cast<int>(channel1_speed_ref),
                               kaco::WriteAccessMethod::pdo);
          std::cout << "Channel 1 speed command = " << std::dec
                    << channel1_speed_ref << std::endl;
          int16_t ch1_speed_feedback = device[i]->get_entry(
              "qry_abspeed/channel_1",
              kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Channel 1 speed feedback = " << std::dec
                    << (ch1_speed_feedback) << std::endl;
          device[i]->set_entry("cmd_cango/cmd_cango_2",
                               static_cast<int>(channel2_speed_ref),
                               kaco::WriteAccessMethod::pdo);
          std::cout << "Channel 2 speed command = " << std::dec
                    << channel1_speed_ref << std::endl;
          int16_t ch2_speed_feedback = device[i]->get_entry(
              "qry_abspeed/channel_2",
              kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Channel 2 speed feedback = " << std::dec
                    << ch2_speed_feedback << std::endl;
          uint16_t v_int = device[i]->get_entry(
              "qry_volts/v_int", kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Internal Voltage = " << std::dec
                    << static_cast<float>(static_cast<float>(v_int) /
                                          static_cast<float>(10))
                    << "V" << std::endl;
          uint16_t v_bat = device[i]->get_entry(
              "qry_volts/v_bat", kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Battery Voltage = " << std::dec
                    << static_cast<float>(static_cast<float>(v_bat) /
                                          static_cast<float>(10))
                    << "V" << std::endl;
          uint16_t v_5vout = device[i]->get_entry(
              "qry_volts/v_5vout",
              kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Internal 5V supply = " << std::dec
                    << static_cast<float>(static_cast<float>(v_5vout) /
                                          static_cast<float>(1000))
                    << "V" << std::endl;
          uint16_t digout = device[i]->get_entry(
              "qry_digout", kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Status of Digital Outs = " << std::hex << digout << ""
                    << std::endl;

          std::cout << "Step in main!" << std::endl;
        } catch (...) {
          std::cout << "Exception in main!" << std::endl;
        }
      } else if (device[i]->get_node_id() == 3) {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "Controlling device with node id=3" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        printDeviceInfo(device[i]);
      }
    }
    i++;
    if (i > 1) {
      i = 0;
    }
    // Sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
}
