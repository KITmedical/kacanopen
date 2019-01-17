﻿/*
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

#include <ros/package.h>
#include <signal.h>
#include <boost/filesystem.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include "kacanopen/core/canopen_error.h"
#include "kacanopen/core/core.h"
#include "kacanopen/core/logger.h"
#include "kacanopen/master/device.h"
#include "kacanopen/master/master.h"
#include "kacanopen/tools/device_rpdo.h"
#include "kacanopen/tools/device_tpdo.h"
static volatile int keepRunning = 1;

void intHandler(int dummy) {
  (void)dummy;
  keepRunning = 0;
}

bool printDeviceInfo(std::shared_ptr<kaco::Device> device) {
  try {
    std::string display_device_type;
    auto device_type =
        device->get_entry(0x1000, 0x0, kaco::ReadAccessMethod::sdo);
    if (131474 == static_cast<uint32_t>(device_type))
      display_device_type = "DS402";  // 131474 is the type number for DS-402;
    auto device_name =
        device->get_entry(0x1008, 0x0, kaco::ReadAccessMethod::sdo);
    auto vendor_id =
        device->get_entry(0x1018, 0x01, kaco::ReadAccessMethod::sdo);
    auto product_id =
        device->get_entry(0x1018, 0x02, kaco::ReadAccessMethod::sdo);
    auto revision =
        device->get_entry(0x1018, 0x03, kaco::ReadAccessMethod::sdo);

    auto serial_no =
        device->get_entry(0x1018, 0x04, kaco::ReadAccessMethod::sdo);

    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "* Device Name found as '" << device_name << "'" << std::endl;
    std::cout << "* Device Type found as CiA-" << display_device_type << "'"
              << std::endl;
    std::cout << "* Vendor ID=" << vendor_id << std::endl;
    std::cout << "* Product ID=" << product_id << std::endl;
    std::cout << "* Serial Number=" << serial_no << std::endl;
    std::cout << "* Revision Number=" << revision << std::endl;
    //    std::cout << "* Hardware version=" << hardware_version << std::endl;
    //    std::cout << "* Firmware version=" << firmware_version << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    return true;
  } catch (...) {
    std::cout << "Exception occured while acquiring device information.!"
              << std::endl;
    return false;
  }
}

void initializeDevice(std::shared_ptr<kaco::Device> device,
                      uint16_t heartbeat_interval, uint8_t node_id) {
  // set the our desired heartbeat_interval time
  device->set_entry(0x1017, 0x0, heartbeat_interval,
                    kaco::WriteAccessMethod::sdo);

  // Mater side Periodic Tranmit pdo1 value initialization
  device->set_entry("target_velocity", 0x0, kaco::WriteAccessMethod::pdo);
  // Mater side Periodic Tranmit pdo2 value initialization
  //  device->set_entry("cmd_cango/cmd_cango_2", static_cast<int>(0x0),
  //                    kaco::WriteAccessMethod::pdo);
  // Master side tpdo1 mapping
  device->add_transmit_pdo_mapping(0x200 + node_id, {{"target_velocity", 0}},
                                   kaco::TransmissionType::PERIODIC,
                                   std::chrono::milliseconds(250));
  /***************** TPDO MAPPING in DEVICE *****************/
  std::vector<uint32_t> tpdo1_entries_to_be_mapped = {0x60410010, 0x606C0020};
  map_tpdo_in_device(tpdo1, tpdo1_entries_to_be_mapped, 255, device);

  /***************** RPDO MAPPING in DEVICE *****************/
  std::vector<uint32_t> rpdo1_entries_to_be_mapped = {0x60FF0020};
  map_rpdo_in_device(rpdo1, rpdo1_entries_to_be_mapped, 255, device);
}

int main() {
  // Signal handleing
  signal(SIGINT, intHandler);

  // ----------- //
  // Preferences //
  // ----------- //

  // A Roboteq motor driver with firmware version v2.0beta07032018 was used to
  // test this program.//

  // The node ID of the slave we want to communicate with.
  const uint8_t node_id = 1;

  // Set the name of your CAN bus. "slcan0" is a common bus name
  // for the first SocketCAN device on a Linux system.
  const std::string busname = "slcan0";

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
  kaco::Core core;
  volatile bool found_node = false;
  volatile bool device_connected = false;
  std::mutex device_mutex;

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

  std::cout << "Registering a callback which is called when a device is "
               "detected via NMT..."
            << std::endl;
  // make sure that the node is reset and goes back to NMT preoperational
  core.nmt.send_nmt_message(node_id, kaco::NMT::Command::reset_node);
  core.nmt.register_device_alive_callback([&](const uint8_t new_node_id) {
    // Check if this is the node we are looking for.
    if (new_node_id == node_id) {
      // lock
      if (!found_node) {
        found_node = true;
        // Lock device mutex
        std::lock_guard<std::mutex> lock(device_mutex);
        try {
          // Initialize the device

          device.reset(new kaco::Device(core, node_id));

          std::string path = ros::package::getPath("kacanopen");
          boost::filesystem::path full_path =
              path + "/resources/eds_library/MaxonMotor/maxon_motor_EPOS4.eds";
          device->load_dictionary_from_eds(full_path.string());
          std::cout << "Printing Device Object Dictionary" << std::endl;
          device->print_dictionary();
          core.nmt.send_nmt_message(node_id,
                                    kaco::NMT::Command::enter_preoperational);
          initializeDevice(device, heartbeat_interval, node_id);
          device->start();
          printDeviceInfo(device);
          uint16_t control_word_value = {0x0006};
          device->set_entry(0x6040, 0x00, control_word_value,
                            kaco::WriteAccessMethod::sdo);
          int8_t set_mode_of_operation = 3;
          device->set_entry(0x6060, 0x00, set_mode_of_operation,
                            kaco::WriteAccessMethod::sdo);
          device_connected = true;

        } catch (...) {
          std::cout << "Exception in device alive!" << std::endl;
          found_node = false;
          device_connected = false;
        }
      }
    }
  });
  core.nmt.register_device_dead_callback([&](const uint8_t new_node_id) {
    if ((device_connected && found_node) && (new_node_id == node_id)) {
      // Lock device mutex
      std::lock_guard<std::mutex> lock(device_mutex);
      // Check if our node is disconnected.
      found_node = false;
      device_connected = false;
      device.reset();
      std::cout << "Device with Node ID=0x" << std::hex << node_id
                << " is disconnected...." << std::endl;
    }
  });

  while (keepRunning) {
    if (device_connected) {
      // Lock device mutex
      std::lock_guard<std::mutex> lock(device_mutex);

      try {
        std::cout << "main loop running!" << std::endl;

        device->set_entry("target_velocity", static_cast<int>(2000),
                          kaco::WriteAccessMethod::pdo);
        uint16_t control_word_value = {0x000F};
        device->set_entry(0x6040, 0x00, control_word_value,
                          kaco::WriteAccessMethod::sdo);
      } catch (...) {
        std::cout << "Exception in main!" << std::endl;
      }
    }

    // Sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
}