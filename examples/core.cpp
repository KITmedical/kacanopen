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
 
#include <chrono>
#include <vector>
#include <iostream>
#include "core.h"
int main() {
    // ----------- //
    // Preferences //
    // ----------- //
    // The node ID of the slave we want to communicate with.
    const uint8_t node_id = 2;
    // Set the name of your CAN bus. "slcan0" is a common bus name
    // for the first SocketCAN device on a Linux system.
    const std::string busname = "slcan0";
    // Set the baudrate of your CAN bus. Most drivers support the values
    // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
    const std::string baudrate = "500K";
    // Set the object dictionary index to write to (download).
    // Here: CiA-401 (I/O device) digital output.
    const uint16_t index = 0x6200;
    // Alternative: CiA-402 (motor) control word:
    //const uint16_t index = 0x6040;
    // Set the object dictionary sub-index to write to (download).
    // Here: CiA-401 (I/O device) digital output - second byte.
    const uint8_t subindex = 0x01;
    // Alternative: CiA-402 (motor) control word:
    //const uint8_t subindex = 0x00;
    // Set the data to write (download).
    const std::vector<uint8_t> data { 0x7F };
    // Alternative: CiA-402 (motor) control word has two bytes. Command: shutdown (little-endian!)
    //const std::vector<uint8_t> data { 0x06, 0x00 };
    // -------------- //
    // Initialization //
    // -------------- //
    std::cout << "This is an example which shows the usage of the Core library." << std::endl;
    // Create core.
    kaco::Core core;
    // This will be set to true by the callback below.
    bool found_node = false;
    std::cout << "Registering a callback which is called when a device is detected via NMT..." << std::endl;
    core.nmt.register_device_alive_callback([&] (const uint8_t new_node_id) {
        std::cout << "Device says it's alive! ID = " << (unsigned) new_node_id << std::endl;
        // Check if this is the node we are looking for.
        if (new_node_id == node_id) {
            found_node = true;
        }
    });
    std::cout << "Starting Core (connect to the driver and start the receiver thread)..." << std::endl;
    if (!core.start(busname, baudrate)) {
        std::cout << "Starting core failed." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Asking all devices to reset. You don't need to do that, but it makes"
        << " sure all slaves are in a reproducible state." << std::endl;
    core.nmt.reset_all_nodes();
    // As an alternative you can request the slaves to announce
    // themselves:
    // core.nmt.discover_nodes();
    std::cout << "Giving the devices one second time to respond..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // Check if the device has been detected (see the callback above).
    if (!found_node) {
        std::cout << "Node with ID " << (unsigned) node_id << " has not been found."<< std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Asking the device to start up..." << std::endl;
    core.nmt.send_nmt_message(node_id,kaco::NMT::Command::start_node);
    std::cout << "Giving the devices one second time to boot up..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // ------------ //
    // Device usage //
    // ------------ //
    std::cout << "Writing to a dictionary entry (CANopen speech: \"download\")..." << std::endl;
    core.sdo.download(node_id, index, subindex, data.size(), data);
    std::cout << "Reading the device type (\"upload\" 0x1000)... Little-endian!" << std::endl;
    std::vector<uint8_t> device_type = core.sdo.upload(node_id,0x1000,0x0);
    for (uint8_t device_type_byte : device_type) {
        std::cout << "  byte 0x" << std::hex << (unsigned) device_type_byte << std::endl;
    }
    std::cout << "Reading the device name (\"upload\" 0x1008 - usually using segmented transfer)..." << std::endl;
    std::vector<uint8_t> device_name = core.sdo.upload(node_id,0x1008,0x0);
    std::string result(reinterpret_cast<char const*>(device_name.data()), device_name.size());
    std::cout << "  " << result << std::endl;
    std::cout << "Finished." << std::endl;
    return EXIT_SUCCESS;
}
