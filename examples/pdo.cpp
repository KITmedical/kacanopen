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
#include <thread>
#include <chrono>
#include <cstdint>
#include "master.h"
#include "logger.h"
int main() {
    // Set the name of your CAN bus. "slcan0" is a common bus name
    // for the first SocketCAN device on a Linux system.
    const std::string busname = "slcan0";
    // Set the baudrate of your CAN bus. Most drivers support the values
    // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
    const std::string baudrate = "500K";
    PRINT("This example runs a counter completely without SDO transfers.");
    PRINT("There must be a CiA 401 device which is configured to send 'Read input 8-bit/Digital Inputs 1-8'");
    PRINT("and 'Read input 8-bit/Digital Inputs 9-16' via TPDO1 and to receive 'Write output 8-bit/Digital Outputs 1-8' via RPDO1.");
    kaco::Master master;
    if (!master.start(busname, baudrate)) {
        PRINT("Starting Master failed.");
        return EXIT_FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (master.num_devices()<1) {
        ERROR("No devices found.");
        return EXIT_FAILURE;
    }
    size_t index;
    bool found = false;
    for (size_t i=0; i<master.num_devices(); ++i) {
        kaco::Device& device = master.get_device(i);
        device.start();
        if (device.get_device_profile_number()==401) {
            index = i;
            found = true;
            PRINT("Found CiA 401 device with node ID "<<device.get_node_id());
        }
    }
    if (!found) {
        ERROR("This example is intended for use with a CiA 401 device but I can't find one.");
        return EXIT_FAILURE;
    }
    kaco::Device& device = master.get_device(index);
    const auto node_id = device.get_node_id();
    // device.start(); // already started
    device.load_dictionary_from_library();
    DUMP(device.get_entry("Manufacturer device name"));
    // TODO: first configure PDO on device side?
    device.add_receive_pdo_mapping(0x180+node_id, "Read input 8-bit/Digital Inputs 1-8", 0); // offset 0,
    device.add_receive_pdo_mapping(0x180+node_id, "Read input 8-bit/Digital Inputs 9-16", 1); // offset 1
    
    // transmit PDO on change
    device.add_transmit_pdo_mapping(0x200+node_id, {{"Write output 8-bit/Digital Outputs 1-8", 0}}); // offset 0
    // transmit PDO every 500ms
    //device.add_transmit_pdo_mapping(0x20A, {{"write_output", 0, 0, 0}}, kaco::TransmissionType::PERIODIC, std::chrono::milliseconds(500));
    for (uint8_t i=0; i<10; ++i) {
        PRINT("Set output to 0x"<<std::hex<<i<<" (via cache!) and wait 1 second");
        device.set_entry("Write output 8-bit/Digital Outputs 1-8", i, kaco::WriteAccessMethod::cache);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        DUMP_HEX(device.get_entry("Write output 8-bit/Digital Outputs 1-8",kaco::ReadAccessMethod::cache));
        DUMP_HEX(device.get_entry("Read input 8-bit/Digital Inputs 1-8",kaco::ReadAccessMethod::cache));
        DUMP_HEX(device.get_entry("Read input 8-bit/Digital Inputs 9-16",kaco::ReadAccessMethod::cache));
    }
}
