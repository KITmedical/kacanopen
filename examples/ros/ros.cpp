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
#include "bridge.h"
#include "logger.h"
#include "entry_publisher.h"
#include "entry_subscriber.h"
 
#include <thread>
#include <chrono>
#include <memory>
int main(int argc, char* argv[]) {
    // Set the name of your CAN bus. "slcan0" is a common bus name
    // for the first SocketCAN device on a Linux system.
    const std::string busname = "slcan0";
    // Set the baudrate of your CAN bus. Most drivers support the values
    // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
    const std::string baudrate = "500K";
    kaco::Master master;
    if (!master.start(busname, baudrate)) {
        ERROR("Starting master failed.");
        return EXIT_FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (master.num_devices()<1) {
        ERROR("No devices found.");
        return EXIT_FAILURE;
    }
    // should be a 401 device
    kaco::Device& device = master.get_device(0);
    device.start();
    device.load_dictionary_from_library();
    uint16_t profile = device.get_device_profile_number();
    if (profile != 401) {
        ERROR("This example is intended for use with a CiA 401 device. You plugged a device with profile number "<<std::dec<<profile);
        return EXIT_FAILURE;
    }
    device.print_dictionary();
    DUMP(device.get_entry("Manufacturer device name"));
    // map PDOs (optional)
    device.add_receive_pdo_mapping(0x188, "Read input 8-bit/Digital Inputs 1-8", 0); // offset 0
    device.add_receive_pdo_mapping(0x188, "Read input 8-bit/Digital Inputs 9-16", 1); // offset 1
    // set some output (optional)
    device.set_entry("Write output 8-bit/Digital Outputs 1-8", (uint8_t) 0xFF);
    ros::init(argc, argv, "canopen_bridge");
    // Create bridge
    kaco::Bridge bridge;
    // create a publisher for reading second 8-bit input and add it to the bridge
    // communication via POD
    // publishing rate = 10 Hz
    auto iopub = std::make_shared<kaco::EntryPublisher>(device, "Read input 8-bit/Digital Inputs 9-16");
    bridge.add_publisher(iopub,1);
    // create a subscriber for editing IO output and add it to the bridge
    // communication via SOD
    auto iosub = std::make_shared<kaco::EntrySubscriber>(device, "Write output 8-bit/Digital Outputs 1-8");
    bridge.add_subscriber(iosub);
    // run ROS loop
    bridge.run();
    master.stop();
}