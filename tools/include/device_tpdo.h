#pragma once
#include <iostream>
#include <vector>
#include <vector>
#include "core.h"
#include "device.h"
#include "master.h"

enum TPDO_NO {
  tpdo1,
  tpdo2,
  tpdo3,
  tpdo4

};

void write_entry(uint16_t index, std::vector<uint32_t> entries,
                 std::shared_ptr<kaco::Device> device);
void map_tpdo_in_device(TPDO_NO tpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type, uint16_t inhibit_time,
                        uint16_t event_timer,
                        std::shared_ptr<kaco::Device> device);
