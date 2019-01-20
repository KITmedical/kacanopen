#include "device_tpdo.h"
#include <cstdint>
void write_entry(uint16_t index, std::vector<uint32_t> entries,
                 std::shared_ptr<kaco::Device> device) {
  uint8_t offset = 0;
  for (uint8_t i = 0; i < entries.size(); i++) {
    offset++;
    device->set_entry(index, offset, static_cast<uint32_t>(entries.at(i)),
                      kaco::WriteAccessMethod::sdo);
  }
}
void map_tpdo_in_device(TPDO_NO tpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type, uint16_t inhibit_time,
                        uint16_t event_timer,
                        std::shared_ptr<kaco::Device> device) {
  uint32_t cob_id = 0;
  switch (tpdo_no) {
    case tpdo1:
      std::cout << "Mapping TPDO1 in the device" << std::endl;
      // disable tpdo
      cob_id = device->get_entry(0x1800, 01, kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1800, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A00, 0x00, static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A00, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A00, 0x00,
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1800, 0x02, transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1800, 0x03, inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // set event timer i.e transmit frequency
      device->set_entry(0x1800, 0x05, event_timer,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo1
      cob_id ^= static_cast<uint32_t>((-0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1800, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case tpdo2:
      std::cout << "Mapping TPDO2 in the device" << std::endl;
      cob_id = device->get_entry(0x1801, 01, kaco::ReadAccessMethod::sdo);
//      std::cout << "TPDO2 cob id found = " << std::hex <<cob_id << std::endl;;
//      if (cob_id!=0x40000280+node_id){
//        cob_id=0x40000280+node_id;
//      }
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      // disable tpdo2
      device->set_entry(0x1801, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A01, 0x00, static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A01, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A01, 0x00,
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                       kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1801, 0x02, transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1801, 0x03, inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // set event timer i.e transmit frequency
      device->set_entry(0x1801, 0x05, event_timer,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo2
      cob_id ^= static_cast<uint32_t>((-0 ^ cob_id) & (1UL << 31));

      device->set_entry(0x1801, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case tpdo3:
      std::cout << "Mapping TPDO3 in the device" << std::endl;
      // disable tpdo3
      cob_id = device->get_entry(0x1802, 01, kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1802, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A02, 0x00, static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A02, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A02, 0x00,
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1802, 0x02, transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1802, 0x03, inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // set event timer i.e transmit frequency
      device->set_entry(0x1802, 0x05, event_timer,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo3
      cob_id ^= static_cast<uint32_t>((-0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1802, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case tpdo4:
      std::cout << "Mapping TPDO4 in the device" << std::endl;
      // disable tpdo4
      cob_id = device->get_entry(0x1803, 01, kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1803, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A03, 0x00, static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A03, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A03, 0x00,
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1803, 0x02, transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1803, 0x03, inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // set event timer i.e transmit frequency
      device->set_entry(0x1803, 0x05, event_timer,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo4
      cob_id ^= static_cast<uint32_t>((-0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1803, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;

    default:
      std::cout << "Maximum 4 PDOs is supported" << std::endl;
      std::cout << "Invalid pdo_no" << std::endl;
  }
}
