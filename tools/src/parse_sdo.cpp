#include "parse_sdo.h"
#include <cstdint>

int32_t parse_sdo_read(std::vector<uint8_t> sdo_to_read) {
  unsigned char sdo_read_byte[] = {0x00, 0x00, 0x00, 0x00};
  for (unsigned int i = 0; i < sdo_to_read.size(); i++) {
    sdo_read_byte[i] = sdo_to_read.at(i);
  }
  int16_t sdo_read_w0 = (sdo_read_byte[1] << 8) + sdo_read_byte[0];
  int16_t sdo_read_w1 = (sdo_read_byte[3] << 8) + sdo_read_byte[2];
  int32_t sdo_read_dw = (sdo_read_w1 << 16) + sdo_read_w0;
  if (2 < sdo_to_read.size()) {
    return sdo_read_dw;
  } else {
    short int ret = sdo_read_dw;
    return ret;
  }
}
