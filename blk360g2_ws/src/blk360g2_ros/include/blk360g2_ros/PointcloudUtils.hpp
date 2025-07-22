#pragma once

#include <fstream>
#include <vector>
#include <cstdint>
#include <cstring>  // memcpy
#include <cmath>
#include <limits>
namespace common
{
void writeChunk(std::ofstream&, const char*, std::uint64_t);
std::vector<uint8_t> convertPolarBufferToPointCloud2Data(const std::vector<uint8_t>& full_data);

}
