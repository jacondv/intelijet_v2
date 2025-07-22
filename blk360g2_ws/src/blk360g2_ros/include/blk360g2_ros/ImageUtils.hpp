#pragma once

#include "BLK360G2.h"

namespace common
{
void saveImageToFile(uint64_t, const char*, const Blk360G2_ImageMetadata&);
void saveImageToFile(uint64_t, const char*, const Blk360G2_ImageMetadata&, const Blk360G2_ImageMetadata&);
void saveImageToFile(uint64_t, const char*, const Blk360G2_ImageMetadata&, unsigned);
void saveImageToFile(uint64_t, const char*, const Blk360G2_ImageMetadata&, const char* suffix);
void saveHdrBracketToFile(uint64_t, const char*, const Blk360G2_ImageMetadata&, unsigned, unsigned);
void writeCalibration(const Blk360G2_ImageCalibration&, const Blk360G2_ImageMetadata&);
}
