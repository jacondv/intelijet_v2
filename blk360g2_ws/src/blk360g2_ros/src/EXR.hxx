#pragma once

/*
 * https://github.com/aras-p/miniexr
 *
 * miniexr.cpp - v0.2 - public domain - 2013 Aras Pranckevicius / Unity Technologies
 * Minor code changes by MTIJ:
 * - replaced old style casts by new style casts;
 * - replaced unsigned by uint32_t;
 * - replaced unsigned char by uint8_t;
 * - fixed comparisons between signed and unsigned;
 * - replaced malloc by std::vector
 * - no longer return a pointer to allocated memory, but rather just write the file, and return a boolean
 * - added some error handling.
 * Minor code changes by ALEB:
 * - Write float instead of half_float
 * - Reformat with clang-format
 */

#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace utils
{
// Write floating point buffer as EXR file.
// Input:
//   - (width) x (height) image,
//   - channels == 4: 16 bytes per pixel (R,G,B,A order, 32 bit float per channel; alpha ignored), or
//   - channels == 3: 12 bytes per pixel (R,G,B   order, 32 bit float per channel).
// Returns true if all went fine, false otherwise.
inline bool WriteEXR(uint32_t width, uint32_t height, uint32_t channels, const float* rgba, const std::string& filename);
}

#define ARRAY_SIZE(x) sizeof(x) / sizeof(x[0])

namespace utils
{
bool WriteEXR(uint32_t width, uint32_t height, uint32_t channels, const float* rgba, const std::string& filename)
{
    if ((channels != 3 && channels != 4) || rgba == nullptr || filename.empty())
        return false;

    const uint32_t ww = width - 1;
    const uint32_t hh = height - 1;
    // clang-format off
    const uint8_t kHeader[] = {
        0x76, 0x2f, 0x31, 0x01, // magic
        2, 0, 0, 0, // version, scanline
        // channels
        'c','h','a','n','n','e','l','s',0,
        'c','h','l','i','s','t',0,
        55,0,0,0,
        'B',0, 2,0,0,0, 0, 0,0,0,1,0,0,0,1,0,0,0, // B, float
        'G',0, 2,0,0,0, 0, 0,0,0,1,0,0,0,1,0,0,0, // G, float
        'R',0, 2,0,0,0, 0, 0,0,0,1,0,0,0,1,0,0,0, // R, float
        0,
        // compression
        'c','o','m','p','r','e','s','s','i','o','n',0,
        'c','o','m','p','r','e','s','s','i','o','n',0,
        1,0,0,0,
        0, // no compression
        // dataWindow
        'd','a','t','a','W','i','n','d','o','w',0,
        'b','o','x','2','i',0,
        16,0,0,0,
        0,0,0,0,0,0,0,0,
        static_cast<uint8_t>(ww & 0xFF), static_cast<uint8_t>((ww >> 8) & 0xFF), static_cast<uint8_t>((ww >> 16) & 0xFF), static_cast<uint8_t>((ww >> 24) & 0xFF),
        static_cast<uint8_t>(hh & 0xFF), static_cast<uint8_t>((hh >> 8) & 0xFF), static_cast<uint8_t>((hh >> 16) & 0xFF), static_cast<uint8_t>((hh >> 24) & 0xFF),
        // displayWindow
        'd','i','s','p','l','a','y','W','i','n','d','o','w',0,
        'b','o','x','2','i',0,
        16,0,0,0,
        0,0,0,0,0,0,0,0,
        static_cast<uint8_t>(ww & 0xFF), static_cast<uint8_t>((ww >> 8) & 0xFF), static_cast<uint8_t>((ww >> 16) & 0xFF), static_cast<uint8_t>((ww >> 24) & 0xFF),
        static_cast<uint8_t>(hh & 0xFF), static_cast<uint8_t>((hh >> 8) & 0xFF), static_cast<uint8_t>((hh >> 16) & 0xFF), static_cast<uint8_t>((hh >> 24) & 0xFF),
        // lineOrder
        'l','i','n','e','O','r','d','e','r',0,
        'l','i','n','e','O','r','d','e','r',0,
        1,0,0,0,
        0, // increasing Y
        // pixelAspectRatio
        'p','i','x','e','l','A','s','p','e','c','t','R','a','t','i','o',0,
        'f','l','o','a','t',0,
        4,0,0,0,
        0,0,0x80,0x3f, // 1.0f
        // screenWindowCenter
        's','c','r','e','e','n','W','i','n','d','o','w','C','e','n','t','e','r',0,
        'v','2','f',0,
        8,0,0,0,
        0,0,0,0, 0,0,0,0,
        // screenWindowWidth
        's','c','r','e','e','n','W','i','n','d','o','w','W','i','d','t','h',0,
        'f','l','o','a','t',0,
        4,0,0,0,
        0,0,0x80,0x3f, // 1.0f
        // end of header
        0,
    };
    // clang-format on
    constexpr int kHeaderSize = ARRAY_SIZE(kHeader);

    const int kScanlineTableSize = 8 * height;
    const uint32_t pixelRowSize = width * 3 * sizeof(float);
    const uint32_t fullRowSize = pixelRowSize + 8;

    const uint32_t bufSize = kHeaderSize + kScanlineTableSize + height * fullRowSize;
    std::vector<uint8_t> vector;
    vector.reserve(bufSize);
    uint8_t* const buf = vector.data();

    // copy in header
    std::memcpy(buf, kHeader, kHeaderSize);

    // line offset table
    uint32_t ofs = kHeaderSize + kScanlineTableSize;
    uint8_t* ptr = buf + kHeaderSize;
    for (uint32_t y = 0; y < height; ++y)
    {
        *ptr++ = ofs & 0xFF;
        *ptr++ = (ofs >> 8) & 0xFF;
        *ptr++ = (ofs >> 16) & 0xFF;
        *ptr++ = (ofs >> 24) & 0xFF;
        *ptr++ = 0;
        *ptr++ = 0;
        *ptr++ = 0;
        *ptr++ = 0;
        ofs += fullRowSize;
    }

    // scanline data
    const uint8_t* src = reinterpret_cast<const uint8_t*>(rgba);
    const int stride = channels * sizeof(float);
    for (uint32_t y = 0; y < height; ++y)
    {
        // coordinate
        *ptr++ = y & 0xFF;
        *ptr++ = (y >> 8) & 0xFF;
        *ptr++ = (y >> 16) & 0xFF;
        *ptr++ = (y >> 24) & 0xFF;

        // data size
        *ptr++ = pixelRowSize & 0xFF;
        *ptr++ = (pixelRowSize >> 8) & 0xFF;
        *ptr++ = (pixelRowSize >> 16) & 0xFF;
        *ptr++ = (pixelRowSize >> 24) & 0xFF;

        // B, G, R
        const uint8_t* chsrc;
        chsrc = src + 2 * sizeof(float);
        for (uint32_t x = 0; x < width; ++x)
        {
            *ptr++ = chsrc[0];
            *ptr++ = chsrc[1];
            *ptr++ = chsrc[2];
            *ptr++ = chsrc[3];
            chsrc += stride;
        }
        chsrc = src + 1 * sizeof(float);
        for (uint32_t x = 0; x < width; ++x)
        {
            *ptr++ = chsrc[0];
            *ptr++ = chsrc[1];
            *ptr++ = chsrc[2];
            *ptr++ = chsrc[3];
            chsrc += stride;
        }
        chsrc = src + 0 * sizeof(float);
        for (uint32_t x = 0; x < width; ++x)
        {
            *ptr++ = chsrc[0];
            *ptr++ = chsrc[1];
            *ptr++ = chsrc[2];
            *ptr++ = chsrc[3];
            chsrc += stride;
        }

        src += width * stride;
    }

    if (ptr - buf != bufSize)
    {
        std::cerr << "Not enough space when dumping .exr image." << std::endl;
        return false;
    }

    std::ofstream ostrm(filename, std::ios::binary);
    ostrm.write(reinterpret_cast<const char*>(buf), bufSize);

    return ostrm.good();
}
}
