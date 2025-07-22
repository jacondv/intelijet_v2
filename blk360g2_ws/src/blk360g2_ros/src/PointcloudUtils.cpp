// #include "PointcloudUtils.hpp"
#include <blk360g2_ros/PointcloudUtils.hpp>
#include <vector>
#include <cstdint>
#include <cstring>  // memcpy
#include <cmath>
#include <limits>

namespace common
{
struct CartesianPoint
{
    float x, y, z, intensity;
};

struct PolarPoint
{
    float hAngle, vAngle, distance, intensity;

    CartesianPoint toCartesian() const
    {
        CartesianPoint cartesian;

        cartesian.x = -distance * std::sin(vAngle) * std::sin(hAngle);
        cartesian.y = -distance * std::cos(vAngle);
        cartesian.z = -distance * std::sin(vAngle) * std::cos(hAngle);
        cartesian.intensity = intensity;

        return cartesian;
    }

    bool isInvalid() const
    {
        return std::abs(hAngle) <= std::numeric_limits<float>::epsilon()
               && std::abs(vAngle) <= std::numeric_limits<float>::epsilon()
               && std::abs(distance) <= std::numeric_limits<float>::epsilon()
               && std::abs(intensity) <= std::numeric_limits<float>::epsilon();
    }
};

// ⭐ HÀM CHUYỂN ĐỔI NHANH GỌN
std::vector<uint8_t> convertPolarBufferToPointCloud2Data(const std::vector<uint8_t>& full_data)
{
    const size_t num_points = full_data.size() / sizeof(PolarPoint);
    const PolarPoint* polar_ptr = reinterpret_cast<const PolarPoint*>(full_data.data());

    std::vector<uint8_t> cloud_data;
    cloud_data.reserve(num_points * 16); // mỗi điểm: 4 * float = 16 byte

    for (size_t i = 0; i < num_points; ++i)
    {
        const PolarPoint& polar = polar_ptr[i];
        if (polar.isInvalid()) continue;

        CartesianPoint cart = polar.toCartesian();

        uint8_t buffer[16];
        std::memcpy(buffer + 0,  &cart.x,         4);
        std::memcpy(buffer + 4,  &cart.y,         4);
        std::memcpy(buffer + 8,  &cart.z,         4);
        std::memcpy(buffer + 12, &cart.intensity, 4);

        cloud_data.insert(cloud_data.end(), buffer, buffer + 16);
    }

    return cloud_data;
}


void writeChunk(std::ofstream& outFile, const char* chunkData, std::uint64_t chunkSize)
{
    const std::size_t totalElements = chunkSize / sizeof(PolarPoint);

    const auto* dataStart = reinterpret_cast<const PolarPoint*>(chunkData);
    const auto* dataEnd = dataStart + totalElements;

    for (const auto* polarPoint = dataStart; polarPoint != dataEnd; polarPoint++)
    {
        if (polarPoint->isInvalid())
        {
            continue; // omit this point
        }

        const auto cartesianPoint = polarPoint->toCartesian();
        outFile << cartesianPoint.x << ';' << cartesianPoint.y << ';' << cartesianPoint.z << ';' << cartesianPoint.intensity << '\n';
    }
}
}
