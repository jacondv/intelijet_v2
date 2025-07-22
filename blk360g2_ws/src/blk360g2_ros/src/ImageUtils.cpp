#include "ImageUtils.hpp"
#include "EXR.hxx"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string_view>

namespace common
{
constexpr int IMAGE_CHANNELS = 3;

const char* strExtension(const Blk360G2_ImageMetadata& metadata)
{
    switch (metadata.imageFormat)
    {
    case Blk360G2_ImageFormat_DNG:
        return ".dng";
    case Blk360G2_ImageFormat_DNL:
        return ".dnl";
    case Blk360G2_ImageFormat_JPG:
        return ".jpg";
    case Blk360G2_ImageFormat_JXR:
        return ".jxr";
    default:
        return "";
    }
}

const char* strImageType(const Blk360G2_ImageMetadata& metadata)
{
    switch (metadata.imageType)
    {
    case Blk360G2_ImageType_LDR:
        return "_LDR";
    case Blk360G2_ImageType_HDR:
        return "_HDR";
    default:
        return "";
    }
}

const char* strCameraPosition(const Blk360G2_ImageMetadata& metadata)
{
    switch (metadata.cameraPosition)
    {
    case Blk360G2_CameraPosition_RightBottom:
        return "_RB";
    case Blk360G2_CameraPosition_RightTop:
        return "_RT";
    case Blk360G2_CameraPosition_LeftBottom:
        return "_LB";
    case Blk360G2_CameraPosition_LeftTop:
        return "_LT";
    default:
        return "";
    }
}

const char* strImageDirection(const Blk360G2_ImageMetadata& metadata)
{
    switch (metadata.imageDirection)
    {
    case Blk360G2_ImageDirection_Front:
        return "_F";
    case Blk360G2_ImageDirection_Back:
        return "_B";
    default:
        return "";
    }
}

std::ostream& operator<<(std::ostream& os, const Blk360G2_Matrix4& m)
{
    for (unsigned row = 0; row < 4; ++row)
    {
        for (unsigned column = 0; column < 4; ++column)
        {
            os << m.elements[column + row * 4] << ',';
        }

        os << '\n';
    }
    return os;
}

void saveImageToFile(uint64_t dataSize, const char* dataPtr, const Blk360G2_ImageMetadata& metadata, unsigned image)
{
    std::stringstream filename;

    filename << "image_" << std::setw(4) << image << strImageType(metadata)
             << strImageDirection(metadata) << strCameraPosition(metadata)
             << strExtension(metadata);

    std::ofstream file(filename.str(), std::ios_base::out | std::ios_base::binary);
    file << std::string_view(dataPtr, dataSize);
}

void saveHdrBracketToFile(uint64_t dataSize, const char* dataPtr, const Blk360G2_ImageMetadata& metadata, unsigned image, unsigned bracket)
{
    std::stringstream filename;

    filename << "image_" << std::setw(4) << image << strImageType(metadata)
             << strImageDirection(metadata) << strCameraPosition(metadata)
             << "_bracket_" << std::setw(4) << bracket << strExtension(metadata);

    std::ofstream file(filename.str(), std::ios_base::out | std::ios_base::binary);
    file << std::string_view(dataPtr, dataSize);
}

template<class T>
void convertAndSave(uint64_t dataSize, const T* dataPtr, uint32_t width, uint32_t height, const std::string& filename)
{
    if (dataSize != width * height * IMAGE_CHANNELS * sizeof(T))
    {
        std::cerr << "Invalid image data size!" << std::endl;
        return;
    }

    const auto elements = dataSize / sizeof(T);

    std::vector<float> convertedData(width * height * IMAGE_CHANNELS);
    std::transform(dataPtr, dataPtr + elements, std::begin(convertedData),
        [](const T value) -> float {
            return static_cast<float>(value) / static_cast<float>(std::numeric_limits<T>::max());
        });

    if (utils::WriteEXR(width, height, IMAGE_CHANNELS, convertedData.data(), filename))
    {
        std::cout << "Image saved!\n";
    }
    else
    {
        std::cerr << "Failed to write file: " << filename << std::endl;
    }
}

void saveImageToFileCommon(uint64_t dataSize, const char* dataPtr, const Blk360G2_ImageMetadata& metadata, std::string filename)
{
    std::cout << "Saving image \"" << filename << "\"...\n";

    const auto imageWidth = metadata.imageWidth;
    const auto imageHeight = metadata.imageHeight;

    switch (metadata.imageFormat)
    {
    case Blk360G2_ImageFormat_RGB_FLOAT:
        if (dataSize != imageWidth * imageHeight * IMAGE_CHANNELS * sizeof(float))
        {
            std::cerr << "Invalid data size!" << std::endl;
            return;
        }

        if (utils::WriteEXR(imageWidth, imageHeight, IMAGE_CHANNELS, reinterpret_cast<const float*>(dataPtr), filename))
        {
            std::cout << "Image saved!\n";
        }
        else
        {
            std::cerr << "Failed to write file: " << filename << std::endl;
        }

        break;
    case Blk360G2_ImageFormat_RGB_UINT8:
        convertAndSave(dataSize, reinterpret_cast<const std::uint8_t*>(dataPtr), imageWidth, imageHeight, filename);
        break;
    case Blk360G2_ImageFormat_RGB_UINT16:
        convertAndSave(dataSize, reinterpret_cast<const std::uint16_t*>(dataPtr), imageWidth, imageHeight, filename);
        break;
    default:
        std::cerr << "Image format `" << metadata.imageFormat << "` is not handled" << std::endl;
    }
}

void saveImageToFile(uint64_t dataSize, const char* dataPtr, const Blk360G2_ImageMetadata& metadata)
{
    std::stringstream filenameBuilder;

    filenameBuilder << "image"
                    << strImageDirection(metadata)
                    << strCameraPosition(metadata)
                    << ".exr";

    auto filename = filenameBuilder.str();

    saveImageToFileCommon(dataSize, dataPtr, metadata, std::move(filename));
}

void saveImageToFile(uint64_t dataSize, const char* dataPtr, const Blk360G2_ImageMetadata& metadata, const Blk360G2_ImageMetadata& originalImageMetadata)
{
    std::stringstream filenameBuilder;

    filenameBuilder << "image"
                    << strImageDirection(metadata)
                    << strCameraPosition(metadata)
                    << strImageType(originalImageMetadata)
                    << ".exr";

    auto filename = filenameBuilder.str();

    saveImageToFileCommon(dataSize, dataPtr, metadata, std::move(filename));
}

void saveImageToFile(uint64_t dataSize, const char* dataPtr, const Blk360G2_ImageMetadata& metadata, const char* suffix)
{
    std::stringstream filenameBuilder;

    filenameBuilder << "image"
                    << strImageDirection(metadata)
                    << strCameraPosition(metadata)
                    << suffix
                    << ".exr";

    auto filename = filenameBuilder.str();

    saveImageToFileCommon(dataSize, dataPtr, metadata, std::move(filename));
}

void writeCalibration(const Blk360G2_ImageCalibration& calibration, const Blk360G2_ImageMetadata& metadata)
{
    std::stringstream filenameBuilder;

    filenameBuilder << "calibration"
                    << strImageDirection(metadata)
                    << strCameraPosition(metadata)
                    << ".txt";

    const auto filename = filenameBuilder.str();
    std::cout << "Saving calibration \"" << filename << "\"...\n";

    std::ofstream out(filename);
    if (!out.is_open())
    {
        std::cerr << "Failed to open file \"" << filename << "\" for writing!\n";
        return;
    }

    out << "Projection:\n"
        << calibration.projection
        << "\nTransformation:\n"
        << calibration.transformation;

    std::cout << "Calibration saved!\n";
}
}
