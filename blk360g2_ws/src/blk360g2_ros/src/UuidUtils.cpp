#include "UuidUtils.hpp"

#include <iostream>
#include <random>

namespace common
{
Blk360G2_UUID generateUuid()
{
    Blk360G2_UUID uuid{};
    std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> distribution(0, 255);

    for (uint8_t i = 0u; i < static_cast<uint8_t>(std::size(uuid.uuid)); i++)
    {
        uuid.uuid[i] = static_cast<uint8_t>(distribution(gen));
    }
    std::cout << "Generated uuid = " << Blk360G2_UUID_Serialize(uuid).uuid << std::endl;
    return uuid;
}

Blk360G2_StringUUID getStringUuid(const Blk360G2_UUID& uuid)
{
    Blk360G2_StringUUID result = Blk360G2_UUID_Serialize(uuid);
    return result;
}
}
