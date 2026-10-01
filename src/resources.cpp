// resources.cpp - access to the files embedded at build time.
#include "love.hpp"

#include "resources.h"

#include <cstring>

namespace love
{

const unsigned char *resource(const char *name, unsigned int *length)
{
    struct Entry
    {
        const char *name;
        const unsigned char *data;
        unsigned int length;
    };

    static const Entry entries[] = {
        {"boot.lua", boot_lua, boot_lua_len},
        {"nogame.lua", nogame_lua, nogame_lua_len},
        {"DejaVuSans.ttf", DejaVuSans_ttf, DejaVuSans_ttf_len},
    };

    for (const Entry &entry : entries)
    {
        if (std::strcmp(entry.name, name) == 0)
        {
            if (length != nullptr)
            {
                *length = entry.length;
            }
            return entry.data;
        }
    }

    if (length != nullptr)
    {
        *length = 0;
    }
    return nullptr;
}

} // namespace love
