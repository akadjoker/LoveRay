#include <string>
#include <ostream>
#include <iostream>
#include <cstring>

#define LOVE_IMPLEMENTATION
#include "Love.hpp"

// #define CHIP_PHYSICS_IMPLEMENTATION
// #include "Chipmunk.hpp"

#define PHYSICS_IMPLEMENTATION
#include "Physics.hpp"

int main(int argc, char **argv)
{

    lua_State *L = luaL_newstate();
    luaL_openlibs(L);
    CreateLove(L);

    lua_getglobal(L, "love");
    if (!lua_isnil(L, -1))
    {
        lua_newtable(L);
        int i;
        for (i = 0; i < argc; i++)
        {
            lua_pushstring(L, argv[i]);
            lua_rawseti(L, -2, i + 1);
        }
        lua_setfield(L, -2, "argv");
    }
    lua_pop(L, 1);

#include "nogame_lua.h"
#include "boot_lua.h"

    struct
    {
        const char *name, *data;
        int size;
    } items[] = 
    {
        {"nogame.lua", nogame_lua, sizeof(nogame_lua)},
        {"boot.lua", boot_lua, sizeof(boot_lua)},
        {NULL, NULL, 0}};
    int i;
    for (i = 0; items[i].name; i++)
    {
        int err = luaL_loadbuffer(L, items[i].data, items[i].size, items[i].name);
        if (err || lua_pcall(L, 0, 0, 0) != 0)
        {
            
            Log(LOG_ERROR, "Error executing %s : %s", items[i].name,lua_tostring(L, -1));
            
        }
    }

    // if (luaL_loadfile(L, "nogame.lua") || lua_pcall(L, 0, 0, 0))
    // {
    //     Log(LOG_ERROR, "Error executing nogame.lua : %s", lua_tostring(L, -1));
    //     lua_pop(L, 1);
    // }
    // if (luaL_loadfile(L, "boot.lua") || lua_pcall(L, 0, 0, 0))
    // {
    //     Log(LOG_ERROR, "Error executing boot.lua : %s", lua_tostring(L, -1));
    //     lua_pop(L, 1);
    // }

    CloseLove(L);
    return 0;
}