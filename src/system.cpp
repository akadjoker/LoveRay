// system.cpp - love.system
#include "love.hpp"
#include "luax.hpp"

#include <cstdlib>
#include <string>
#include <thread>

namespace love
{
namespace
{

int l_getOS(lua_State *L)
{
#if defined(_WIN32)
    lua_pushstring(L, "Windows");
#elif defined(__APPLE__)
    lua_pushstring(L, "OS X");
#elif defined(__EMSCRIPTEN__)
    lua_pushstring(L, "Web");
#elif defined(__ANDROID__)
    lua_pushstring(L, "Android");
#else
    lua_pushstring(L, "Linux");
#endif
    return 1;
}

int l_getProcessorCount(lua_State *L)
{
    unsigned int count = std::thread::hardware_concurrency();
    lua_pushinteger(L, count == 0 ? 1 : count);
    return 1;
}

int l_getClipboardText(lua_State *L)
{
    if (!window::isOpen())
    {
        lua_pushstring(L, "");
        return 1;
    }
    const char *text = GetClipboardText();
    lua_pushstring(L, text != nullptr ? text : "");
    return 1;
}

int l_setClipboardText(lua_State *L)
{
    const char *text = luaL_checkstring(L, 1);
    if (window::isOpen())
    {
        SetClipboardText(text);
    }
    return 0;
}

int l_openURL(lua_State *L)
{
    const char *url = luaL_checkstring(L, 1);
    OpenURL(url);
    lua_pushboolean(L, 1);
    return 1;
}

int l_getPowerInfo(lua_State *L)
{
    lua_pushstring(L, "unknown");
    lua_pushnil(L);
    lua_pushnil(L);
    return 3;
}

int l_vibrate(lua_State *L)
{
    return 0;
}

int l_hasBackgroundMusic(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

int l_getPreferredLocales(lua_State *L)
{
    lua_newtable(L);
    std::string locale = "en_US";
    for (const char *var : {"LC_ALL", "LC_MESSAGES", "LANG"})
    {
        const char *value = std::getenv(var);
        if (value != nullptr && value[0] != '\0')
        {
            locale = value;
            break;
        }
    }
    size_t dot = locale.find('.');
    if (dot != std::string::npos)
    {
        locale.erase(dot);
    }
    if (locale == "C" || locale == "POSIX")
    {
        locale = "en_US";
    }
    lua_pushstring(L, locale.c_str());
    lua_rawseti(L, -2, 1);
    return 1;
}

const luaL_Reg FUNCS[] = {
    {"getOS", l_getOS},
    {"getProcessorCount", l_getProcessorCount},
    {"getClipboardText", l_getClipboardText},
    {"setClipboardText", l_setClipboardText},
    {"openURL", l_openURL},
    {"getPowerInfo", l_getPowerInfo},
    {"vibrate", l_vibrate},
    {"hasBackgroundMusic", l_hasBackgroundMusic},
    {"getPreferredLocales", l_getPreferredLocales},
    {nullptr, nullptr},
};

} // namespace

int open_system(lua_State *L)
{
    luaL_newlib(L, FUNCS);
    return 1;
}

} // namespace love
