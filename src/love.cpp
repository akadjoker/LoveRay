// love.cpp - the `love` table, logging and the boot sequence.
#include "love.hpp"
#include "luax.hpp"

#include <box2d/b2_common.h>

#include <cstdarg>
#include <cstdio>
#include <ctime>

namespace love
{

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

void log(LogLevel level, const char *fmt, ...)
{
    const char *label = "info";
    const char *color = "\033[1;32m";
    FILE *stream = stdout;
    switch (level)
    {
    case LogLevel::Warning:
        label = "warning";
        color = "\033[1;35m";
        break;
    case LogLevel::Error:
        label = "error";
        color = "\033[1;31m";
        stream = stderr;
        break;
    default:
        break;
    }

    std::time_t now = std::time(nullptr);
    char stamp[16];
    std::strftime(stamp, sizeof(stamp), "%H:%M:%S", std::localtime(&now));

    std::fprintf(stream, "\033[0;36m[%s]\033[0m %s%s\033[0m: ", stamp, color, label);
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stream, fmt, args);
    va_end(args);
    std::fputc('\n', stream);
    std::fflush(stream);
}

// ---------------------------------------------------------------------------
// love.* top level functions
// ---------------------------------------------------------------------------

namespace
{

int l_getVersion(lua_State *L)
{
    lua_pushinteger(L, LOVE_API_VERSION_MAJOR);
    lua_pushinteger(L, LOVE_API_VERSION_MINOR);
    lua_pushinteger(L, 0);
    lua_pushstring(L, "LoveRay");
    return 4;
}

int l_isVersionCompatible(lua_State *L)
{
    int major = 0;
    int minor = 0;
    if (lua_type(L, 1) == LUA_TSTRING)
    {
        std::sscanf(lua_tostring(L, 1), "%d.%d", &major, &minor);
    }
    else
    {
        major = static_cast<int>(luaL_checkinteger(L, 1));
        minor = static_cast<int>(luaL_optinteger(L, 2, 0));
    }
    lua_pushboolean(L, major == LOVE_API_VERSION_MAJOR && minor <= LOVE_API_VERSION_MINOR);
    return 1;
}

int l_log(lua_State *L)
{
    static const char *const names[] = {"info", "warning", "error"};
    static const int values[] = {0, 1, 2};
    int level = 0;
    int msgIndex = 1;
    if (lua_gettop(L) >= 2)
    {
        level = luax::checkenum(L, 1, names, values, "log level");
        msgIndex = 2;
    }
    const char *msg = luaL_checkstring(L, msgIndex);
    log(static_cast<LogLevel>(level), "%s", msg);
    return 0;
}

// love._loadResource(name) -> chunk  (embedded Lua scripts)
int l_loadResource(lua_State *L)
{
    const char *name = luaL_checkstring(L, 1);
    unsigned int length = 0;
    const unsigned char *data = resource(name, &length);
    if (data == nullptr)
    {
        return luaL_error(L, "Unknown embedded resource '%s'", name);
    }
    std::string chunkname = "=[loveray \"";
    chunkname += name;
    chunkname += "\"]";
    if (luaL_loadbuffer(L, reinterpret_cast<const char *>(data), length, chunkname.c_str()) != LUA_OK)
    {
        return lua_error(L);
    }
    return 1;
}

int open_love(lua_State *L)
{
    static const luaL_Reg funcs[] = {
        {"getVersion", l_getVersion},
        {"isVersionCompatible", l_isVersionCompatible},
        {"log", l_log},
        {"_loadResource", l_loadResource},
        {nullptr, nullptr},
    };
    luaL_newlib(L, funcs);

    lua_pushstring(L, LOVE_API_VERSION_STRING);
    lua_setfield(L, -2, "_version");
    lua_pushinteger(L, LOVE_API_VERSION_MAJOR);
    lua_setfield(L, -2, "_version_major");
    lua_pushinteger(L, LOVE_API_VERSION_MINOR);
    lua_setfield(L, -2, "_version_minor");
    lua_pushinteger(L, 0);
    lua_setfield(L, -2, "_version_revision");
    lua_pushstring(L, "LoveRay");
    lua_setfield(L, -2, "_version_codename");

#if defined(_WIN32)
    lua_pushstring(L, "Windows");
#elif defined(__APPLE__)
    lua_pushstring(L, "OS X");
#elif defined(__EMSCRIPTEN__)
    lua_pushstring(L, "Web");
#else
    lua_pushstring(L, "Linux");
#endif
    lua_setfield(L, -2, "_os");

    // Runtime information specific to this implementation.
    lua_newtable(L);
    lua_pushstring(L, LOVERAY_VERSION);
    lua_setfield(L, -2, "version");
    lua_pushstring(L, RAYLIB_VERSION);
    lua_setfield(L, -2, "raylib");
    lua_pushstring(L, LUA_RELEASE);
    lua_setfield(L, -2, "lua");
    lua_pushfstring(L, "%d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);
    lua_setfield(L, -2, "box2d");
    lua_setfield(L, -2, "loveray");

    struct Module
    {
        const char *name;
        lua_CFunction open;
    };
    static const Module modules[] = {
        {"filesystem", open_filesystem},
        {"window", open_window},
        {"graphics", open_graphics},
        {"event", open_event},
        {"keyboard", open_keyboard},
        {"mouse", open_mouse},
        {"joystick", open_joystick},
        {"timer", open_timer},
        {"math", open_math},
        {"audio", open_audio},
        {"system", open_system},
        {"image", open_image},
        {"physics", open_physics},
    };
    for (const Module &module : modules)
    {
        module.open(L);
        lua_setfield(L, -2, module.name);
    }
    return 1;
}

} // namespace

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

BootResult boot(int argc, char **argv)
{
    BootResult result;

    lua_State *L = luaL_newstate();
    luaL_openlibs(L);

    luaL_requiref(L, "love", open_love, 1);
    lua_pop(L, 1);

    // Global `arg` table, following the Love2D layout: arg[0] is the
    // executable and arg[1..n] are the command line arguments.
    lua_newtable(L);
    for (int i = 0; i < argc; ++i)
    {
        lua_pushstring(L, argv[i]);
        lua_rawseti(L, -2, i);
    }
    lua_setglobal(L, "arg");

    unsigned int length = 0;
    const unsigned char *bootScript = resource("boot.lua", &length);

    lua_pushcfunction(L, luax::traceback);
    int status = luaL_loadbuffer(L, reinterpret_cast<const char *>(bootScript), length, "=[loveray \"boot.lua\"]");
    if (status == LUA_OK)
    {
        status = lua_pcall(L, 0, 1, -2);
    }

    if (status != LUA_OK)
    {
        log(LogLevel::Error, "%s", lua_tostring(L, -1));
        result.exitCode = 1;
    }
    else if (lua_type(L, -1) == LUA_TSTRING)
    {
        result.restart = std::strcmp(lua_tostring(L, -1), "restart") == 0;
    }
    else if (lua_isnumber(L, -1))
    {
        result.exitCode = static_cast<int>(lua_tointeger(L, -1));
    }
    lua_settop(L, 0);

    // Closing the state runs the finalizers of every GPU/audio object, so the
    // window (GL context) and the audio device must still be alive here.
    lua_close(L);
    graphics::shutdown();
    audio::shutdown();
    return result;
}

} // namespace love
