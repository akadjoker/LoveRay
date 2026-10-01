// timer.cpp - love.timer
#include "love.hpp"
#include "luax.hpp"

#include <chrono>
#include <thread>

namespace love
{
namespace timer
{

namespace
{

using Clock = std::chrono::steady_clock;

const Clock::time_point g_start = Clock::now();
double g_previous = 0.0;
double g_delta = 0.0;

// Rolling one-second window used for getFPS / getAverageDelta.
double g_windowStart = 0.0;
int g_windowFrames = 0;
int g_fps = 0;
double g_averageDelta = 0.0;

double now()
{
    return std::chrono::duration<double>(Clock::now() - g_start).count();
}

} // namespace

double step()
{
    double current = now();
    if (g_previous == 0.0)
    {
        g_previous = current;
    }
    g_delta = current - g_previous;
    g_previous = current;

    ++g_windowFrames;
    double elapsed = current - g_windowStart;
    if (elapsed >= 1.0)
    {
        g_fps = static_cast<int>(g_windowFrames / elapsed + 0.5);
        g_averageDelta = elapsed / g_windowFrames;
        g_windowFrames = 0;
        g_windowStart = current;
    }
    return g_delta;
}

double getDelta()
{
    return g_delta;
}

double getAverageDelta()
{
    return g_averageDelta;
}

int getFPS()
{
    return g_fps;
}

namespace
{

int l_getTime(lua_State *L)
{
    lua_pushnumber(L, now());
    return 1;
}

int l_getDelta(lua_State *L)
{
    lua_pushnumber(L, g_delta);
    return 1;
}

int l_getAverageDelta(lua_State *L)
{
    lua_pushnumber(L, g_averageDelta);
    return 1;
}

int l_getFPS(lua_State *L)
{
    lua_pushinteger(L, g_fps);
    return 1;
}

int l_step(lua_State *L)
{
    lua_pushnumber(L, step());
    return 1;
}

int l_sleep(lua_State *L)
{
    double seconds = luaL_checknumber(L, 1);
    if (seconds > 0.0)
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    }
    return 0;
}

const luaL_Reg FUNCS[] = {
    {"getTime", l_getTime},
    {"getDelta", l_getDelta},
    {"getAverageDelta", l_getAverageDelta},
    {"getFPS", l_getFPS},
    {"step", l_step},
    {"sleep", l_sleep},
    {nullptr, nullptr},
};

} // namespace
} // namespace timer

int open_timer(lua_State *L)
{
    luaL_newlib(L, timer::FUNCS);
    return 1;
}

} // namespace love
