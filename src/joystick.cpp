// joystick.cpp - love.joystick (raylib gamepads)
#include "love.hpp"
#include "luax.hpp"

#include <cstring>

namespace love
{
namespace joystick
{

namespace
{

const char *JOYSTICK_TYPE = "Joystick";
const int MAX_PADS = 4;

struct Joystick
{
    int index;
};

// Stable addresses used as identity keys for the object registry.
Joystick g_pads[MAX_PADS] = {{0}, {1}, {2}, {3}};

struct ButtonName
{
    int button;
    const char *name;
};

const ButtonName BUTTONS[] = {
    {GAMEPAD_BUTTON_RIGHT_FACE_DOWN, "a"},
    {GAMEPAD_BUTTON_RIGHT_FACE_RIGHT, "b"},
    {GAMEPAD_BUTTON_RIGHT_FACE_LEFT, "x"},
    {GAMEPAD_BUTTON_RIGHT_FACE_UP, "y"},
    {GAMEPAD_BUTTON_MIDDLE_LEFT, "back"},
    {GAMEPAD_BUTTON_MIDDLE, "guide"},
    {GAMEPAD_BUTTON_MIDDLE_RIGHT, "start"},
    {GAMEPAD_BUTTON_LEFT_THUMB, "leftstick"},
    {GAMEPAD_BUTTON_RIGHT_THUMB, "rightstick"},
    {GAMEPAD_BUTTON_LEFT_TRIGGER_1, "leftshoulder"},
    {GAMEPAD_BUTTON_RIGHT_TRIGGER_1, "rightshoulder"},
    {GAMEPAD_BUTTON_LEFT_FACE_UP, "dpup"},
    {GAMEPAD_BUTTON_LEFT_FACE_DOWN, "dpdown"},
    {GAMEPAD_BUTTON_LEFT_FACE_LEFT, "dpleft"},
    {GAMEPAD_BUTTON_LEFT_FACE_RIGHT, "dpright"},
};

const ButtonName AXES[] = {
    {GAMEPAD_AXIS_LEFT_X, "leftx"},
    {GAMEPAD_AXIS_LEFT_Y, "lefty"},
    {GAMEPAD_AXIS_RIGHT_X, "rightx"},
    {GAMEPAD_AXIS_RIGHT_Y, "righty"},
    {GAMEPAD_AXIS_LEFT_TRIGGER, "triggerleft"},
    {GAMEPAD_AXIS_RIGHT_TRIGGER, "triggerright"},
};

int buttonFromName(const char *name)
{
    for (const ButtonName &b : BUTTONS)
    {
        if (std::strcmp(b.name, name) == 0)
        {
            return b.button;
        }
    }
    return -1;
}

int axisFromName(const char *name)
{
    for (const ButtonName &a : AXES)
    {
        if (std::strcmp(a.name, name) == 0)
        {
            return a.button;
        }
    }
    return -1;
}

Joystick *checkJoystick(lua_State *L, int idx)
{
    return luax::checkobject<Joystick>(L, idx, JOYSTICK_TYPE);
}

bool connected(const Joystick *js)
{
    return window::isOpen() && IsGamepadAvailable(js->index);
}

int js_isConnected(lua_State *L)
{
    lua_pushboolean(L, connected(checkJoystick(L, 1)));
    return 1;
}

int js_getName(lua_State *L)
{
    Joystick *js = checkJoystick(L, 1);
    const char *name = connected(js) ? GetGamepadName(js->index) : nullptr;
    lua_pushstring(L, name != nullptr ? name : "Gamepad");
    return 1;
}

int js_getID(lua_State *L)
{
    Joystick *js = checkJoystick(L, 1);
    lua_pushinteger(L, js->index + 1);
    lua_pushinteger(L, js->index + 1);
    return 2;
}

int js_getGUID(lua_State *L)
{
    Joystick *js = checkJoystick(L, 1);
    lua_pushfstring(L, "raylib-gamepad-%d", js->index);
    return 1;
}

int js_getAxisCount(lua_State *L)
{
    Joystick *js = checkJoystick(L, 1);
    lua_pushinteger(L, connected(js) ? GetGamepadAxisCount(js->index) : 0);
    return 1;
}

int js_getButtonCount(lua_State *L)
{
    lua_pushinteger(L, GAMEPAD_BUTTON_RIGHT_THUMB);
    return 1;
}

int js_getHatCount(lua_State *L)
{
    lua_pushinteger(L, 0);
    return 1;
}

int js_getAxis(lua_State *L)
{
    Joystick *js = checkJoystick(L, 1);
    int axis = static_cast<int>(luaL_checkinteger(L, 2)) - 1;
    float value = connected(js) && axis >= 0 ? GetGamepadAxisMovement(js->index, axis) : 0.0f;
    lua_pushnumber(L, value);
    return 1;
}

int js_getAxes(lua_State *L)
{
    Joystick *js = checkJoystick(L, 1);
    int count = connected(js) ? GetGamepadAxisCount(js->index) : 0;
    for (int i = 0; i < count; ++i)
    {
        lua_pushnumber(L, GetGamepadAxisMovement(js->index, i));
    }
    return count;
}

int js_isDown(lua_State *L)
{
    Joystick *js = checkJoystick(L, 1);
    int n = lua_gettop(L);
    bool down = false;
    for (int i = 2; i <= n && !down; ++i)
    {
        int button = static_cast<int>(luaL_checkinteger(L, i));
        down = connected(js) && IsGamepadButtonDown(js->index, button);
    }
    lua_pushboolean(L, down);
    return 1;
}

int js_isGamepad(lua_State *L)
{
    lua_pushboolean(L, 1);
    return 1;
}

int js_isGamepadDown(lua_State *L)
{
    Joystick *js = checkJoystick(L, 1);
    int n = lua_gettop(L);
    bool down = false;
    for (int i = 2; i <= n && !down; ++i)
    {
        int button = buttonFromName(luaL_checkstring(L, i));
        if (button < 0)
        {
            return luaL_error(L, "Invalid gamepad button: %s", lua_tostring(L, i));
        }
        down = connected(js) && IsGamepadButtonDown(js->index, button);
    }
    lua_pushboolean(L, down);
    return 1;
}

int js_getGamepadAxis(lua_State *L)
{
    Joystick *js = checkJoystick(L, 1);
    int axis = axisFromName(luaL_checkstring(L, 2));
    if (axis < 0)
    {
        return luaL_error(L, "Invalid gamepad axis: %s", lua_tostring(L, 2));
    }
    double value = connected(js) ? GetGamepadAxisMovement(js->index, axis) : 0.0;
    if (axis == GAMEPAD_AXIS_LEFT_TRIGGER || axis == GAMEPAD_AXIS_RIGHT_TRIGGER)
    {
        value = (value + 1.0) * 0.5;
    }
    lua_pushnumber(L, value);
    return 1;
}

int js_getHat(lua_State *L)
{
    lua_pushstring(L, "c");
    return 1;
}

int js_isVibrationSupported(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

int js_setVibration(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

int js_getVibration(lua_State *L)
{
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    return 2;
}

int js_getDeviceInfo(lua_State *L)
{
    lua_pushinteger(L, 0);
    lua_pushinteger(L, 0);
    lua_pushinteger(L, 0);
    return 3;
}

int js_getJoystickType(lua_State *L)
{
    lua_pushstring(L, "gamepad");
    return 1;
}

int js_getGamepadType(lua_State *L)
{
    lua_pushstring(L, "unknown");
    return 1;
}

int js_getGamepadMapping(lua_State *L)
{
    lua_pushnil(L);
    return 1;
}

int js_getGamepadMappingString(lua_State *L)
{
    lua_pushnil(L);
    return 1;
}

int js_tostring(lua_State *L)
{
    Joystick *js = checkJoystick(L, 1);
    lua_pushfstring(L, "Joystick: %d", js->index + 1);
    return 1;
}

const luaL_Reg JOYSTICK_METHODS[] = {
    {"isConnected", js_isConnected},
    {"getName", js_getName},
    {"getID", js_getID},
    {"getGUID", js_getGUID},
    {"getAxisCount", js_getAxisCount},
    {"getButtonCount", js_getButtonCount},
    {"getHatCount", js_getHatCount},
    {"getAxis", js_getAxis},
    {"getAxes", js_getAxes},
    {"isDown", js_isDown},
    {"isGamepad", js_isGamepad},
    {"isGamepadDown", js_isGamepadDown},
    {"getGamepadAxis", js_getGamepadAxis},
    {"getHat", js_getHat},
    {"isVibrationSupported", js_isVibrationSupported},
    {"setVibration", js_setVibration},
    {"getVibration", js_getVibration},
    {"getDeviceInfo", js_getDeviceInfo},
    {"getJoystickType", js_getJoystickType},
    {"getGamepadType", js_getGamepadType},
    {"getGamepadMapping", js_getGamepadMapping},
    {"getGamepadMappingString", js_getGamepadMappingString},
    {"__tostring", js_tostring},
    {nullptr, nullptr},
};

int l_getJoysticks(lua_State *L)
{
    lua_newtable(L);
    int n = 0;
    for (int i = 0; i < MAX_PADS; ++i)
    {
        if (window::isOpen() && IsGamepadAvailable(i))
        {
            pushJoystick(L, i);
            lua_rawseti(L, -2, ++n);
        }
    }
    return 1;
}

int l_getJoystickCount(lua_State *L)
{
    int count = 0;
    for (int i = 0; i < MAX_PADS; ++i)
    {
        if (window::isOpen() && IsGamepadAvailable(i))
        {
            ++count;
        }
    }
    lua_pushinteger(L, count);
    return 1;
}

int l_setGamepadMapping(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

int l_loadGamepadMappings(lua_State *L)
{
    const char *mappings = luaL_checkstring(L, 1);
    if (window::isOpen())
    {
        SetGamepadMappings(mappings);
    }
    return 0;
}

int l_saveGamepadMappings(lua_State *L)
{
    lua_pushstring(L, "");
    return 1;
}

int l_getGamepadMappingString(lua_State *L)
{
    lua_pushnil(L);
    return 1;
}

const luaL_Reg FUNCS[] = {
    {"getJoysticks", l_getJoysticks},
    {"getJoystickCount", l_getJoystickCount},
    {"setGamepadMapping", l_setGamepadMapping},
    {"loadGamepadMappings", l_loadGamepadMappings},
    {"saveGamepadMappings", l_saveGamepadMappings},
    {"getGamepadMappingString", l_getGamepadMappingString},
    {nullptr, nullptr},
};

} // namespace

void pushJoystick(lua_State *L, int index)
{
    if (index < 0 || index >= MAX_PADS)
    {
        lua_pushnil(L);
        return;
    }
    if (luax::pushregistered(L, &g_pads[index]))
    {
        return;
    }
    Joystick *js = luax::newobject<Joystick>(L, JOYSTICK_TYPE);
    js->index = index;
    luax::registerobject(L, &g_pads[index], -1);
}

const char *gamepadButtonName(int raylibButton)
{
    for (const ButtonName &b : BUTTONS)
    {
        if (b.button == raylibButton)
        {
            return b.name;
        }
    }
    return nullptr;
}

const char *gamepadAxisName(int raylibAxis)
{
    for (const ButtonName &a : AXES)
    {
        if (a.button == raylibAxis)
        {
            return a.name;
        }
    }
    return nullptr;
}

} // namespace joystick

int open_joystick(lua_State *L)
{
    luax::newtype(L, joystick::JOYSTICK_TYPE, joystick::JOYSTICK_METHODS);
    luaL_newlib(L, joystick::FUNCS);
    return 1;
}

} // namespace love
