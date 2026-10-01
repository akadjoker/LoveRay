// mouse.cpp - love.mouse
#include "love.hpp"
#include "luax.hpp"

#include <cstring>

namespace love
{
namespace mouse
{

namespace
{

const char *CURSOR_TYPE = "Cursor";

struct Cursor
{
    int type;
    const char *name;
};

bool g_grabbed = false;
bool g_relative = false;
Cursor g_current = {MOUSE_CURSOR_DEFAULT, "arrow"};

const char *const CURSOR_NAMES[] = {
    "arrow", "ibeam", "wait", "crosshair", "waitarrow", "sizenwse", "sizenesw",
    "sizewe", "sizens", "sizeall", "no", "hand",
};
const int CURSOR_VALUES[] = {
    MOUSE_CURSOR_ARROW, MOUSE_CURSOR_IBEAM, MOUSE_CURSOR_DEFAULT, MOUSE_CURSOR_CROSSHAIR,
    MOUSE_CURSOR_DEFAULT, MOUSE_CURSOR_RESIZE_NWSE, MOUSE_CURSOR_RESIZE_NESW,
    MOUSE_CURSOR_RESIZE_EW, MOUSE_CURSOR_RESIZE_NS, MOUSE_CURSOR_RESIZE_ALL,
    MOUSE_CURSOR_NOT_ALLOWED, MOUSE_CURSOR_POINTING_HAND,
};

int l_getPosition(lua_State *L)
{
    Vector2 pos = window::isOpen() ? GetMousePosition() : Vector2{0, 0};
    lua_pushnumber(L, pos.x);
    lua_pushnumber(L, pos.y);
    return 2;
}

int l_getX(lua_State *L)
{
    lua_pushnumber(L, window::isOpen() ? GetMouseX() : 0);
    return 1;
}

int l_getY(lua_State *L)
{
    lua_pushnumber(L, window::isOpen() ? GetMouseY() : 0);
    return 1;
}

int l_setPosition(lua_State *L)
{
    int x = static_cast<int>(luaL_checknumber(L, 1));
    int y = static_cast<int>(luaL_checknumber(L, 2));
    if (window::isOpen())
    {
        SetMousePosition(x, y);
    }
    return 0;
}

int l_setX(lua_State *L)
{
    int x = static_cast<int>(luaL_checknumber(L, 1));
    if (window::isOpen())
    {
        SetMousePosition(x, GetMouseY());
    }
    return 0;
}

int l_setY(lua_State *L)
{
    int y = static_cast<int>(luaL_checknumber(L, 1));
    if (window::isOpen())
    {
        SetMousePosition(GetMouseX(), y);
    }
    return 0;
}

int l_isDown(lua_State *L)
{
    int n = lua_gettop(L);
    bool down = false;
    for (int i = 1; i <= n && !down; ++i)
    {
        int button = raylibButton(static_cast<int>(luaL_checkinteger(L, i)));
        down = button >= 0 && window::isOpen() && IsMouseButtonDown(button);
    }
    lua_pushboolean(L, down);
    return 1;
}

int l_setVisible(lua_State *L)
{
    bool visible = luax::checkboolean(L, 1);
    if (window::isOpen())
    {
        if (visible)
        {
            ShowCursor();
        }
        else
        {
            HideCursor();
        }
    }
    return 0;
}

int l_isVisible(lua_State *L)
{
    lua_pushboolean(L, !window::isOpen() || !IsCursorHidden());
    return 1;
}

int l_setGrabbed(lua_State *L)
{
    // raylib has no window-confinement mode; the flag is kept for queries.
    g_grabbed = luax::checkboolean(L, 1);
    return 0;
}

int l_isGrabbed(lua_State *L)
{
    lua_pushboolean(L, g_grabbed);
    return 1;
}

int l_setRelativeMode(lua_State *L)
{
    bool enable = luax::checkboolean(L, 1);
    if (window::isOpen() && enable != g_relative)
    {
        if (enable)
        {
            DisableCursor();
        }
        else
        {
            EnableCursor();
        }
    }
    g_relative = enable;
    lua_pushboolean(L, 1);
    return 1;
}

int l_getRelativeMode(lua_State *L)
{
    lua_pushboolean(L, g_relative);
    return 1;
}

int l_isCursorSupported(lua_State *L)
{
    lua_pushboolean(L, 1);
    return 1;
}

void pushCursor(lua_State *L, int type, const char *name)
{
    Cursor *cursor = luax::newobject<Cursor>(L, CURSOR_TYPE);
    cursor->type = type;
    cursor->name = name;
}

int l_getSystemCursor(lua_State *L)
{
    int type = luax::checkenum(L, 1, CURSOR_NAMES, CURSOR_VALUES, "system cursor");
    pushCursor(L, type, lua_tostring(L, 1));
    return 1;
}

int l_newCursor(lua_State *L)
{
    return luaL_error(L, "love.mouse.newCursor: custom image cursors are not supported by raylib; use love.mouse.getSystemCursor");
}

int l_setCursor(lua_State *L)
{
    if (lua_isnoneornil(L, 1))
    {
        g_current = {MOUSE_CURSOR_DEFAULT, "arrow"};
    }
    else
    {
        Cursor *cursor = luax::checkobject<Cursor>(L, 1, CURSOR_TYPE);
        g_current = *cursor;
    }
    if (window::isOpen())
    {
        SetMouseCursor(g_current.type);
    }
    return 0;
}

int l_getCursor(lua_State *L)
{
    pushCursor(L, g_current.type, g_current.name);
    return 1;
}

int cursor_getType(lua_State *L)
{
    Cursor *cursor = luax::checkobject<Cursor>(L, 1, CURSOR_TYPE);
    lua_pushstring(L, cursor->name);
    return 1;
}

const luaL_Reg CURSOR_METHODS[] = {
    {"getType", cursor_getType},
    {nullptr, nullptr},
};

const luaL_Reg FUNCS[] = {
    {"getPosition", l_getPosition},
    {"getX", l_getX},
    {"getY", l_getY},
    {"setPosition", l_setPosition},
    {"setX", l_setX},
    {"setY", l_setY},
    {"isDown", l_isDown},
    {"setVisible", l_setVisible},
    {"isVisible", l_isVisible},
    {"setGrabbed", l_setGrabbed},
    {"isGrabbed", l_isGrabbed},
    {"setRelativeMode", l_setRelativeMode},
    {"getRelativeMode", l_getRelativeMode},
    {"isCursorSupported", l_isCursorSupported},
    {"getSystemCursor", l_getSystemCursor},
    {"newCursor", l_newCursor},
    {"setCursor", l_setCursor},
    {"getCursor", l_getCursor},
    {nullptr, nullptr},
};

} // namespace

int raylibButton(int loveButton)
{
    switch (loveButton)
    {
    case 1:
        return MOUSE_BUTTON_LEFT;
    case 2:
        return MOUSE_BUTTON_RIGHT;
    case 3:
        return MOUSE_BUTTON_MIDDLE;
    case 4:
        return MOUSE_BUTTON_SIDE;
    case 5:
        return MOUSE_BUTTON_EXTRA;
    case 6:
        return MOUSE_BUTTON_FORWARD;
    case 7:
        return MOUSE_BUTTON_BACK;
    default:
        return -1;
    }
}

int loveButton(int raylibButton)
{
    switch (raylibButton)
    {
    case MOUSE_BUTTON_LEFT:
        return 1;
    case MOUSE_BUTTON_RIGHT:
        return 2;
    case MOUSE_BUTTON_MIDDLE:
        return 3;
    case MOUSE_BUTTON_SIDE:
        return 4;
    case MOUSE_BUTTON_EXTRA:
        return 5;
    case MOUSE_BUTTON_FORWARD:
        return 6;
    case MOUSE_BUTTON_BACK:
        return 7;
    default:
        return 0;
    }
}

} // namespace mouse

int open_mouse(lua_State *L)
{
    luax::newtype(L, mouse::CURSOR_TYPE, mouse::CURSOR_METHODS);
    luaL_newlib(L, mouse::FUNCS);
    return 1;
}

} // namespace love
