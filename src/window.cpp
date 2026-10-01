// window.cpp - love.window
#include "love.hpp"
#include "luax.hpp"

#include <cstdlib>
#include <cstring>
#include <string>

namespace love
{
namespace window
{

namespace
{

struct State
{
    bool open = false;
    bool hidden = false;
    std::string title = "Untitled";
    int width = 800;
    int height = 600;
    bool fullscreen = false;
    std::string fullscreenType = "desktop";
    int vsync = 1;
    int msaa = 0;
    bool resizable = false;
    bool borderless = false;
    bool centered = true;
    int display = 1;
    int minWidth = 1;
    int minHeight = 1;
    bool highdpi = false;
    bool useDpiScale = true;
    bool hasPosition = false;
    int x = 0;
    int y = 0;
    int maxfps = 0;
    bool displaySleep = true;
};

State g_state;

unsigned int configFlags(const State &s)
{
    unsigned int flags = 0;
    if (s.vsync != 0)
    {
        flags |= FLAG_VSYNC_HINT;
    }
    if (s.resizable)
    {
        flags |= FLAG_WINDOW_RESIZABLE;
    }
    if (s.borderless)
    {
        flags |= FLAG_WINDOW_UNDECORATED;
    }
    if (s.msaa > 0)
    {
        flags |= FLAG_MSAA_4X_HINT;
    }
    if (s.highdpi)
    {
        flags |= FLAG_WINDOW_HIGHDPI;
    }
    if (s.hidden)
    {
        flags |= FLAG_WINDOW_HIDDEN;
    }
    if (s.fullscreen)
    {
        flags |= (s.fullscreenType == "exclusive") ? FLAG_FULLSCREEN_MODE : FLAG_BORDERLESS_WINDOWED_MODE;
    }
    return flags;
}

bool openWindow(State &s)
{
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(configFlags(s));
    InitWindow(s.width, s.height, s.title.c_str());
    if (!IsWindowReady())
    {
        return false;
    }
    s.open = true;

    // Love2D never quits on Escape; the game decides.
    SetExitKey(KEY_NULL);
    SetWindowMinSize(s.minWidth, s.minHeight);
    SetTargetFPS(s.maxfps);
    if (s.hasPosition)
    {
        SetWindowPosition(s.x, s.y);
    }
    if (s.display > 1 && s.display <= GetMonitorCount() && s.fullscreen)
    {
        SetWindowMonitor(s.display - 1);
    }
    return true;
}

void applyRuntimeChanges(const State &previous, State &s)
{
    if (previous.title != s.title)
    {
        SetWindowTitle(s.title.c_str());
    }
    if (previous.vsync != s.vsync)
    {
        if (s.vsync != 0)
        {
            SetWindowState(FLAG_VSYNC_HINT);
        }
        else
        {
            ClearWindowState(FLAG_VSYNC_HINT);
        }
    }
    if (previous.resizable != s.resizable)
    {
        if (s.resizable)
        {
            SetWindowState(FLAG_WINDOW_RESIZABLE);
        }
        else
        {
            ClearWindowState(FLAG_WINDOW_RESIZABLE);
        }
    }
    if (previous.borderless != s.borderless)
    {
        if (s.borderless)
        {
            SetWindowState(FLAG_WINDOW_UNDECORATED);
        }
        else
        {
            ClearWindowState(FLAG_WINDOW_UNDECORATED);
        }
    }
    SetWindowMinSize(s.minWidth, s.minHeight);
    SetTargetFPS(s.maxfps);

    bool wantExclusive = s.fullscreen && s.fullscreenType == "exclusive";
    bool wantBorderless = s.fullscreen && !wantExclusive;
    bool isExclusive = IsWindowFullscreen();
    bool isBorderless = IsWindowState(FLAG_BORDERLESS_WINDOWED_MODE);

    if (isExclusive != wantExclusive)
    {
        ToggleFullscreen();
    }
    if (isBorderless != wantBorderless)
    {
        ToggleBorderlessWindowed();
    }
    if (!s.fullscreen)
    {
        if (GetScreenWidth() != s.width || GetScreenHeight() != s.height)
        {
            SetWindowSize(s.width, s.height);
        }
        if (s.hasPosition)
        {
            SetWindowPosition(s.x, s.y);
        }
    }
    if (s.hidden)
    {
        SetWindowState(FLAG_WINDOW_HIDDEN);
    }
    else
    {
        ClearWindowState(FLAG_WINDOW_HIDDEN);
    }
}

void readFlags(lua_State *L, int idx, State &s)
{
    if (lua_isnoneornil(L, idx))
    {
        return;
    }
    luaL_checktype(L, idx, LUA_TTABLE);
    s.fullscreen = luax::getboolfield(L, idx, "fullscreen", s.fullscreen);
    s.fullscreenType = luax::getstringfield(L, idx, "fullscreentype", s.fullscreenType.c_str());
    lua_getfield(L, idx, "vsync");
    if (lua_isboolean(L, -1))
    {
        s.vsync = lua_toboolean(L, -1) ? 1 : 0;
    }
    else if (lua_isnumber(L, -1))
    {
        s.vsync = static_cast<int>(lua_tointeger(L, -1));
    }
    lua_pop(L, 1);
    s.msaa = static_cast<int>(luax::getnumberfield(L, idx, "msaa", s.msaa));
    s.resizable = luax::getboolfield(L, idx, "resizable", s.resizable);
    s.borderless = luax::getboolfield(L, idx, "borderless", s.borderless);
    s.centered = luax::getboolfield(L, idx, "centered", s.centered);
    s.display = static_cast<int>(luax::getnumberfield(L, idx, "display", s.display));
    s.minWidth = static_cast<int>(luax::getnumberfield(L, idx, "minwidth", s.minWidth));
    s.minHeight = static_cast<int>(luax::getnumberfield(L, idx, "minheight", s.minHeight));
    s.highdpi = luax::getboolfield(L, idx, "highdpi", s.highdpi);
    s.useDpiScale = luax::getboolfield(L, idx, "usedpiscale", s.useDpiScale);
    s.maxfps = static_cast<int>(luax::getnumberfield(L, idx, "maxfps", s.maxfps));
    lua_getfield(L, idx, "x");
    bool hasX = lua_isnumber(L, -1);
    int x = hasX ? static_cast<int>(lua_tointeger(L, -1)) : 0;
    lua_pop(L, 1);
    lua_getfield(L, idx, "y");
    bool hasY = lua_isnumber(L, -1);
    int y = hasY ? static_cast<int>(lua_tointeger(L, -1)) : 0;
    lua_pop(L, 1);
    if (hasX && hasY)
    {
        s.hasPosition = true;
        s.x = x;
        s.y = y;
    }
}

int l_setMode(lua_State *L)
{
    State next = g_state;
    next.width = static_cast<int>(luaL_checkinteger(L, 1));
    next.height = static_cast<int>(luaL_checkinteger(L, 2));
    next.hidden = false;
    readFlags(L, 3, next);
    if (next.width <= 0 || next.height <= 0)
    {
        lua_pushboolean(L, 0);
        lua_pushstring(L, "Invalid window dimensions");
        return 2;
    }

    if (!g_state.open)
    {
        g_state = next;
        if (!openWindow(g_state))
        {
            g_state.open = false;
            lua_pushboolean(L, 0);
            lua_pushstring(L, "Could not create window");
            return 2;
        }
    }
    else
    {
        State previous = g_state;
        g_state = next;
        applyRuntimeChanges(previous, g_state);
    }
    lua_pushboolean(L, 1);
    return 1;
}

int l_getMode(lua_State *L)
{
    const State &s = g_state;
    lua_pushinteger(L, s.open ? GetScreenWidth() : s.width);
    lua_pushinteger(L, s.open ? GetScreenHeight() : s.height);
    lua_newtable(L);
    lua_pushboolean(L, s.fullscreen);
    lua_setfield(L, -2, "fullscreen");
    lua_pushstring(L, s.fullscreenType.c_str());
    lua_setfield(L, -2, "fullscreentype");
    lua_pushinteger(L, s.vsync);
    lua_setfield(L, -2, "vsync");
    lua_pushinteger(L, s.msaa);
    lua_setfield(L, -2, "msaa");
    lua_pushboolean(L, s.resizable);
    lua_setfield(L, -2, "resizable");
    lua_pushboolean(L, s.borderless);
    lua_setfield(L, -2, "borderless");
    lua_pushboolean(L, s.centered);
    lua_setfield(L, -2, "centered");
    lua_pushinteger(L, s.open ? GetCurrentMonitor() + 1 : s.display);
    lua_setfield(L, -2, "display");
    lua_pushinteger(L, s.minWidth);
    lua_setfield(L, -2, "minwidth");
    lua_pushinteger(L, s.minHeight);
    lua_setfield(L, -2, "minheight");
    lua_pushboolean(L, s.highdpi);
    lua_setfield(L, -2, "highdpi");
    lua_pushboolean(L, s.useDpiScale);
    lua_setfield(L, -2, "usedpiscale");
    if (s.open)
    {
        Vector2 pos = GetWindowPosition();
        lua_pushinteger(L, static_cast<int>(pos.x));
        lua_setfield(L, -2, "x");
        lua_pushinteger(L, static_cast<int>(pos.y));
        lua_setfield(L, -2, "y");
        lua_pushinteger(L, GetMonitorRefreshRate(GetCurrentMonitor()));
        lua_setfield(L, -2, "refreshrate");
    }
    lua_pushinteger(L, s.maxfps);
    lua_setfield(L, -2, "maxfps");
    return 3;
}

int l_updateMode(lua_State *L)
{
    return l_setMode(L);
}

int l_isOpen(lua_State *L)
{
    lua_pushboolean(L, g_state.open);
    return 1;
}

int l_close(lua_State *L)
{
    shutdown();
    return 0;
}

int l_setTitle(lua_State *L)
{
    g_state.title = luaL_checkstring(L, 1);
    if (g_state.open)
    {
        SetWindowTitle(g_state.title.c_str());
    }
    return 0;
}

int l_getTitle(lua_State *L)
{
    lua_pushstring(L, g_state.title.c_str());
    return 1;
}

int l_setFullscreen(lua_State *L)
{
    State previous = g_state;
    g_state.fullscreen = luax::checkboolean(L, 1);
    if (!lua_isnoneornil(L, 2))
    {
        static const char *const names[] = {"desktop", "exclusive", "normal"};
        static const int values[] = {0, 1, 1};
        g_state.fullscreenType = luax::checkenum(L, 2, names, values, "fullscreen type") == 1 ? "exclusive" : "desktop";
    }
    if (g_state.open)
    {
        applyRuntimeChanges(previous, g_state);
    }
    lua_pushboolean(L, 1);
    return 1;
}

int l_getFullscreen(lua_State *L)
{
    lua_pushboolean(L, g_state.fullscreen);
    lua_pushstring(L, g_state.fullscreenType.c_str());
    return 2;
}

int l_getFullscreenModes(lua_State *L)
{
    int monitor = static_cast<int>(luaL_optinteger(L, 1, 1)) - 1;
    lua_newtable(L);
    if (!g_state.open || monitor < 0 || monitor >= GetMonitorCount())
    {
        return 1;
    }
    static const int common[][2] = {
        {3840, 2160}, {2560, 1440}, {1920, 1200}, {1920, 1080}, {1680, 1050}, {1600, 900},
        {1440, 900}, {1366, 768}, {1280, 1024}, {1280, 800}, {1280, 720}, {1024, 768}, {800, 600},
    };
    int mw = GetMonitorWidth(monitor);
    int mh = GetMonitorHeight(monitor);
    int n = 0;
    for (const int *mode : common)
    {
        if (mode[0] <= mw && mode[1] <= mh)
        {
            lua_newtable(L);
            lua_pushinteger(L, mode[0]);
            lua_setfield(L, -2, "width");
            lua_pushinteger(L, mode[1]);
            lua_setfield(L, -2, "height");
            lua_rawseti(L, -2, ++n);
        }
    }
    return 1;
}

int l_getDisplayCount(lua_State *L)
{
    lua_pushinteger(L, g_state.open ? GetMonitorCount() : 1);
    return 1;
}

int l_getDisplayName(lua_State *L)
{
    int monitor = static_cast<int>(luaL_optinteger(L, 1, 1)) - 1;
    if (!g_state.open || monitor < 0 || monitor >= GetMonitorCount())
    {
        return luaL_error(L, "Invalid display index: %d", monitor + 1);
    }
    lua_pushstring(L, GetMonitorName(monitor));
    return 1;
}

int l_getDesktopDimensions(lua_State *L)
{
    int monitor = static_cast<int>(luaL_optinteger(L, 1, 1)) - 1;
    if (!g_state.open)
    {
        lua_pushinteger(L, g_state.width);
        lua_pushinteger(L, g_state.height);
        return 2;
    }
    if (monitor < 0 || monitor >= GetMonitorCount())
    {
        monitor = GetCurrentMonitor();
    }
    lua_pushinteger(L, GetMonitorWidth(monitor));
    lua_pushinteger(L, GetMonitorHeight(monitor));
    return 2;
}

int l_getPosition(lua_State *L)
{
    if (!g_state.open)
    {
        lua_pushinteger(L, g_state.x);
        lua_pushinteger(L, g_state.y);
        lua_pushinteger(L, g_state.display);
        return 3;
    }
    Vector2 pos = GetWindowPosition();
    lua_pushinteger(L, static_cast<int>(pos.x));
    lua_pushinteger(L, static_cast<int>(pos.y));
    lua_pushinteger(L, GetCurrentMonitor() + 1);
    return 3;
}

int l_setPosition(lua_State *L)
{
    g_state.hasPosition = true;
    g_state.x = static_cast<int>(luaL_checkinteger(L, 1));
    g_state.y = static_cast<int>(luaL_checkinteger(L, 2));
    if (g_state.open)
    {
        SetWindowPosition(g_state.x, g_state.y);
    }
    return 0;
}

int l_hasFocus(lua_State *L)
{
    lua_pushboolean(L, g_state.open && IsWindowFocused());
    return 1;
}

int l_hasMouseFocus(lua_State *L)
{
    lua_pushboolean(L, g_state.open && IsCursorOnScreen());
    return 1;
}

int l_isVisible(lua_State *L)
{
    lua_pushboolean(L, g_state.open && !g_state.hidden && !IsWindowHidden() && !IsWindowMinimized());
    return 1;
}

int l_minimize(lua_State *L)
{
    if (g_state.open)
    {
        MinimizeWindow();
    }
    return 0;
}

int l_maximize(lua_State *L)
{
    if (g_state.open)
    {
        MaximizeWindow();
    }
    return 0;
}

int l_restore(lua_State *L)
{
    if (g_state.open)
    {
        RestoreWindow();
    }
    return 0;
}

int l_isMaximized(lua_State *L)
{
    lua_pushboolean(L, g_state.open && IsWindowMaximized());
    return 1;
}

int l_isMinimized(lua_State *L)
{
    lua_pushboolean(L, g_state.open && IsWindowMinimized());
    return 1;
}

float dpiScale()
{
    if (!g_state.open || !g_state.useDpiScale)
    {
        return 1.0f;
    }
    return GetWindowScaleDPI().x;
}

int l_getDPIScale(lua_State *L)
{
    lua_pushnumber(L, dpiScale());
    return 1;
}

int l_toPixels(lua_State *L)
{
    float scale = dpiScale();
    int n = lua_gettop(L);
    for (int i = 1; i <= n; ++i)
    {
        lua_pushnumber(L, luaL_checknumber(L, i) * scale);
    }
    return n;
}

int l_fromPixels(lua_State *L)
{
    float scale = dpiScale();
    int n = lua_gettop(L);
    for (int i = 1; i <= n; ++i)
    {
        lua_pushnumber(L, luaL_checknumber(L, i) / scale);
    }
    return n;
}

int l_setVSync(lua_State *L)
{
    State previous = g_state;
    if (lua_isboolean(L, 1))
    {
        g_state.vsync = lua_toboolean(L, 1) ? 1 : 0;
    }
    else
    {
        g_state.vsync = static_cast<int>(luaL_checkinteger(L, 1));
    }
    if (g_state.open)
    {
        applyRuntimeChanges(previous, g_state);
    }
    return 0;
}

int l_getVSync(lua_State *L)
{
    lua_pushinteger(L, g_state.vsync);
    return 1;
}

int l_requestAttention(lua_State *L)
{
    return 0;
}

int l_setIcon(lua_State *L)
{
    // Accepts an ImageData (love.image) or a file path.
    if (lua_type(L, 1) == LUA_TSTRING)
    {
        std::string real = filesystem::resolveRead(lua_tostring(L, 1));
        if (real.empty())
        {
            lua_pushboolean(L, 0);
            return 1;
        }
        Image image = LoadImage(real.c_str());
        if (image.data != nullptr && g_state.open)
        {
            SetWindowIcon(image);
        }
        UnloadImage(image);
        lua_pushboolean(L, image.data != nullptr);
        return 1;
    }
    Image *image = luax::checkobject<Image>(L, 1, "ImageData");
    if (g_state.open && image->data != nullptr)
    {
        SetWindowIcon(*image);
    }
    lua_pushboolean(L, 1);
    return 1;
}

int l_getIcon(lua_State *L)
{
    lua_pushnil(L);
    return 1;
}

int l_showMessageBox(lua_State *L)
{
    const char *title = luaL_checkstring(L, 1);
    const char *message = luaL_checkstring(L, 2);
    log(LogLevel::Info, "[%s] %s", title, message);
    if (lua_type(L, 3) == LUA_TTABLE)
    {
        // No native dialog: report the first button as pressed.
        lua_pushinteger(L, 1);
        return 1;
    }
    lua_pushboolean(L, 1);
    return 1;
}

int l_setDisplaySleepEnabled(lua_State *L)
{
    g_state.displaySleep = luax::checkboolean(L, 1);
    return 0;
}

int l_isDisplaySleepEnabled(lua_State *L)
{
    lua_pushboolean(L, g_state.displaySleep);
    return 1;
}

int l_getSafeArea(lua_State *L)
{
    lua_pushinteger(L, 0);
    lua_pushinteger(L, 0);
    lua_pushinteger(L, g_state.open ? GetScreenWidth() : g_state.width);
    lua_pushinteger(L, g_state.open ? GetScreenHeight() : g_state.height);
    return 4;
}

int l_openHidden(lua_State *L)
{
    ensureOpen();
    return 0;
}

const luaL_Reg FUNCS[] = {
    {"setMode", l_setMode},
    {"updateMode", l_updateMode},
    {"getMode", l_getMode},
    {"isOpen", l_isOpen},
    {"close", l_close},
    {"setTitle", l_setTitle},
    {"getTitle", l_getTitle},
    {"setFullscreen", l_setFullscreen},
    {"getFullscreen", l_getFullscreen},
    {"getFullscreenModes", l_getFullscreenModes},
    {"getDisplayCount", l_getDisplayCount},
    {"getDisplayName", l_getDisplayName},
    {"getDesktopDimensions", l_getDesktopDimensions},
    {"getPosition", l_getPosition},
    {"setPosition", l_setPosition},
    {"hasFocus", l_hasFocus},
    {"hasMouseFocus", l_hasMouseFocus},
    {"isVisible", l_isVisible},
    {"minimize", l_minimize},
    {"maximize", l_maximize},
    {"restore", l_restore},
    {"isMaximized", l_isMaximized},
    {"isMinimized", l_isMinimized},
    {"getDPIScale", l_getDPIScale},
    {"toPixels", l_toPixels},
    {"fromPixels", l_fromPixels},
    {"setVSync", l_setVSync},
    {"getVSync", l_getVSync},
    {"requestAttention", l_requestAttention},
    {"setIcon", l_setIcon},
    {"getIcon", l_getIcon},
    {"showMessageBox", l_showMessageBox},
    {"setDisplaySleepEnabled", l_setDisplaySleepEnabled},
    {"isDisplaySleepEnabled", l_isDisplaySleepEnabled},
    {"getSafeArea", l_getSafeArea},
    {"_openHidden", l_openHidden},
    {nullptr, nullptr},
};

} // namespace

bool isOpen()
{
    return g_state.open;
}

void ensureOpen()
{
    if (g_state.open)
    {
        return;
    }
    g_state.hidden = true;
    if (!openWindow(g_state))
    {
        g_state.open = false;
        log(LogLevel::Error, "Could not create the window (is a display available?)");
        std::exit(1);
    }
}

void shutdown()
{
    if (g_state.open)
    {
        CloseWindow();
        g_state.open = false;
    }
}

} // namespace window

int open_window(lua_State *L)
{
    luaL_newlib(L, window::FUNCS);
    return 1;
}

} // namespace love
