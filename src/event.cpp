// event.cpp - love.event
//
// raylib is polled, Love2D is event driven. love.event.pump() bridges the two:
// it compares the current input state with the previous frame and queues the
// corresponding Love2D events, which love.run then dispatches to the callbacks.
#include "love.hpp"
#include "luax.hpp"

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace love
{
namespace event
{

namespace
{

struct Arg
{
    enum Kind
    {
        Nil,
        Boolean,
        Number,
        String,
        Ref
    };
    Kind kind = Nil;
    bool b = false;
    double n = 0.0;
    std::string s;
    std::shared_ptr<luax::Ref> ref;

    static Arg boolean(bool v)
    {
        Arg a;
        a.kind = Boolean;
        a.b = v;
        return a;
    }
    static Arg number(double v)
    {
        Arg a;
        a.kind = Number;
        a.n = v;
        return a;
    }
    static Arg string(const char *v)
    {
        Arg a;
        a.kind = String;
        a.s = v != nullptr ? v : "";
        return a;
    }
};

struct Event
{
    std::string name;
    std::vector<Arg> args;
};

std::deque<Event> g_queue;

// Previous-frame state used to detect changes.
bool g_wasFocused = true;
bool g_wasMouseOnScreen = true;
bool g_wasVisible = true;
int g_lastWidth = 0;
int g_lastHeight = 0;
bool g_gamepadConnected[4] = {false, false, false, false};
float g_gamepadAxes[4][8] = {};

void push(const char *name, std::vector<Arg> args = {})
{
    g_queue.push_back({name, std::move(args)});
}

void pushArg(lua_State *L, const Arg &arg)
{
    switch (arg.kind)
    {
    case Arg::Boolean:
        lua_pushboolean(L, arg.b);
        break;
    case Arg::Number:
        lua_pushnumber(L, arg.n);
        break;
    case Arg::String:
        lua_pushlstring(L, arg.s.data(), arg.s.size());
        break;
    case Arg::Ref:
        arg.ref->push(L);
        break;
    default:
        lua_pushnil(L);
        break;
    }
}

Arg argFromLua(lua_State *L, int idx)
{
    switch (lua_type(L, idx))
    {
    case LUA_TBOOLEAN:
        return Arg::boolean(lua_toboolean(L, idx) != 0);
    case LUA_TNUMBER:
        return Arg::number(lua_tonumber(L, idx));
    case LUA_TSTRING:
        return Arg::string(lua_tostring(L, idx));
    case LUA_TNIL:
    case LUA_TNONE:
        return Arg();
    default:
    {
        Arg a;
        a.kind = Arg::Ref;
        a.ref = std::make_shared<luax::Ref>();
        a.ref->set(L, idx);
        return a;
    }
    }
}

// Joystick arguments are pushed as a reference to the shared Joystick object.
Arg joystickArg(lua_State *L, int index)
{
    joystick::pushJoystick(L, index);
    Arg a = argFromLua(L, -1);
    lua_pop(L, 1);
    return a;
}

void encodeUtf8(int codepoint, std::string &out)
{
    if (codepoint < 0x80)
    {
        out.push_back(static_cast<char>(codepoint));
    }
    else if (codepoint < 0x800)
    {
        out.push_back(static_cast<char>(0xC0 | (codepoint >> 6)));
        out.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }
    else if (codepoint < 0x10000)
    {
        out.push_back(static_cast<char>(0xE0 | (codepoint >> 12)));
        out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }
    else
    {
        out.push_back(static_cast<char>(0xF0 | (codepoint >> 18)));
        out.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }
}

void pumpKeyboard()
{
    for (int key : keyboard::allKeys())
    {
        const char *name = keyboard::nameFromKey(key);
        if (IsKeyPressed(key))
        {
            push("keypressed", {Arg::string(name), Arg::string(name), Arg::boolean(false)});
        }
        else if (keyboard::keyRepeatEnabled() && IsKeyPressedRepeat(key))
        {
            push("keypressed", {Arg::string(name), Arg::string(name), Arg::boolean(true)});
        }
        if (IsKeyReleased(key))
        {
            push("keyreleased", {Arg::string(name), Arg::string(name)});
        }
    }

    int codepoint;
    while ((codepoint = GetCharPressed()) != 0)
    {
        if (!keyboard::textInputEnabled())
        {
            continue;
        }
        std::string text;
        encodeUtf8(codepoint, text);
        push("textinput", {Arg::string(text.c_str())});
    }
}

void pumpMouse()
{
    Vector2 pos = GetMousePosition();
    Vector2 delta = GetMouseDelta();
    if (delta.x != 0.0f || delta.y != 0.0f)
    {
        push("mousemoved", {Arg::number(pos.x), Arg::number(pos.y), Arg::number(delta.x), Arg::number(delta.y), Arg::boolean(false)});
    }
    for (int button = MOUSE_BUTTON_LEFT; button <= MOUSE_BUTTON_BACK; ++button)
    {
        int loveButton = mouse::loveButton(button);
        if (IsMouseButtonPressed(button))
        {
            push("mousepressed", {Arg::number(pos.x), Arg::number(pos.y), Arg::number(loveButton), Arg::boolean(false), Arg::number(1)});
        }
        if (IsMouseButtonReleased(button))
        {
            push("mousereleased", {Arg::number(pos.x), Arg::number(pos.y), Arg::number(loveButton), Arg::boolean(false), Arg::number(1)});
        }
    }
    Vector2 wheel = GetMouseWheelMoveV();
    if (wheel.x != 0.0f || wheel.y != 0.0f)
    {
        push("wheelmoved", {Arg::number(wheel.x), Arg::number(wheel.y)});
    }
}

void pumpWindow()
{
    int width = GetScreenWidth();
    int height = GetScreenHeight();
    if (IsWindowResized() || width != g_lastWidth || height != g_lastHeight)
    {
        if (g_lastWidth != 0)
        {
            push("resize", {Arg::number(width), Arg::number(height)});
        }
        g_lastWidth = width;
        g_lastHeight = height;
    }

    bool focused = IsWindowFocused();
    if (focused != g_wasFocused)
    {
        g_wasFocused = focused;
        push("focus", {Arg::boolean(focused)});
    }

    bool onScreen = IsCursorOnScreen();
    if (onScreen != g_wasMouseOnScreen)
    {
        g_wasMouseOnScreen = onScreen;
        push("mousefocus", {Arg::boolean(onScreen)});
    }

    bool visible = !IsWindowMinimized() && !IsWindowHidden();
    if (visible != g_wasVisible)
    {
        g_wasVisible = visible;
        push("visible", {Arg::boolean(visible)});
    }

    if (IsFileDropped())
    {
        FilePathList files = LoadDroppedFiles();
        for (unsigned int i = 0; i < files.count; ++i)
        {
            push(DirectoryExists(files.paths[i]) ? "directorydropped" : "filedropped", {Arg::string(files.paths[i])});
        }
        UnloadDroppedFiles(files);
    }

    if (WindowShouldClose())
    {
        push("quit");
    }
}

void pumpGamepads(lua_State *L)
{
    for (int pad = 0; pad < 4; ++pad)
    {
        bool connected = IsGamepadAvailable(pad);
        if (connected != g_gamepadConnected[pad])
        {
            g_gamepadConnected[pad] = connected;
            push(connected ? "joystickadded" : "joystickremoved", {joystickArg(L, pad)});
            for (float &axis : g_gamepadAxes[pad])
            {
                axis = 0.0f;
            }
        }
        if (!connected)
        {
            continue;
        }

        for (int button = GAMEPAD_BUTTON_LEFT_FACE_UP; button <= GAMEPAD_BUTTON_RIGHT_THUMB; ++button)
        {
            const char *name = joystick::gamepadButtonName(button);
            if (IsGamepadButtonPressed(pad, button))
            {
                push("joystickpressed", {joystickArg(L, pad), Arg::number(button)});
                if (name != nullptr)
                {
                    push("gamepadpressed", {joystickArg(L, pad), Arg::string(name)});
                }
            }
            if (IsGamepadButtonReleased(pad, button))
            {
                push("joystickreleased", {joystickArg(L, pad), Arg::number(button)});
                if (name != nullptr)
                {
                    push("gamepadreleased", {joystickArg(L, pad), Arg::string(name)});
                }
            }
        }

        int axisCount = GetGamepadAxisCount(pad);
        if (axisCount > 8)
        {
            axisCount = 8;
        }
        for (int axis = 0; axis < axisCount; ++axis)
        {
            float value = GetGamepadAxisMovement(pad, axis);
            if (value != g_gamepadAxes[pad][axis])
            {
                g_gamepadAxes[pad][axis] = value;
                push("joystickaxis", {joystickArg(L, pad), Arg::number(axis + 1), Arg::number(value)});
                const char *name = joystick::gamepadAxisName(axis);
                if (name != nullptr)
                {
                    double mapped = value;
                    if (axis == GAMEPAD_AXIS_LEFT_TRIGGER || axis == GAMEPAD_AXIS_RIGHT_TRIGGER)
                    {
                        mapped = (value + 1.0) * 0.5;
                    }
                    push("gamepadaxis", {joystickArg(L, pad), Arg::string(name), Arg::number(mapped)});
                }
            }
        }
    }
}

int l_pump(lua_State *L)
{
    pump(L);
    return 0;
}

int poll_iterator(lua_State *L)
{
    if (g_queue.empty())
    {
        return 0;
    }
    Event event = std::move(g_queue.front());
    g_queue.pop_front();
    lua_pushstring(L, event.name.c_str());
    for (const Arg &arg : event.args)
    {
        pushArg(L, arg);
    }
    return 1 + static_cast<int>(event.args.size());
}

int l_poll(lua_State *L)
{
    lua_pushcfunction(L, poll_iterator);
    return 1;
}

int l_push(lua_State *L)
{
    const char *name = luaL_checkstring(L, 1);
    Event event;
    event.name = name;
    int n = lua_gettop(L);
    for (int i = 2; i <= n; ++i)
    {
        event.args.push_back(argFromLua(L, i));
    }
    g_queue.push_back(std::move(event));
    return 0;
}

int l_quit(lua_State *L)
{
    Event event;
    event.name = "quit";
    if (!lua_isnoneornil(L, 1))
    {
        event.args.push_back(argFromLua(L, 1));
    }
    g_queue.push_back(std::move(event));
    return 0;
}

int l_clear(lua_State *L)
{
    g_queue.clear();
    return 0;
}

int l_wait(lua_State *L)
{
    // raylib only polls input while presenting a frame, so waiting cannot
    // block; this returns the next queued event, if any.
    if (g_queue.empty())
    {
        pump(L);
    }
    return poll_iterator(L);
}

const luaL_Reg FUNCS[] = {
    {"pump", l_pump},
    {"poll", l_poll},
    {"push", l_push},
    {"quit", l_quit},
    {"clear", l_clear},
    {"wait", l_wait},
    {nullptr, nullptr},
};

} // namespace

void pump(lua_State *L)
{
    audio::update();
    if (!window::isOpen())
    {
        return;
    }
    pumpWindow();
    pumpKeyboard();
    pumpMouse();
    pumpGamepads(L);
}

} // namespace event

int open_event(lua_State *L)
{
    luaL_newlib(L, event::FUNCS);
    return 1;
}

} // namespace love
