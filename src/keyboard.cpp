// keyboard.cpp - love.keyboard and the KeyConstant <-> raylib key mapping.
#include "love.hpp"
#include "luax.hpp"

#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

namespace love
{
namespace keyboard
{

namespace
{

struct KeyName
{
    const char *name;
    int key;
};

// Love2D KeyConstants (https://love2d.org/wiki/KeyConstant) mapped onto raylib.
const KeyName KEYS[] = {
    {"a", KEY_A}, {"b", KEY_B}, {"c", KEY_C}, {"d", KEY_D}, {"e", KEY_E}, {"f", KEY_F},
    {"g", KEY_G}, {"h", KEY_H}, {"i", KEY_I}, {"j", KEY_J}, {"k", KEY_K}, {"l", KEY_L},
    {"m", KEY_M}, {"n", KEY_N}, {"o", KEY_O}, {"p", KEY_P}, {"q", KEY_Q}, {"r", KEY_R},
    {"s", KEY_S}, {"t", KEY_T}, {"u", KEY_U}, {"v", KEY_V}, {"w", KEY_W}, {"x", KEY_X},
    {"y", KEY_Y}, {"z", KEY_Z},
    {"0", KEY_ZERO}, {"1", KEY_ONE}, {"2", KEY_TWO}, {"3", KEY_THREE}, {"4", KEY_FOUR},
    {"5", KEY_FIVE}, {"6", KEY_SIX}, {"7", KEY_SEVEN}, {"8", KEY_EIGHT}, {"9", KEY_NINE},
    {"space", KEY_SPACE}, {"'", KEY_APOSTROPHE}, {",", KEY_COMMA}, {"-", KEY_MINUS},
    {".", KEY_PERIOD}, {"/", KEY_SLASH}, {";", KEY_SEMICOLON}, {"=", KEY_EQUAL},
    {"[", KEY_LEFT_BRACKET}, {"\\", KEY_BACKSLASH}, {"]", KEY_RIGHT_BRACKET}, {"`", KEY_GRAVE},
    {"escape", KEY_ESCAPE}, {"return", KEY_ENTER}, {"tab", KEY_TAB}, {"backspace", KEY_BACKSPACE},
    {"insert", KEY_INSERT}, {"delete", KEY_DELETE},
    {"right", KEY_RIGHT}, {"left", KEY_LEFT}, {"down", KEY_DOWN}, {"up", KEY_UP},
    {"pageup", KEY_PAGE_UP}, {"pagedown", KEY_PAGE_DOWN}, {"home", KEY_HOME}, {"end", KEY_END},
    {"capslock", KEY_CAPS_LOCK}, {"scrolllock", KEY_SCROLL_LOCK}, {"numlock", KEY_NUM_LOCK},
    {"printscreen", KEY_PRINT_SCREEN}, {"pause", KEY_PAUSE},
    {"f1", KEY_F1}, {"f2", KEY_F2}, {"f3", KEY_F3}, {"f4", KEY_F4}, {"f5", KEY_F5}, {"f6", KEY_F6},
    {"f7", KEY_F7}, {"f8", KEY_F8}, {"f9", KEY_F9}, {"f10", KEY_F10}, {"f11", KEY_F11}, {"f12", KEY_F12},
    {"lshift", KEY_LEFT_SHIFT}, {"lctrl", KEY_LEFT_CONTROL}, {"lalt", KEY_LEFT_ALT}, {"lgui", KEY_LEFT_SUPER},
    {"rshift", KEY_RIGHT_SHIFT}, {"rctrl", KEY_RIGHT_CONTROL}, {"ralt", KEY_RIGHT_ALT}, {"rgui", KEY_RIGHT_SUPER},
    {"menu", KEY_KB_MENU},
    {"kp0", KEY_KP_0}, {"kp1", KEY_KP_1}, {"kp2", KEY_KP_2}, {"kp3", KEY_KP_3}, {"kp4", KEY_KP_4},
    {"kp5", KEY_KP_5}, {"kp6", KEY_KP_6}, {"kp7", KEY_KP_7}, {"kp8", KEY_KP_8}, {"kp9", KEY_KP_9},
    {"kp.", KEY_KP_DECIMAL}, {"kp/", KEY_KP_DIVIDE}, {"kp*", KEY_KP_MULTIPLY}, {"kp-", KEY_KP_SUBTRACT},
    {"kp+", KEY_KP_ADD}, {"kpenter", KEY_KP_ENTER}, {"kp=", KEY_KP_EQUAL},
};

const std::unordered_map<std::string, int> &nameToKey()
{
    static const std::unordered_map<std::string, int> map = [] {
        std::unordered_map<std::string, int> m;
        for (const KeyName &k : KEYS)
        {
            m.emplace(k.name, k.key);
        }
        return m;
    }();
    return map;
}

const std::unordered_map<int, const char *> &keyToName()
{
    static const std::unordered_map<int, const char *> map = [] {
        std::unordered_map<int, const char *> m;
        for (const KeyName &k : KEYS)
        {
            m.emplace(k.key, k.name);
        }
        return m;
    }();
    return map;
}

bool g_keyRepeat = false;
bool g_textInput = true;

int checkKey(lua_State *L, int idx)
{
    const char *name = luaL_checkstring(L, idx);
    int key = keyFromName(name);
    if (key == KEY_NULL)
    {
        // Unknown names are not errors in Love2D either; they are never down.
        return KEY_NULL;
    }
    return key;
}

int l_isDown(lua_State *L)
{
    int n = lua_gettop(L);
    bool down = false;
    for (int i = 1; i <= n && !down; ++i)
    {
        int key = checkKey(L, i);
        down = key != KEY_NULL && window::isOpen() && IsKeyDown(key);
    }
    lua_pushboolean(L, down);
    return 1;
}

int l_setKeyRepeat(lua_State *L)
{
    g_keyRepeat = luax::checkboolean(L, 1);
    return 0;
}

int l_hasKeyRepeat(lua_State *L)
{
    lua_pushboolean(L, g_keyRepeat);
    return 1;
}

int l_setTextInput(lua_State *L)
{
    g_textInput = luax::checkboolean(L, 1);
    return 0;
}

int l_hasTextInput(lua_State *L)
{
    lua_pushboolean(L, g_textInput);
    return 1;
}

int l_hasScreenKeyboard(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

// Scancodes are reported as key names (raylib has no layout-independent codes).
int l_getKeyFromScancode(lua_State *L)
{
    lua_pushvalue(L, 1);
    return 1;
}

int l_getScancodeFromKey(lua_State *L)
{
    lua_pushvalue(L, 1);
    return 1;
}

int l_isModifierActive(lua_State *L)
{
    static const char *const names[] = {"numlock", "capslock", "scrolllock", "mode"};
    static const int values[] = {KEY_NUM_LOCK, KEY_CAPS_LOCK, KEY_SCROLL_LOCK, KEY_NULL};
    int key = luax::checkenum(L, 1, names, values, "modifier key");
    lua_pushboolean(L, key != KEY_NULL && window::isOpen() && IsKeyDown(key));
    return 1;
}

const luaL_Reg FUNCS[] = {
    {"isDown", l_isDown},
    {"isScancodeDown", l_isDown},
    {"setKeyRepeat", l_setKeyRepeat},
    {"hasKeyRepeat", l_hasKeyRepeat},
    {"setTextInput", l_setTextInput},
    {"hasTextInput", l_hasTextInput},
    {"hasScreenKeyboard", l_hasScreenKeyboard},
    {"getKeyFromScancode", l_getKeyFromScancode},
    {"getScancodeFromKey", l_getScancodeFromKey},
    {"isModifierActive", l_isModifierActive},
    {nullptr, nullptr},
};

} // namespace

int keyFromName(const char *name)
{
    auto it = nameToKey().find(name);
    return it == nameToKey().end() ? KEY_NULL : it->second;
}

const char *nameFromKey(int key)
{
    auto it = keyToName().find(key);
    return it == keyToName().end() ? nullptr : it->second;
}

bool keyRepeatEnabled()
{
    return g_keyRepeat;
}

bool textInputEnabled()
{
    return g_textInput;
}

const std::vector<int> &allKeys()
{
    static const std::vector<int> keys = [] {
        std::vector<int> v;
        for (const KeyName &k : KEYS)
        {
            v.push_back(k.key);
        }
        return v;
    }();
    return keys;
}

} // namespace keyboard

int open_keyboard(lua_State *L)
{
    luaL_newlib(L, keyboard::FUNCS);
    return 1;
}

} // namespace love
