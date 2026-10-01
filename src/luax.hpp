// luax.hpp - small helpers on top of the Lua C API used by every module.
#pragma once

#include <lua.hpp>

#include <cstdint>
#include <cstring>
#include <new>
#include <string>

namespace luax
{

// ---------------------------------------------------------------------------
// Types (userdata with a metatable)
// ---------------------------------------------------------------------------

namespace detail
{

inline const char *supertype(const char *name)
{
    struct Pair
    {
        const char *type;
        const char *parent;
    };
    static const Pair pairs[] = {
        {"Image", "Texture"},       {"Canvas", "Texture"},       {"Texture", "Drawable"},
        {"SpriteBatch", "Drawable"}, {"Text", "Drawable"},       {"FileData", "Data"},
        {"ImageData", "Data"},      {"CircleShape", "Shape"},    {"PolygonShape", "Shape"},
        {"EdgeShape", "Shape"},     {"ChainShape", "Shape"},     {"DistanceJoint", "Joint"},
        {"RevoluteJoint", "Joint"}, {"PrismaticJoint", "Joint"}, {"WheelJoint", "Joint"},
        {"WeldJoint", "Joint"},     {"MouseJoint", "Joint"},     {"MotorJoint", "Joint"},
        {"FrictionJoint", "Joint"}, {"RopeJoint", "Joint"},      {"PulleyJoint", "Joint"},
        {"GearJoint", "Joint"},
    };
    for (const Pair &p : pairs)
    {
        if (std::strcmp(p.type, name) == 0)
        {
            return p.parent;
        }
    }
    return nullptr;
}

inline const char *metaname(lua_State *L, int idx)
{
    if (!lua_getmetatable(L, idx))
    {
        return nullptr;
    }
    lua_getfield(L, -1, "__name");
    const char *name = lua_tostring(L, -1);
    lua_pop(L, 2);
    return name;
}

inline int object_type(lua_State *L)
{
    const char *name = metaname(L, 1);
    lua_pushstring(L, name != nullptr ? name : "Object");
    return 1;
}

inline int object_typeOf(lua_State *L)
{
    const char *wanted = luaL_checkstring(L, 2);
    const char *name = metaname(L, 1);
    bool match = std::strcmp(wanted, "Object") == 0;
    while (!match && name != nullptr)
    {
        match = std::strcmp(wanted, name) == 0;
        name = supertype(name);
    }
    lua_pushboolean(L, match);
    return 1;
}

inline int released_index(lua_State *L)
{
    return luaL_error(L, "Attempt to use an object that was released");
}

inline int object_release(lua_State *L)
{
    if (!lua_getmetatable(L, 1))
    {
        lua_pushboolean(L, 0);
        return 1;
    }
    lua_getfield(L, -1, "__gc");
    if (lua_isfunction(L, -1))
    {
        lua_pushvalue(L, 1);
        lua_call(L, 1, 0);
    }
    else
    {
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    if (luaL_newmetatable(L, "ReleasedObject"))
    {
        lua_pushcfunction(L, released_index);
        lua_setfield(L, -2, "__index");
    }
    lua_setmetatable(L, 1);
    lua_pushboolean(L, 1);
    return 1;
}

} // namespace detail

// Create (or fetch) the metatable `name`, install `methods` as __index and the
// optional `gc` finalizer. Every type also gets the Love2D Object methods
// type(), typeOf() and release(). Leaves the stack untouched.
inline void newtype(lua_State *L, const char *name, const luaL_Reg *methods, lua_CFunction gc = nullptr)
{
    if (luaL_newmetatable(L, name) == 0)
    {
        lua_pop(L, 1);
        return;
    }

    lua_newtable(L);
    if (methods != nullptr)
    {
        luaL_setfuncs(L, methods, 0);
    }
    lua_pushcfunction(L, detail::object_type);
    lua_setfield(L, -2, "type");
    lua_pushcfunction(L, detail::object_typeOf);
    lua_setfield(L, -2, "typeOf");
    lua_pushcfunction(L, detail::object_release);
    lua_setfield(L, -2, "release");
    lua_setfield(L, -2, "__index");

    if (gc != nullptr)
    {
        lua_pushcfunction(L, gc);
        lua_setfield(L, -2, "__gc");
    }
    for (const char *meta : {"__tostring", "__mul", "__eq"})
    {
        lua_getfield(L, -1, "__index");
        lua_getfield(L, -1, meta);
        lua_remove(L, -2);
        if (lua_isfunction(L, -1))
        {
            lua_setfield(L, -2, meta);
        }
        else
        {
            lua_pop(L, 1);
        }
    }

    lua_pushstring(L, name);
    lua_setfield(L, -2, "__name");
    lua_pop(L, 1);
}

// Allocate a userdata holding a T constructed with `args`, set its metatable
// and leave it on the stack.
template <class T, class... Args>
T *newobject(lua_State *L, const char *tname, Args &&...args)
{
    void *mem = lua_newuserdatauv(L, sizeof(T), 1);
    T *obj = new (mem) T(static_cast<Args &&>(args)...);
    luaL_setmetatable(L, tname);
    return obj;
}

template <class T>
T *checkobject(lua_State *L, int idx, const char *tname)
{
    return static_cast<T *>(luaL_checkudata(L, idx, tname));
}

template <class T>
T *testobject(lua_State *L, int idx, const char *tname)
{
    return static_cast<T *>(luaL_testudata(L, idx, tname));
}

// Generic __gc that runs the destructor of the userdata payload.
template <class T>
int gcobject(lua_State *L)
{
    T *obj = static_cast<T *>(lua_touserdata(L, 1));
    if (obj != nullptr)
    {
        obj->~T();
    }
    return 0;
}

// Returns the type name of the value at `idx` ("Image", "number", ...).
inline const char *typename_(lua_State *L, int idx)
{
    if (lua_getmetatable(L, idx))
    {
        lua_getfield(L, -1, "__name");
        const char *name = lua_tostring(L, -1);
        lua_pop(L, 2);
        if (name != nullptr)
        {
            return name;
        }
    }
    return luaL_typename(L, idx);
}

// ---------------------------------------------------------------------------
// Argument helpers
// ---------------------------------------------------------------------------

inline float checkfloat(lua_State *L, int idx)
{
    return static_cast<float>(luaL_checknumber(L, idx));
}

inline float optfloat(lua_State *L, int idx, float def)
{
    return static_cast<float>(luaL_optnumber(L, idx, def));
}

inline int optint(lua_State *L, int idx, int def)
{
    return static_cast<int>(luaL_optinteger(L, idx, def));
}

inline bool optboolean(lua_State *L, int idx, bool def)
{
    if (lua_isnoneornil(L, idx))
    {
        return def;
    }
    return lua_toboolean(L, idx) != 0;
}

inline bool checkboolean(lua_State *L, int idx)
{
    luaL_checktype(L, idx, LUA_TBOOLEAN);
    return lua_toboolean(L, idx) != 0;
}

// Maps a string argument onto an enum through parallel name/value arrays.
// Raises a Lua error listing the valid names when the string is unknown.
template <size_t N>
int checkenum(lua_State *L, int idx, const char *const (&names)[N], const int (&values)[N], const char *what)
{
    const char *str = luaL_checkstring(L, idx);
    for (size_t i = 0; i < N; ++i)
    {
        if (std::strcmp(str, names[i]) == 0)
        {
            return values[i];
        }
    }

    std::string valid;
    for (size_t i = 0; i < N; ++i)
    {
        valid += (i == 0 ? "'" : ", '");
        valid += names[i];
        valid += "'";
    }
    return luaL_error(L, "Invalid %s '%s' (expected one of %s)", what, str, valid.c_str());
}

template <size_t N>
const char *enumname(const char *const (&names)[N], const int (&values)[N], int value)
{
    for (size_t i = 0; i < N; ++i)
    {
        if (values[i] == value)
        {
            return names[i];
        }
    }
    return "unknown";
}

// Reads field `key` from the table at `idx` as a number, with a default.
inline double getnumberfield(lua_State *L, int idx, const char *key, double def)
{
    lua_getfield(L, idx, key);
    double value = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : def;
    lua_pop(L, 1);
    return value;
}

inline bool getboolfield(lua_State *L, int idx, const char *key, bool def)
{
    lua_getfield(L, idx, key);
    bool value = lua_isnil(L, -1) ? def : lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    return value;
}

inline std::string getstringfield(lua_State *L, int idx, const char *key, const char *def)
{
    lua_getfield(L, idx, key);
    const char *value = lua_isstring(L, -1) ? lua_tostring(L, -1) : def;
    std::string result = value != nullptr ? value : "";
    lua_pop(L, 1);
    return result;
}

// ---------------------------------------------------------------------------
// References
// ---------------------------------------------------------------------------

// Owning reference into the Lua registry.
class Ref
{
public:
    Ref() = default;
    Ref(const Ref &) = delete;
    Ref &operator=(const Ref &) = delete;
    Ref(Ref &&other) noexcept : ref_(other.ref_)
    {
        other.ref_ = LUA_NOREF;
    }
    Ref &operator=(Ref &&other) noexcept
    {
        ref_ = other.ref_;
        other.ref_ = LUA_NOREF;
        return *this;
    }

    bool valid() const
    {
        return ref_ != LUA_NOREF && ref_ != LUA_REFNIL;
    }

    // Takes the value at `idx` (does not pop it).
    void set(lua_State *L, int idx)
    {
        clear(L);
        lua_pushvalue(L, idx);
        ref_ = luaL_ref(L, LUA_REGISTRYINDEX);
    }

    void push(lua_State *L) const
    {
        if (valid())
        {
            lua_rawgeti(L, LUA_REGISTRYINDEX, ref_);
        }
        else
        {
            lua_pushnil(L);
        }
    }

    void clear(lua_State *L)
    {
        if (valid())
        {
            luaL_unref(L, LUA_REGISTRYINDEX, ref_);
        }
        ref_ = LUA_NOREF;
    }

private:
    int ref_ = LUA_NOREF;
};

// ---------------------------------------------------------------------------
// Object identity registry
//
// Native pointers that must map onto exactly one Lua userdata (physics bodies,
// fixtures, joints...) are tracked in a weak-valued registry table. This makes
// `a == b` work for handles obtained through different paths and lets a
// destroyed object invalidate every handle at once.
// ---------------------------------------------------------------------------

// Weak registry: entries disappear once Lua drops the object. Strong registry:
// entries live until unregisterobject(), for native objects whose lifetime is
// controlled from C++ (physics bodies, fixtures, joints).
inline void pushobjectregistry(lua_State *L, bool strong)
{
    const char *key = strong ? "loveray.objects.strong" : "loveray.objects.weak";
    lua_getfield(L, LUA_REGISTRYINDEX, key);
    if (lua_isnil(L, -1))
    {
        lua_pop(L, 1);
        lua_newtable(L);
        if (!strong)
        {
            lua_newtable(L);
            lua_pushstring(L, "v");
            lua_setfield(L, -2, "__mode");
            lua_setmetatable(L, -2);
        }
        lua_pushvalue(L, -1);
        lua_setfield(L, LUA_REGISTRYINDEX, key);
    }
}

// Pushes the userdata registered for `key`; returns false (pushing nothing)
// when none exists or it was collected.
inline bool pushregistered(lua_State *L, const void *key)
{
    for (bool strong : {true, false})
    {
        pushobjectregistry(L, strong);
        lua_rawgetp(L, -1, key);
        if (!lua_isnil(L, -1))
        {
            lua_remove(L, -2);
            return true;
        }
        lua_pop(L, 2);
    }
    return false;
}

// Registers the userdata at `idx` under `key`.
inline void registerobject(lua_State *L, const void *key, int idx, bool strong = false)
{
    idx = lua_absindex(L, idx);
    pushobjectregistry(L, strong);
    lua_pushvalue(L, idx);
    lua_rawsetp(L, -2, key);
    lua_pop(L, 1);
}

inline void unregisterobject(lua_State *L, const void *key)
{
    for (bool strong : {true, false})
    {
        pushobjectregistry(L, strong);
        lua_pushnil(L);
        lua_rawsetp(L, -2, key);
        lua_pop(L, 1);
    }
}

// ---------------------------------------------------------------------------
// Misc
// ---------------------------------------------------------------------------

// Message handler that appends a traceback to the error message.
inline int traceback(lua_State *L)
{
    const char *msg = lua_tostring(L, 1);
    if (msg == nullptr)
    {
        if (luaL_callmeta(L, 1, "__tostring") && lua_type(L, -1) == LUA_TSTRING)
        {
            return 1;
        }
        msg = lua_pushfstring(L, "(error object is a %s value)", luaL_typename(L, 1));
    }
    luaL_traceback(L, L, msg, 1);
    return 1;
}

} // namespace luax
