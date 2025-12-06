#pragma once
#include <lua.hpp>
#include <utility>
#include <type_traits>

// === Conversões ===
inline void luaPush(lua_State *L, int v) { lua_pushinteger(L, v); }
inline void luaPush(lua_State *L, float v) { lua_pushnumber(L, v); }
inline void luaPush(lua_State *L, double v) { lua_pushnumber(L, v); }
inline void luaPush(lua_State *L, bool v) { lua_pushboolean(L, v); }
inline void luaPush(lua_State *L, const char *v) { lua_pushstring(L, v); }

inline int luaGet(lua_State *L, int i, int *) { return luaL_checkinteger(L, i); }
inline float luaGet(lua_State *L, int i, float *) { return luaL_checknumber(L, i); }
inline double luaGet(lua_State *L, int i, double *) { return luaL_checknumber(L, i); }
inline bool luaGet(lua_State *L, int i, bool *) { return lua_toboolean(L, i); }
inline const char *luaGet(lua_State *L, int i, const char **) { return luaL_checkstring(L, i); }

template <typename T, typename... Args, size_t... I>
T *newT(lua_State *L, std::index_sequence<I...>)
{
    return new T(luaGet(L, I + 1, (Args *)0)...);
}

template <typename T, typename... Args, size_t... I>
void callVoid(lua_State *L, T *obj, void (T::*f)(Args...), std::index_sequence<I...>)
{
    (obj->*f)(luaGet(L, I + 2, (Args *)0)...);
}

template <typename T, typename R, typename... Args, size_t... I>
R callRet(lua_State *L, T *obj, R (T::*f)(Args...), std::index_sequence<I...>)
{
    return (obj->*f)(luaGet(L, I + 2, (Args *)0)...);
}

template <typename R, typename... Args, size_t... I>
R callFunc(lua_State *L, R (*f)(Args...), std::index_sequence<I...>)
{
    return f(luaGet(L, I + 1, (Args *)0)...);
}

template <typename... Args, size_t... I>
void callFuncVoid(lua_State *L, void (*f)(Args...), std::index_sequence<I...>)
{
    f(luaGet(L, I + 1, (Args *)0)...);
}

// === Push múltiplos ===
inline void luaPushArgs(lua_State *L) {}
template <typename T, typename... Args>
inline void luaPushArgs(lua_State *L, T first, Args... rest)
{
    luaPush(L, first);
    luaPushArgs(L, rest...);
}

// === Callback ===
class LuaRef
{
    lua_State *L;
    int ref;

public:
    LuaRef(lua_State *L, int ref) : L(L), ref(ref) {}
    ~LuaRef()
    {
        if (ref != LUA_NOREF)
            luaL_unref(L, LUA_REGISTRYINDEX, ref);
    }

    LuaRef(const LuaRef &) = delete;
    LuaRef &operator=(const LuaRef &) = delete;
    LuaRef(LuaRef &&o) noexcept : L(o.L), ref(o.ref) { o.ref = LUA_NOREF; }
    LuaRef &operator=(LuaRef &&o) noexcept
    {
        if (this != &o)
        {
            if (ref != LUA_NOREF)
                luaL_unref(L, LUA_REGISTRYINDEX, ref);
            L = o.L;
            ref = o.ref;
            o.ref = LUA_NOREF;
        }
        return *this;
    }

    template <typename... Args>
    void call(Args... args)
    {
        if (ref == LUA_NOREF)
            return;
        lua_rawgeti(L, LUA_REGISTRYINDEX, ref);
        luaPushArgs(L, args...);
        lua_pcall(L, sizeof...(args), 0, 0);
    }

    template <typename R, typename... Args>
    R callReturn(Args... args)
    {
        if (ref == LUA_NOREF)
            return R();
        lua_rawgeti(L, LUA_REGISTRYINDEX, ref);
        luaPushArgs(L, args...);
        if (lua_pcall(L, sizeof...(args), 1, 0) == LUA_OK)
        {
            R result = luaGet(L, -1, (R *)0);
            lua_pop(L, 1);
            return result;
        }
        return R();
    }
    bool isValid() { return ref != LUA_NOREF; }
};

// === Wrapper ===
class LuaBridge
{
public:
    lua_State *L;

    LuaBridge()
    {
        L = luaL_newstate();
        luaL_openlibs(L);
    }
    ~LuaBridge() { lua_close(L); }

    bool doFile(const char *p) { return luaL_dofile(L, p) == LUA_OK; }
    bool doString(const char *c) { return luaL_dostring(L, c) == LUA_OK; }
    const char *error()
    {
        const char *e = lua_tostring(L, -1);
        lua_pop(L, 1);
        return e;
    }

    // === FUNÇÃO (com retorno) ===
    template <typename R, typename... Args>
    typename std::enable_if<!std::is_void<R>::value, void>::type
    bindFunc(const char *n, R (*f)(Args...))
    {
        *(R(**)(Args...))lua_newuserdata(L, sizeof(f)) = f;
        lua_pushcclosure(L, [](lua_State *L)
                         {
            auto fn = *(R(**)(Args...))lua_touserdata(L, lua_upvalueindex(1));
            R result = callFunc<R, Args...>(L, fn, std::index_sequence_for<Args...>{});
            luaPush(L, result);
            return 1; }, 1);
        lua_setglobal(L, n);
    }

    // === FUNÇÃO (void) ===
    template <typename R, typename... Args>
    typename std::enable_if<std::is_void<R>::value, void>::type
    bindFunc(const char *n, R (*f)(Args...))
    {
        *(R(**)(Args...))lua_newuserdata(L, sizeof(f)) = f;
        lua_pushcclosure(L, [](lua_State *L)
                         {
            auto fn = *(void(**)(Args...))lua_touserdata(L, lua_upvalueindex(1));
            callFuncVoid<Args...>(L, fn, std::index_sequence_for<Args...>{});
            return 0; }, 1);
        lua_setglobal(L, n);
    }

    // === CONSTANTE ===
    void setInt(const char *n, int v)
    {
        lua_pushinteger(L, v);
        lua_setglobal(L, n);
    }
    void setFloat(const char *n, float v)
    {
        lua_pushnumber(L, v);
        lua_setglobal(L, n);
    }
    void setStr(const char *n, const char *v)
    {
        lua_pushstring(L, v);
        lua_setglobal(L, n);
    }

    // === ENUM ===
    template <typename E>
    void setEnum(const char *name, int count, const char **keys, E *vals)
    {
        lua_newtable(L);
        for (int i = 0; i < count; i++)
        {
            lua_pushinteger(L, (int)vals[i]);
            lua_setfield(L, -2, keys[i]);
        }
        lua_setglobal(L, name);
    }

    // === NAMESPACE ===
    void beginNamespace(const char *ns)
    {
        lua_getglobal(L, ns);
        if (!lua_istable(L, -1))
        {
            lua_pop(L, 1);
            lua_newtable(L);
            lua_pushvalue(L, -1);
            lua_setglobal(L, ns);
        }
    }
    void endNamespace() { lua_pop(L, 1); }

    // === FUNÇÃO NO NAMESPACE (com retorno) ===
    template <typename R, typename... Args>
    typename std::enable_if<!std::is_void<R>::value, void>::type
    nsFunc(const char *n, R (*f)(Args...))
    {
        *(R(**)(Args...))lua_newuserdata(L, sizeof(f)) = f;
        lua_pushcclosure(L, [](lua_State *L)
                         {
            auto fn = *(R(**)(Args...))lua_touserdata(L, lua_upvalueindex(1));
            R result = callFunc<R, Args...>(L, fn, std::index_sequence_for<Args...>{});
            luaPush(L, result);
            return 1; }, 1);
        lua_setfield(L, -2, n);
    }

    // === FUNÇÃO NO NAMESPACE (void) ===
    template <typename R, typename... Args>
    typename std::enable_if<std::is_void<R>::value, void>::type
    nsFunc(const char *n, R (*f)(Args...))
    {
        *(void (**)(Args...))lua_newuserdata(L, sizeof(f)) = f;
        lua_pushcclosure(L, [](lua_State *L)
                         {
            auto fn = *(void(**)(Args...))lua_touserdata(L, lua_upvalueindex(1));
            callFuncVoid<Args...>(L, fn, std::index_sequence_for<Args...>{});
            return 0; }, 1);
        lua_setfield(L, -2, n);
    }

    void nsInt(const char *n, int v)
    {
        lua_pushinteger(L, v);
        lua_setfield(L, -2, n);
    }
    void nsFloat(const char *n, float v)
    {
        lua_pushnumber(L, v);
        lua_setfield(L, -2, n);
    }

    template <typename E>
    void nsEnum(const char *name, int count, const char **keys, E *vals)
    {
        lua_newtable(L);
        for (int i = 0; i < count; i++)
        {
            lua_pushinteger(L, (int)vals[i]);
            lua_setfield(L, -2, keys[i]);
        }
        lua_setfield(L, -2, name);
    }

    template <typename T>
    void bindDestroy(const char *n)
    {
        luaL_getmetatable(L, n);
        lua_pushstring(L, n);
        lua_pushcclosure(L, [](lua_State *L)
                         {
            T** p = (T**)luaL_checkudata(L, 1, lua_tostring(L, lua_upvalueindex(1)));
            if (p && *p) {
                delete *p;
                *p = nullptr;
            }
            return 0; }, 1);
        lua_setfield(L, -2, "destroy");
        lua_pop(L, 1);
    }

    // === CLASSE (GC automático) ===
    template <typename T>
    void bindClass(const char *n)
    {
        luaL_newmetatable(L, n);

        // __gc
        lua_pushstring(L, n);
        lua_pushcclosure(L, [](lua_State *L)
                         {
            const char* className = lua_tostring(L, lua_upvalueindex(1));
            T** p = (T**)luaL_checkudata(L, 1, className);
            if (p && *p) {
                printf("[GC] Deleting %s at %p\n", className, (void*)*p);
                delete *p;
                *p = nullptr;
            }
            return 0; }, 1);
        lua_setfield(L, -2, "__gc");

        // __index = self
        lua_pushvalue(L, -1);
        lua_setfield(L, -2, "__index");

        // Tabela global
        lua_newtable(L);
        lua_pushvalue(L, -1);
        lua_setglobal(L, n);
        lua_pop(L, 2);
    }

    template <typename T, typename... Args>
    void bindNew(const char *n)
    {
        lua_getglobal(L, n);
        lua_pushstring(L, n);
        lua_pushcclosure(L, [](lua_State *L)
                         {
            const char* className = lua_tostring(L, lua_upvalueindex(1));
            
            // Cria userdata
            T** p = (T**)lua_newuserdata(L, sizeof(T*));
            
            // Associa metatable
            luaL_getmetatable(L, className);
            lua_setmetatable(L, -2);
            
           
            *p = newT<T, Args...>(L, std::index_sequence_for<Args...>{});
            printf("[NEW] Created %s at %p\n", className, (void*)*p);
            
            return 1; }, 1);
        lua_setfield(L, -2, "new");
        lua_pop(L, 1);
    }

    template <typename T, typename... Args>
    void bindMethod(const char *n, const char *m, void (T::*f)(Args...))
    {
        luaL_getmetatable(L, n);
        lua_pushstring(L, n);
        *(void(T::**)(Args...))lua_newuserdata(L, sizeof(f)) = f;
        lua_pushcclosure(L, [](lua_State *L)
                         {
            auto fn = *(void(T::**)(Args...))lua_touserdata(L, lua_upvalueindex(2));
            T* o = *(T**)luaL_checkudata(L, 1, lua_tostring(L, lua_upvalueindex(1)));
            callMethod(L, o, fn, std::index_sequence_for<Args...>{});
            return 0; }, 2);
        lua_setfield(L, -2, m);
        lua_pop(L, 1);
    }

    template <typename T, typename R, typename... Args>
    void bindMethod(const char *n, const char *m, R (T::*f)(Args...))
    {
        luaL_getmetatable(L, n);
        lua_pushstring(L, n);
        *(R(T::**)(Args...))lua_newuserdata(L, sizeof(f)) = f;
        lua_pushcclosure(L, [](lua_State *L)
                         {
            auto fn = *(R(T::**)(Args...))lua_touserdata(L, lua_upvalueindex(2));
            T* o = *(T**)luaL_checkudata(L, 1, lua_tostring(L, lua_upvalueindex(1)));
            R result = callMethodRet<T, R, Args...>(L, o, fn, std::index_sequence_for<Args...>{});
            luaPush(L, result);
            return 1; }, 2);
        lua_setfield(L, -2, m);
        lua_pop(L, 1);
    }

    // === CALLBACK ===
    LuaRef getFunc(const char *name)
    {
        lua_getglobal(L, name);
        if (lua_isfunction(L, -1))
        {
            int ref = luaL_ref(L, LUA_REGISTRYINDEX);
            return LuaRef(L, ref);
        }
        lua_pop(L, 1);
        return LuaRef(L, LUA_NOREF);
    }

    template <typename... Args>
    void callFunc(const char *name, Args... args)
    {
        lua_getglobal(L, name);
        if (lua_isfunction(L, -1))
        {
            luaPushArgs(L, args...);
            lua_pcall(L, sizeof...(args), 0, 0);
        }
        else
        {
            lua_pop(L, 1);
        }
    }

private:
    template <typename T, typename... Args, size_t... I>
    static T *newObj(lua_State *L, std::index_sequence<I...>)
    {
        return new T(luaGet(L, I + 2, (Args *)0)...);
    }

    template <typename T, typename... Args, size_t... I>
    static void callMethod(lua_State *L, T *o, void (T::*f)(Args...), std::index_sequence<I...>)
    {
        (o->*f)(luaGet(L, I + 2, (Args *)0)...);
    }

    template <typename T, typename R, typename... Args, size_t... I>
    static R callMethodRet(lua_State *L, T *o, R (T::*f)(Args...), std::index_sequence<I...>)
    {
        return (o->*f)(luaGet(L, I + 2, (Args *)0)...);
    }

    template <typename R, typename... Args, size_t... I>
    static R callFunc(lua_State *L, R (*f)(Args...), std::index_sequence<I...>)
    {
        return f(luaGet(L, I + 1, (Args *)0)...);
    }

    template <typename... Args, size_t... I>
    static void callFuncVoid(lua_State *L, void (*f)(Args...), std::index_sequence<I...>)
    {
        f(luaGet(L, I + 1, (Args *)0)...);
    }
};
