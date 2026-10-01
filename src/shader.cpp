// shader.cpp - love.graphics.newShader: Love's GLSL dialect on top of raylib shaders.
#include "graphics_internal.hpp"

#include <cstdarg>
#include <cstdio>
#include <map>
#include <regex>

namespace love
{
namespace graphics
{

namespace
{

enum class UniformKind
{
    Float,
    Int,
    Bool,
    Sampler,
    Matrix4,
    Unsupported
};

struct UniformInfo
{
    UniformKind kind = UniformKind::Unsupported;
    int components = 1;
    int count = 1;
    int location = -1;
    std::string glslType;
};

struct SamplerBinding
{
    int location = -1;
    Texture2D texture = {};
    luax::Ref ref;
};

} // namespace

struct ShaderObj
{
    ::Shader shader = {};
    std::map<std::string, UniformInfo> uniforms;
    std::map<std::string, SamplerBinding> samplers;
    int screenSizeLocation = -1;
    int targetWidth = 0;
    int targetHeight = 0;
    lua_State *state = nullptr;

    ~ShaderObj()
    {
        if (shader.id != 0 && shader.id != rlGetShaderIdDefault())
        {
            UnloadShader(shader);
        }
        shader = {};
    }
};

namespace
{

ShaderObj *g_active = nullptr;
std::string g_compileLog;

void captureLog(int level, const char *text, va_list args)
{
    char buffer[4096];
    std::vsnprintf(buffer, sizeof(buffer), text, args);
    if (level >= LOG_WARNING)
    {
        g_compileLog += buffer;
        g_compileLog += '\n';
    }
}

std::string stripComments(const std::string &code)
{
    std::string out;
    out.reserve(code.size());
    for (size_t i = 0; i < code.size(); ++i)
    {
        if (code.compare(i, 2, "//") == 0)
        {
            while (i < code.size() && code[i] != '\n')
            {
                ++i;
            }
            out.push_back('\n');
        }
        else if (code.compare(i, 2, "/*") == 0)
        {
            size_t end = code.find("*/", i + 2);
            for (size_t k = i; k < (end == std::string::npos ? code.size() : end + 2); ++k)
            {
                if (code[k] == '\n')
                {
                    out.push_back('\n');
                }
            }
            if (end == std::string::npos)
            {
                break;
            }
            i = end + 1;
        }
        else
        {
            out.push_back(code[i]);
        }
    }
    return out;
}

bool hasFunction(const std::string &code, const char *name)
{
    std::regex pattern(std::string("\\bvec4\\s+") + name + "\\s*\\(");
    return std::regex_search(stripComments(code), pattern);
}

std::string translateKeywords(const std::string &code)
{
    return std::regex_replace(code, std::regex("\\bextern\\b"), "uniform");
}

#if defined(PLATFORM_WEB)

const char *VERTEX_PRELUDE = R"(#version 100
precision highp float;
attribute vec3 vertexPosition;
attribute vec2 vertexTexCoord;
attribute vec4 vertexColor;
uniform mat4 mvp;
varying vec2 fragTexCoord;
varying vec4 fragColor;
#define number float
#define Image sampler2D
#define TransformProjectionMatrix mvp
#define VaryingTexCoord vec4(vertexTexCoord, 0.0, 1.0)
#define VaryingColor vertexColor
)";

const char *VERTEX_MAIN = R"(
void main()
{
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    gl_Position = position(mvp, vec4(vertexPosition, 1.0));
}
)";

const char *FRAGMENT_PRELUDE = R"(#version 100
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
varying vec2 fragTexCoord;
varying vec4 fragColor;
uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec4 love_ScreenSize;
#define number float
#define Image sampler2D
#define Texel(tex, uv) texture2D(tex, uv)
#define VaryingTexCoord vec4(fragTexCoord, 0.0, 1.0)
#define VaryingColor fragColor
)";

const char *FRAGMENT_MAIN = R"(
void main()
{
    vec2 screenCoords = vec2(gl_FragCoord.x, love_ScreenSize.y - gl_FragCoord.y);
    gl_FragColor = effect(fragColor, texture0, fragTexCoord, screenCoords);
}
)";

#else

const char *VERTEX_PRELUDE = R"(#version 330
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec4 vertexColor;
uniform mat4 mvp;
out vec2 fragTexCoord;
out vec4 fragColor;
#define varying out
#define number float
#define Image sampler2D
#define TransformProjectionMatrix mvp
#define VaryingTexCoord vec4(vertexTexCoord, 0.0, 1.0)
#define VaryingColor vertexColor
)";

const char *VERTEX_MAIN = R"(
void main()
{
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    gl_Position = position(mvp, vec4(vertexPosition, 1.0));
}
)";

const char *FRAGMENT_PRELUDE = R"(#version 330
in vec2 fragTexCoord;
in vec4 fragColor;
uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec4 love_ScreenSize;
out vec4 love_FragColor;
#define varying in
#define number float
#define Image sampler2D
#define Texel(tex, uv) texture(tex, uv)
#define texture2D texture
#define VaryingTexCoord vec4(fragTexCoord, 0.0, 1.0)
#define VaryingColor fragColor
)";

const char *FRAGMENT_MAIN = R"(
void main()
{
    vec2 screenCoords = vec2(gl_FragCoord.x, love_ScreenSize.y - gl_FragCoord.y);
    love_FragColor = effect(fragColor, texture0, fragTexCoord, screenCoords);
}
)";

#endif

const char *DEFAULT_POSITION = "vec4 position(mat4 transform_projection, vec4 vertex_position)\n"
                               "{\n    return transform_projection * vertex_position;\n}\n";
const char *DEFAULT_EFFECT = "vec4 effect(vec4 color, Image tex, vec2 texture_coords, vec2 screen_coords)\n"
                             "{\n    return Texel(tex, texture_coords) * color;\n}\n";

std::string buildVertex(const std::string &userCode)
{
    std::string source = VERTEX_PRELUDE;
    if (userCode.empty())
    {
        source += DEFAULT_POSITION;
    }
    else
    {
        source += "#line 1\n";
        source += translateKeywords(userCode);
    }
    source += VERTEX_MAIN;
    return source;
}

std::string buildFragment(const std::string &userCode)
{
    std::string source = FRAGMENT_PRELUDE;
    if (userCode.empty())
    {
        source += DEFAULT_EFFECT;
    }
    else
    {
        source += "#line 1\n";
        source += translateKeywords(userCode);
    }
    source += FRAGMENT_MAIN;
    return source;
}

struct TypeInfo
{
    const char *name;
    UniformKind kind;
    int components;
};

const TypeInfo TYPES[] = {
    {"float", UniformKind::Float, 1},     {"number", UniformKind::Float, 1}, {"vec2", UniformKind::Float, 2},
    {"vec3", UniformKind::Float, 3},      {"vec4", UniformKind::Float, 4},   {"int", UniformKind::Int, 1},
    {"ivec2", UniformKind::Int, 2},       {"ivec3", UniformKind::Int, 3},    {"ivec4", UniformKind::Int, 4},
    {"bool", UniformKind::Bool, 1},       {"mat4", UniformKind::Matrix4, 16}, {"mat2", UniformKind::Unsupported, 4},
    {"mat3", UniformKind::Unsupported, 9}, {"sampler2D", UniformKind::Sampler, 1}, {"Image", UniformKind::Sampler, 1},
};

void parseUniforms(const std::string &code, std::map<std::string, UniformInfo> &out)
{
    std::string clean = stripComments(code);
    std::regex pattern("\\b(?:extern|uniform)\\s+(?:(?:highp|mediump|lowp)\\s+)?(\\w+)\\s+(\\w+)\\s*(?:\\[\\s*(\\d+)\\s*\\])?");
    for (auto it = std::sregex_iterator(clean.begin(), clean.end(), pattern); it != std::sregex_iterator(); ++it)
    {
        std::string type = (*it)[1];
        std::string name = (*it)[2];
        UniformInfo info;
        info.glslType = type;
        info.count = (*it)[3].matched ? std::atoi((*it)[3].str().c_str()) : 1;
        for (const TypeInfo &t : TYPES)
        {
            if (type == t.name)
            {
                info.kind = t.kind;
                info.components = t.components;
                break;
            }
        }
        out[name] = info;
    }
}

bool readSource(lua_State *L, int idx, std::string &out)
{
    size_t len = 0;
    const char *text = luaL_checklstring(L, idx, &len);
    std::string code(text, len);
    bool looksLikeCode = code.find('\n') != std::string::npos || code.find('{') != std::string::npos ||
                         code.find(';') != std::string::npos;
    if (looksLikeCode)
    {
        out = code;
        return true;
    }
    std::vector<unsigned char> data;
    if (!filesystem::readFile(code, data))
    {
        luaL_error(L, "Could not open file %s. Does not exist.", code.c_str());
    }
    out.assign(data.begin(), data.end());
    return true;
}

// Compiles and links; on failure fills `error` and returns false.
bool compile(const std::string &vertex, const std::string &fragment, ::Shader &shader, std::string &error)
{
    window::ensureOpen();
    g_compileLog.clear();
    SetTraceLogCallback(captureLog);
    shader = LoadShaderFromMemory(vertex.c_str(), fragment.c_str());
    SetTraceLogCallback(nullptr);
    if (shader.id == 0 || shader.id == rlGetShaderIdDefault())
    {
        error = g_compileLog;
        if (error.empty())
        {
            error = "Unknown shader compilation error";
        }
        shader = {};
        return false;
    }
    return true;
}

// Splits the user's arguments into pixel and vertex code like Love2D does.
void readShaderArgs(lua_State *L, std::string &pixel, std::string &vertex)
{
    std::string first;
    readSource(L, 1, first);
    if (lua_isnoneornil(L, 2))
    {
        bool isPixel = hasFunction(first, "effect");
        bool isVertex = hasFunction(first, "position");
        if (!isPixel && !isVertex)
        {
            luaL_error(L, "Could not parse shader code (missing 'effect' or 'position' function?)");
        }
        pixel = isPixel ? first : "";
        vertex = isVertex ? first : "";
        return;
    }
    readSource(L, 2, vertex);
    pixel = first;
    if (!hasFunction(pixel, "effect"))
    {
        luaL_error(L, "Could not parse pixel shader code (missing 'effect' function?)");
    }
    if (!hasFunction(vertex, "position"))
    {
        luaL_error(L, "Could not parse vertex shader code (missing 'position' function?)");
    }
}

int l_newShader(lua_State *L)
{
    std::string pixel, vertex;
    readShaderArgs(L, pixel, vertex);

    ::Shader compiled = {};
    std::string error;
    if (!compile(buildVertex(vertex), buildFragment(pixel), compiled, error))
    {
        return luaL_error(L, "Cannot compile shader:\n%s", error.c_str());
    }

    ShaderObj *obj = luax::newobject<ShaderObj>(L, SHADER_TYPE);
    obj->shader = compiled;
    obj->state = L;
    parseUniforms(pixel, obj->uniforms);
    parseUniforms(vertex, obj->uniforms);
    for (auto &entry : obj->uniforms)
    {
        entry.second.location = GetShaderLocation(obj->shader, entry.first.c_str());
    }
    obj->screenSizeLocation = GetShaderLocation(obj->shader, "love_ScreenSize");
    return 1;
}

int l_validateShader(lua_State *L)
{
    // validateShader(gles, pixelcode [, vertexcode])
    lua_remove(L, 1);
    std::string pixel, vertex;
    readShaderArgs(L, pixel, vertex);
    ::Shader compiled = {};
    std::string error;
    bool ok = compile(buildVertex(vertex), buildFragment(pixel), compiled, error);
    if (ok)
    {
        UnloadShader(compiled);
        lua_pushboolean(L, 1);
        return 1;
    }
    lua_pushboolean(L, 0);
    lua_pushstring(L, error.c_str());
    return 2;
}

// ---------------------------------------------------------------------------
// Shader:send
// ---------------------------------------------------------------------------

void flattenNumbers(lua_State *L, int idx, std::vector<float> &out)
{
    idx = lua_absindex(L, idx);
    switch (lua_type(L, idx))
    {
    case LUA_TNUMBER:
        out.push_back(static_cast<float>(lua_tonumber(L, idx)));
        break;
    case LUA_TBOOLEAN:
        out.push_back(lua_toboolean(L, idx) ? 1.0f : 0.0f);
        break;
    case LUA_TTABLE:
    {
        lua_Integer n = luaL_len(L, idx);
        for (lua_Integer i = 1; i <= n; ++i)
        {
            lua_rawgeti(L, idx, i);
            flattenNumbers(L, -1, out);
            lua_pop(L, 1);
        }
        break;
    }
    default:
        luaL_error(L, "Invalid shader uniform value of type %s", luax::typename_(L, idx));
    }
}

void flushBatch()
{
    if (graphics::frameActive())
    {
        rlDrawRenderBatchActive();
    }
}

int shader_send(lua_State *L)
{
    ShaderObj *obj = checkShader(L, 1);
    const char *name = luaL_checkstring(L, 2);
    auto found = obj->uniforms.find(name);
    if (found == obj->uniforms.end() || found->second.location < 0)
    {
        return luaL_error(L, "Shader uniform '%s' does not exist.\nA common error is to define but not use the variable.", name);
    }
    UniformInfo &info = found->second;
    if (lua_gettop(L) < 3)
    {
        return luaL_error(L, "No value to send to uniform '%s'", name);
    }

    switch (info.kind)
    {
    case UniformKind::Sampler:
    {
        DrawSource src = checkDrawSource(L, 3);
        SamplerBinding &binding = obj->samplers[name];
        binding.location = info.location;
        binding.texture = src.texture;
        binding.ref.set(L, 3);
        if (g_active == obj)
        {
            flushBatch();
            rlSetUniformSampler(binding.location, binding.texture.id);
        }
        return 0;
    }
    case UniformKind::Matrix4:
    {
        std::vector<float> values;
        int first = 3;
        bool columnMajor = false;
        if (lua_type(L, 3) == LUA_TSTRING)
        {
            columnMajor = std::string(lua_tostring(L, 3)) == "column";
            first = 4;
        }
        for (int i = first; i <= lua_gettop(L); ++i)
        {
            flattenNumbers(L, i, values);
        }
        if (static_cast<int>(values.size()) != 16 * info.count)
        {
            return luaL_error(L, "Value count mismatch for uniform '%s': expected %d numbers, got %d", name,
                              16 * info.count, static_cast<int>(values.size()));
        }
        flushBatch();
        rlEnableShader(obj->shader.id);
        for (int m = 0; m < info.count; ++m)
        {
            Matrix matrix;
            float *dst = reinterpret_cast<float *>(&matrix);
            for (int i = 0; i < 16; ++i)
            {
                int row = columnMajor ? i % 4 : i / 4;
                int col = columnMajor ? i / 4 : i % 4;
                dst[row * 4 + col] = values[static_cast<size_t>(m * 16 + i)];
            }
            rlSetUniformMatrix(info.location + m, matrix);
        }
        rlDisableShader();
        return 0;
    }
    case UniformKind::Unsupported:
        return luaL_error(L, "Uniform '%s' has type %s which is not supported by LoveRay", name, info.glslType.c_str());
    default:
        break;
    }

    std::vector<float> values;
    for (int i = 3; i <= lua_gettop(L); ++i)
    {
        flattenNumbers(L, i, values);
    }
    int expected = info.components * info.count;
    if (static_cast<int>(values.size()) != expected)
    {
        return luaL_error(L, "Value count mismatch for uniform '%s': expected %d numbers, got %d", name, expected,
                          static_cast<int>(values.size()));
    }
    flushBatch();
    if (info.kind == UniformKind::Float)
    {
        static const int types[] = {SHADER_UNIFORM_FLOAT, SHADER_UNIFORM_VEC2, SHADER_UNIFORM_VEC3, SHADER_UNIFORM_VEC4};
        SetShaderValueV(obj->shader, info.location, values.data(), types[info.components - 1], info.count);
    }
    else
    {
        std::vector<int> ints(values.begin(), values.end());
        static const int types[] = {SHADER_UNIFORM_INT, SHADER_UNIFORM_IVEC2, SHADER_UNIFORM_IVEC3, SHADER_UNIFORM_IVEC4};
        SetShaderValueV(obj->shader, info.location, ints.data(), types[info.components - 1], info.count);
    }
    return 0;
}

int shader_sendColor(lua_State *L)
{
    // Colors are sent as-is, there is no gamma-correct pipeline.
    return shader_send(L);
}

int shader_hasUniform(lua_State *L)
{
    ShaderObj *obj = checkShader(L, 1);
    auto found = obj->uniforms.find(luaL_checkstring(L, 2));
    lua_pushboolean(L, found != obj->uniforms.end() && found->second.location >= 0);
    return 1;
}

int shader_getWarnings(lua_State *L)
{
    checkShader(L, 1);
    lua_pushstring(L, "");
    return 1;
}

int shader_tostring(lua_State *L)
{
    checkShader(L, 1);
    lua_pushstring(L, "Shader");
    return 1;
}

int shader_gc(lua_State *L)
{
    ShaderObj *obj = static_cast<ShaderObj *>(lua_touserdata(L, 1));
    if (g_active == obj)
    {
        g_active = nullptr;
    }
    for (auto &entry : obj->samplers)
    {
        entry.second.ref.clear(L);
    }
    obj->~ShaderObj();
    return 0;
}

const luaL_Reg SHADER_METHODS[] = {
    {"send", shader_send},
    {"sendColor", shader_sendColor},
    {"hasUniform", shader_hasUniform},
    {"getWarnings", shader_getWarnings},
    {"__tostring", shader_tostring},
    {nullptr, nullptr},
};

} // namespace

ShaderObj *checkShader(lua_State *L, int idx)
{
    return luax::checkobject<ShaderObj>(L, idx, SHADER_TYPE);
}

void shaderSetTargetSize(ShaderObj *shader, int width, int height)
{
    shader->targetWidth = width;
    shader->targetHeight = height;
    if (shader->screenSizeLocation >= 0)
    {
        flushBatch();
        float size[4] = {static_cast<float>(width), static_cast<float>(height), 1.0f, 1.0f};
        SetShaderValue(shader->shader, shader->screenSizeLocation, size, SHADER_UNIFORM_VEC4);
    }
}

void shaderActivate(ShaderObj *shader, int targetWidth, int targetHeight)
{
    BeginShaderMode(shader->shader);
    g_active = shader;
    shaderSetTargetSize(shader, targetWidth, targetHeight);
    shaderBeforeDraw(shader);
}

void shaderDeactivate()
{
    EndShaderMode();
    g_active = nullptr;
}

void shaderBeforeDraw(ShaderObj *shader)
{
    for (auto &entry : shader->samplers)
    {
        rlSetUniformSampler(entry.second.location, entry.second.texture.id);
    }
}

void registerShaderType(lua_State *L)
{
    luax::newtype(L, SHADER_TYPE, SHADER_METHODS, shader_gc);
}

const luaL_Reg SHADER_FUNCS[] = {
    {"newShader", l_newShader},
    {"validateShader", l_validateShader},
    {nullptr, nullptr},
};

} // namespace graphics
} // namespace love
