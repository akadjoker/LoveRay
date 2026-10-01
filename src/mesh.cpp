// mesh.cpp - love.graphics.newMesh
//
// Vertices carry a position, a texture coordinate and a color, drawn through
// rlgl so they work with shaders, canvases and the transform stack. Custom
// vertex attributes are not available because rlgl batches fixed vertex data.
#include "graphics_internal.hpp"

#include <algorithm>

namespace love
{
namespace graphics
{

namespace
{

enum class DrawMode
{
    Fan,
    Strip,
    Triangles,
    Points
};

const char *const MODE_NAMES[] = {"fan", "strip", "triangles", "points"};
const int MODE_VALUES[] = {static_cast<int>(DrawMode::Fan), static_cast<int>(DrawMode::Strip),
                           static_cast<int>(DrawMode::Triangles), static_cast<int>(DrawMode::Points)};

const char *const USAGE_NAMES[] = {"stream", "dynamic", "static"};
const int USAGE_VALUES[] = {0, 1, 2};

struct Vertex
{
    float x = 0, y = 0;
    float u = 0, v = 0;
    float r = 1, g = 1, b = 1, a = 1;
};

} // namespace

struct MeshObj
{
    std::vector<Vertex> vertices;
    std::vector<unsigned int> map;
    luax::Ref texture;
    DrawMode mode = DrawMode::Fan;
    int usage = 1;
    int rangeStart = -1;
    int rangeCount = -1;
};

namespace
{

MeshObj *check(lua_State *L, int idx)
{
    return luax::checkobject<MeshObj>(L, idx, MESH_TYPE);
}

int mesh_gc(lua_State *L)
{
    MeshObj *mesh = static_cast<MeshObj *>(lua_touserdata(L, 1));
    mesh->texture.clear(L);
    mesh->~MeshObj();
    return 0;
}

float optNumber(lua_State *L, int idx, float fallback)
{
    return lua_isnumber(L, idx) ? static_cast<float>(lua_tonumber(L, idx)) : fallback;
}

// Reads a vertex from {x, y, u, v, r, g, b, a} at table index `idx`.
Vertex readVertexTable(lua_State *L, int idx)
{
    luaL_checktype(L, idx, LUA_TTABLE);
    float v[8] = {0, 0, 0, 0, 1, 1, 1, 1};
    for (int i = 0; i < 8; ++i)
    {
        lua_rawgeti(L, idx, i + 1);
        v[i] = optNumber(L, -1, v[i]);
        lua_pop(L, 1);
    }
    return {v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]};
}

// Reads a vertex from loose numbers starting at `idx`.
Vertex readVertexArgs(lua_State *L, int idx)
{
    Vertex v;
    v.x = luax::checkfloat(L, idx);
    v.y = luax::checkfloat(L, idx + 1);
    v.u = optNumber(L, idx + 2, 0.0f);
    v.v = optNumber(L, idx + 3, 0.0f);
    v.r = optNumber(L, idx + 4, 1.0f);
    v.g = optNumber(L, idx + 5, 1.0f);
    v.b = optNumber(L, idx + 6, 1.0f);
    v.a = optNumber(L, idx + 7, 1.0f);
    return v;
}

int pushVertex(lua_State *L, const Vertex &v)
{
    lua_pushnumber(L, v.x);
    lua_pushnumber(L, v.y);
    lua_pushnumber(L, v.u);
    lua_pushnumber(L, v.v);
    lua_pushnumber(L, v.r);
    lua_pushnumber(L, v.g);
    lua_pushnumber(L, v.b);
    lua_pushnumber(L, v.a);
    return 8;
}

// A format is {{name, datatype, components}, ...}. Only the standard
// attributes are drawable, so anything else is rejected with a clear error.
bool looksLikeFormat(lua_State *L, int idx)
{
    if (!lua_istable(L, idx))
    {
        return false;
    }
    lua_rawgeti(L, idx, 1);
    bool result = false;
    if (lua_istable(L, -1))
    {
        lua_rawgeti(L, -1, 1);
        result = lua_type(L, -1) == LUA_TSTRING;
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return result;
}

void validateFormat(lua_State *L, int idx)
{
    lua_Integer n = luaL_len(L, idx);
    for (lua_Integer i = 1; i <= n; ++i)
    {
        lua_rawgeti(L, idx, i);
        lua_rawgeti(L, -1, 1);
        std::string name = luaL_checkstring(L, -1);
        lua_pop(L, 1);
        lua_rawgeti(L, -1, 2);
        std::string type = luaL_checkstring(L, -1);
        lua_pop(L, 1);
        lua_rawgeti(L, -1, 3);
        lua_Integer components = luaL_checkinteger(L, -1);
        lua_pop(L, 2);
        bool ok = (name == "VertexPosition" && type == "float" && components == 2) ||
                  (name == "VertexTexCoord" && type == "float" && components == 2) ||
                  (name == "VertexColor" && type == "byte" && components == 4);
        if (!ok)
        {
            luaL_error(L, "Vertex attribute '%s' (%s x%d) is not supported by LoveRay: only VertexPosition (float x2), "
                          "VertexTexCoord (float x2) and VertexColor (byte x4)",
                       name.c_str(), type.c_str(), static_cast<int>(components));
        }
    }
}

// newMesh([format,] vertices | count [, mode [, usage]])
int l_newMesh(lua_State *L)
{
    int idx = 1;
    if (looksLikeFormat(L, idx))
    {
        validateFormat(L, idx);
        ++idx;
    }

    std::vector<Vertex> vertices;
    if (lua_istable(L, idx))
    {
        lua_Integer n = luaL_len(L, idx);
        vertices.reserve(static_cast<size_t>(n));
        for (lua_Integer i = 1; i <= n; ++i)
        {
            lua_rawgeti(L, idx, i);
            vertices.push_back(readVertexTable(L, lua_gettop(L)));
            lua_pop(L, 1);
        }
    }
    else
    {
        lua_Integer count = luaL_checkinteger(L, idx);
        if (count < 1)
        {
            return luaL_error(L, "Invalid number of vertices (%d)", static_cast<int>(count));
        }
        vertices.assign(static_cast<size_t>(count), Vertex());
    }
    if (vertices.empty())
    {
        return luaL_error(L, "Cannot create a Mesh without vertices");
    }
    int mode = lua_isnoneornil(L, idx + 1) ? static_cast<int>(DrawMode::Fan)
                                           : luax::checkenum(L, idx + 1, MODE_NAMES, MODE_VALUES, "mesh draw mode");
    int usage = lua_isnoneornil(L, idx + 2) ? 1 : luax::checkenum(L, idx + 2, USAGE_NAMES, USAGE_VALUES, "usage hint");

    MeshObj *mesh = luax::newobject<MeshObj>(L, MESH_TYPE);
    mesh->vertices = std::move(vertices);
    mesh->mode = static_cast<DrawMode>(mode);
    mesh->usage = usage;
    return 1;
}

int mesh_setVertices(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    luaL_checktype(L, 2, LUA_TTABLE);
    lua_Integer start = luaL_optinteger(L, 3, 1);
    lua_Integer n = luaL_len(L, 2);
    if (start < 1 || start - 1 + n > static_cast<lua_Integer>(mesh->vertices.size()))
    {
        return luaL_error(L, "Invalid vertex range: the Mesh has %d vertices and its size cannot change",
                          static_cast<int>(mesh->vertices.size()));
    }
    for (lua_Integer i = 1; i <= n; ++i)
    {
        lua_rawgeti(L, 2, i);
        mesh->vertices[static_cast<size_t>(start - 1 + i - 1)] = readVertexTable(L, lua_gettop(L));
        lua_pop(L, 1);
    }
    return 0;
}

int mesh_getVertices(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    lua_createtable(L, static_cast<int>(mesh->vertices.size()), 0);
    int n = 0;
    for (const Vertex &v : mesh->vertices)
    {
        const float values[8] = {v.x, v.y, v.u, v.v, v.r, v.g, v.b, v.a};
        lua_createtable(L, 8, 0);
        for (int i = 0; i < 8; ++i)
        {
            lua_pushnumber(L, values[i]);
            lua_rawseti(L, -2, i + 1);
        }
        lua_rawseti(L, -2, ++n);
    }
    return 1;
}

size_t vertexIndex(lua_State *L, MeshObj *mesh, int idx)
{
    lua_Integer i = luaL_checkinteger(L, idx);
    if (i < 1 || i > static_cast<lua_Integer>(mesh->vertices.size()))
    {
        luaL_error(L, "Invalid vertex index: %d", static_cast<int>(i));
    }
    return static_cast<size_t>(i - 1);
}

int mesh_setVertex(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    size_t i = vertexIndex(L, mesh, 2);
    mesh->vertices[i] = lua_istable(L, 3) ? readVertexTable(L, 3) : readVertexArgs(L, 3);
    return 0;
}

int mesh_getVertex(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    return pushVertex(L, mesh->vertices[vertexIndex(L, mesh, 2)]);
}

// Attribute indices follow the default format: 1 position, 2 texcoord, 3 color.
int mesh_setVertexAttribute(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    Vertex &v = mesh->vertices[vertexIndex(L, mesh, 2)];
    lua_Integer attribute = luaL_checkinteger(L, 3);
    switch (attribute)
    {
    case 1:
        v.x = luax::checkfloat(L, 4);
        v.y = luax::checkfloat(L, 5);
        break;
    case 2:
        v.u = luax::checkfloat(L, 4);
        v.v = luax::checkfloat(L, 5);
        break;
    case 3:
        v.r = luax::checkfloat(L, 4);
        v.g = luax::checkfloat(L, 5);
        v.b = luax::checkfloat(L, 6);
        v.a = luax::optfloat(L, 7, 1.0f);
        break;
    default:
        return luaL_error(L, "Invalid vertex attribute index: %d", static_cast<int>(attribute));
    }
    return 0;
}

int mesh_getVertexAttribute(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    const Vertex &v = mesh->vertices[vertexIndex(L, mesh, 2)];
    switch (luaL_checkinteger(L, 3))
    {
    case 1:
        lua_pushnumber(L, v.x);
        lua_pushnumber(L, v.y);
        return 2;
    case 2:
        lua_pushnumber(L, v.u);
        lua_pushnumber(L, v.v);
        return 2;
    case 3:
        lua_pushnumber(L, v.r);
        lua_pushnumber(L, v.g);
        lua_pushnumber(L, v.b);
        lua_pushnumber(L, v.a);
        return 4;
    default:
        return luaL_error(L, "Invalid vertex attribute index: %d", static_cast<int>(luaL_checkinteger(L, 3)));
    }
}

int mesh_getVertexCount(lua_State *L)
{
    lua_pushinteger(L, static_cast<lua_Integer>(check(L, 1)->vertices.size()));
    return 1;
}

int mesh_getVertexFormat(lua_State *L)
{
    check(L, 1);
    struct Entry
    {
        const char *name;
        const char *type;
        int components;
    };
    static const Entry entries[] = {{"VertexPosition", "float", 2}, {"VertexTexCoord", "float", 2}, {"VertexColor", "byte", 4}};
    lua_createtable(L, 3, 0);
    int n = 0;
    for (const Entry &e : entries)
    {
        lua_createtable(L, 3, 0);
        lua_pushstring(L, e.name);
        lua_rawseti(L, -2, 1);
        lua_pushstring(L, e.type);
        lua_rawseti(L, -2, 2);
        lua_pushinteger(L, e.components);
        lua_rawseti(L, -2, 3);
        lua_rawseti(L, -2, ++n);
    }
    return 1;
}

int mesh_setDrawMode(lua_State *L)
{
    check(L, 1)->mode = static_cast<DrawMode>(luax::checkenum(L, 2, MODE_NAMES, MODE_VALUES, "mesh draw mode"));
    return 0;
}

int mesh_getDrawMode(lua_State *L)
{
    lua_pushstring(L, luax::enumname(MODE_NAMES, MODE_VALUES, static_cast<int>(check(L, 1)->mode)));
    return 1;
}

int mesh_setDrawRange(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    if (lua_isnoneornil(L, 2))
    {
        mesh->rangeStart = mesh->rangeCount = -1;
        return 0;
    }
    lua_Integer start = luaL_checkinteger(L, 2);
    lua_Integer count = luaL_checkinteger(L, 3);
    if (start < 1 || count < 1)
    {
        return luaL_error(L, "Invalid draw range");
    }
    mesh->rangeStart = static_cast<int>(start) - 1;
    mesh->rangeCount = static_cast<int>(count);
    return 0;
}

int mesh_getDrawRange(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    if (mesh->rangeStart < 0)
    {
        return 0;
    }
    lua_pushinteger(L, mesh->rangeStart + 1);
    lua_pushinteger(L, mesh->rangeCount);
    return 2;
}

int mesh_setTexture(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    if (lua_isnoneornil(L, 2))
    {
        mesh->texture.clear(L);
        return 0;
    }
    checkDrawSource(L, 2);
    mesh->texture.set(L, 2);
    return 0;
}

int mesh_getTexture(lua_State *L)
{
    check(L, 1)->texture.push(L);
    return 1;
}

// setVertexMap(i1, i2, ...) | setVertexMap({i1, i2, ...}) | setVertexMap() to clear
int mesh_setVertexMap(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    std::vector<unsigned int> map;
    auto add = [&](lua_Integer value) {
        if (value < 1 || value > static_cast<lua_Integer>(mesh->vertices.size()))
        {
            luaL_error(L, "Invalid vertex map value: %d", static_cast<int>(value));
        }
        map.push_back(static_cast<unsigned int>(value - 1));
    };
    if (lua_istable(L, 2))
    {
        lua_Integer n = luaL_len(L, 2);
        for (lua_Integer i = 1; i <= n; ++i)
        {
            lua_rawgeti(L, 2, i);
            add(luaL_checkinteger(L, -1));
            lua_pop(L, 1);
        }
    }
    else
    {
        for (int i = 2; i <= lua_gettop(L); ++i)
        {
            add(luaL_checkinteger(L, i));
        }
    }
    mesh->map = std::move(map);
    return 0;
}

int mesh_getVertexMap(lua_State *L)
{
    MeshObj *mesh = check(L, 1);
    if (mesh->map.empty())
    {
        return 0;
    }
    lua_createtable(L, static_cast<int>(mesh->map.size()), 0);
    int n = 0;
    for (unsigned int value : mesh->map)
    {
        lua_pushinteger(L, value + 1);
        lua_rawseti(L, -2, ++n);
    }
    return 1;
}

int mesh_flush(lua_State *L)
{
    return 0;
}

int mesh_tostring(lua_State *L)
{
    lua_pushfstring(L, "Mesh: %d vertices", static_cast<int>(check(L, 1)->vertices.size()));
    return 1;
}

const luaL_Reg MESH_METHODS[] = {
    {"setVertices", mesh_setVertices},
    {"getVertices", mesh_getVertices},
    {"setVertex", mesh_setVertex},
    {"getVertex", mesh_getVertex},
    {"setVertexAttribute", mesh_setVertexAttribute},
    {"getVertexAttribute", mesh_getVertexAttribute},
    {"getVertexCount", mesh_getVertexCount},
    {"getVertexFormat", mesh_getVertexFormat},
    {"setDrawMode", mesh_setDrawMode},
    {"getDrawMode", mesh_getDrawMode},
    {"setDrawRange", mesh_setDrawRange},
    {"getDrawRange", mesh_getDrawRange},
    {"setTexture", mesh_setTexture},
    {"getTexture", mesh_getTexture},
    {"setVertexMap", mesh_setVertexMap},
    {"getVertexMap", mesh_getVertexMap},
    {"flush", mesh_flush},
    {"__tostring", mesh_tostring},
    {nullptr, nullptr},
};

unsigned char byteOf(float value)
{
    return static_cast<unsigned char>(std::min(1.0f, std::max(0.0f, value)) * 255.0f + 0.5f);
}

} // namespace

void drawMesh(lua_State *L, MeshObj &mesh, const Matrix &transform)
{
    DrawSource src = {};
    bool textured = false;
    if (mesh.texture.valid())
    {
        mesh.texture.push(L);
        src = checkDrawSource(L, lua_gettop(L));
        lua_pop(L, 1);
        textured = true;
    }

    // Indices into the vertex array: the vertex map when set, otherwise 0..n-1.
    std::vector<unsigned int> order;
    if (!mesh.map.empty())
    {
        order = mesh.map;
    }
    else
    {
        order.resize(mesh.vertices.size());
        for (size_t i = 0; i < order.size(); ++i)
        {
            order[i] = static_cast<unsigned int>(i);
        }
    }
    if (mesh.rangeStart >= 0)
    {
        size_t start = std::min(order.size(), static_cast<size_t>(mesh.rangeStart));
        size_t end = std::min(order.size(), start + static_cast<size_t>(mesh.rangeCount));
        order = std::vector<unsigned int>(order.begin() + static_cast<long>(start), order.begin() + static_cast<long>(end));
    }
    if (order.empty())
    {
        return;
    }

    // Triangle corner list for every mode except points.
    std::vector<unsigned int> corners;
    switch (mesh.mode)
    {
    case DrawMode::Triangles:
        for (size_t i = 0; i + 2 < order.size(); i += 3)
        {
            corners.insert(corners.end(), {order[i], order[i + 1], order[i + 2]});
        }
        break;
    case DrawMode::Fan:
        for (size_t i = 1; i + 1 < order.size(); ++i)
        {
            corners.insert(corners.end(), {order[0], order[i], order[i + 1]});
        }
        break;
    case DrawMode::Strip:
        for (size_t i = 0; i + 2 < order.size(); ++i)
        {
            if (i % 2 == 0)
            {
                corners.insert(corners.end(), {order[i], order[i + 1], order[i + 2]});
            }
            else
            {
                corners.insert(corners.end(), {order[i + 1], order[i], order[i + 2]});
            }
        }
        break;
    case DrawMode::Points:
        break;
    }

    ensureFrame();
    Color tint = currentColor();
    auto emit = [&](const Vertex &v) {
        rlColor4ub(static_cast<unsigned char>(byteOf(v.r) * tint.r / 255.0f + 0.5f),
                   static_cast<unsigned char>(byteOf(v.g) * tint.g / 255.0f + 0.5f),
                   static_cast<unsigned char>(byteOf(v.b) * tint.b / 255.0f + 0.5f),
                   static_cast<unsigned char>(byteOf(v.a) * tint.a / 255.0f + 0.5f));
        float tv = (textured && src.flipY) ? 1.0f - v.v : v.v;
        rlTexCoord2f(v.u, tv);
        rlVertex2f(v.x, v.y);
    };

    // The mesh winding is up to the user, so back-face culling must be off.
    rlDrawRenderBatchActive();
    rlDisableBackfaceCulling();
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloat(transform));
    const unsigned int textureId = textured ? src.texture.id : rlGetTextureIdDefault();

    if (mesh.mode == DrawMode::Points)
    {
        float half = currentPointSize() * 0.5f;
        for (size_t i = 0; i < order.size(); i += 500)
        {
            size_t count = std::min<size_t>(500, order.size() - i);
            rlCheckRenderBatchLimit(static_cast<int>(count) * 6);
            rlBegin(RL_TRIANGLES);
            rlSetTexture(textureId);
            for (size_t k = 0; k < count; ++k)
            {
                Vertex v = mesh.vertices[order[i + k]];
                Vertex a = v, b = v, c = v, d = v;
                a.x -= half; a.y -= half;
                b.x += half; b.y -= half;
                c.x += half; c.y += half;
                d.x -= half; d.y += half;
                emit(a); emit(b); emit(c);
                emit(a); emit(c); emit(d);
            }
            rlEnd();
        }
    }
    else
    {
        for (size_t i = 0; i < corners.size(); i += 3000)
        {
            size_t count = std::min<size_t>(3000, corners.size() - i);
            rlCheckRenderBatchLimit(static_cast<int>(count));
            rlBegin(RL_TRIANGLES);
            rlSetTexture(textureId);
            for (size_t k = 0; k < count; ++k)
            {
                emit(mesh.vertices[corners[i + k]]);
            }
            rlEnd();
        }
    }

    rlSetTexture(0);
    rlPopMatrix();
    rlDrawRenderBatchActive();
    rlEnableBackfaceCulling();
}

void registerMeshType(lua_State *L)
{
    luax::newtype(L, MESH_TYPE, MESH_METHODS, mesh_gc);
}

const luaL_Reg MESH_FUNCS[] = {
    {"newMesh", l_newMesh},
    {nullptr, nullptr},
};

} // namespace graphics
} // namespace love
