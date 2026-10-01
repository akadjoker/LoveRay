// graphics.cpp - love.graphics: state, frame handling, shapes, text and the
// coordinate transform stack.
#include "graphics_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace love
{
namespace graphics
{

namespace
{

constexpr int MAX_STACK_DEPTH = 31; // rlgl allows 32 matrices

enum BlendMode
{
    BLEND_LOVE_ALPHA,
    BLEND_LOVE_ADD,
    BLEND_LOVE_SUBTRACT,
    BLEND_LOVE_MULTIPLY,
    BLEND_LOVE_LIGHTEN,
    BLEND_LOVE_DARKEN,
    BLEND_LOVE_SCREEN,
    BLEND_LOVE_REPLACE,
    BLEND_LOVE_NONE,
};

const char *const BLEND_NAMES[] = {"alpha", "add", "subtract", "multiply", "lighten", "darken", "screen", "replace", "none"};
const int BLEND_VALUES[] = {BLEND_LOVE_ALPHA, BLEND_LOVE_ADD, BLEND_LOVE_SUBTRACT, BLEND_LOVE_MULTIPLY, BLEND_LOVE_LIGHTEN,
                            BLEND_LOVE_DARKEN, BLEND_LOVE_SCREEN, BLEND_LOVE_REPLACE, BLEND_LOVE_NONE};

struct State
{
    Color color = WHITE;
    Color background = BLACK;
    float lineWidth = 1.0f;
    std::string lineStyle = "smooth";
    std::string lineJoin = "miter";
    float pointSize = 1.0f;
    int blendMode = BLEND_LOVE_ALPHA;
    bool premultiplied = false;
    bool scissor = false;
    Rectangle scissorRect = {0, 0, 0, 0};
    bool colorMask[4] = {true, true, true, true};
    bool wireframe = false;
    int stencilMode = 0; // index into STENCIL_NAMES, 0 is "always" (test off)
    int stencilValue = 0;
    luax::Ref font;
    luax::Ref canvas;
    luax::Ref shader;
};

State g_state;
ShaderObj *g_shader = nullptr;
std::vector<State *> g_stateStack; // snapshots for push("all")
std::vector<bool> g_pushKinds;     // true = "all"
std::string g_defaultFilterMin = "linear";
std::string g_defaultFilterMag = "linear";
bool g_frameActive = false;
bool g_blendDirty = true;
// Pending love.graphics.captureScreenshot
std::string g_screenshotPath;
luax::Ref g_screenshotCallback;
bool g_screenshotPending = false;
lua_State *g_screenshotState = nullptr;

Color toColor(float r, float g, float b, float a)
{
    auto clamp = [](float v) {
        if (v < 0.0f)
        {
            v = 0.0f;
        }
        if (v > 1.0f)
        {
            v = 1.0f;
        }
        return static_cast<unsigned char>(v * 255.0f + 0.5f);
    };
    return Color{clamp(r), clamp(g), clamp(b), clamp(a)};
}

// Reads a color from numbers or a table starting at `idx`. Returns the index
// after the color arguments. Alpha defaults to 1 like Love2D 11.
int readColor(lua_State *L, int idx, Color &out, bool required = true)
{
    if (lua_istable(L, idx))
    {
        float c[4] = {0, 0, 0, 1};
        for (int i = 0; i < 4; ++i)
        {
            lua_rawgeti(L, idx, i + 1);
            if (lua_isnumber(L, -1))
            {
                c[i] = static_cast<float>(lua_tonumber(L, -1));
            }
            lua_pop(L, 1);
        }
        out = toColor(c[0], c[1], c[2], c[3]);
        return idx + 1;
    }
    if (!required && lua_isnoneornil(L, idx))
    {
        return idx;
    }
    float r = luax::checkfloat(L, idx);
    float g = luax::checkfloat(L, idx + 1);
    float b = luax::checkfloat(L, idx + 2);
    float a = luax::optfloat(L, idx + 3, 1.0f);
    out = toColor(r, g, b, a);
    return idx + 4;
}

void pushColor(lua_State *L, Color c)
{
    lua_pushnumber(L, c.r / 255.0);
    lua_pushnumber(L, c.g / 255.0);
    lua_pushnumber(L, c.b / 255.0);
    lua_pushnumber(L, c.a / 255.0);
}

void applyBlendMode()
{
    g_blendDirty = false;
    int src = RL_SRC_ALPHA;
    int dst = RL_ONE_MINUS_SRC_ALPHA;
    int srcA = RL_ONE;
    int dstA = RL_ONE_MINUS_SRC_ALPHA;
    int eq = RL_FUNC_ADD;
    switch (g_state.blendMode)
    {
    case BLEND_LOVE_ALPHA:
        src = g_state.premultiplied ? RL_ONE : RL_SRC_ALPHA;
        break;
    case BLEND_LOVE_ADD:
        src = g_state.premultiplied ? RL_ONE : RL_SRC_ALPHA;
        dst = RL_ONE;
        srcA = RL_ZERO;
        dstA = RL_ONE;
        break;
    case BLEND_LOVE_SUBTRACT:
        src = g_state.premultiplied ? RL_ONE : RL_SRC_ALPHA;
        dst = RL_ONE;
        srcA = RL_ZERO;
        dstA = RL_ONE;
        eq = RL_FUNC_REVERSE_SUBTRACT;
        break;
    case BLEND_LOVE_MULTIPLY:
        src = RL_DST_COLOR;
        dst = RL_ONE_MINUS_SRC_ALPHA;
        srcA = RL_DST_COLOR;
        dstA = RL_ONE_MINUS_SRC_ALPHA;
        break;
    case BLEND_LOVE_LIGHTEN:
        src = RL_ONE;
        dst = RL_ONE;
        srcA = RL_ONE;
        dstA = RL_ONE;
        eq = RL_MAX;
        break;
    case BLEND_LOVE_DARKEN:
        src = RL_ONE;
        dst = RL_ONE;
        srcA = RL_ONE;
        dstA = RL_ONE;
        eq = RL_MIN;
        break;
    case BLEND_LOVE_SCREEN:
        src = RL_ONE;
        dst = RL_ONE_MINUS_SRC_COLOR;
        srcA = RL_ONE;
        dstA = RL_ONE_MINUS_SRC_ALPHA;
        break;
    case BLEND_LOVE_REPLACE:
    case BLEND_LOVE_NONE:
        src = RL_ONE;
        dst = RL_ZERO;
        srcA = RL_ONE;
        dstA = RL_ZERO;
        break;
    }
    rlSetBlendFactorsSeparate(src, dst, srcA, dstA, eq, eq);
    BeginBlendMode(BLEND_CUSTOM_SEPARATE);
}

void applyScissor()
{
    if (g_state.scissor)
    {
        BeginScissorMode(static_cast<int>(g_state.scissorRect.x), static_cast<int>(g_state.scissorRect.y),
                         static_cast<int>(g_state.scissorRect.width), static_cast<int>(g_state.scissorRect.height));
    }
    else
    {
        EndScissorMode();
    }
}

const char *const STENCIL_NAMES[] = {"always", "equal", "notequal", "less", "lequal", "greater", "gequal"};
const int STENCIL_VALUES[] = {0, 1, 2, 3, 4, 5, 6};
// OpenGL compares the reference against the buffer, Love2D compares the buffer
// against the value, so the orderings are mirrored.
const int STENCIL_GL_FUNCS[] = {0x0207, 0x0202, 0x0205, 0x0204, 0x0206, 0x0201, 0x0203};

const char *const STENCIL_ACTION_NAMES[] = {"replace", "increment", "decrement", "incrementwrap", "decrementwrap", "invert"};
const int STENCIL_ACTION_VALUES[] = {0x1E01, 0x1E02, 0x1E03, 0x8507, 0x8508, 0x150A};

constexpr int GL_KEEP_OP = 0x1E00;
constexpr int GL_ALWAYS_FUNC = 0x0207;

void applyStencilTest()
{
    rlDrawRenderBatchActive();
    if (g_state.stencilMode == 0)
    {
        rlDisableStencilTest();
        return;
    }
    rlEnableStencilTest();
    rlStencilFunc(STENCIL_GL_FUNCS[g_state.stencilMode], g_state.stencilValue, 0xFF);
    rlStencilOp(GL_KEEP_OP, GL_KEEP_OP, GL_KEEP_OP);
    rlStencilMask(0);
}

CanvasObj *activeCanvas(lua_State *L)
{
    if (!g_state.canvas.valid())
    {
        return nullptr;
    }
    g_state.canvas.push(L);
    CanvasObj *canvas = luax::testobject<CanvasObj>(L, -1, CANVAS_TYPE);
    lua_pop(L, 1);
    return canvas;
}

void currentTargetSize(lua_State *L, int &width, int &height)
{
    if (CanvasObj *canvas = activeCanvas(L))
    {
        width = canvas->target.texture.width;
        height = canvas->target.texture.height;
        return;
    }
    width = GetScreenWidth();
    height = GetScreenHeight();
}

void performScreenshot()
{
    if (!g_screenshotPending)
    {
        return;
    }
    g_screenshotPending = false;
    Image image = LoadImageFromScreen();
    if (!g_screenshotPath.empty())
    {
        std::string real = filesystem::resolveWrite(g_screenshotPath);
        if (real.empty() || !ExportImage(image, real.c_str()))
        {
            log(LogLevel::Error, "Could not save screenshot to '%s'", g_screenshotPath.c_str());
        }
        else
        {
            log(LogLevel::Info, "Screenshot saved to %s", real.c_str());
        }
        g_screenshotPath.clear();
    }
    else if (g_screenshotCallback.valid() && g_screenshotState != nullptr)
    {
        lua_State *L = g_screenshotState;
        g_screenshotCallback.push(L);
        g_screenshotCallback.clear(L);
        Image *data = luax::newobject<Image>(L, "ImageData");
        *data = ImageCopy(image);
        if (lua_pcall(L, 1, 0, 0) != LUA_OK)
        {
            log(LogLevel::Error, "captureScreenshot callback: %s", lua_tostring(L, -1));
            lua_pop(L, 1);
        }
    }
    UnloadImage(image);
}

} // namespace

// ---------------------------------------------------------------------------
// Services for other modules
// ---------------------------------------------------------------------------

void ensureFrame()
{
    if (g_frameActive)
    {
        if (g_shader != nullptr)
        {
            shaderBeforeDraw(g_shader);
        }
        return;
    }
    window::ensureOpen();
    BeginDrawing();
    g_frameActive = true;
    g_pushKinds.clear();
    applyBlendMode();
    applyScissor();
    applyStencilTest();
    if (g_shader != nullptr)
    {
        shaderActivate(g_shader, GetScreenWidth(), GetScreenHeight());
    }
    if (g_state.wireframe)
    {
        rlEnableWireMode();
    }
}

bool frameActive()
{
    return g_frameActive;
}

void present()
{
    if (!g_frameActive)
    {
        ensureFrame();
    }
    // Unbalanced push() calls would otherwise leak into the next frame.
    while (!g_pushKinds.empty())
    {
        rlPopMatrix();
        g_pushKinds.pop_back();
    }
    if (g_state.scissor)
    {
        EndScissorMode();
    }
    EndBlendMode();
    performScreenshot();
    EndDrawing();
    g_frameActive = false;
    g_blendDirty = true;
}

Color currentColor()
{
    return g_state.color;
}

float currentPointSize()
{
    return g_state.pointSize;
}

const std::string &defaultFilterMin()
{
    return g_defaultFilterMin;
}

const std::string &defaultFilterMag()
{
    return g_defaultFilterMag;
}

Matrix multiply(const Matrix &a, const Matrix &b)
{
    const float *fa = reinterpret_cast<const float *>(&a);
    const float *fb = reinterpret_cast<const float *>(&b);
    Matrix r;
    float *fr = reinterpret_cast<float *>(&r);
    // raylib stores the matrix row-major in memory: element(row, col) = f[row * 4 + col].
    for (int col = 0; col < 4; ++col)
    {
        for (int row = 0; row < 4; ++row)
        {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k)
            {
                sum += fa[row * 4 + k] * fb[k * 4 + col];
            }
            fr[row * 4 + col] = sum;
        }
    }
    return r;
}

Matrix makeTransform(float x, float y, float angle, float sx, float sy, float ox, float oy, float kx, float ky)
{
    // Same closed form Love2D uses: translate * rotate * scale * shear * origin.
    Matrix e = MatrixIdentity();
    float c = std::cos(angle);
    float s = std::sin(angle);
    e.m0 = c * sx - ky * s * sy;
    e.m1 = s * sx + ky * c * sy;
    e.m4 = kx * c * sx - s * sy;
    e.m5 = kx * s * sx + c * sy;
    e.m12 = x - ox * e.m0 - oy * e.m4;
    e.m13 = y - ox * e.m1 - oy * e.m5;
    return e;
}

void drawTextured(const DrawSource &src, const Matrix &transform, Color color)
{
    ensureFrame();
    float tw = static_cast<float>(src.texture.width);
    float th = static_cast<float>(src.texture.height);
    if (tw <= 0.0f || th <= 0.0f)
    {
        return;
    }
    float u0 = src.source.x / tw;
    float u1 = (src.source.x + src.source.width) / tw;
    float v0 = src.source.y / th;
    float v1 = (src.source.y + src.source.height) / th;
    if (src.flipY)
    {
        std::swap(v0, v1);
    }
    float w = src.source.width;
    float h = src.source.height;

    rlSetTexture(src.texture.id);
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloat(transform));
    rlBegin(RL_QUADS);
    rlColor4ub(color.r, color.g, color.b, color.a);
    rlNormal3f(0.0f, 0.0f, 1.0f);
    rlTexCoord2f(u0, v0);
    rlVertex2f(0.0f, 0.0f);
    rlTexCoord2f(u0, v1);
    rlVertex2f(0.0f, h);
    rlTexCoord2f(u1, v1);
    rlVertex2f(w, h);
    rlTexCoord2f(u1, v0);
    rlVertex2f(w, 0.0f);
    rlEnd();
    rlPopMatrix();
    rlSetTexture(0);
}

void shutdown()
{
    for (State *s : g_stateStack)
    {
        delete s;
    }
    g_stateStack.clear();
    g_pushKinds.clear();
    g_state = State();
    g_shader = nullptr;
    g_frameActive = false;
    g_screenshotPending = false;
    g_screenshotState = nullptr;
    shutdownObjects();
}

// ---------------------------------------------------------------------------
// Shape helpers
// ---------------------------------------------------------------------------

namespace
{

enum DrawMode
{
    MODE_FILL,
    MODE_LINE
};

int checkDrawMode(lua_State *L, int idx)
{
    static const char *const names[] = {"fill", "line"};
    static const int values[] = {MODE_FILL, MODE_LINE};
    return luax::checkenum(L, idx, names, values, "draw mode");
}

// Reads a flat list of coordinates (numbers or one table) starting at `idx`.
void readPoints(lua_State *L, int idx, std::vector<Vector2> &points)
{
    points.clear();
    if (lua_istable(L, idx))
    {
        lua_Integer n = luaL_len(L, idx);
        if (n % 2 != 0)
        {
            luaL_error(L, "Number of vertex components must be a multiple of two");
        }
        for (lua_Integer i = 1; i <= n; i += 2)
        {
            lua_rawgeti(L, idx, i);
            lua_rawgeti(L, idx, i + 1);
            points.push_back({static_cast<float>(luaL_checknumber(L, -2)), static_cast<float>(luaL_checknumber(L, -1))});
            lua_pop(L, 2);
        }
        return;
    }
    int n = lua_gettop(L) - idx + 1;
    if (n % 2 != 0)
    {
        luaL_error(L, "Number of vertex components must be a multiple of two");
    }
    for (int i = idx; i < idx + n; i += 2)
    {
        points.push_back({luax::checkfloat(L, i), luax::checkfloat(L, i + 1)});
    }
}

void drawPolyline(const std::vector<Vector2> &points, bool closed)
{
    if (points.size() < 2)
    {
        return;
    }
    float width = g_state.lineWidth;
    Color color = g_state.color;
    size_t count = points.size();
    size_t segments = closed ? count : count - 1;
    if (width <= 1.0f)
    {
        rlBegin(RL_LINES);
        rlColor4ub(color.r, color.g, color.b, color.a);
        for (size_t i = 0; i < segments; ++i)
        {
            const Vector2 &a = points[i];
            const Vector2 &b = points[(i + 1) % count];
            rlVertex2f(a.x, a.y);
            rlVertex2f(b.x, b.y);
        }
        rlEnd();
        return;
    }
    for (size_t i = 0; i < segments; ++i)
    {
        DrawLineEx(points[i], points[(i + 1) % count], width, color);
    }
    // Round the joints so thick lines do not show gaps at corners.
    size_t jointStart = closed ? 0 : 1;
    size_t jointEnd = closed ? count : count - 1;
    for (size_t i = jointStart; i < jointEnd; ++i)
    {
        DrawCircleV(points[i], width * 0.5f, color);
    }
}

// raylib culls back faces; on screen (y down) a visible triangle has a
// negative shoelace area, so every fill below is emitted in that winding.
float signedArea(const std::vector<Vector2> &points)
{
    float area = 0.0f;
    for (size_t i = 0; i < points.size(); ++i)
    {
        const Vector2 &a = points[i];
        const Vector2 &b = points[(i + 1) % points.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return area * 0.5f;
}

void drawFan(const std::vector<Vector2> &points)
{
    if (points.size() < 3)
    {
        return;
    }
    bool reverse = signedArea(points) > 0.0f;
    Color c = g_state.color;
    rlBegin(RL_TRIANGLES);
    rlColor4ub(c.r, c.g, c.b, c.a);
    const Vector2 &origin = points[0];
    for (size_t i = 1; i + 1 < points.size(); ++i)
    {
        const Vector2 &b = reverse ? points[i + 1] : points[i];
        const Vector2 &d = reverse ? points[i] : points[i + 1];
        rlVertex2f(origin.x, origin.y);
        rlVertex2f(b.x, b.y);
        rlVertex2f(d.x, d.y);
    }
    rlEnd();
}

void fillPolygon(lua_State *L, const std::vector<Vector2> &points)
{
    if (points.size() < 3)
    {
        return;
    }
    std::vector<float> flat;
    flat.reserve(points.size() * 2);
    for (const Vector2 &p : points)
    {
        flat.push_back(p.x);
        flat.push_back(p.y);
    }
    if (math::isConvex(flat))
    {
        drawFan(points);
        return;
    }
    std::vector<int> indices;
    if (!math::triangulate(flat, indices))
    {
        luaL_error(L, "Could not triangulate polygon (is it simple and non-degenerate?)");
    }
    Color c = g_state.color;
    rlBegin(RL_TRIANGLES);
    rlColor4ub(c.r, c.g, c.b, c.a);
    for (size_t i = 0; i + 2 < indices.size(); i += 3)
    {
        const Vector2 &a = points[indices[i]];
        const Vector2 &b = points[indices[i + 1]];
        const Vector2 &d = points[indices[i + 2]];
        float cross = (b.x - a.x) * (d.y - a.y) - (b.y - a.y) * (d.x - a.x);
        if (cross > 0.0f)
        {
            rlVertex2f(a.x, a.y);
            rlVertex2f(d.x, d.y);
            rlVertex2f(b.x, b.y);
        }
        else
        {
            rlVertex2f(a.x, a.y);
            rlVertex2f(b.x, b.y);
            rlVertex2f(d.x, d.y);
        }
    }
    rlEnd();
}

void arcPoints(float x, float y, float radius, float a1, float a2, int segments, std::vector<Vector2> &out)
{
    out.clear();
    for (int i = 0; i <= segments; ++i)
    {
        float t = a1 + (a2 - a1) * static_cast<float>(i) / static_cast<float>(segments);
        out.push_back({x + std::cos(t) * radius, y + std::sin(t) * radius});
    }
}

int defaultSegments(float radius)
{
    int segments = static_cast<int>(radius * 0.5f) + 12;
    return std::min(std::max(segments, 12), 128);
}

// ---------------------------------------------------------------------------
// love.graphics.* : state
// ---------------------------------------------------------------------------

int l_setColor(lua_State *L)
{
    readColor(L, 1, g_state.color);
    return 0;
}

int l_getColor(lua_State *L)
{
    pushColor(L, g_state.color);
    return 4;
}

int l_setBackgroundColor(lua_State *L)
{
    readColor(L, 1, g_state.background);
    return 0;
}

int l_getBackgroundColor(lua_State *L)
{
    pushColor(L, g_state.background);
    return 4;
}

int l_setLineWidth(lua_State *L)
{
    g_state.lineWidth = luax::checkfloat(L, 1);
    return 0;
}

int l_getLineWidth(lua_State *L)
{
    lua_pushnumber(L, g_state.lineWidth);
    return 1;
}

int l_setLineStyle(lua_State *L)
{
    static const char *const names[] = {"rough", "smooth"};
    static const int values[] = {0, 1};
    luax::checkenum(L, 1, names, values, "line style");
    g_state.lineStyle = lua_tostring(L, 1);
    return 0;
}

int l_getLineStyle(lua_State *L)
{
    lua_pushstring(L, g_state.lineStyle.c_str());
    return 1;
}

int l_setLineJoin(lua_State *L)
{
    static const char *const names[] = {"miter", "none", "bevel"};
    static const int values[] = {0, 1, 2};
    luax::checkenum(L, 1, names, values, "line join");
    g_state.lineJoin = lua_tostring(L, 1);
    return 0;
}

int l_getLineJoin(lua_State *L)
{
    lua_pushstring(L, g_state.lineJoin.c_str());
    return 1;
}

int l_setPointSize(lua_State *L)
{
    g_state.pointSize = luax::checkfloat(L, 1);
    return 0;
}

int l_getPointSize(lua_State *L)
{
    lua_pushnumber(L, g_state.pointSize);
    return 1;
}

int l_setBlendMode(lua_State *L)
{
    g_state.blendMode = luax::checkenum(L, 1, BLEND_NAMES, BLEND_VALUES, "blend mode");
    if (!lua_isnoneornil(L, 2))
    {
        static const char *const names[] = {"alphamultiply", "premultiplied"};
        static const int values[] = {0, 1};
        g_state.premultiplied = luax::checkenum(L, 2, names, values, "blend alpha mode") == 1;
    }
    else
    {
        g_state.premultiplied = false;
    }
    if (g_state.blendMode == BLEND_LOVE_MULTIPLY && !g_state.premultiplied)
    {
        return luaL_error(L, "The 'multiply' blend mode must be used with premultiplied alpha.");
    }
    if (g_frameActive)
    {
        applyBlendMode();
    }
    return 0;
}

int l_getBlendMode(lua_State *L)
{
    lua_pushstring(L, luax::enumname(BLEND_NAMES, BLEND_VALUES, g_state.blendMode));
    lua_pushstring(L, g_state.premultiplied ? "premultiplied" : "alphamultiply");
    return 2;
}

int l_setScissor(lua_State *L)
{
    if (lua_isnoneornil(L, 1))
    {
        g_state.scissor = false;
    }
    else
    {
        g_state.scissor = true;
        g_state.scissorRect = {luax::checkfloat(L, 1), luax::checkfloat(L, 2), luax::checkfloat(L, 3), luax::checkfloat(L, 4)};
    }
    if (g_frameActive)
    {
        applyScissor();
    }
    return 0;
}

int l_intersectScissor(lua_State *L)
{
    Rectangle r = {luax::checkfloat(L, 1), luax::checkfloat(L, 2), luax::checkfloat(L, 3), luax::checkfloat(L, 4)};
    if (g_state.scissor)
    {
        Rectangle c = g_state.scissorRect;
        float x0 = std::max(r.x, c.x);
        float y0 = std::max(r.y, c.y);
        float x1 = std::min(r.x + r.width, c.x + c.width);
        float y1 = std::min(r.y + r.height, c.y + c.height);
        r = {x0, y0, std::max(0.0f, x1 - x0), std::max(0.0f, y1 - y0)};
    }
    g_state.scissor = true;
    g_state.scissorRect = r;
    if (g_frameActive)
    {
        applyScissor();
    }
    return 0;
}

int l_getScissor(lua_State *L)
{
    if (!g_state.scissor)
    {
        return 0;
    }
    lua_pushnumber(L, g_state.scissorRect.x);
    lua_pushnumber(L, g_state.scissorRect.y);
    lua_pushnumber(L, g_state.scissorRect.width);
    lua_pushnumber(L, g_state.scissorRect.height);
    return 4;
}

int l_setColorMask(lua_State *L)
{
    if (lua_isnoneornil(L, 1))
    {
        for (bool &m : g_state.colorMask)
        {
            m = true;
        }
    }
    else
    {
        for (int i = 0; i < 4; ++i)
        {
            g_state.colorMask[i] = luax::checkboolean(L, i + 1);
        }
    }
    if (g_frameActive)
    {
        rlDrawRenderBatchActive();
    }
    rlColorMask(g_state.colorMask[0], g_state.colorMask[1], g_state.colorMask[2], g_state.colorMask[3]);
    return 0;
}

int l_getColorMask(lua_State *L)
{
    for (bool m : g_state.colorMask)
    {
        lua_pushboolean(L, m);
    }
    return 4;
}

int l_setWireframe(lua_State *L)
{
    g_state.wireframe = luax::checkboolean(L, 1);
    if (g_frameActive)
    {
        rlDrawRenderBatchActive();
        if (g_state.wireframe)
        {
            rlEnableWireMode();
        }
        else
        {
            rlDisableWireMode();
        }
    }
    return 0;
}

int l_isWireframe(lua_State *L)
{
    lua_pushboolean(L, g_state.wireframe);
    return 1;
}

int l_setDefaultFilter(lua_State *L)
{
    static const char *const names[] = {"linear", "nearest"};
    static const int values[] = {0, 1};
    luax::checkenum(L, 1, names, values, "filter mode");
    g_defaultFilterMin = lua_tostring(L, 1);
    if (lua_isnoneornil(L, 2))
    {
        g_defaultFilterMag = g_defaultFilterMin;
    }
    else
    {
        luax::checkenum(L, 2, names, values, "filter mode");
        g_defaultFilterMag = lua_tostring(L, 2);
    }
    return 0;
}

int l_getDefaultFilter(lua_State *L)
{
    lua_pushstring(L, g_defaultFilterMin.c_str());
    lua_pushstring(L, g_defaultFilterMag.c_str());
    lua_pushnumber(L, 1);
    return 3;
}

int l_reset(lua_State *L)
{
    g_state.color = WHITE;
    g_state.background = BLACK;
    g_state.lineWidth = 1.0f;
    g_state.pointSize = 1.0f;
    g_state.blendMode = BLEND_LOVE_ALPHA;
    g_state.premultiplied = false;
    g_state.scissor = false;
    g_state.wireframe = false;
    g_state.stencilMode = 0;
    g_state.stencilValue = 0;
    for (bool &m : g_state.colorMask)
    {
        m = true;
    }
    g_state.canvas.clear(L);
    g_state.font.clear(L);
    g_state.shader.clear(L);
    if (g_frameActive)
    {
        EndTextureMode();
        if (g_shader != nullptr)
        {
            shaderDeactivate();
        }
        applyBlendMode();
        applyScissor();
        applyStencilTest();
        rlColorMask(true, true, true, true);
        rlDisableWireMode();
        while (!g_pushKinds.empty())
        {
            rlPopMatrix();
            g_pushKinds.pop_back();
        }
        rlLoadIdentity();
    }
    g_shader = nullptr;
    return 0;
}

// ---------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------

int l_clear(lua_State *L)
{
    ensureFrame();
    Color c = g_state.background;
    int next = 1;
    if (lua_isnumber(L, 1) || lua_istable(L, 1))
    {
        next = readColor(L, 1, c);
    }
    ClearBackground(c);

    // clear(r, g, b, a, clearstencil, cleardepth): the stencil buffer is cleared
    // unless told otherwise, and a number clears it to that value.
    int stencilValue = 0;
    bool clearStencil = true;
    if (lua_isboolean(L, next))
    {
        clearStencil = lua_toboolean(L, next) != 0;
    }
    else if (lua_isnumber(L, next))
    {
        stencilValue = static_cast<int>(lua_tointeger(L, next));
    }
    if (clearStencil)
    {
        rlClearStencil(stencilValue);
        rlStencilMask(0);
    }
    return 0;
}

int l_discard(lua_State *L)
{
    return 0;
}

int l_present(lua_State *L)
{
    present();
    return 0;
}

int l_isActive(lua_State *L)
{
    lua_pushboolean(L, window::isOpen());
    return 1;
}

int l_isCreated(lua_State *L)
{
    lua_pushboolean(L, window::isOpen());
    return 1;
}

int l_getWidth(lua_State *L)
{
    lua_pushinteger(L, window::isOpen() ? GetScreenWidth() : 0);
    return 1;
}

int l_getHeight(lua_State *L)
{
    lua_pushinteger(L, window::isOpen() ? GetScreenHeight() : 0);
    return 1;
}

int l_getDimensions(lua_State *L)
{
    lua_pushinteger(L, window::isOpen() ? GetScreenWidth() : 0);
    lua_pushinteger(L, window::isOpen() ? GetScreenHeight() : 0);
    return 2;
}

int l_getPixelWidth(lua_State *L)
{
    lua_pushinteger(L, window::isOpen() ? GetRenderWidth() : 0);
    return 1;
}

int l_getPixelHeight(lua_State *L)
{
    lua_pushinteger(L, window::isOpen() ? GetRenderHeight() : 0);
    return 1;
}

int l_getPixelDimensions(lua_State *L)
{
    lua_pushinteger(L, window::isOpen() ? GetRenderWidth() : 0);
    lua_pushinteger(L, window::isOpen() ? GetRenderHeight() : 0);
    return 2;
}

int l_getDPIScale(lua_State *L)
{
    lua_pushnumber(L, window::isOpen() ? GetWindowScaleDPI().x : 1.0);
    return 1;
}

int l_captureScreenshot(lua_State *L)
{
    g_screenshotPending = true;
    g_screenshotState = L;
    if (lua_type(L, 1) == LUA_TFUNCTION)
    {
        g_screenshotPath.clear();
        g_screenshotCallback.set(L, 1);
    }
    else
    {
        g_screenshotPath = luaL_checkstring(L, 1);
        g_screenshotCallback.clear(L);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Transform stack
// ---------------------------------------------------------------------------

int l_push(lua_State *L)
{
    ensureFrame();
    if (g_pushKinds.size() >= static_cast<size_t>(MAX_STACK_DEPTH))
    {
        return luaL_error(L, "Maximum stack depth reached (more pushes than pops?)");
    }
    bool all = false;
    if (!lua_isnoneornil(L, 1))
    {
        static const char *const names[] = {"transform", "all"};
        static const int values[] = {0, 1};
        all = luax::checkenum(L, 1, names, values, "graphics stack type") == 1;
    }
    rlPushMatrix();
    g_pushKinds.push_back(all);
    if (all)
    {
        State *snapshot = new State();
        snapshot->color = g_state.color;
        snapshot->background = g_state.background;
        snapshot->lineWidth = g_state.lineWidth;
        snapshot->lineStyle = g_state.lineStyle;
        snapshot->lineJoin = g_state.lineJoin;
        snapshot->pointSize = g_state.pointSize;
        snapshot->blendMode = g_state.blendMode;
        snapshot->premultiplied = g_state.premultiplied;
        snapshot->scissor = g_state.scissor;
        snapshot->scissorRect = g_state.scissorRect;
        snapshot->wireframe = g_state.wireframe;
        snapshot->stencilMode = g_state.stencilMode;
        snapshot->stencilValue = g_state.stencilValue;
        std::memcpy(snapshot->colorMask, g_state.colorMask, sizeof(snapshot->colorMask));
        g_state.font.push(L);
        snapshot->font.set(L, -1);
        lua_pop(L, 1);
        g_state.canvas.push(L);
        snapshot->canvas.set(L, -1);
        lua_pop(L, 1);
        g_state.shader.push(L);
        snapshot->shader.set(L, -1);
        lua_pop(L, 1);
        g_stateStack.push_back(snapshot);
    }
    return 0;
}

int setCanvasImpl(lua_State *L, int idx);
int setShaderImpl(lua_State *L, int idx);

int l_pop(lua_State *L)
{
    if (g_pushKinds.empty())
    {
        return luaL_error(L, "Minimum stack depth reached (more pops than pushes?)");
    }
    rlPopMatrix();
    bool all = g_pushKinds.back();
    g_pushKinds.pop_back();
    if (all && !g_stateStack.empty())
    {
        State *snapshot = g_stateStack.back();
        g_stateStack.pop_back();
        g_state.color = snapshot->color;
        g_state.background = snapshot->background;
        g_state.lineWidth = snapshot->lineWidth;
        g_state.lineStyle = snapshot->lineStyle;
        g_state.lineJoin = snapshot->lineJoin;
        g_state.pointSize = snapshot->pointSize;
        g_state.blendMode = snapshot->blendMode;
        g_state.premultiplied = snapshot->premultiplied;
        g_state.scissor = snapshot->scissor;
        g_state.scissorRect = snapshot->scissorRect;
        g_state.wireframe = snapshot->wireframe;
        g_state.stencilMode = snapshot->stencilMode;
        g_state.stencilValue = snapshot->stencilValue;
        std::memcpy(g_state.colorMask, snapshot->colorMask, sizeof(g_state.colorMask));
        snapshot->font.push(L);
        if (lua_isnil(L, -1))
        {
            g_state.font.clear(L);
        }
        else
        {
            g_state.font.set(L, -1);
        }
        lua_pop(L, 1);
        snapshot->canvas.push(L);
        setCanvasImpl(L, lua_gettop(L));
        lua_pop(L, 1);
        snapshot->shader.push(L);
        setShaderImpl(L, lua_gettop(L));
        lua_pop(L, 1);
        snapshot->font.clear(L);
        snapshot->canvas.clear(L);
        snapshot->shader.clear(L);
        delete snapshot;
        applyBlendMode();
        applyScissor();
        applyStencilTest();
        rlColorMask(g_state.colorMask[0], g_state.colorMask[1], g_state.colorMask[2], g_state.colorMask[3]);
    }
    return 0;
}

int l_translate(lua_State *L)
{
    ensureFrame();
    rlTranslatef(luax::checkfloat(L, 1), luax::checkfloat(L, 2), 0.0f);
    return 0;
}

int l_rotate(lua_State *L)
{
    ensureFrame();
    rlRotatef(luax::checkfloat(L, 1) * RAD2DEG, 0.0f, 0.0f, 1.0f);
    return 0;
}

int l_scale(lua_State *L)
{
    ensureFrame();
    float sx = luax::checkfloat(L, 1);
    float sy = luax::optfloat(L, 2, sx);
    rlScalef(sx, sy, 1.0f);
    return 0;
}

int l_shear(lua_State *L)
{
    ensureFrame();
    Matrix shear = MatrixIdentity();
    shear.m4 = luax::checkfloat(L, 1); // kx
    shear.m1 = luax::checkfloat(L, 2); // ky
    rlMultMatrixf(MatrixToFloat(shear));
    return 0;
}

int l_origin(lua_State *L)
{
    ensureFrame();
    rlLoadIdentity();
    return 0;
}

int l_applyTransform(lua_State *L)
{
    ensureFrame();
    Matrix *m = math::checkTransform(L, 1);
    rlMultMatrixf(MatrixToFloat(*m));
    return 0;
}

int l_replaceTransform(lua_State *L)
{
    ensureFrame();
    Matrix *m = math::checkTransform(L, 1);
    rlLoadIdentity();
    rlMultMatrixf(MatrixToFloat(*m));
    return 0;
}

Matrix currentMatrix()
{
    // rlgl applies the pushed transform stack first, then the modelview.
    return multiply(rlGetMatrixModelview(), rlGetMatrixTransform());
}

int l_transformPoint(lua_State *L)
{
    ensureFrame();
    Vector3 p = {luax::checkfloat(L, 1), luax::checkfloat(L, 2), 0.0f};
    Vector3 r = Vector3Transform(p, currentMatrix());
    lua_pushnumber(L, r.x);
    lua_pushnumber(L, r.y);
    return 2;
}

int l_inverseTransformPoint(lua_State *L)
{
    ensureFrame();
    Vector3 p = {luax::checkfloat(L, 1), luax::checkfloat(L, 2), 0.0f};
    Vector3 r = Vector3Transform(p, MatrixInvert(currentMatrix()));
    lua_pushnumber(L, r.x);
    lua_pushnumber(L, r.y);
    return 2;
}

// ---------------------------------------------------------------------------
// Shapes
// ---------------------------------------------------------------------------

int l_rectangle(lua_State *L)
{
    ensureFrame();
    int mode = checkDrawMode(L, 1);
    Rectangle rec = {luax::checkfloat(L, 2), luax::checkfloat(L, 3), luax::checkfloat(L, 4), luax::checkfloat(L, 5)};
    float rx = luax::optfloat(L, 6, 0.0f);
    float ry = luax::optfloat(L, 7, rx);
    int segments = luax::optint(L, 8, 10);
    Color color = g_state.color;

    if (rx > 0.0f || ry > 0.0f)
    {
        float radius = std::max(rx, ry);
        float minSide = std::min(std::fabs(rec.width), std::fabs(rec.height));
        float roundness = minSide > 0.0f ? std::min(1.0f, radius / (minSide * 0.5f)) : 0.0f;
        if (mode == MODE_FILL)
        {
            DrawRectangleRounded(rec, roundness, segments, color);
        }
        else
        {
            DrawRectangleRoundedLinesEx(rec, roundness, segments, g_state.lineWidth, color);
        }
        return 0;
    }

    if (mode == MODE_FILL)
    {
        DrawRectangleRec(rec, color);
    }
    else
    {
        std::vector<Vector2> pts = {{rec.x, rec.y}, {rec.x + rec.width, rec.y}, {rec.x + rec.width, rec.y + rec.height}, {rec.x, rec.y + rec.height}};
        drawPolyline(pts, true);
    }
    return 0;
}

int l_circle(lua_State *L)
{
    ensureFrame();
    int mode = checkDrawMode(L, 1);
    float x = luax::checkfloat(L, 2);
    float y = luax::checkfloat(L, 3);
    float radius = luax::checkfloat(L, 4);
    int segments = luax::optint(L, 5, defaultSegments(radius));
    if (mode == MODE_FILL)
    {
        DrawCircleSector({x, y}, radius, 0.0f, 360.0f, segments, g_state.color);
    }
    else
    {
        std::vector<Vector2> pts;
        arcPoints(x, y, radius, 0.0f, 2.0f * PI, segments, pts);
        pts.pop_back();
        drawPolyline(pts, true);
    }
    return 0;
}

int l_ellipse(lua_State *L)
{
    ensureFrame();
    int mode = checkDrawMode(L, 1);
    float x = luax::checkfloat(L, 2);
    float y = luax::checkfloat(L, 3);
    float rx = luax::checkfloat(L, 4);
    float ry = luax::optfloat(L, 5, rx);
    int segments = luax::optint(L, 6, defaultSegments(std::max(rx, ry)));
    std::vector<Vector2> pts;
    for (int i = 0; i < segments; ++i)
    {
        float t = 2.0f * PI * static_cast<float>(i) / static_cast<float>(segments);
        pts.push_back({x + std::cos(t) * rx, y + std::sin(t) * ry});
    }
    if (mode == MODE_FILL)
    {
        drawFan(pts);
    }
    else
    {
        drawPolyline(pts, true);
    }
    return 0;
}

int l_arc(lua_State *L)
{
    ensureFrame();
    int mode = checkDrawMode(L, 1);
    int arcType = 0; // 0 pie, 1 open, 2 closed
    int idx = 2;
    if (lua_type(L, 2) == LUA_TSTRING)
    {
        static const char *const names[] = {"pie", "open", "closed"};
        static const int values[] = {0, 1, 2};
        arcType = luax::checkenum(L, 2, names, values, "arc type");
        idx = 3;
    }
    float x = luax::checkfloat(L, idx);
    float y = luax::checkfloat(L, idx + 1);
    float radius = luax::checkfloat(L, idx + 2);
    float a1 = luax::checkfloat(L, idx + 3);
    float a2 = luax::checkfloat(L, idx + 4);
    int segments = luax::optint(L, idx + 5, defaultSegments(radius));
    if (std::fabs(a1 - a2) >= 2.0f * PI)
    {
        a2 = a1 + 2.0f * PI;
    }

    std::vector<Vector2> pts;
    arcPoints(x, y, radius, a1, a2, segments, pts);

    if (mode == MODE_FILL)
    {
        if (arcType == 0)
        {
            pts.insert(pts.begin(), {x, y});
        }
        fillPolygon(L, pts);
        return 0;
    }
    if (arcType == 0)
    {
        pts.insert(pts.begin(), {x, y});
        drawPolyline(pts, true);
    }
    else
    {
        drawPolyline(pts, arcType == 2);
    }
    return 0;
}

int l_polygon(lua_State *L)
{
    ensureFrame();
    int mode = checkDrawMode(L, 1);
    std::vector<Vector2> pts;
    readPoints(L, 2, pts);
    if (pts.size() < 3)
    {
        return luaL_error(L, "Need at least three vertices to draw a polygon");
    }
    if (mode == MODE_FILL)
    {
        fillPolygon(L, pts);
    }
    else
    {
        drawPolyline(pts, true);
    }
    return 0;
}

int l_line(lua_State *L)
{
    ensureFrame();
    std::vector<Vector2> pts;
    readPoints(L, 1, pts);
    if (pts.size() < 2)
    {
        return luaL_error(L, "Need at least two vertices to draw a line");
    }
    drawPolyline(pts, false);
    return 0;
}

int l_points(lua_State *L)
{
    ensureFrame();
    float size = g_state.pointSize;
    Color color = g_state.color;
    // points(x, y, ...) | points({x, y, ...}) | points({{x, y, r, g, b, a}, ...})
    if (lua_istable(L, 1))
    {
        lua_rawgeti(L, 1, 1);
        bool nested = lua_istable(L, -1);
        lua_pop(L, 1);
        if (nested)
        {
            lua_Integer n = luaL_len(L, 1);
            for (lua_Integer i = 1; i <= n; ++i)
            {
                lua_rawgeti(L, 1, i);
                float v[6] = {0, 0, 1, 1, 1, 1};
                for (int k = 0; k < 6; ++k)
                {
                    lua_rawgeti(L, -1, k + 1);
                    if (lua_isnumber(L, -1))
                    {
                        v[k] = static_cast<float>(lua_tonumber(L, -1));
                    }
                    lua_pop(L, 1);
                }
                lua_pop(L, 1);
                Color c = toColor(v[2] * color.r / 255.0f, v[3] * color.g / 255.0f, v[4] * color.b / 255.0f, v[5] * color.a / 255.0f);
                DrawRectangleRec({v[0] - size * 0.5f, v[1] - size * 0.5f, size, size}, c);
            }
            return 0;
        }
    }
    std::vector<Vector2> pts;
    readPoints(L, 1, pts);
    for (const Vector2 &p : pts)
    {
        DrawRectangleRec({p.x - size * 0.5f, p.y - size * 0.5f, size, size}, color);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Drawables
// ---------------------------------------------------------------------------

// Reads the x, y, r, sx, sy, ox, oy, kx, ky arguments (or a Transform).
Matrix readTransformArgs(lua_State *L, int idx)
{
    if (Matrix *t = luax::testobject<Matrix>(L, idx, math::TRANSFORM_TYPE))
    {
        return *t;
    }
    float x = luax::optfloat(L, idx, 0.0f);
    float y = luax::optfloat(L, idx + 1, 0.0f);
    float r = luax::optfloat(L, idx + 2, 0.0f);
    float sx = luax::optfloat(L, idx + 3, 1.0f);
    float sy = luax::optfloat(L, idx + 4, sx);
    float ox = luax::optfloat(L, idx + 5, 0.0f);
    float oy = luax::optfloat(L, idx + 6, 0.0f);
    float kx = luax::optfloat(L, idx + 7, 0.0f);
    float ky = luax::optfloat(L, idx + 8, 0.0f);
    return makeTransform(x, y, r, sx, sy, ox, oy, kx, ky);
}

void drawSpriteBatch(lua_State *L, SpriteBatchObj &batch, const Matrix &transform)
{
    batch.texture.push(L);
    DrawSource src = checkDrawSource(L, lua_gettop(L));
    lua_pop(L, 1);
    ensureFrame();
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloat(transform));
    size_t start = batch.rangeStart >= 0 ? static_cast<size_t>(batch.rangeStart) : 0;
    size_t end = batch.rangeCount >= 0 ? std::min(batch.sprites.size(), start + static_cast<size_t>(batch.rangeCount)) : batch.sprites.size();
    Color tint = g_state.color;
    for (size_t i = start; i < end; ++i)
    {
        const Sprite &s = batch.sprites[i];
        DrawSource part = src;
        part.source = s.source;
        Color c = tint;
        if (s.hasColor)
        {
            c = toColor(s.color.r / 255.0f * tint.r / 255.0f, s.color.g / 255.0f * tint.g / 255.0f,
                        s.color.b / 255.0f * tint.b / 255.0f, s.color.a / 255.0f * tint.a / 255.0f);
        }
        drawTextured(part, s.transform, c);
    }
    rlPopMatrix();
}

void drawTextObject(lua_State *L, TextObj &text, const Matrix &transform)
{
    text.font.push(L);
    FontObj *font = luax::testobject<FontObj>(L, -1, FONT_TYPE);
    if (font == nullptr)
    {
        lua_pop(L, 1);
        font = pushCurrentFont(L);
    }
    ensureFrame();
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloat(transform));
    for (const TextLine &line : text.lines)
    {
        Matrix m = line.hasTransform ? line.transform : MatrixIdentity();
        lua_pushlstring(L, line.text.data(), line.text.size());
        printText(L, *font, lua_gettop(L), line.x, line.y, line.limit, line.align.c_str(), m);
        lua_pop(L, 1);
    }
    rlPopMatrix();
    lua_pop(L, 1);
}

int l_draw(lua_State *L)
{
    if (SpriteBatchObj *batch = luax::testobject<SpriteBatchObj>(L, 1, SPRITEBATCH_TYPE))
    {
        drawSpriteBatch(L, *batch, readTransformArgs(L, 2));
        return 0;
    }
    if (TextObj *text = luax::testobject<TextObj>(L, 1, TEXT_TYPE))
    {
        drawTextObject(L, *text, readTransformArgs(L, 2));
        return 0;
    }
    if (ParticleSystemObj *particles = luax::testobject<ParticleSystemObj>(L, 1, PARTICLES_TYPE))
    {
        drawParticleSystem(L, *particles, readTransformArgs(L, 2));
        return 0;
    }
    if (MeshObj *mesh = luax::testobject<MeshObj>(L, 1, MESH_TYPE))
    {
        drawMesh(L, *mesh, readTransformArgs(L, 2));
        return 0;
    }

    DrawSource src = checkDrawSource(L, 1);
    int argIndex = 2;
    if (QuadObj *quad = luax::testobject<QuadObj>(L, 2, QUAD_TYPE))
    {
        // Quads are defined against reference dimensions, map them onto the
        // actual texture size.
        float scaleX = src.texture.width / quad->sw;
        float scaleY = src.texture.height / quad->sh;
        src.source = {quad->x * scaleX, quad->y * scaleY, quad->w * scaleX, quad->h * scaleY};
        argIndex = 3;
    }
    Matrix transform = readTransformArgs(L, argIndex);
    drawTextured(src, transform, g_state.color);
    return 0;
}

// ---------------------------------------------------------------------------
// Text
// ---------------------------------------------------------------------------

int l_print(lua_State *L)
{
    FontObj *font = pushCurrentFont(L);
    lua_pop(L, 1);
    Matrix transform = readTransformArgs(L, 2);
    printText(L, *font, 1, 0.0f, 0.0f, 0.0f, "left", transform);
    return 0;
}

int l_printf(lua_State *L)
{
    FontObj *font = pushCurrentFont(L);
    lua_pop(L, 1);
    const char *align = "left";
    Matrix transform;
    float limit;
    if (Matrix *t = luax::testobject<Matrix>(L, 2, math::TRANSFORM_TYPE))
    {
        // printf(text, transform, limit, align)
        transform = *t;
        limit = luax::checkfloat(L, 3);
        align = luaL_optstring(L, 4, "left");
    }
    else
    {
        float x = luax::checkfloat(L, 2);
        float y = luax::checkfloat(L, 3);
        limit = luax::checkfloat(L, 4);
        align = luaL_optstring(L, 5, "left");
        float r = luax::optfloat(L, 6, 0.0f);
        float sx = luax::optfloat(L, 7, 1.0f);
        float sy = luax::optfloat(L, 8, sx);
        float ox = luax::optfloat(L, 9, 0.0f);
        float oy = luax::optfloat(L, 10, 0.0f);
        float kx = luax::optfloat(L, 11, 0.0f);
        float ky = luax::optfloat(L, 12, 0.0f);
        transform = makeTransform(x, y, r, sx, sy, ox, oy, kx, ky);
    }
    static const char *const names[] = {"left", "center", "right", "justify"};
    static const int values[] = {0, 1, 2, 3};
    lua_pushstring(L, align);
    luax::checkenum(L, lua_gettop(L), names, values, "alignment");
    lua_pop(L, 1);
    printText(L, *font, 1, 0.0f, 0.0f, limit, align, transform);
    return 0;
}

// ---------------------------------------------------------------------------
// Canvas binding
// ---------------------------------------------------------------------------

int setCanvasImpl(lua_State *L, int idx)
{
    ensureFrame();
    CanvasObj *next = nullptr;
    if (!lua_isnoneornil(L, idx))
    {
        if (lua_istable(L, idx))
        {
            lua_rawgeti(L, idx, 1);
            next = luax::checkobject<CanvasObj>(L, -1, CANVAS_TYPE);
            lua_pop(L, 1);
            lua_rawgeti(L, idx, 2);
            if (!lua_isnil(L, -1))
            {
                lua_pop(L, 1);
                return luaL_error(L, "Multi-canvas rendering is not supported");
            }
            lua_pop(L, 1);
        }
        else
        {
            next = luax::checkobject<CanvasObj>(L, idx, CANVAS_TYPE);
        }
    }
    CanvasObj *current = activeCanvas(L);
    if (current == next)
    {
        return 0;
    }
    // raylib resets the modelview when switching render targets; Love2D keeps
    // the transform stack, so save and restore it around the switch.
    Matrix modelview = rlGetMatrixModelview();
    if (current != nullptr)
    {
        EndTextureMode();
    }
    if (next != nullptr)
    {
        BeginTextureMode(next->target);
        if (lua_istable(L, idx))
        {
            lua_rawgeti(L, idx, 1);
            g_state.canvas.set(L, -1);
            lua_pop(L, 1);
        }
        else
        {
            g_state.canvas.set(L, idx);
        }
    }
    else
    {
        g_state.canvas.clear(L);
    }
    rlSetMatrixModelview(modelview);
    applyScissor();
    if (g_shader != nullptr)
    {
        int width, height;
        currentTargetSize(L, width, height);
        shaderSetTargetSize(g_shader, width, height);
    }
    return 0;
}

int setShaderImpl(lua_State *L, int idx)
{
    ensureFrame();
    ShaderObj *next = lua_isnoneornil(L, idx) ? nullptr : checkShader(L, idx);
    if (next == g_shader)
    {
        return 0;
    }
    if (next == nullptr)
    {
        shaderDeactivate();
        g_shader = nullptr;
        g_state.shader.clear(L);
        return 0;
    }
    int width, height;
    currentTargetSize(L, width, height);
    shaderActivate(next, width, height);
    g_shader = next;
    g_state.shader.set(L, idx);
    return 0;
}

int l_setShader(lua_State *L)
{
    return setShaderImpl(L, 1);
}

int l_getShader(lua_State *L)
{
    g_state.shader.push(L);
    return 1;
}

int l_setCanvas(lua_State *L)
{
    return setCanvasImpl(L, 1);
}

int l_getCanvas(lua_State *L)
{
    g_state.canvas.push(L);
    return 1;
}

// ---------------------------------------------------------------------------
// Information
// ---------------------------------------------------------------------------

int l_getRendererInfo(lua_State *L)
{
    lua_pushstring(L, "OpenGL");
    int version = rlGetVersion();
    const char *versionName = "3.3";
    switch (version)
    {
    case RL_OPENGL_11:
        versionName = "1.1";
        break;
    case RL_OPENGL_21:
        versionName = "2.1";
        break;
    case RL_OPENGL_43:
        versionName = "4.3";
        break;
    case RL_OPENGL_ES_20:
        versionName = "ES 2.0";
        break;
    case RL_OPENGL_ES_30:
        versionName = "ES 3.0";
        break;
    default:
        break;
    }
    lua_pushstring(L, versionName);
    lua_pushstring(L, "raylib");
    lua_pushstring(L, RAYLIB_VERSION);
    return 4;
}

int l_getStats(lua_State *L)
{
    lua_newtable(L);
    lua_pushinteger(L, 0);
    lua_setfield(L, -2, "drawcalls");
    lua_pushinteger(L, 0);
    lua_setfield(L, -2, "canvasswitches");
    lua_pushinteger(L, 0);
    lua_setfield(L, -2, "texturememory");
    lua_pushinteger(L, 0);
    lua_setfield(L, -2, "images");
    lua_pushinteger(L, 0);
    lua_setfield(L, -2, "canvases");
    lua_pushinteger(L, 0);
    lua_setfield(L, -2, "fonts");
    lua_pushinteger(L, 0);
    lua_setfield(L, -2, "shaderswitches");
    lua_pushinteger(L, 0);
    lua_setfield(L, -2, "drawcallsbatched");
    return 1;
}

int l_getSupported(lua_State *L)
{
    lua_newtable(L);
    const char *yes[] = {"canvas", "clampzero", "lighten", "multicanvasformats", "fullnpot"};
    const char *no[] = {"glsl3", "instancing", "pixelshaderhighp", "shaderderivatives", "multicanvas", "glsl4"};
    for (const char *name : yes)
    {
        lua_pushboolean(L, 1);
        lua_setfield(L, -2, name);
    }
    for (const char *name : no)
    {
        lua_pushboolean(L, 0);
        lua_setfield(L, -2, name);
    }
    return 1;
}

int l_getSystemLimits(lua_State *L)
{
    lua_newtable(L);
    lua_pushinteger(L, 1);
    lua_setfield(L, -2, "pointsize");
    lua_pushinteger(L, 8192);
    lua_setfield(L, -2, "texturesize");
    lua_pushinteger(L, 1);
    lua_setfield(L, -2, "multicanvas");
    lua_pushinteger(L, 4);
    lua_setfield(L, -2, "canvasmsaa");
    lua_pushinteger(L, 8);
    lua_setfield(L, -2, "texturelayers");
    lua_pushinteger(L, 16);
    lua_setfield(L, -2, "anisotropy");
    return 1;
}

int l_getCanvasFormats(lua_State *L)
{
    lua_newtable(L);
    lua_pushboolean(L, 1);
    lua_setfield(L, -2, "rgba8");
    lua_pushboolean(L, 1);
    lua_setfield(L, -2, "normal");
    return 1;
}

int l_getImageFormats(lua_State *L)
{
    lua_newtable(L);
    lua_pushboolean(L, 1);
    lua_setfield(L, -2, "rgba8");
    return 1;
}

int l_isGammaCorrect(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

// Features that depend on shaders / stencil are not available on top of the
// raylib batch renderer. They fail with a clear message instead of silently
// drawing nothing.
int l_unsupported(lua_State *L)
{
    const char *name = lua_tostring(L, lua_upvalueindex(1));
    return luaL_error(L, "love.graphics.%s is not supported by LoveRay yet", name);
}

// stencil(fn, action = "replace", value = 1, keepvalues = false)
int l_stencil(lua_State *L)
{
    luaL_checktype(L, 1, LUA_TFUNCTION);
    int action = lua_isnoneornil(L, 2) ? STENCIL_ACTION_VALUES[0]
                                       : luax::checkenum(L, 2, STENCIL_ACTION_NAMES, STENCIL_ACTION_VALUES, "stencil action");
    int value = luax::optint(L, 3, 1);
    bool keep = luax::optboolean(L, 4, false);
    if (value < 0 || value > 255)
    {
        return luaL_error(L, "Stencil value must be between 0 and 255");
    }

    ensureFrame();
    rlDrawRenderBatchActive();
    if (!keep)
    {
        rlClearStencil(0);
    }
    rlEnableStencilTest();
    rlStencilFunc(GL_ALWAYS_FUNC, value, 0xFF);
    rlStencilOp(GL_KEEP_OP, GL_KEEP_OP, action);
    rlStencilMask(0xFF);
    rlColorMask(false, false, false, false);

    lua_pushvalue(L, 1);
    int status = lua_pcall(L, 0, 0, 0);

    rlDrawRenderBatchActive();
    rlColorMask(g_state.colorMask[0], g_state.colorMask[1], g_state.colorMask[2], g_state.colorMask[3]);
    applyStencilTest();
    if (status != LUA_OK)
    {
        return lua_error(L);
    }
    return 0;
}

// setStencilTest() disables the test, setStencilTest(comparemode, comparevalue) enables it.
int l_setStencilTest(lua_State *L)
{
    int mode = 0;
    int value = 0;
    if (!lua_isnoneornil(L, 1))
    {
        mode = luax::checkenum(L, 1, STENCIL_NAMES, STENCIL_VALUES, "compare mode");
        value = static_cast<int>(luaL_checkinteger(L, 2));
        if (value < 0 || value > 255)
        {
            return luaL_error(L, "Stencil test value must be between 0 and 255");
        }
    }
    g_state.stencilMode = mode;
    g_state.stencilValue = value;
    if (g_frameActive)
    {
        applyStencilTest();
    }
    return 0;
}

int l_getStencilTest(lua_State *L)
{
    lua_pushstring(L, STENCIL_NAMES[g_state.stencilMode]);
    lua_pushinteger(L, g_state.stencilValue);
    return 2;
}

int l_noop(lua_State *L)
{
    return 0;
}

int l_getDepthMode(lua_State *L)
{
    lua_pushstring(L, "always");
    lua_pushboolean(L, 0);
    return 2;
}

int l_getMeshCullMode(lua_State *L)
{
    lua_pushstring(L, "none");
    return 1;
}

int l_getFrontFaceWinding(lua_State *L)
{
    lua_pushstring(L, "ccw");
    return 1;
}

const luaL_Reg FUNCS[] = {
    // state
    {"setColor", l_setColor},
    {"getColor", l_getColor},
    {"setBackgroundColor", l_setBackgroundColor},
    {"getBackgroundColor", l_getBackgroundColor},
    {"setLineWidth", l_setLineWidth},
    {"getLineWidth", l_getLineWidth},
    {"setLineStyle", l_setLineStyle},
    {"getLineStyle", l_getLineStyle},
    {"setLineJoin", l_setLineJoin},
    {"getLineJoin", l_getLineJoin},
    {"setPointSize", l_setPointSize},
    {"getPointSize", l_getPointSize},
    {"setBlendMode", l_setBlendMode},
    {"getBlendMode", l_getBlendMode},
    {"setScissor", l_setScissor},
    {"intersectScissor", l_intersectScissor},
    {"getScissor", l_getScissor},
    {"setColorMask", l_setColorMask},
    {"getColorMask", l_getColorMask},
    {"setWireframe", l_setWireframe},
    {"isWireframe", l_isWireframe},
    {"setDefaultFilter", l_setDefaultFilter},
    {"getDefaultFilter", l_getDefaultFilter},
    {"reset", l_reset},
    // frame
    {"clear", l_clear},
    {"discard", l_discard},
    {"present", l_present},
    {"isActive", l_isActive},
    {"isCreated", l_isCreated},
    {"getWidth", l_getWidth},
    {"getHeight", l_getHeight},
    {"getDimensions", l_getDimensions},
    {"getPixelWidth", l_getPixelWidth},
    {"getPixelHeight", l_getPixelHeight},
    {"getPixelDimensions", l_getPixelDimensions},
    {"getDPIScale", l_getDPIScale},
    {"captureScreenshot", l_captureScreenshot},
    // transform
    {"push", l_push},
    {"pop", l_pop},
    {"translate", l_translate},
    {"rotate", l_rotate},
    {"scale", l_scale},
    {"shear", l_shear},
    {"origin", l_origin},
    {"applyTransform", l_applyTransform},
    {"replaceTransform", l_replaceTransform},
    {"transformPoint", l_transformPoint},
    {"inverseTransformPoint", l_inverseTransformPoint},
    // shapes
    {"rectangle", l_rectangle},
    {"circle", l_circle},
    {"ellipse", l_ellipse},
    {"arc", l_arc},
    {"polygon", l_polygon},
    {"line", l_line},
    {"points", l_points},
    // drawables and text
    {"draw", l_draw},
    {"print", l_print},
    {"printf", l_printf},
    {"setCanvas", l_setCanvas},
    {"getCanvas", l_getCanvas},
    // info
    {"getRendererInfo", l_getRendererInfo},
    {"getStats", l_getStats},
    {"getSupported", l_getSupported},
    {"getSystemLimits", l_getSystemLimits},
    {"getCanvasFormats", l_getCanvasFormats},
    {"getImageFormats", l_getImageFormats},
    {"isGammaCorrect", l_isGammaCorrect},
    // partial / unsupported
    {"stencil", l_stencil},
    {"setStencilTest", l_setStencilTest},
    {"getStencilTest", l_getStencilTest},
    {"setDepthMode", l_noop},
    {"getDepthMode", l_getDepthMode},
    {"setMeshCullMode", l_noop},
    {"getMeshCullMode", l_getMeshCullMode},
    {"setFrontFaceWinding", l_noop},
    {"getFrontFaceWinding", l_getFrontFaceWinding},
    {"setShader", l_setShader},
    {"getShader", l_getShader},
    {"flushBatch", l_noop},
    {nullptr, nullptr},
};

} // namespace
} // namespace graphics

int open_graphics(lua_State *L)
{
    using namespace graphics;
    registerObjectTypes(L);
    luaL_newlib(L, FUNCS);
    luaL_setfuncs(L, OBJECT_FUNCS, 0);

    luaL_setfuncs(L, SHADER_FUNCS, 0);
    luaL_setfuncs(L, PARTICLE_FUNCS, 0);
    luaL_setfuncs(L, MESH_FUNCS, 0);
    const char *unsupported[] = {"newVideo", "newArrayImage", "newCubeImage",
                                 "newVolumeImage"};
    for (const char *name : unsupported)
    {
        lua_pushstring(L, name);
        lua_pushcclosure(L, l_unsupported, 1);
        lua_setfield(L, -2, name);
    }
    return 1;
}

} // namespace love
