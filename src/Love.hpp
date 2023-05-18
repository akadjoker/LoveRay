#ifndef LOVE_H
#define LOVE_H

#include <string>
#include <vector>
#include <map>
#include <lua.hpp>
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#define Push_int(L, value) lua_pushinteger(L, value)
#define Push_float(L, value) lua_pushnumber(L, value);
#define Push_double(L, value) lua_pushnumber(L, value);
#define Push_bool(L, value) lua_pushboolean(L, value)
#define Push_string(L, value) lua_pushstring(L, value)

#define Get_ptr (void *)luaL_checkinteger
#define Get_int (int)luaL_checkinteger
#define Get_bool (bool)lua_toboolean
#define Get_unsigned (unsigned)luaL_checkinteger
#define Get_char (char)luaL_checkinteger
#define Get_float (float)luaL_checknumber
#define Get_double luaL_checknumber
#define Get_string luaL_checkstring

#define CONSOLE_COLOR_RESET "\033[0m"
#define CONSOLE_COLOR_GREEN "\033[1;32m"
#define CONSOLE_COLOR_RED "\033[1;31m"
#define CONSOLE_COLOR_PURPLE "\033[1;35m"
#define CONSOLE_COLOR_CYAN "\033[0;36m"
#define LOVE_VERSION "0.0.1"

inline void Log(int severity, const char *fmt, ...)
{
    /* Determine strings for the type and colour */
    const char *type;
    const char *color;
    switch (severity)
    {
    case LOG_INFO:
        type = "info";
        color = CONSOLE_COLOR_GREEN;
        break;
    case LOG_ERROR:
        type = "error";
        color = CONSOLE_COLOR_RED;
        break;
    case LOG_WARNING:
        type = "warning";
        color = CONSOLE_COLOR_PURPLE;
        break;
    default:
        break; /* Unreachable */
    }

    /* Obtain the current date and time */
    time_t rawTime;
    struct tm *timeInfo;
    char timeBuffer[80];

    time(&rawTime);
    timeInfo = localtime(&rawTime);

    strftime(timeBuffer, sizeof(timeBuffer), "[%H:%M:%S]", timeInfo);

    /* Format for printing to the console (with colours) */
    char consoleFormat[1024];
    snprintf(consoleFormat, 1024, "%s%s %s%s%s: %s\n", CONSOLE_COLOR_CYAN,
             timeBuffer, color, type, CONSOLE_COLOR_RESET, fmt);

    va_list argptr;

    /* Print to the console */
    va_start(argptr, fmt);
    vprintf(consoleFormat, argptr);
    va_end(argptr);
}

typedef struct
{
    uint32_t type;
} luaobj_head_t;

struct lQuad
{

    lQuad()
    {
        x = 0;
        y = 0;
        width = 1;
        height = 1;
        sw = 0;
        sh = 0;
    }
    lQuad(float x, float y, float width, float height, float sw, float sh)
    {
        this->x = x;
        this->y = y;
        this->width = width;
        this->height = height;
        this->sw = sw;
        this->sh = sh;
    }
    ~lQuad()
    {
        Log(LOG_INFO, "Quad unloaded");
    }

    float x;
    float y;
    float width;
    float height;
    float sw;
    float sh;
};
class MatrixStack
{
public:
    MatrixStack();

    void push();
    void pop();
    void rotate(float radians);
    void translate(float sx, float sy);
    void scale(float sx, float sy);
    void shear(float kx, float ky);
    void applyMatrix(const Matrix &matrix);
    void applyTransformations() const;

private:
    std::vector<Matrix> transformationStack;
    Matrix currentTransformation;
};

void CloseLove(lua_State *L);
void CreateLove(lua_State *L);
#endif

#ifdef LOVE_IMPLEMENTATION

#include "Physics.hpp"
#include <string>
#include <cstring>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <map>

MatrixStack::MatrixStack()
{
    currentTransformation = MatrixIdentity();
}

void MatrixStack::push()
{
    transformationStack.push_back(currentTransformation);
}

void MatrixStack::pop()
{

    if (!transformationStack.empty())
    {
        currentTransformation = transformationStack.back();
        transformationStack.pop_back();
    }
    rlPopMatrix();
}

void MatrixStack::rotate(float radians)
{
    currentTransformation = MatrixMultiply(currentTransformation, MatrixRotateZ(radians));
}

void MatrixStack::scale(float sx, float sy)
{
    currentTransformation = MatrixMultiply(currentTransformation, MatrixScale(sx, sy, 1.0f));
}

void MatrixStack::translate(float sx, float sy)
{
    currentTransformation = MatrixMultiply(currentTransformation, MatrixTranslate(sx, sy, 0.0f));
}

void MatrixStack::shear(float kx, float ky)
{
    Matrix shearMatrix = {
        1, kx, 0, 0,
        ky, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1};

    currentTransformation = MatrixMultiply(currentTransformation, shearMatrix);
}

void MatrixStack::applyMatrix(const Matrix &matrix)
{
    currentTransformation = MatrixMultiply(currentTransformation, matrix);
}
void MatrixStack::applyTransformations() const
{
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloat(currentTransformation));
}

static Color _color;
static Color _backgroundColor;
static MatrixStack matrixStack;
static int screenWidth = 800;
static int screenHeight = 600;
static int screenFps = 60;
static std::string screenTitle;
static bool screenVsync = false;
static bool screenFullscreen = false;
static bool screenBorderless = false;
static bool screenResizable = true;
static bool screenInit = false;
static std::map<std::string, int> _keys;
namespace nGraphics
{

#define FIX_ARTIFACTS_BY_STRECHING_TEXEL true

    typedef struct rVertex
    {
        float x, y, z;
        Color col;
        float tx, ty;

    } rVertex;

    typedef struct rQuad
    {
        rVertex v[4];
        Texture2D tex;
        int blend;
    } rQuad;

    void RenderQuadMatrix(const rQuad *quad, const Matrix *mat)
    {

        rlCheckRenderBatchLimit(4); // Make sure there is enough free space on the batch buffer
        rlSetTexture(quad->tex.id);

        matrixStack.push();

        matrixStack.applyMatrix(*mat);
        matrixStack.applyTransformations();

        rlBegin(RL_QUADS);

        Color a = quad->v[1].col;
        Color b = quad->v[0].col;
        Color c = quad->v[3].col;
        Color d = quad->v[2].col;

        rlNormal3f(0.0f, 0.0f, 1.0f);

        rlColor4ub(a.r, a.g, a.b, a.a);
        rlTexCoord2f(quad->v[1].tx, quad->v[1].ty);
        rlVertex3f(quad->v[1].x, quad->v[1].y, quad->v[1].z);

        rlColor4ub(b.r, b.g, b.b, b.a);
        rlTexCoord2f(quad->v[0].tx, quad->v[0].ty);
        rlVertex3f(quad->v[0].x, quad->v[0].y, quad->v[0].z);

        rlColor4ub(c.r, c.g, c.b, c.a);
        rlTexCoord2f(quad->v[3].tx, quad->v[3].ty);
        rlVertex3f(quad->v[3].x, quad->v[3].y, quad->v[3].z);

        rlColor4ub(d.r, d.g, d.b, d.a);
        rlTexCoord2f(quad->v[2].tx, quad->v[2].ty);
        rlVertex3f(quad->v[2].x, quad->v[2].y, quad->v[2].z);

        rlEnd();

        matrixStack.pop();
    }

    void RenderClipFlip(Texture2D texture, int width, int height, Rectangle clip, bool flipX, bool flipY, Color color, const Matrix *matrix, int blend)
    {

        rQuad quad;
        quad.tex = texture;
        quad.blend = blend;

        int widthTex = texture.width;
        int heightTex = texture.height;

        float left;
        float right;
        float top;
        float bottom;

        if (FIX_ARTIFACTS_BY_STRECHING_TEXEL)
        {
            left = (2 * clip.x + 1) / (2 * widthTex);
            right = left + (clip.width * 2 - 2) / (2 * widthTex);
            top = (2 * clip.y + 1) / (2 * heightTex);
            bottom = top + (clip.height * 2 - 2) / (2 * heightTex);
        }
        else
        {
            left = clip.x / widthTex;
            right = (clip.x + clip.width) / widthTex;
            top = clip.y / heightTex;
            bottom = (clip.y + clip.height) / heightTex;
        }

        if (flipX)
        {
            float tmp = left;
            left = right;
            right = tmp;
        }

        if (flipY)
        {
            float tmp = top;
            top = bottom;
            bottom = tmp;
        }

        float TempX1 = 0;
        float TempY1 = 0;
        float TempX2 = width;
        float TempY2 = height;

        quad.v[1].x = TempX1;
        quad.v[1].y = TempY1;
        quad.v[1].tx = left;
        quad.v[1].ty = top;

        quad.v[0].x = TempX1;
        quad.v[0].y = TempY2;
        quad.v[0].tx = left;
        quad.v[0].ty = bottom;

        quad.v[3].x = TempX2;
        quad.v[3].y = TempY2;
        quad.v[3].tx = right;
        quad.v[3].ty = bottom;

        quad.v[2].x = TempX2;
        quad.v[2].y = TempY1;
        quad.v[2].tx = right;
        quad.v[2].ty = top;

        quad.v[0].z = quad.v[1].z = quad.v[2].z = quad.v[3].z = 0.0f;
        quad.v[0].col = quad.v[1].col = quad.v[2].col = quad.v[3].col = color;

        RenderQuadMatrix(&quad, matrix);
    }

    static Matrix e = MatrixIdentity();
    Matrix setTransformation(float x, float y, float angle, float sx, float sy, float ox, float oy, float kx, float ky)
    {
        // Matrix e = MatrixIdentity();
        float c = cosf(angle), s = sinf(angle);
        // matrix multiplication carried out on paper:
        // |1     x| |c -s    | |sx       | | 1 ky    | |1     -ox|
        // |  1   y| |s  c    | |   sy    | |kx  1    | |  1   -oy|
        // |    1  | |     1  | |      1  | |      1  | |    1    |
        // |      1| |       1| |        1| |        1| |       1 |
        //   move      rotate      scale       skew       origin
        e.m10 = e.m15 = 1.0f;
        e.m0 = c * sx - ky * s * sy; // = a
        e.m1 = s * sx + ky * c * sy; // = b
        e.m4 = kx * c * sx - s * sy; // = c
        e.m5 = kx * s * sx + c * sy; // = d
        e.m12 = x - ox * e.m0 - oy * e.m4;
        e.m13 = y - ox * e.m1 - oy * e.m5;
        return e;
    }

    static int love_graphics_setBackgroundColor(lua_State *L)
    {
        if (lua_gettop(L) < 3)
        {
            return luaL_error(L, "setBackgroundColor function requires 3/4 arguments");
        }

        float r = (float)luaL_checknumber(L, 1);
        float g = (float)luaL_checknumber(L, 2);
        float b = (float)luaL_checknumber(L, 3);

        if (lua_gettop(L) == 4)
        {
            _backgroundColor.a = (float)luaL_checknumber(L, 4) * 255;
        }

        _backgroundColor.r = r * 255;
        _backgroundColor.g = g * 255;
        _backgroundColor.b = b * 255;

        return 0;
    }

    static int love_graphics_getWidth(lua_State *L)
    {
        lua_pushnumber(L, GetScreenWidth());
        return 1;
    }

    static int love_graphics_getHeight(lua_State *L)
    {
        lua_pushnumber(L, GetScreenHeight());
        return 1;
    }

    static int love_graphics_print(lua_State *L)
    {
        if (lua_gettop(L) < 3)
        {
            return luaL_error(L, "print function requires 3 arguments");
        }

        const char *text = luaL_checkstring(L, 1);
        float x = (float)luaL_checknumber(L, 2);
        float y = (float)luaL_checknumber(L, 3);
        int size = 20;
        if (lua_gettop(L) == 4)
        {
            size = (int)luaL_checknumber(L, 4);
        }
        matrixStack.push();
        matrixStack.applyTransformations();
        DrawText(text, x, y, size, _color);
        matrixStack.pop();
        return 0;
    }

    static int love_graphics_rectangle(lua_State *L)
    {
        if (lua_gettop(L) < 5)
        {
            return luaL_error(L, "rectangle function requires 5 arguments");
        }

        const char *fill = luaL_checkstring(L, 1);
        float x = (float)luaL_checknumber(L, 2);
        float y = (float)luaL_checknumber(L, 3);
        float width = (float)luaL_checknumber(L, 4);
        float height = (float)luaL_checknumber(L, 5);
        matrixStack.push();
        matrixStack.applyTransformations();
        if (strcmp(fill, "fill") == 0)
        {
            DrawRectangle(x, y, width, height, _color);
        }
        else
        {
            DrawRectangleLines(x, y, width, height, _color);
        }
        matrixStack.pop();
        return 0;
    }

    static int love_graphics_circle(lua_State *L)
    {
        if (lua_gettop(L) < 4)
        {
            return luaL_error(L, "circle function requires 4 arguments");
        }

        const char *fill = luaL_checkstring(L, 1);
        float x = (float)luaL_checknumber(L, 2);
        float y = (float)luaL_checknumber(L, 3);
        float radius = (float)luaL_checknumber(L, 4);
        matrixStack.push();
        matrixStack.applyTransformations();
        if (strcmp(fill, "fill") == 0)
        {
            DrawCircle(x, y, radius, _color);
        }
        else
        {
            DrawCircleLines(x, y, radius, _color);
        }
        matrixStack.pop();
        return 0;
    }

    static int love_graphics_line(lua_State *L)
    {
        if (lua_gettop(L) < 4)
        {
            return luaL_error(L, "line function requires 4 arguments");
        }

        float x1 = (float)luaL_checknumber(L, 1);
        float y1 = (float)luaL_checknumber(L, 2);
        float x2 = (float)luaL_checknumber(L, 3);
        float y2 = (float)luaL_checknumber(L, 4);
        matrixStack.push();
        matrixStack.applyTransformations();

        DrawLine(x1, y1, x2, y2, _color);
        matrixStack.pop();

        return 0;
    }

    static int love_graphics_setColor(lua_State *L)
    {
        if (lua_gettop(L) < 3)
        {
            return luaL_error(L, "setColor function requires 3 arguments");
        }

        float r = (float)luaL_checknumber(L, 1);
        float g = (float)luaL_checknumber(L, 2);
        float b = (float)luaL_checknumber(L, 3);

        if (lua_gettop(L) == 4)
        {
            _color.a = (float)luaL_checknumber(L, 4) * 255;
        }

        _color.r = r * 255;
        _color.g = g * 255;
        _color.b = b * 255;

        return 0;
    }
    // 40201  32
    static Matrix mat;
    static Rectangle source;

    static int love_graphics_draw(lua_State *L)
    {

        if (lua_gettop(L) < 3)
        {
            return luaL_error(L, "draw function requires 3+ arguments");
        }

        Texture2D *image_ptr = (Texture2D *)luaL_checkudata(L, 1, "Image");
        if (image_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Image object");
        }
        Texture2D image = (*image_ptr);

        Rectangle *quad_ptr = (Rectangle *)luaL_testudata(L, 2, "Quad");
        float x = 0;
        float y = 0;
        float angle = 0;
        float sx = 1;
        float sy = 1;
        float ox = 0;
        float oy = 0;
        float kx = 0;
        float ky = 0;

        if (quad_ptr != NULL)
        {

            if (quad_ptr == nullptr)
            {
                return luaL_error(L, "Invalid Quad object");
            }
            Rectangle source = (*(quad_ptr));

            x = (float)luaL_checknumber(L, 3);
            y = (float)luaL_checknumber(L, 4);

            if (lua_gettop(L) > 4)
            {
                if (lua_gettop(L) < 11)
                {
                    return luaL_error(L, "draw image quad function requires 11 arguments");
                }
                angle = (float)luaL_checknumber(L, 5);
                sx = (float)luaL_checknumber(L, 6);
                sy = (float)luaL_checknumber(L, 7);
                ox = (float)luaL_checknumber(L, 8);
                oy = (float)luaL_checknumber(L, 9);

                kx = (float)luaL_checknumber(L, 10);
                ky = (float)luaL_checknumber(L, 11);

                mat = setTransformation(x, y, angle, sx, sy, ox, oy, kx, ky);
                RenderClipFlip(image, source.width, source.height, source, false, false, _color, &mat, 0);

                return 0;
            }

            mat = setTransformation(x, y, 0, 1, 1, 0, 0, 0, 0);
            RenderClipFlip(image, source.width, source.height, source, false, false, _color, &mat, 0);
        }
        else
        {
            x = (float)luaL_checknumber(L, 2);
            y = (float)luaL_checknumber(L, 3);

            if (lua_gettop(L) > 3)
            {
                if (lua_gettop(L) < 10)
                {
                    return luaL_error(L, "draw image function requires 10 arguments");
                }
                angle = (float)luaL_checknumber(L, 4);
                sx = (float)luaL_checknumber(L, 5);
                sy = (float)luaL_checknumber(L, 6);
                ox = (float)luaL_checknumber(L, 7);
                oy = (float)luaL_checknumber(L, 8);
                kx = (float)luaL_checknumber(L, 9);
                ky = (float)luaL_checknumber(L, 10);

                // Rectangle source;
                source.x = 0;
                source.y = 0;
                source.width = image.width;
                source.height = image.height;
                mat = setTransformation(x, y, angle, sx, sy, ox, oy, kx, ky);
                RenderClipFlip(image, source.width, source.height, source, false, false, _color, &mat, 0);

                return 0;
            }

            /// DrawTexture(image, x, y, _color);
            source.x = 0;
            source.y = 0;
            source.width = image.width;
            source.height = image.height;
            mat = setTransformation(x, y, 0, 1, 1, 0, 0, 0, 0);
            RenderClipFlip(image, source.width, source.height, source, false, false, _color, &mat, 0);
        }

        return 0;
    }

    static int love_graphics_point(lua_State *L)
    {
        if (lua_gettop(L) < 2)
        {
            return luaL_error(L, "point function requires 2 arguments");
        }

        float x = (float)luaL_checknumber(L, 1);
        float y = (float)luaL_checknumber(L, 2);

        DrawPixel(x, y, _color);

        return 0;
    }

    static int love_graphics_clear(lua_State *L)
    {

        (void)L;
        BeginDrawing();
        ClearBackground(_backgroundColor);

        return 0;
    }

    static int love_graphics_begin(lua_State *L)
    {
        (void)L;
        return 0;
    }
    static int love_graphics_end(lua_State *L)
    {
        (void)L;
        EndDrawing();
        return 0;
    }

    static int love_graphics_getFont(lua_State *L)
    {
        Font *userdata = (Font *)lua_newuserdata(L, sizeof(Font));
        *userdata = GetFontDefault();
        luaL_getmetatable(L, "Font");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int love_graphics_push(lua_State *L)
    {
        (void)L;

        rlLoadIdentity();          // Reset current matrix (modelview)
        rlDrawRenderBatchActive(); // Update and draw internal render batch
        matrixStack.push();
        matrixStack.applyTransformations();

        return 0;
    }

    static int love_graphics_pop(lua_State *L)
    {
        (void)L;
        rlDrawRenderBatchActive(); // Update and draw internal render batch
        rlLoadIdentity();          // Reset current matrix (modelview)
        matrixStack.pop();
        return 0;
    }

    static int love_graphics_translate(lua_State *L)
    {
        if (lua_gettop(L) < 2)
        {
            return luaL_error(L, "translate function requires 2 arguments");
        }

        float x = (float)luaL_checknumber(L, 1);
        float y = (float)luaL_checknumber(L, 2);

        matrixStack.translate(x, y);
        return 0;
    }

    static int love_graphics_rotate(lua_State *L)
    {
        if (lua_gettop(L) < 1)
        {
            return luaL_error(L, "rotate function requires 1 argument");
        }

        float angle = (float)luaL_checknumber(L, 1);

        matrixStack.rotate(angle);
        return 0;
    }

    static int love_graphics_scale(lua_State *L)
    {
        if (lua_gettop(L) < 2)
        {
            return luaL_error(L, "scale function requires 2 arguments");
        }

        float x = (float)luaL_checknumber(L, 1);
        float y = (float)luaL_checknumber(L, 2);

        matrixStack.scale(x, y);
        return 0;
    }

    static int love_graphics_shear(lua_State *L)
    {
        if (lua_gettop(L) < 2)
        {
            return luaL_error(L, "shear function requires 2 arguments");
        }

        float x = (float)luaL_checknumber(L, 1);
        float y = (float)luaL_checknumber(L, 2);
        matrixStack.shear(x, y);

        return 0;
    }

}

namespace nQuad
{

    static int newQuad(lua_State *L)
    {

        float x = luaL_checknumber(L, 1);
        float y = luaL_checknumber(L, 2);
        float width = luaL_checknumber(L, 3);
        float height = luaL_checknumber(L, 4);
        // float sw = 0;
        // float sh = 0;

        if (lua_gettop(L) >= 5)
        {
            //   sw = luaL_checknumber(L, 5);
            //  sh = luaL_checknumber(L, 6);
        }

        // Empilhe a instância de Quad no topo da pilha como um userdata (substitua Quad pela sua própria implementação)
        Rectangle *quad_ptr = (Rectangle *)lua_newuserdata(L, sizeof(Rectangle));
        quad_ptr->x = x;
        quad_ptr->y = y;
        quad_ptr->width = width;
        quad_ptr->height = height;
        luaL_getmetatable(L, "Quad");
        lua_setmetatable(L, -2);

        return 1;
    }

    // Exemplo de função de ligação para um método da classe Quad (substitua pelo seu próprio método)
    static int quad_getWidth(lua_State *L)
    {
        // Recupera a instância de Quad da pilha (substitua Quad pela sua própria implementação)

        Rectangle *quad_ptr = (Rectangle *)luaL_checkudata(L, 1, "Quad");

        // Verifica se a instância de Quad é nula e retorna uma mensagem de erro, se necessário
        if (quad_ptr == nullptr)
        {

            return luaL_error(L, "Invalid Quad object");
        }

        Rectangle quad = (*(quad_ptr));

        //    Chama o método getWidth na instância de Quad (substitua pelo seu próprio método)
        float v = quad.width;

        // Empilha o resultado na pilha Lua
        lua_pushnumber(L, v);

        // Número de valores retornados na pilha (no caso, 1 valor)
        return 1;
    }

    static int quad_getHeight(lua_State *L)
    {
        Rectangle *quad_ptr = (Rectangle *)luaL_checkudata(L, 1, "Quad");
        if (quad_ptr == nullptr)
        {

            return luaL_error(L, "Invalid Quad object");
        }

        Rectangle quad = (*(quad_ptr));
        float v = quad.height;
        lua_pushnumber(L, v);
        return 1;
    }

    static int gc_quad(lua_State *L)
    {
        Rectangle *quad_ptr = (Rectangle *)luaL_checkudata(L, 1, "Quad");
        if (quad_ptr == nullptr)
        {

            return luaL_error(L, "Invalid Quad object");
        }

        // Rectangle quad = (*(quad_ptr));
        return 0;
    }

    // Função para registrar o metatable e os métodos da classe Quad
    int registerQuad(lua_State *L)
    {
        // Crie e registre o metatable Quad_mt
        luaL_newmetatable(L, "Quad");
        lua_pushcfunction(L, gc_quad);
        lua_setfield(L, -2, "__gc");

        // Adicione os métodos da classe Quad ao metatable
        lua_newtable(L);
        lua_pushcfunction(L, quad_getWidth);
        lua_setfield(L, -2, "getWidth");

        lua_pushcfunction(L, quad_getHeight);
        lua_setfield(L, -2, "getHeight");

        // Atribuir a tabela de métodos ao campo __index do metatable
        lua_setfield(L, -2, "__index");

        // Remove o metatable da pilha
        lua_pop(L, 1);
        return 0;
    }

}

namespace nFont
{
    static int newFont(lua_State *L)
    {
        // Recupere os argumentos: filename
        const char *filename = luaL_checkstring(L, 1);
        if (FileExists(filename) == false)
        {
            lua_pushnil(L);
            luaL_error(L, "File not found: %s", filename);
            return 1;
        }
        Font *userdata = (Font *)lua_newuserdata(L, sizeof(Font));
        *userdata = LoadFont(filename);
        luaL_getmetatable(L, "Font");
        lua_setmetatable(L, -2);
        return 1;
    }
    static int font_getWidth(lua_State *L)
    {
        Font *font_ptr = (Font *)luaL_checkudata(L, 1, "Font");
        if (font_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Font object");
        }
        Font font = (*font_ptr);

        const char *text = luaL_checkstring(L, 2);
        float spacing = 1;

        if (lua_gettop(L) == 3)
        {
            spacing = luaL_checknumber(L, 3);
        }
        Vector2 s = MeasureTextEx(font, text, font.baseSize, spacing);
        lua_pushnumber(L, s.x);
        return 1;
    }

    static int font_gc(lua_State *L)
    {
        Font *font_ptr = (Font *)luaL_checkudata(L, 1, "Font");
        if (font_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Font object");
        }
        // Log(LOG_INFO, "Font free");
        Font font = (*font_ptr);
        if (font.texture.id != 0)
        {
            UnloadFont(font);
        }

        return 0;
    }

    int registerFont(lua_State *L)
    {
        luaL_newmetatable(L, "Font");

        lua_pushcfunction(L, font_gc);
        lua_setfield(L, -2, "__gc");

        lua_newtable(L);
        lua_pushcfunction(L, font_getWidth);
        lua_setfield(L, -2, "getWidth");

        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);
        return 0;
    }

}

namespace nImage
{

    static int newImage(lua_State *L)
    {
        // Recupere os argumentos: filename
        const char *filename = luaL_checkstring(L, 1);
        if (FileExists(filename) == false)
        {
            lua_pushnil(L);
            luaL_error(L, "File not found: %s", filename);
            return 1;
        }
        Texture2D *userdata = (Texture2D *)lua_newuserdata(L, sizeof(Texture2D));
        *userdata = LoadTexture(filename);
        luaL_getmetatable(L, "Image");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int image_getWidth(lua_State *L)
    {
        Texture2D *image_ptr = (Texture2D *)luaL_checkudata(L, 1, "Image");
        if (image_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Image object");
        }
        Texture2D image = (*image_ptr);
        float v = image.width;
        lua_pushnumber(L, v);
        return 1;
    }

    static int image_getHeight(lua_State *L)
    {
        Texture2D *image_ptr = (Texture2D *)luaL_checkudata(L, 1, "Image");
        if (image_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Image object");
        }
        Texture2D image = (*image_ptr);
        float v = image.height;

        lua_pushnumber(L, v);
        return 1;
    }
    static int image_gc(lua_State *L)
    {
        Texture2D *image_ptr = (Texture2D *)luaL_checkudata(L, 1, "Image");

        if (image_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Image object");
        }
        Texture2D image = (*image_ptr);
        if (image.id != 0)
        {
            UnloadTexture(image);
        }
        return 0;
    }

    int registerImage(lua_State *L)
    {
        luaL_newmetatable(L, "Image");
        lua_pushcfunction(L, image_gc);
        lua_setfield(L, -2, "__gc");

        lua_newtable(L);
        lua_pushcfunction(L, image_getWidth);
        lua_setfield(L, -2, "getWidth");
        lua_pushcfunction(L, image_getHeight);
        lua_setfield(L, -2, "getHeight");
        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);
        return 0;
    }

}

namespace nInput
{

    int coreKeyToRaylib(const char *key)
    {
        if (_keys.find(key) == _keys.end())
        {
            Log(LOG_WARNING, "Key not found: %s", key);
            return -1;
        }
        int key_code = _keys[key];
        return key_code;
    }

    static int love_keyboard_isDown(lua_State *L)
    {
        bool result = false;
        if (lua_gettop(L) == 1)
        {
            const char *key = luaL_checkstring(L, 1);

            int key_code = coreKeyToRaylib(key);

            result = IsKeyDown(key_code);
        }
        else if (lua_gettop(L) == 2)
        {
            const char *key1 = luaL_checkstring(L, 1);
            const char *key2 = luaL_checkstring(L, 2);
            int key_code1 = coreKeyToRaylib(key1);
            int key_code2 = coreKeyToRaylib(key2);
            result = IsKeyDown(key_code1) || IsKeyDown(key_code2);
          
        }
        else
        {
            luaL_error(L, "Invalid number of arguments, expected 1 or 2");
        }

        lua_pushboolean(L, result);

        return 1;
    }

    static int love_keyboard_check(lua_State *L)
    {
        int result = 0;
        if (lua_gettop(L) == 1)
        {
            const char *key = luaL_checkstring(L, 1);

            int key_code = coreKeyToRaylib(key);

            result = IsKeyDown(key_code);
        }
        else if (lua_gettop(L) == 2)
        {
            const char *key1 = luaL_checkstring(L, 1);
            const char *key2 = luaL_checkstring(L, 2);
            int key_code1 = coreKeyToRaylib(key1);
            int key_code2 = coreKeyToRaylib(key2);
            result = (int)IsKeyDown(key_code1) || IsKeyDown(key_code2);
        }
        else
        {
            luaL_error(L, "Invalid number of arguments, expected 1 or 2");
        }

        lua_pushinteger(L, result);
        return 1;
    }

    static int love_keyboard_keyPressed(lua_State *L)
    {
        lua_pushinteger(L, GetKeyPressed());
        return 1;
    }
    static int love_keyboard_charPressed(lua_State *L)
    {
        lua_pushinteger(L, GetCharPressed());
        return 1;
    }

    static int love_keyboard_down(lua_State *L)
    {
        bool result = false;
        if (lua_gettop(L) == 1)
        {
            const char *key = luaL_checkstring(L, 1);

            int key_code = coreKeyToRaylib(key);

            result = IsKeyDown(key_code);
        }
        else if (lua_gettop(L) == 2)
        {
            const char *key1 = luaL_checkstring(L, 1);
            const char *key2 = luaL_checkstring(L, 2);
            int key_code1 = coreKeyToRaylib(key1);
            int key_code2 = coreKeyToRaylib(key2);
            result = IsKeyDown(key_code1) || IsKeyDown(key_code2);
        }
        else
        {
            luaL_error(L, "Invalid number of arguments, expected 1 or 2");
        }

        lua_pushboolean(L, result);

        return 1;
    }
    static int love_keyboard_press(lua_State *L)
    {
        bool result = false;
        if (lua_gettop(L) == 1)
        {
            const char *key = luaL_checkstring(L, 1);

            int key_code = coreKeyToRaylib(key);

            result = IsKeyPressed(key_code);
        }
        else if (lua_gettop(L) == 2)
        {
            const char *key1 = luaL_checkstring(L, 1);
            const char *key2 = luaL_checkstring(L, 2);
            int key_code1 = coreKeyToRaylib(key1);
            int key_code2 = coreKeyToRaylib(key2);
            result = IsKeyPressed(key_code1) || IsKeyPressed(key_code2);
        }
        else
        {
            luaL_error(L, "Invalid number of arguments, expected 1 or 2");
        }

        lua_pushboolean(L, result);

        return 1;
    }

    static int love_keyboard_release(lua_State *L)
    {
        bool result = false;
        if (lua_gettop(L) == 1)
        {
            const char *key = luaL_checkstring(L, 1);

            int key_code = coreKeyToRaylib(key);

            result = IsKeyReleased(key_code);
        }
        else if (lua_gettop(L) == 2)
        {
            const char *key1 = luaL_checkstring(L, 1);
            const char *key2 = luaL_checkstring(L, 2);
            int key_code1 = coreKeyToRaylib(key1);
            int key_code2 = coreKeyToRaylib(key2);
            result = IsKeyReleased(key_code1) || IsKeyReleased(key_code2);
        }
        else
        {
            luaL_error(L, "Invalid number of arguments, expected 1 or 2");
        }

        lua_pushboolean(L, result);
        return 1;
    }
    static int love_keyboard_up(lua_State *L)
    {
        bool result = false;
        if (lua_gettop(L) == 1)
        {
            const char *key = luaL_checkstring(L, 1);

            int key_code = coreKeyToRaylib(key);

            result = IsKeyReleased(key_code);
        }
        else if (lua_gettop(L) == 2)
        {
            const char *key1 = luaL_checkstring(L, 1);
            const char *key2 = luaL_checkstring(L, 2);
            int key_code1 = coreKeyToRaylib(key1);
            int key_code2 = coreKeyToRaylib(key2);
            result = IsKeyUp(key_code1) || IsKeyUp(key_code2);
        }
        else
        {
            luaL_error(L, "Invalid number of arguments, expected 1 or 2");
        }

        lua_pushboolean(L, result);
        return 1;
    }
    static int love_keyboard_setExitKey(lua_State *L)
    {
        const char *key = luaL_checkstring(L, 1);
        int key_code = coreKeyToRaylib(key);
        SetExitKey(key_code);
        return 0;
    }

    static int luaopen_keyboard(lua_State *L)
    {
         _keys["a"] = KEY_A;
    _keys["b"] = KEY_B;
    _keys["c"] = KEY_C;
    _keys["d"] = KEY_D;
    _keys["e"] = KEY_E;
    _keys["f"] = KEY_F;
    _keys["g"] = KEY_G;
    _keys["h"] = KEY_H;
    _keys["i"] = KEY_I;
    _keys["j"] = KEY_J;
    _keys["k"] = KEY_K;
    _keys["l"] = KEY_L;
    _keys["m"] = KEY_M;
    _keys["n"] = KEY_N;
    _keys["o"] = KEY_O;
    _keys["p"] = KEY_P;
    _keys["q"] = KEY_Q;
    _keys["r"] = KEY_R;
    _keys["s"] = KEY_S;
    _keys["t"] = KEY_T;
    _keys["u"] = KEY_U;
    _keys["v"] = KEY_V;
    _keys["w"] = KEY_W;
    _keys["x"] = KEY_X;
    _keys["y"] = KEY_Y;
    _keys["z"] = KEY_Z;
    _keys["0"] = KEY_ZERO;
    _keys["1"] = KEY_ONE;
    _keys["2"] = KEY_TWO;
    _keys["3"] = KEY_THREE;
    _keys["4"] = KEY_FOUR;
    _keys["5"] = KEY_FIVE;
    _keys["6"] = KEY_SIX;
    _keys["7"] = KEY_SEVEN;
    _keys["8"] = KEY_EIGHT;
    _keys["9"] = KEY_NINE;
    _keys["space"] = KEY_SPACE;
    _keys["esc"] = KEY_ESCAPE;
    _keys["enter"] = KEY_ENTER;
    _keys["backspace"] = KEY_BACKSPACE;
    _keys["tab"] = KEY_TAB;
    _keys["left"] = KEY_LEFT;
    _keys["right"] = KEY_RIGHT;
    _keys["up"] = KEY_UP;
    _keys["down"] = KEY_DOWN;
    _keys["f1"] = KEY_F1;
    _keys["f2"] = KEY_F2;
    _keys["f3"] = KEY_F3;
    _keys["f4"] = KEY_F4;
    _keys["f5"] = KEY_F5;
    _keys["f6"] = KEY_F6;
    _keys["f7"] = KEY_F7;
    _keys["f8"] = KEY_F8;
    _keys["f9"] = KEY_F9;
    _keys["f10"] = KEY_F10;
    _keys["f11"] = KEY_F11;
    _keys["f12"] = KEY_F12;
    _keys["pause"] = KEY_PAUSE;
    _keys["insert"] = KEY_INSERT;
    _keys["home"] = KEY_HOME;
    _keys["pageup"] = KEY_PAGE_UP;
    _keys["pagedown"] = KEY_PAGE_DOWN;
    _keys["end"] = KEY_END;
    _keys["del"] = KEY_DELETE;
    _keys["capslock"] = KEY_CAPS_LOCK;
    _keys["scrolllock"] = KEY_SCROLL_LOCK;
    _keys["numlock"] = KEY_NUM_LOCK;
    _keys["printscreen"] = KEY_PRINT_SCREEN;
    _keys["rightshift"] = KEY_RIGHT_SHIFT;
    _keys["leftshift"] = KEY_LEFT_SHIFT;
    _keys["rightcontrol"] = KEY_RIGHT_CONTROL;
    _keys["leftcontrol"] = KEY_LEFT_CONTROL;
    _keys["rightalt"] = KEY_RIGHT_ALT;
    _keys["leftalt"] = KEY_LEFT_ALT;
    _keys["rightsuper"] = KEY_RIGHT_SUPER;
        luaL_Reg reg[] = {
            {"isDown", love_keyboard_isDown},
            {"check", love_keyboard_check},
            {"down", love_keyboard_down},
            {"press", love_keyboard_press},
            {"release", love_keyboard_release},
            {"up", love_keyboard_up},
            {"keyPressed", love_keyboard_keyPressed},
            {"charPressed", love_keyboard_charPressed},
            {"setExitKey", love_keyboard_setExitKey},
            {0, 0},
            {0, 0},
        };
        luaL_newlib(L, reg);
        return 1;
    }

    static int love_mouse_isDown(lua_State *L)
    {
        int button = luaL_checkinteger(L, 1);
        lua_pushboolean(L, IsMouseButtonDown(button));
        return 1;
    }
    static int love_mouse_press(lua_State *L)
    {
        int button = luaL_checkinteger(L, 1);
        lua_pushboolean(L, IsMouseButtonPressed(button));
        return 1;
    }
    static int love_mouse_release(lua_State *L)
    {
        int button = luaL_checkinteger(L, 1);
        lua_pushboolean(L, IsMouseButtonReleased(button));
        return 1;
    }

    static int love_mouse_up(lua_State *L)
    {
        int button = luaL_checkinteger(L, 1);
        lua_pushboolean(L, IsMouseButtonUp(button));
        return 1;
    }

    static int love_mouse_check(lua_State *L)
    {
        int button = luaL_checkinteger(L, 1);
        lua_pushinteger(L, IsMouseButtonDown(button));
        return 1;
    }

    static int love_mouse_getPosition(lua_State *L)
    {
        Vector2 pos = GetMousePosition();
        lua_pushnumber(L, pos.x);
        lua_pushnumber(L, pos.y);
        return 2;
    }

    static int love_mouse_getDelta(lua_State *L)
    {
        Vector2 pos = GetMouseDelta();
        lua_pushnumber(L, pos.x);
        lua_pushnumber(L, pos.y);
        return 2;
    }

    static int love_mouse_getX(lua_State *L)
    {
        lua_pushnumber(L, GetMouseX());
        return 1;
    }
    static int love_mouse_getY(lua_State *L)
    {
        lua_pushnumber(L, GetMouseY());
        return 1;
    }

    int luaopen_mouse(lua_State *L)
    {
        luaL_Reg reg[] = {
            {"getPosition", love_mouse_getPosition},
            {"getX", love_mouse_getX},
            {"getY", love_mouse_getY},
            {"isDown", love_mouse_isDown},
            {"check", love_mouse_check},
            {"getDelta", love_mouse_getDelta},
            {"press", love_mouse_press},
            {"release", love_mouse_release},
            {"up", love_mouse_up},
            {0, 0},
            {0, 0},
        };
        luaL_newlib(L, reg);
        return 1;
    }
}

namespace nTimer
{

    static int love_timer_getTime(lua_State *L)
    {
        lua_pushnumber(L, GetTime());
        return 1;
    }
    static int l_timer_getDelta(lua_State *L)
    {
        lua_pushnumber(L, GetFrameTime());
        return 1;
    }

    static int l_timer_getFPS(lua_State *L)
    {
        lua_pushnumber(L, GetFPS());
        return 1;
    }
    int luaopen_timer(lua_State *L)
    {
        luaL_Reg reg[] = {
            {"getTime", love_timer_getTime},
            {"getDelta", l_timer_getDelta},
            {"getFPS", l_timer_getFPS},

            {0, 0},
        };
        luaL_newlib(L, reg);
        return 1;
    }

}

namespace nSystem
{
    int l_system_getOS(lua_State *L)
    {
        lua_pushstring(L, "Linux");
        return 1;
    }

    int l_system_getMemUsage(lua_State *L)
    {
        lua_pushnumber(L, lua_gc(L, LUA_GCCOUNT, 0) / 1024);
        return 1;
    }

    int luaopen_system(lua_State *L)
    {
        luaL_Reg reg[] = {
            {"getOS", l_system_getOS},
            {"getMemUsage", l_system_getMemUsage},
            {0, 0},
        };
        luaL_newlib(L, reg);
        return 1;
    }
}

namespace nFileSystem
{

    static int l_filesystem_exists(lua_State *L)
    {
        const char *path = luaL_checkstring(L, 1);
        lua_pushboolean(L, FileExists(path));
        return 1;
    }

    static int l_filesystem_isFile(lua_State *L)
    {
        const char *path = luaL_checkstring(L, 1);
        lua_pushboolean(L, FileExists(path));
        return 1;
    }

    static int l_filesystem_isDirectory(lua_State *L)
    {
        const char *path = luaL_checkstring(L, 1);
        lua_pushboolean(L, DirectoryExists(path));
        return 1;
    }

    static int l_filesystem_read(lua_State *L)
    {
        const char *path = luaL_checkstring(L, 1);
        char *buffer = LoadFileText(path);
        if (buffer == NULL)
        {
            lua_pushnil(L);
            lua_pushstring(L, "File not found");
            return 2;
        }
        lua_pushstring(L, buffer);
        free(buffer);
        return 1;
    }

    static int l_filesystem_load(lua_State *L)
    {
        const char *path = luaL_checkstring(L, 1);
        char *buffer = LoadFileText(path);
        if (buffer == NULL)
        {
            lua_pushstring(L, "File not found");
            return 1;
        }
        if (luaL_loadstring(L, buffer) != LUA_OK)
        {

            free(buffer);
            lua_pushstring(L, "Error loading file");
            return 1;
        }
        free(buffer);
        int status = lua_pcall(L, 0, LUA_MULTRET, 0);
        if (status != LUA_OK)
        {
            const char *error_message = lua_tostring(L, -1);
            Log(LOG_ERROR, "%s\n", error_message);

            lua_pushstring(L, error_message);

            return 1;
        }

        lua_pushvalue(L, -1);

        return 1;
    }

    static int l_filesystem_getInfo(lua_State *L)
    {
        const char *path = luaL_checkstring(L, 1);
        lua_newtable(L);
        lua_pushstring(L, "size");
        lua_pushnumber(L, GetFileLength(path));
        lua_settable(L, -3);
        lua_pushstring(L, "modtime");
        lua_pushnumber(L, GetFileModTime(path));
        lua_settable(L, -3);
        return 1;
    }

    static int l_filesystem_getWorkingDirectory(lua_State *L)
    {
        lua_pushstring(L, GetWorkingDirectory());
        return 1;
    }

    static int l_filesystem_getfilePath(lua_State *L)
    {
        const char *path = luaL_checkstring(L, 1);
        lua_pushstring(L, GetDirectoryPath(path));
        return 1;
    }

    int luaopen_filesystem(lua_State *L)
    {
        luaL_Reg reg[] = {
            {"exists", l_filesystem_exists},
            {"isFile", l_filesystem_isFile},
            {"getInfo", l_filesystem_getInfo},
            {"isDirectory", l_filesystem_isDirectory},
            {"getWorkingDirectory", l_filesystem_getWorkingDirectory},
            {"getPath", l_filesystem_getfilePath},
            {"read", l_filesystem_read},
            {"load", l_filesystem_load},

            {0, 0},
        };
        luaL_newlib(L, reg);
        return 1;
    }

}

namespace nSound
{

    static int newSound(lua_State *L)
    {
        const char *filename = luaL_checkstring(L, 1);
        if (FileExists(filename) == false)
        {
            lua_pushnil(L);
            luaL_error(L, "[newSound] File not found: %s", filename);
            return 1;
        }
        Sound *userdata = (Sound *)lua_newuserdata(L, sizeof(Sound));
        *userdata = LoadSound(filename);
        luaL_getmetatable(L, "Sound");
        lua_setmetatable(L, -2);
        return 1;
    }
    static int sound_gc(lua_State *L)
    {
        Sound *sound_ptr = (Sound *)luaL_checkudata(L, 1, "Sound");

        if (sound_ptr == nullptr)
        {
            return luaL_error(L, "[free] Invalid Sound object");
        }
        Sound sound = (*sound_ptr);
        if (sound.stream.buffer != nullptr)
        {
            UnloadSound(sound);
        }

        return 0;
    }

    static int sound_is_playing(lua_State *L)
    {
        Sound *sound_ptr = (Sound *)luaL_checkudata(L, 1, "Sound");

        if (sound_ptr == nullptr)
        {
            return luaL_error(L, "[isPlaying] Invalid Sound object");
        }
        Sound sound = (*sound_ptr);
        if (sound.stream.buffer != nullptr)
        {
            lua_pushboolean(L, IsSoundPlaying(sound));
        }
        else
        {
            lua_pushboolean(L, false);
        }
        return 1;
    }

    static int love_audio_play(lua_State *L)
    {
        Sound *sound_ptr = (Sound *)luaL_checkudata(L, 1, "Sound");

        if (sound_ptr == nullptr)
        {
            return luaL_error(L, "[play] Invalid Sound object");
        }
        Sound sound = (*sound_ptr);
        if (sound.stream.buffer != nullptr)
        {
            PlaySound(sound);
        }

        return 0;
    }

    static int love_audio_pause(lua_State *L)
    {
        Sound *sound_ptr = (Sound *)luaL_checkudata(L, 1, "Sound");

        if (sound_ptr == nullptr)
        {
            return luaL_error(L, "[pause] Invalid Sound object");
        }
        Sound sound = (*sound_ptr);
        if (sound.stream.buffer != nullptr)
        {
            PlaySound(sound);
        }

        return 0;
    }

    static int love_audio_stop(lua_State *L)
    {
        Sound *sound_ptr = (Sound *)luaL_checkudata(L, 1, "Sound");

        if (sound_ptr == nullptr)
        {
            return luaL_error(L, "[stop] Invalid Sound object");
        }
        Sound sound = (*sound_ptr);
        if (sound.stream.buffer != nullptr)
        {
            PlaySound(sound);
        }

        return 0;
    }

    static int love_audio_resume(lua_State *L)
    {
        Sound *sound_ptr = (Sound *)luaL_checkudata(L, 1, "Sound");

        if (sound_ptr == nullptr)
        {
            return luaL_error(L, "[resume] Invalid Sound object");
        }
        Sound sound = (*sound_ptr);
        if (sound.stream.buffer != nullptr)
        {
            PlaySound(sound);
        }

        return 0;
    }
    static int love_audio_setMasterVolume(lua_State *L)
    {
        float volume = luaL_checknumber(L, 1);
        SetMasterVolume(volume);
        return 0;
    }

    static int love_audio_isPlay(lua_State *L)
    {
        Sound *sound_ptr = (Sound *)luaL_checkudata(L, 1, "Sound");
        if (sound_ptr == nullptr)
        {
            return luaL_error(L, "[isPlaying] Invalid Sound object");
        }
        Sound sound = (*sound_ptr);
        bool b = false;
        if (sound.stream.buffer != nullptr)
        {
            b = IsSoundPlaying(sound);
        }
        lua_pushboolean(L, b);

        return 1;
    }

    static int love_audio_setVolume(lua_State *L)
    {
        Sound *sound_ptr = (Sound *)luaL_checkudata(L, 1, "Sound");
        float volume = luaL_checknumber(L, 2);
        if (sound_ptr == nullptr)
        {
            return luaL_error(L, "[setVolume] Invalid Sound object");
        }
        Sound sound = (*sound_ptr);
        if (sound.stream.buffer != nullptr)
            SetSoundVolume(sound, volume);
        return 0;
    }

    static int love_audio_setPitch(lua_State *L)
    {
        Sound *sound_ptr = (Sound *)luaL_checkudata(L, 1, "Sound");
        float pitch = luaL_checknumber(L, 2);
        if (sound_ptr == nullptr)
        {
            return luaL_error(L, "[setPitch] Invalid Sound object");
        }
        Sound sound = (*sound_ptr);
        if (sound.stream.buffer != nullptr)
            SetSoundPitch(sound, pitch);
        return 0;
    }

    static int love_audio_setPan(lua_State *L)
    {
        Sound *sound_ptr = (Sound *)luaL_checkudata(L, 1, "Sound");
        float pan = luaL_checknumber(L, 2);
        if (sound_ptr == nullptr)
        {
            return luaL_error(L, "[setPan] Invalid Sound object");
        }
        Sound sound = (*sound_ptr);
        if (sound.stream.buffer != nullptr)
            SetSoundPan(sound, pan);
        return 0;
    }

    int registerSound(lua_State *L)
    {

        luaL_newmetatable(L, "Sound");
        lua_pushcfunction(L, sound_gc);
        lua_setfield(L, -2, "__gc");

        lua_newtable(L);
        lua_pushcfunction(L, sound_is_playing);
        lua_setfield(L, -2, "isPlaying");
        lua_pushcfunction(L, love_audio_play);
        lua_setfield(L, -2, "play");
        lua_pushcfunction(L, love_audio_pause);
        lua_setfield(L, -2, "pause");
        lua_pushcfunction(L, love_audio_stop);
        lua_setfield(L, -2, "stop");
        lua_pushcfunction(L, love_audio_resume);
        lua_setfield(L, -2, "resume");
        lua_pushcfunction(L, love_audio_setVolume);
        lua_setfield(L, -2, "setVolume");
        lua_pushcfunction(L, love_audio_setPitch);
        lua_setfield(L, -2, "setPitch");
        lua_pushcfunction(L, love_audio_setPan);
        lua_setfield(L, -2, "setPan");

        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);

        return 0;
    }

    static int music_is_playing(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[isPlaying] Invalid Music object");
        }
        Music music = (*music_ptr);
        bool b = IsMusicStreamPlaying(music);
        lua_pushboolean(L, b);
        return 1;
    }

    static int music_play(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[play] Invalid Music object");
        }
        Music music = (*music_ptr);
        PlayMusicStream(music);
        return 0;
    }

    static int music_pause(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[pause] Invalid Music object");
        }
        Music music = (*music_ptr);
        PauseMusicStream(music);
        return 0;
    }

    static int music_stop(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[stop] Invalid Music object");
        }
        Music music = (*music_ptr);
        StopMusicStream(music);
        return 0;
    }

    static int music_resume(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[resume] Invalid Music object");
        }
        Music music = (*music_ptr);
        ResumeMusicStream(music);
        return 0;
    }

    static int music_seek(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        float position = luaL_checknumber(L, 2);
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[seek] Invalid Music object");
        }
        Music music = (*music_ptr);
        SeekMusicStream(music, position);
        return 0;
    }

    static int music_set_volume(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        float volume = luaL_checknumber(L, 2);
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[setVolume] Invalid Music object");
        }
        Music music = (*music_ptr);
        SetMusicVolume(music, volume);
        return 0;
    }

    static int music_set_pitch(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        float pitch = luaL_checknumber(L, 2);
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[setPitch] Invalid Music object");
        }
        Music music = (*music_ptr);
        SetMusicPitch(music, pitch);
        return 0;
    }

    static int music_set_pan(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        float pan = luaL_checknumber(L, 2);
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[setPan] Invalid Music object");
        }
        Music music = (*music_ptr);
        SetMusicPan(music, pan);
        return 0;
    }

    static int music_get_time_length(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[getTimeLength] Invalid Music object");
        }
        Music music = (*music_ptr);
        float time = GetMusicTimeLength(music);
        lua_pushnumber(L, time);
        return 1;
    }

    static int music_get_time_played(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[getTimePlayed] Invalid Music object");
        }
        Music music = (*music_ptr);
        float time = GetMusicTimePlayed(music);
        lua_pushnumber(L, time);
        return 1;
    }

    static int newMusic(lua_State *L)
    {
        const char *filename = luaL_checkstring(L, 1);
        if (FileExists(filename) == false)
        {
            lua_pushnil(L);
            luaL_error(L, "[newMusic] File not found: %s", filename);
            return 1;
        }
        Music *userdata = (Music *)lua_newuserdata(L, sizeof(Music));
        *userdata = LoadMusicStream(filename);
        luaL_getmetatable(L, "Music");
        lua_setmetatable(L, -2);
        return 1;
    }
    static int music_gc(lua_State *L)
    {
        Music *music_ptr = (Music *)luaL_checkudata(L, 1, "Music");
        if (music_ptr == nullptr)
        {
            return luaL_error(L, "[free] Invalid Music object");
        }
        Music music = (*music_ptr);
        if (music.stream.buffer != nullptr)
        {
            UnloadMusicStream(music);
        }

        return 0;
    }

    int registerMusic(lua_State *L)
    {

        luaL_newmetatable(L, "Music");
        lua_pushcfunction(L, music_gc);
        lua_setfield(L, -2, "__gc");

        lua_newtable(L);
        lua_pushcfunction(L, music_is_playing);
        lua_setfield(L, -2, "isPlaying");
        lua_pushcfunction(L, music_play);
        lua_setfield(L, -2, "play");
        lua_pushcfunction(L, music_pause);
        lua_setfield(L, -2, "pause");
        lua_pushcfunction(L, music_stop);
        lua_setfield(L, -2, "stop");
        lua_pushcfunction(L, music_resume);
        lua_setfield(L, -2, "resume");
        lua_pushcfunction(L, music_seek);
        lua_setfield(L, -2, "seek");
        lua_pushcfunction(L, music_set_volume);
        lua_setfield(L, -2, "setVolume");
        lua_pushcfunction(L, music_set_pitch);
        lua_setfield(L, -2, "setPitch");
        lua_pushcfunction(L, music_set_pan);
        lua_setfield(L, -2, "setPan");
        lua_pushcfunction(L, music_get_time_length);
        lua_setfield(L, -2, "getTimeLength");
        lua_pushcfunction(L, music_get_time_played);
        lua_setfield(L, -2, "getTimePlayed");
        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);

        return 0;
    }

    /// sound function

    int luaopen_sound(lua_State *L)
    {
        luaL_Reg reg[] = {
            {"setMasterVolume", love_audio_setMasterVolume},
            {"newSound", newSound},
            {"newMusic", newMusic},
            {0, 0},
        };
        luaL_newlib(L, reg);

        return 1;
    }

}

namespace nLove
{
    int l_love_getVersion(lua_State *L)
    {
        lua_pushstring(L, LOVE_VERSION);
        return 1;
    }

    int l_love_setup(lua_State *L)
    {
        screenWidth = luaL_checkinteger(L, 1);
        screenHeight = luaL_checkinteger(L, 2);
        screenFps = luaL_checkinteger(L, 3);
        screenTitle = luaL_checkstring(L, 4);
        screenVsync = lua_toboolean(L, 5);
        screenFullscreen = lua_toboolean(L, 6);
        screenBorderless = lua_toboolean(L, 7);
        screenResizable = lua_toboolean(L, 8);

        return 0;
    }
    int l_love_init(lua_State *L)
    {
        (void)L;
        int flags = 0;
        if (screenVsync)
        {
            flags |= FLAG_VSYNC_HINT;
        }
        if (screenFullscreen)
        {
            flags |= FLAG_FULLSCREEN_MODE;
        }
        if (screenBorderless)
        {
            flags |= FLAG_WINDOW_UNDECORATED;
        }
        if (screenResizable)
        {
            flags |= FLAG_WINDOW_RESIZABLE;
        }
        SetConfigFlags(flags);

        InitWindow(screenWidth, screenHeight, screenTitle.c_str());
        SetTargetFPS(screenFps);
        InitAudioDevice();
        init_physics();
        screenInit = true;

        return 0;
    }

    static int l_love_close(lua_State *L)
    {
        lua_pushboolean(L, WindowShouldClose());
        return 1;
    }

    static int l_love_log(lua_State *L)
    {
        if (lua_gettop(L) != 2)
        {
            return luaL_error(L, "love.log: wrong number of arguments (expected 2, got %d)", lua_gettop(L));
        }

        int type = luaL_checkinteger(L, 1);
        const char *msg = luaL_checkstring(L, 2);

        switch (type)
        {
        case 0:
            Log(LOG_INFO, "%s", msg);
            break;
        case 1:
            Log(LOG_WARNING, "%s", msg);
            break;
        case 2:
            Log(LOG_ERROR, "%s", msg);
            break;
        }

        return 0;
    }

    int love_api(lua_State *L)
    {
        luaL_Reg reg[] = {
            {"getVersion", l_love_getVersion},
            {"init", l_love_init},
            {"setup", l_love_setup},
            {"close", l_love_close},
            {"log", l_love_log},
            {0, 0},
        };
        luaL_newlib(L, reg);
        return 1;
    }
}

int luaopen_graphics(lua_State *L)
{

    using namespace nGraphics;

    luaL_Reg reg[] = {
        {"getWidth", love_graphics_getWidth},
        {"getHeight", love_graphics_getHeight},
        {"setBackgroundColor", love_graphics_setBackgroundColor},

        {"setColor", love_graphics_setColor},
        {"getFont", love_graphics_getFont},

        {"pop", love_graphics_pop},
        {"push", love_graphics_push},
        {"rotate", love_graphics_rotate},
        {"scale", love_graphics_scale},
        {"translate", love_graphics_translate},
        {"shear", love_graphics_shear},

        {"clear", love_graphics_clear},
        {"begin", love_graphics_begin},
        {"present", love_graphics_end},
        {"draw", love_graphics_draw},
        {"point", love_graphics_point},
        {"line", love_graphics_line},
        {"rectangle", love_graphics_rectangle},
        {"circle", love_graphics_circle},
        {"print", love_graphics_print},
        {"newImage", nImage::newImage},
        {"newQuad", nQuad::newQuad},
        {"newFont", nFont::newFont},
        {0, 0},
    };
    luaL_newlib(L, reg);
    return 1;
}
int luaopen_love(lua_State *L)
{
    using namespace nLove;

    love_api(L);

    nSound::registerMusic(L);
    nSound::registerSound(L);
    nImage::registerImage(L);
    nQuad::registerQuad(L);
    nFont::registerFont(L);
    luaclass_physics(L);

    /* Init submodules */
    nSystem::luaopen_system(L);
    lua_setfield(L, -2, "system");

    nFileSystem::luaopen_filesystem(L);
    lua_setfield(L, -2, "filesystem");

    luaopen_graphics(L);
    lua_setfield(L, -2, "graphics");

    nTimer::luaopen_timer(L);
    lua_setfield(L, -2, "timer");

    nInput::luaopen_keyboard(L);
    lua_setfield(L, -2, "keyboard");

    nInput::luaopen_mouse(L);
    lua_setfield(L, -2, "mouse");

    nSound::luaopen_sound(L);
    lua_setfield(L, -2, "audio");

    luaopen_physics(L);
    lua_setfield(L, -2, "physics");

    return 1;
}

void CreateLove(lua_State *L)
{

    using namespace nQuad;
    using namespace nImage;
    using namespace nInput;

    _color = WHITE;
    _backgroundColor = BLACK;
    //  luaL_dostring(L, "package.path = package.path .. ';scripts/?.lua;assets/scripts/?.lua;assets/scripts/utils/?.lua'");

    registerQuad(L);
    registerImage(L);
    luaL_requiref(L, "love", luaopen_love, 1);
}

void CloseLove(lua_State *L)
{
    // if (GetSoundsPlaying() > 0)
    //     StopSoundMulti();

    lua_close(L);
    if (screenInit)
    {
        free_physics();
        CloseAudioDevice();
        CloseWindow();
    }
}

#endif