// graphics_internal.hpp - declarations shared by graphics.cpp and
// graphics_objects.cpp (not part of the public love.hpp API).
#pragma once

#include "love.hpp"
#include "luax.hpp"

#include <raymath.h>
#include <rlgl.h>

#include <string>
#include <vector>

namespace love
{
namespace graphics
{

constexpr const char *IMAGE_TYPE = "Image";
constexpr const char *QUAD_TYPE = "Quad";
constexpr const char *CANVAS_TYPE = "Canvas";
constexpr const char *FONT_TYPE = "Font";
constexpr const char *SPRITEBATCH_TYPE = "SpriteBatch";
constexpr const char *TEXT_TYPE = "Text";
constexpr const char *SHADER_TYPE = "Shader";

// Filter / wrap names shared with love.image.
int textureFilterFromNames(const std::string &min, const std::string &mag);
int textureWrapFromName(const std::string &name);

struct ImageObj
{
    Texture2D texture = {};
    std::string filterMin = "linear";
    std::string filterMag = "linear";
    std::string wrapH = "clamp";
    std::string wrapV = "clamp";
    bool mipmaps = false;

    ~ImageObj();
};

struct QuadObj
{
    float x = 0, y = 0, w = 1, h = 1;
    float sw = 1, sh = 1; // reference texture dimensions
};

struct CanvasObj
{
    RenderTexture2D target = {};
    std::string filterMin = "linear";
    std::string filterMag = "linear";
    std::string wrapH = "clamp";
    std::string wrapV = "clamp";

    ~CanvasObj();
};

struct FontObj
{
    Font font = {};
    int size = 12;
    float lineHeight = 1.0f;
    bool owned = true; // false for the shared default-font cache entries
    std::string filterMin = "linear";
    std::string filterMag = "linear";

    ~FontObj();

    float height() const
    {
        return static_cast<float>(size);
    }
};

struct Sprite
{
    Rectangle source;    // in texture pixels
    Matrix transform;
    Color color;
    bool hasColor;
};

struct SpriteBatchObj
{
    luax::Ref texture;     // Image or Canvas userdata
    std::vector<Sprite> sprites;
    int bufferSize = 1000;
    Color color = WHITE;
    bool hasColor = false;
    int rangeStart = -1;
    int rangeCount = -1;
};

struct TextLine
{
    std::string text;
    float x = 0, y = 0;
    float limit = 0;      // 0 = no wrapping
    std::string align = "left";
    Matrix transform;
    bool hasTransform = false;
};

struct TextObj
{
    luax::Ref font;
    std::vector<TextLine> lines;
};

// What love.graphics.draw needs from any drawable.
struct DrawSource
{
    Texture2D texture;
    Rectangle source; // pixels
    bool flipY;
};

// Reads an Image or Canvas at `idx`; raises an error for anything else.
DrawSource checkDrawSource(lua_State *L, int idx);

// Love2D transform matrix: translate, rotate, scale, shear, origin.
Matrix makeTransform(float x, float y, float angle, float sx, float sy, float ox, float oy, float kx, float ky);

// Mathematical product a * b (apply b first, then a).
Matrix multiply(const Matrix &a, const Matrix &b);

// Draws `src` through `transform` (quad space -> screen) tinted with `color`.
void drawTextured(const DrawSource &src, const Matrix &transform, Color color);

Color currentColor();
const std::string &defaultFilterMin();
const std::string &defaultFilterMag();

// Fonts ---------------------------------------------------------------------

FontObj *checkFont(lua_State *L, int idx);
// Pushes the current font (creating the default one when needed) and returns it.
FontObj *pushCurrentFont(lua_State *L);
void setCurrentFont(lua_State *L, int idx);
Font loadEmbeddedFont(int size);
Font loadFontFromMemory(const char *extension, const unsigned char *data, int length, int size);
float measureText(const FontObj &font, const std::string &text);
void drawText(const FontObj &font, const std::string &text, float x, float y, Color color);
void wrapText(const FontObj &font, const std::string &text, float limit, std::vector<std::string> &lines, float &maxWidth);
void applyFontFilter(FontObj &font);

// Draws text (string or colored-text table at `textIndex`) with wrapping and
// alignment, under the transform built from the remaining arguments.
void printText(lua_State *L, FontObj &font, int textIndex, float x, float y, float limit, const char *align,
               const Matrix &transform);

// Shaders ----------------------------------------------------------------------

struct ShaderObj;

ShaderObj *checkShader(lua_State *L, int idx);
// Makes `shader` the active raylib shader and primes its built-in uniforms.
void shaderActivate(ShaderObj *shader, int targetWidth, int targetHeight);
void shaderDeactivate();
// Cheap per-draw refresh: re-registers extra textures, which raylib forgets after every batch flush.
void shaderBeforeDraw(ShaderObj *shader);
void shaderSetTargetSize(ShaderObj *shader, int width, int height);
void registerShaderType(lua_State *L);
extern const luaL_Reg SHADER_FUNCS[];

// Registration ---------------------------------------------------------------

void registerObjectTypes(lua_State *L);
extern const luaL_Reg OBJECT_FUNCS[];
void shutdownObjects();

} // namespace graphics
} // namespace love
