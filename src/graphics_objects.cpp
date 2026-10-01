// graphics_objects.cpp - Image, Quad, Canvas, Font, SpriteBatch and Text.
#include "graphics_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace love
{
namespace graphics
{

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

int textureFilterFromNames(const std::string &min, const std::string &mag)
{
    if (min == "nearest" && mag == "nearest")
    {
        return TEXTURE_FILTER_POINT;
    }
    return TEXTURE_FILTER_BILINEAR;
}

int textureWrapFromName(const std::string &name)
{
    if (name == "repeat")
    {
        return TEXTURE_WRAP_REPEAT;
    }
    if (name == "mirroredrepeat")
    {
        return TEXTURE_WRAP_MIRROR_REPEAT;
    }
    return TEXTURE_WRAP_CLAMP;
}

namespace
{

const char *const FILTER_NAMES[] = {"linear", "nearest"};
const int FILTER_VALUES[] = {0, 1};
const char *const WRAP_NAMES[] = {"clamp", "repeat", "mirroredrepeat", "clampzero"};
const int WRAP_VALUES[] = {0, 1, 2, 3};

std::string fileExtension(const std::string &path)
{
    size_t dot = path.find_last_of('.');
    if (dot == std::string::npos)
    {
        return "";
    }
    std::string ext = path.substr(dot);
    for (char &c : ext)
    {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return ext;
}

// Codepoints baked into TTF atlases: ASCII, Latin-1, Latin Extended-A/B and
// the common punctuation block. Enough for most western languages.
std::vector<int> &fontCodepoints()
{
    static std::vector<int> points = [] {
        std::vector<int> v;
        for (int c = 32; c <= 126; ++c)
        {
            v.push_back(c);
        }
        for (int c = 160; c <= 591; ++c)
        {
            v.push_back(c);
        }
        for (int c = 0x2010; c <= 0x2027; ++c)
        {
            v.push_back(c);
        }
        for (int c = 0x2030; c <= 0x205E; ++c)
        {
            v.push_back(c);
        }
        v.push_back(0x20AC); // euro
        v.push_back(0x2122); // trademark
        return v;
    }();
    return points;
}

// Default font atlases are shared between every Font object of the same size.
std::map<int, Font> g_defaultFonts;

} // namespace

ImageObj::~ImageObj()
{
    if (texture.id != 0)
    {
        UnloadTexture(texture);
        texture.id = 0;
    }
}

CanvasObj::~CanvasObj()
{
    if (target.id != 0)
    {
        UnloadRenderTexture(target);
        target.id = 0;
    }
}

FontObj::~FontObj()
{
    if (owned && font.texture.id != 0)
    {
        UnloadFont(font);
        font.texture.id = 0;
    }
}

Font loadFontFromMemory(const char *extension, const unsigned char *data, int length, int size)
{
    std::vector<int> &points = fontCodepoints();
    SetTraceLogLevel(LOG_ERROR);
    Font font = LoadFontFromMemory(extension, data, length, size, points.data(), static_cast<int>(points.size()));
    SetTraceLogLevel(LOG_WARNING);
    if (font.texture.id != 0)
    {
        SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);
    }
    return font;
}

Font loadEmbeddedFont(int size)
{
    auto it = g_defaultFonts.find(size);
    if (it != g_defaultFonts.end())
    {
        return it->second;
    }
    window::ensureOpen();
    unsigned int length = 0;
    const unsigned char *data = resource("DejaVuSans.ttf", &length);
    Font font = loadFontFromMemory(".ttf", data, static_cast<int>(length), size);
    g_defaultFonts[size] = font;
    return font;
}

void applyFontFilter(FontObj &font)
{
    if (font.font.texture.id != 0)
    {
        SetTextureFilter(font.font.texture, textureFilterFromNames(font.filterMin, font.filterMag));
    }
}

float measureText(const FontObj &font, const std::string &text)
{
    if (text.empty())
    {
        return 0.0f;
    }
    return MeasureTextEx(font.font, text.c_str(), static_cast<float>(font.size), 0.0f).x;
}

void drawText(const FontObj &font, const std::string &text, float x, float y, Color color)
{
    if (text.empty())
    {
        return;
    }
    DrawTextEx(font.font, text.c_str(), {x, y}, static_cast<float>(font.size), 0.0f, color);
}

namespace
{

// UTF-8 aware split of `line` into words (keeping the separating spaces).
void splitWords(const std::string &line, std::vector<std::string> &words)
{
    words.clear();
    std::string current;
    for (size_t i = 0; i < line.size(); ++i)
    {
        char c = line[i];
        current.push_back(c);
        if (c == ' ')
        {
            words.push_back(current);
            current.clear();
        }
    }
    if (!current.empty())
    {
        words.push_back(current);
    }
}

size_t utf8Length(const std::string &s, size_t pos)
{
    unsigned char c = static_cast<unsigned char>(s[pos]);
    if (c < 0x80)
    {
        return 1;
    }
    if ((c >> 5) == 0x6)
    {
        return 2;
    }
    if ((c >> 4) == 0xE)
    {
        return 3;
    }
    return 4;
}

// Breaks a single word that is wider than the limit into pieces.
void breakWord(const FontObj &font, const std::string &word, float limit, std::vector<std::string> &lines)
{
    std::string current;
    size_t pos = 0;
    while (pos < word.size())
    {
        size_t len = utf8Length(word, pos);
        std::string next = current + word.substr(pos, len);
        if (!current.empty() && measureText(font, next) > limit)
        {
            lines.push_back(current);
            current = word.substr(pos, len);
        }
        else
        {
            current = next;
        }
        pos += len;
    }
    if (!current.empty())
    {
        lines.push_back(current);
    }
}

std::string trimRight(const std::string &s)
{
    size_t end = s.find_last_not_of(' ');
    return end == std::string::npos ? "" : s.substr(0, end + 1);
}

} // namespace

void wrapText(const FontObj &font, const std::string &text, float limit, std::vector<std::string> &lines, float &maxWidth)
{
    lines.clear();
    maxWidth = 0.0f;
    size_t start = 0;
    while (start <= text.size())
    {
        size_t end = text.find('\n', start);
        std::string line = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }

        if (limit <= 0.0f || measureText(font, line) <= limit)
        {
            lines.push_back(line);
        }
        else
        {
            std::vector<std::string> words;
            splitWords(line, words);
            std::string current;
            for (const std::string &word : words)
            {
                std::string candidate = current + word;
                if (measureText(font, trimRight(candidate)) <= limit)
                {
                    current = candidate;
                    continue;
                }
                if (!current.empty())
                {
                    lines.push_back(trimRight(current));
                    current.clear();
                }
                if (measureText(font, trimRight(word)) > limit)
                {
                    std::vector<std::string> pieces;
                    breakWord(font, trimRight(word), limit, pieces);
                    for (size_t i = 0; i + 1 < pieces.size(); ++i)
                    {
                        lines.push_back(pieces[i]);
                    }
                    current = pieces.empty() ? "" : pieces.back();
                    if (word.back() == ' ')
                    {
                        current.push_back(' ');
                    }
                }
                else
                {
                    current = word;
                }
            }
            lines.push_back(trimRight(current));
        }

        if (end == std::string::npos)
        {
            break;
        }
        start = end + 1;
    }
    for (const std::string &l : lines)
    {
        maxWidth = std::max(maxWidth, measureText(font, l));
    }
}

namespace
{

struct ColoredRun
{
    std::string text;
    Color color;
};

// Reads a string or a Love2D colored-text table {color, string, color, ...}.
void readColoredText(lua_State *L, int idx, std::vector<ColoredRun> &runs)
{
    runs.clear();
    Color base = currentColor();
    if (lua_istable(L, idx))
    {
        lua_Integer n = luaL_len(L, idx);
        Color current = base;
        for (lua_Integer i = 1; i <= n; ++i)
        {
            lua_rawgeti(L, idx, i);
            if (lua_istable(L, -1))
            {
                float c[4] = {1, 1, 1, 1};
                for (int k = 0; k < 4; ++k)
                {
                    lua_rawgeti(L, -1, k + 1);
                    if (lua_isnumber(L, -1))
                    {
                        c[k] = static_cast<float>(lua_tonumber(L, -1));
                    }
                    lua_pop(L, 1);
                }
                auto byte = [](float v) { return static_cast<unsigned char>(std::min(1.0f, std::max(0.0f, v)) * 255.0f + 0.5f); };
                current = Color{byte(c[0]), byte(c[1]), byte(c[2]), byte(c[3])};
            }
            else
            {
                size_t len = 0;
                const char *s = luaL_tolstring(L, -1, &len);
                runs.push_back({std::string(s, len), current});
                lua_pop(L, 1);
            }
            lua_pop(L, 1);
        }
        return;
    }
    size_t len = 0;
    const char *s = luaL_tolstring(L, idx, &len);
    runs.push_back({std::string(s, len), base});
    lua_pop(L, 1);
}

} // namespace

void printText(lua_State *L, FontObj &font, int textIndex, float x, float y, float limit, const char *align,
               const Matrix &transform)
{
    std::vector<ColoredRun> runs;
    readColoredText(L, textIndex, runs);

    // Flatten runs into one string for wrapping, remembering run boundaries.
    std::string full;
    for (const ColoredRun &run : runs)
    {
        full += run.text;
    }
    std::vector<std::string> lines;
    float maxWidth = 0.0f;
    wrapText(font, full, limit, lines, maxWidth);

    ensureFrame();
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloat(transform));

    float lineAdvance = font.height() * font.lineHeight;
    float width = limit > 0.0f ? limit : maxWidth;

    // Walk the runs in parallel with the wrapped lines so colors follow text.
    size_t runIndex = 0;
    size_t runOffset = 0;
    size_t consumed = 0; // characters of `full` already drawn

    for (size_t i = 0; i < lines.size(); ++i)
    {
        const std::string &line = lines[i];
        float lineWidth = measureText(font, line);
        float startX = x;
        if (std::strcmp(align, "center") == 0)
        {
            startX += (width - lineWidth) * 0.5f;
        }
        else if (std::strcmp(align, "right") == 0)
        {
            startX += width - lineWidth;
        }
        float penX = startX;
        float penY = y + static_cast<float>(i) * lineAdvance;

        // Skip the separator (newline or space) removed by wrapping.
        while (consumed < full.size() && runIndex < runs.size())
        {
            if (line.empty())
            {
                break;
            }
            if (full.compare(consumed, line.size(), line) == 0)
            {
                break;
            }
            ++consumed;
            ++runOffset;
            while (runIndex < runs.size() && runOffset >= runs[runIndex].text.size())
            {
                runOffset = 0;
                ++runIndex;
            }
        }

        size_t remaining = line.size();
        size_t lineOffset = 0;
        if (runs.size() == 1)
        {
            drawText(font, line, penX, penY, runs[0].color);
            consumed += line.size();
            continue;
        }
        while (remaining > 0 && runIndex < runs.size())
        {
            const ColoredRun &run = runs[runIndex];
            size_t available = run.text.size() - runOffset;
            size_t take = std::min(available, remaining);
            std::string piece = line.substr(lineOffset, take);
            drawText(font, piece, penX, penY, run.color);
            penX += measureText(font, piece);
            lineOffset += take;
            remaining -= take;
            consumed += take;
            runOffset += take;
            if (runOffset >= run.text.size())
            {
                runOffset = 0;
                ++runIndex;
            }
        }
        if (remaining > 0)
        {
            drawText(font, line.substr(lineOffset), penX, penY, currentColor());
            consumed += remaining;
        }
    }
    rlPopMatrix();
}

// ---------------------------------------------------------------------------
// Fonts
// ---------------------------------------------------------------------------

FontObj *checkFont(lua_State *L, int idx)
{
    return luax::checkobject<FontObj>(L, idx, FONT_TYPE);
}

namespace
{

luax::Ref g_currentFont;

FontObj *newDefaultFont(lua_State *L, int size)
{
    FontObj *font = luax::newobject<FontObj>(L, FONT_TYPE);
    font->font = loadEmbeddedFont(size);
    font->size = size;
    font->owned = false;
    return font;
}

} // namespace

FontObj *pushCurrentFont(lua_State *L)
{
    g_currentFont.push(L);
    FontObj *font = luax::testobject<FontObj>(L, -1, FONT_TYPE);
    if (font != nullptr)
    {
        return font;
    }
    lua_pop(L, 1);
    font = newDefaultFont(L, 12);
    g_currentFont.set(L, -1);
    return font;
}

void setCurrentFont(lua_State *L, int idx)
{
    checkFont(L, idx);
    g_currentFont.set(L, idx);
}

namespace
{

int font_getWidth(lua_State *L)
{
    FontObj *font = checkFont(L, 1);
    size_t len = 0;
    const char *text = luaL_checklstring(L, 2, &len);
    std::vector<std::string> lines;
    float width = 0.0f;
    wrapText(*font, std::string(text, len), 0.0f, lines, width);
    lua_pushnumber(L, width);
    return 1;
}

int font_getHeight(lua_State *L)
{
    lua_pushnumber(L, checkFont(L, 1)->height());
    return 1;
}

int font_getWrap(lua_State *L)
{
    FontObj *font = checkFont(L, 1);
    size_t len = 0;
    const char *text = luaL_checklstring(L, 2, &len);
    float limit = luax::checkfloat(L, 3);
    std::vector<std::string> lines;
    float width = 0.0f;
    wrapText(*font, std::string(text, len), limit, lines, width);
    lua_pushnumber(L, width);
    lua_newtable(L);
    for (size_t i = 0; i < lines.size(); ++i)
    {
        lua_pushlstring(L, lines[i].data(), lines[i].size());
        lua_rawseti(L, -2, static_cast<lua_Integer>(i + 1));
    }
    return 2;
}

int font_setLineHeight(lua_State *L)
{
    checkFont(L, 1)->lineHeight = luax::checkfloat(L, 2);
    return 0;
}

int font_getLineHeight(lua_State *L)
{
    lua_pushnumber(L, checkFont(L, 1)->lineHeight);
    return 1;
}

int font_getAscent(lua_State *L)
{
    lua_pushnumber(L, std::ceil(checkFont(L, 1)->height() * 0.8f));
    return 1;
}

int font_getDescent(lua_State *L)
{
    lua_pushnumber(L, -std::ceil(checkFont(L, 1)->height() * 0.2f));
    return 1;
}

int font_getBaseline(lua_State *L)
{
    lua_pushnumber(L, std::ceil(checkFont(L, 1)->height() * 0.8f));
    return 1;
}

int font_hasGlyphs(lua_State *L)
{
    FontObj *font = checkFont(L, 1);
    int n = lua_gettop(L);
    for (int i = 2; i <= n; ++i)
    {
        if (lua_isinteger(L, i))
        {
            int cp = static_cast<int>(lua_tointeger(L, i));
            int index = GetGlyphIndex(font->font, cp);
            if (font->font.glyphs == nullptr || font->font.glyphs[index].value != cp)
            {
                lua_pushboolean(L, 0);
                return 1;
            }
            continue;
        }
        size_t len = 0;
        const char *text = luaL_checklstring(L, i, &len);
        int count = 0;
        int *points = LoadCodepoints(text, &count);
        bool ok = true;
        for (int k = 0; k < count && ok; ++k)
        {
            int index = GetGlyphIndex(font->font, points[k]);
            ok = font->font.glyphs != nullptr && font->font.glyphs[index].value == points[k];
        }
        UnloadCodepoints(points);
        if (!ok)
        {
            lua_pushboolean(L, 0);
            return 1;
        }
    }
    lua_pushboolean(L, 1);
    return 1;
}

int font_setFilter(lua_State *L)
{
    FontObj *font = checkFont(L, 1);
    luax::checkenum(L, 2, FILTER_NAMES, FILTER_VALUES, "filter mode");
    font->filterMin = lua_tostring(L, 2);
    if (lua_isnoneornil(L, 3))
    {
        font->filterMag = font->filterMin;
    }
    else
    {
        luax::checkenum(L, 3, FILTER_NAMES, FILTER_VALUES, "filter mode");
        font->filterMag = lua_tostring(L, 3);
    }
    applyFontFilter(*font);
    return 0;
}

int font_getFilter(lua_State *L)
{
    FontObj *font = checkFont(L, 1);
    lua_pushstring(L, font->filterMin.c_str());
    lua_pushstring(L, font->filterMag.c_str());
    lua_pushnumber(L, 1);
    return 3;
}

int font_getDPIScale(lua_State *L)
{
    lua_pushnumber(L, 1);
    return 1;
}

int font_getKerning(lua_State *L)
{
    lua_pushnumber(L, 0);
    return 1;
}

int font_setFallbacks(lua_State *L)
{
    return 0;
}

int font_tostring(lua_State *L)
{
    FontObj *font = checkFont(L, 1);
    lua_pushfstring(L, "Font: %dpx", font->size);
    return 1;
}

const luaL_Reg FONT_METHODS[] = {
    {"getWidth", font_getWidth},
    {"getHeight", font_getHeight},
    {"getWrap", font_getWrap},
    {"setLineHeight", font_setLineHeight},
    {"getLineHeight", font_getLineHeight},
    {"getAscent", font_getAscent},
    {"getDescent", font_getDescent},
    {"getBaseline", font_getBaseline},
    {"hasGlyphs", font_hasGlyphs},
    {"setFilter", font_setFilter},
    {"getFilter", font_getFilter},
    {"getDPIScale", font_getDPIScale},
    {"getKerning", font_getKerning},
    {"setFallbacks", font_setFallbacks},
    {"__tostring", font_tostring},
    {nullptr, nullptr},
};

// newFont([filename,] [size]) | newFont(FileData, size)
int l_newFont(lua_State *L)
{
    int size = 12;
    int sizeIndex = 1;
    std::string path;
    std::vector<unsigned char> data;
    std::string extension = ".ttf";
    bool fromMemory = false;

    if (lua_type(L, 1) == LUA_TSTRING)
    {
        path = lua_tostring(L, 1);
        sizeIndex = 2;
        if (!filesystem::readFile(path, data))
        {
            return luaL_error(L, "Could not open file %s. Does not exist.", path.c_str());
        }
        extension = fileExtension(path);
        fromMemory = true;
    }
    else if (lua_isuserdata(L, 1))
    {
        // FileData
        lua_getfield(L, 1, "getString");
        lua_pushvalue(L, 1);
        lua_call(L, 1, 1);
        size_t len = 0;
        const char *bytes = lua_tolstring(L, -1, &len);
        data.assign(bytes, bytes + len);
        lua_pop(L, 1);
        lua_getfield(L, 1, "getFilename");
        lua_pushvalue(L, 1);
        lua_call(L, 1, 1);
        extension = fileExtension(lua_tostring(L, -1) ? lua_tostring(L, -1) : "");
        lua_pop(L, 1);
        sizeIndex = 2;
        fromMemory = true;
    }
    size = luax::optint(L, sizeIndex, 12);
    if (size <= 0)
    {
        return luaL_error(L, "Invalid font size: %d", size);
    }

    if (!fromMemory)
    {
        newDefaultFont(L, size);
        return 1;
    }

    window::ensureOpen();
    FontObj *font = luax::newobject<FontObj>(L, FONT_TYPE);
    font->size = size;
    if (extension == ".fnt")
    {
        // BMFont descriptors reference a texture by path; raylib needs a file.
        std::string real = filesystem::resolveRead(path);
        font->font = LoadFont(real.c_str());
    }
    else
    {
        font->font = loadFontFromMemory(extension.c_str(), data.data(), static_cast<int>(data.size()), size);
    }
    if (font->font.texture.id == 0 || font->font.glyphCount == 0)
    {
        return luaL_error(L, "Could not load font '%s'", path.c_str());
    }
    return 1;
}

int l_setFont(lua_State *L)
{
    if (lua_isnoneornil(L, 1))
    {
        g_currentFont.clear(L);
        pushCurrentFont(L);
        lua_pop(L, 1);
        return 0;
    }
    setCurrentFont(L, 1);
    return 0;
}

int l_getFont(lua_State *L)
{
    pushCurrentFont(L);
    return 1;
}

int l_setNewFont(lua_State *L)
{
    l_newFont(L);
    setCurrentFont(L, lua_gettop(L));
    return 1;
}

// ---------------------------------------------------------------------------
// Images
// ---------------------------------------------------------------------------

ImageObj *checkImage(lua_State *L, int idx)
{
    return luax::checkobject<ImageObj>(L, idx, IMAGE_TYPE);
}

void applyImageSettings(ImageObj &image)
{
    SetTextureFilter(image.texture, textureFilterFromNames(image.filterMin, image.filterMag));
    SetTextureWrap(image.texture, textureWrapFromName(image.wrapH));
}

int l_newImage(lua_State *L)
{
    window::ensureOpen();
    Image source = {};
    bool ownsSource = true;

    if (lua_type(L, 1) == LUA_TSTRING)
    {
        std::string path = lua_tostring(L, 1);
        std::vector<unsigned char> data;
        if (!filesystem::readFile(path, data))
        {
            return luaL_error(L, "Could not open file %s. Does not exist.", path.c_str());
        }
        std::string ext = fileExtension(path);
        source = LoadImageFromMemory(ext.c_str(), data.data(), static_cast<int>(data.size()));
        if (source.data == nullptr)
        {
            return luaL_error(L, "Could not decode image '%s'", path.c_str());
        }
    }
    else if (Image *imageData = luax::testobject<Image>(L, 1, "ImageData"))
    {
        source = *imageData;
        ownsSource = false;
    }
    else if (lua_isuserdata(L, 1))
    {
        // FileData
        lua_getfield(L, 1, "getString");
        lua_pushvalue(L, 1);
        lua_call(L, 1, 1);
        size_t len = 0;
        const char *bytes = lua_tolstring(L, -1, &len);
        lua_getfield(L, 1, "getFilename");
        lua_pushvalue(L, 1);
        lua_call(L, 1, 1);
        std::string ext = fileExtension(lua_tostring(L, -1) ? lua_tostring(L, -1) : "");
        source = LoadImageFromMemory(ext.c_str(), reinterpret_cast<const unsigned char *>(bytes), static_cast<int>(len));
        lua_pop(L, 2);
        if (source.data == nullptr)
        {
            return luaL_error(L, "Could not decode image data");
        }
    }
    else
    {
        return luaL_error(L, "bad argument #1 to 'newImage' (filename, ImageData or FileData expected, got %s)",
                          luax::typename_(L, 1));
    }

    bool mipmaps = false;
    if (lua_istable(L, 2))
    {
        mipmaps = luax::getboolfield(L, 2, "mipmaps", false);
    }

    ImageObj *image = luax::newobject<ImageObj>(L, IMAGE_TYPE);
    image->texture = LoadTextureFromImage(source);
    if (ownsSource)
    {
        UnloadImage(source);
    }
    if (image->texture.id == 0)
    {
        return luaL_error(L, "Could not create texture");
    }
    image->filterMin = defaultFilterMin();
    image->filterMag = defaultFilterMag();
    image->mipmaps = mipmaps;
    if (mipmaps)
    {
        GenTextureMipmaps(&image->texture);
    }
    applyImageSettings(*image);
    return 1;
}

int image_getWidth(lua_State *L)
{
    lua_pushinteger(L, checkImage(L, 1)->texture.width);
    return 1;
}

int image_getHeight(lua_State *L)
{
    lua_pushinteger(L, checkImage(L, 1)->texture.height);
    return 1;
}

int image_getDimensions(lua_State *L)
{
    ImageObj *image = checkImage(L, 1);
    lua_pushinteger(L, image->texture.width);
    lua_pushinteger(L, image->texture.height);
    return 2;
}

int image_setFilter(lua_State *L)
{
    ImageObj *image = checkImage(L, 1);
    luax::checkenum(L, 2, FILTER_NAMES, FILTER_VALUES, "filter mode");
    image->filterMin = lua_tostring(L, 2);
    if (lua_isnoneornil(L, 3))
    {
        image->filterMag = image->filterMin;
    }
    else
    {
        luax::checkenum(L, 3, FILTER_NAMES, FILTER_VALUES, "filter mode");
        image->filterMag = lua_tostring(L, 3);
    }
    applyImageSettings(*image);
    return 0;
}

int image_getFilter(lua_State *L)
{
    ImageObj *image = checkImage(L, 1);
    lua_pushstring(L, image->filterMin.c_str());
    lua_pushstring(L, image->filterMag.c_str());
    lua_pushnumber(L, 1);
    return 3;
}

int image_setWrap(lua_State *L)
{
    ImageObj *image = checkImage(L, 1);
    luax::checkenum(L, 2, WRAP_NAMES, WRAP_VALUES, "wrap mode");
    image->wrapH = lua_tostring(L, 2);
    image->wrapV = lua_isnoneornil(L, 3) ? image->wrapH : lua_tostring(L, 3);
    applyImageSettings(*image);
    return 0;
}

int image_getWrap(lua_State *L)
{
    ImageObj *image = checkImage(L, 1);
    lua_pushstring(L, image->wrapH.c_str());
    lua_pushstring(L, image->wrapV.c_str());
    return 2;
}

int image_setMipmapFilter(lua_State *L)
{
    return 0;
}

int image_getMipmapFilter(lua_State *L)
{
    ImageObj *image = checkImage(L, 1);
    if (!image->mipmaps)
    {
        return 0;
    }
    lua_pushstring(L, "linear");
    lua_pushnumber(L, 0);
    return 2;
}

int image_getMipmapCount(lua_State *L)
{
    lua_pushinteger(L, checkImage(L, 1)->texture.mipmaps);
    return 1;
}

int image_isReadable(lua_State *L)
{
    lua_pushboolean(L, 1);
    return 1;
}

int image_getFormat(lua_State *L)
{
    lua_pushstring(L, "rgba8");
    return 1;
}

int image_getDPIScale(lua_State *L)
{
    lua_pushnumber(L, 1);
    return 1;
}

int image_getTextureType(lua_State *L)
{
    lua_pushstring(L, "2d");
    return 1;
}

int image_replacePixels(lua_State *L)
{
    ImageObj *image = checkImage(L, 1);
    Image *data = luax::checkobject<Image>(L, 2, "ImageData");
    if (data->width != image->texture.width || data->height != image->texture.height)
    {
        return luaL_error(L, "ImageData dimensions must match the Image dimensions");
    }
    Image copy = ImageCopy(*data);
    ImageFormat(&copy, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    UpdateTexture(image->texture, copy.data);
    UnloadImage(copy);
    return 0;
}

int image_tostring(lua_State *L)
{
    ImageObj *image = checkImage(L, 1);
    lua_pushfstring(L, "Image: %dx%d", image->texture.width, image->texture.height);
    return 1;
}

const luaL_Reg IMAGE_METHODS[] = {
    {"getWidth", image_getWidth},
    {"getHeight", image_getHeight},
    {"getDimensions", image_getDimensions},
    {"getPixelWidth", image_getWidth},
    {"getPixelHeight", image_getHeight},
    {"getPixelDimensions", image_getDimensions},
    {"setFilter", image_setFilter},
    {"getFilter", image_getFilter},
    {"setWrap", image_setWrap},
    {"getWrap", image_getWrap},
    {"setMipmapFilter", image_setMipmapFilter},
    {"getMipmapFilter", image_getMipmapFilter},
    {"getMipmapCount", image_getMipmapCount},
    {"isReadable", image_isReadable},
    {"getFormat", image_getFormat},
    {"getDPIScale", image_getDPIScale},
    {"getTextureType", image_getTextureType},
    {"replacePixels", image_replacePixels},
    {"__tostring", image_tostring},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// Quads
// ---------------------------------------------------------------------------

QuadObj *checkQuad(lua_State *L, int idx)
{
    return luax::checkobject<QuadObj>(L, idx, QUAD_TYPE);
}

// newQuad(x, y, width, height, sw, sh) | newQuad(x, y, width, height, texture)
int l_newQuad(lua_State *L)
{
    QuadObj *quad = luax::newobject<QuadObj>(L, QUAD_TYPE);
    quad->x = luax::checkfloat(L, 1);
    quad->y = luax::checkfloat(L, 2);
    quad->w = luax::checkfloat(L, 3);
    quad->h = luax::checkfloat(L, 4);
    if (lua_isnumber(L, 5))
    {
        quad->sw = luax::checkfloat(L, 5);
        quad->sh = luax::checkfloat(L, 6);
    }
    else
    {
        DrawSource src = checkDrawSource(L, 5);
        quad->sw = static_cast<float>(src.texture.width);
        quad->sh = static_cast<float>(src.texture.height);
    }
    if (quad->sw <= 0.0f || quad->sh <= 0.0f)
    {
        return luaL_error(L, "Quad reference dimensions must be positive");
    }
    return 1;
}

int quad_getViewport(lua_State *L)
{
    QuadObj *quad = checkQuad(L, 1);
    lua_pushnumber(L, quad->x);
    lua_pushnumber(L, quad->y);
    lua_pushnumber(L, quad->w);
    lua_pushnumber(L, quad->h);
    return 4;
}

int quad_setViewport(lua_State *L)
{
    QuadObj *quad = checkQuad(L, 1);
    quad->x = luax::checkfloat(L, 2);
    quad->y = luax::checkfloat(L, 3);
    quad->w = luax::checkfloat(L, 4);
    quad->h = luax::checkfloat(L, 5);
    if (lua_isnumber(L, 6))
    {
        quad->sw = luax::checkfloat(L, 6);
        quad->sh = luax::checkfloat(L, 7);
    }
    return 0;
}

int quad_getTextureDimensions(lua_State *L)
{
    QuadObj *quad = checkQuad(L, 1);
    lua_pushnumber(L, quad->sw);
    lua_pushnumber(L, quad->sh);
    return 2;
}

int quad_setLayer(lua_State *L)
{
    return 0;
}

int quad_getLayer(lua_State *L)
{
    lua_pushinteger(L, 1);
    return 1;
}

const luaL_Reg QUAD_METHODS[] = {
    {"getViewport", quad_getViewport},
    {"setViewport", quad_setViewport},
    {"getTextureDimensions", quad_getTextureDimensions},
    {"setLayer", quad_setLayer},
    {"getLayer", quad_getLayer},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// Canvas
// ---------------------------------------------------------------------------

CanvasObj *checkCanvas(lua_State *L, int idx)
{
    return luax::checkobject<CanvasObj>(L, idx, CANVAS_TYPE);
}

void applyCanvasSettings(CanvasObj &canvas)
{
    SetTextureFilter(canvas.target.texture, textureFilterFromNames(canvas.filterMin, canvas.filterMag));
    SetTextureWrap(canvas.target.texture, textureWrapFromName(canvas.wrapH));
}

int l_newCanvas(lua_State *L)
{
    window::ensureOpen();
    int width = luax::optint(L, 1, GetScreenWidth());
    int height = luax::optint(L, 2, GetScreenHeight());
    if (width <= 0 || height <= 0)
    {
        return luaL_error(L, "Canvas dimensions must be positive");
    }
    CanvasObj *canvas = luax::newobject<CanvasObj>(L, CANVAS_TYPE);
    canvas->target = LoadRenderTexture(width, height);
    if (canvas->target.id == 0)
    {
        return luaL_error(L, "Could not create canvas");
    }
    canvas->filterMin = defaultFilterMin();
    canvas->filterMag = defaultFilterMag();
    applyCanvasSettings(*canvas);

    // Love2D canvases start transparent.
    BeginTextureMode(canvas->target);
    ClearBackground(BLANK);
    EndTextureMode();
    return 1;
}

int canvas_getWidth(lua_State *L)
{
    lua_pushinteger(L, checkCanvas(L, 1)->target.texture.width);
    return 1;
}

int canvas_getHeight(lua_State *L)
{
    lua_pushinteger(L, checkCanvas(L, 1)->target.texture.height);
    return 1;
}

int canvas_getDimensions(lua_State *L)
{
    CanvasObj *canvas = checkCanvas(L, 1);
    lua_pushinteger(L, canvas->target.texture.width);
    lua_pushinteger(L, canvas->target.texture.height);
    return 2;
}

int canvas_setFilter(lua_State *L)
{
    CanvasObj *canvas = checkCanvas(L, 1);
    luax::checkenum(L, 2, FILTER_NAMES, FILTER_VALUES, "filter mode");
    canvas->filterMin = lua_tostring(L, 2);
    if (lua_isnoneornil(L, 3))
    {
        canvas->filterMag = canvas->filterMin;
    }
    else
    {
        luax::checkenum(L, 3, FILTER_NAMES, FILTER_VALUES, "filter mode");
        canvas->filterMag = lua_tostring(L, 3);
    }
    applyCanvasSettings(*canvas);
    return 0;
}

int canvas_getFilter(lua_State *L)
{
    CanvasObj *canvas = checkCanvas(L, 1);
    lua_pushstring(L, canvas->filterMin.c_str());
    lua_pushstring(L, canvas->filterMag.c_str());
    lua_pushnumber(L, 1);
    return 3;
}

int canvas_setWrap(lua_State *L)
{
    CanvasObj *canvas = checkCanvas(L, 1);
    luax::checkenum(L, 2, WRAP_NAMES, WRAP_VALUES, "wrap mode");
    canvas->wrapH = lua_tostring(L, 2);
    canvas->wrapV = lua_isnoneornil(L, 3) ? canvas->wrapH : lua_tostring(L, 3);
    applyCanvasSettings(*canvas);
    return 0;
}

int canvas_getWrap(lua_State *L)
{
    CanvasObj *canvas = checkCanvas(L, 1);
    lua_pushstring(L, canvas->wrapH.c_str());
    lua_pushstring(L, canvas->wrapV.c_str());
    return 2;
}

int canvas_renderTo(lua_State *L)
{
    checkCanvas(L, 1);
    luaL_checktype(L, 2, LUA_TFUNCTION);
    // Equivalent to: local old = getCanvas(); setCanvas(self); fn(...); setCanvas(old)
    lua_getglobal(L, "love");
    lua_getfield(L, -1, "graphics");
    lua_remove(L, -2);
    int graphicsIdx = lua_gettop(L);

    lua_getfield(L, graphicsIdx, "getCanvas");
    lua_call(L, 0, 1);
    int oldIdx = lua_gettop(L);

    lua_getfield(L, graphicsIdx, "setCanvas");
    lua_pushvalue(L, 1);
    lua_call(L, 1, 0);

    int nargs = lua_gettop(L) - oldIdx; // nothing extra beyond old canvas yet
    (void)nargs;
    lua_pushvalue(L, 2);
    int extra = 0;
    for (int i = 3; i <= graphicsIdx - 1; ++i)
    {
        lua_pushvalue(L, i);
        ++extra;
    }
    int status = lua_pcall(L, extra, 0, 0);

    lua_getfield(L, graphicsIdx, "setCanvas");
    lua_pushvalue(L, oldIdx);
    lua_call(L, 1, 0);

    if (status != LUA_OK)
    {
        return lua_error(L);
    }
    return 0;
}

int canvas_newImageData(lua_State *L)
{
    CanvasObj *canvas = checkCanvas(L, 1);
    Image *data = luax::newobject<Image>(L, "ImageData");
    *data = LoadImageFromTexture(canvas->target.texture);
    ImageFlipVertical(data);
    ImageFormat(data, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    return 1;
}

int canvas_getMSAA(lua_State *L)
{
    lua_pushinteger(L, 0);
    return 1;
}

int canvas_getFormat(lua_State *L)
{
    lua_pushstring(L, "rgba8");
    return 1;
}

int canvas_isReadable(lua_State *L)
{
    lua_pushboolean(L, 1);
    return 1;
}

int canvas_getDPIScale(lua_State *L)
{
    lua_pushnumber(L, 1);
    return 1;
}

int canvas_getTextureType(lua_State *L)
{
    lua_pushstring(L, "2d");
    return 1;
}

int canvas_getMipmapMode(lua_State *L)
{
    lua_pushstring(L, "none");
    return 1;
}

int canvas_tostring(lua_State *L)
{
    CanvasObj *canvas = checkCanvas(L, 1);
    lua_pushfstring(L, "Canvas: %dx%d", canvas->target.texture.width, canvas->target.texture.height);
    return 1;
}

const luaL_Reg CANVAS_METHODS[] = {
    {"getWidth", canvas_getWidth},
    {"getHeight", canvas_getHeight},
    {"getDimensions", canvas_getDimensions},
    {"getPixelWidth", canvas_getWidth},
    {"getPixelHeight", canvas_getHeight},
    {"getPixelDimensions", canvas_getDimensions},
    {"setFilter", canvas_setFilter},
    {"getFilter", canvas_getFilter},
    {"setWrap", canvas_setWrap},
    {"getWrap", canvas_getWrap},
    {"renderTo", canvas_renderTo},
    {"newImageData", canvas_newImageData},
    {"getMSAA", canvas_getMSAA},
    {"getFormat", canvas_getFormat},
    {"isReadable", canvas_isReadable},
    {"getDPIScale", canvas_getDPIScale},
    {"getTextureType", canvas_getTextureType},
    {"getMipmapMode", canvas_getMipmapMode},
    {"__tostring", canvas_tostring},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// SpriteBatch
// ---------------------------------------------------------------------------

SpriteBatchObj *checkSpriteBatch(lua_State *L, int idx)
{
    return luax::checkobject<SpriteBatchObj>(L, idx, SPRITEBATCH_TYPE);
}

int l_newSpriteBatch(lua_State *L)
{
    checkDrawSource(L, 1);
    int maxSprites = luax::optint(L, 2, 1000);
    SpriteBatchObj *batch = luax::newobject<SpriteBatchObj>(L, SPRITEBATCH_TYPE);
    batch->texture.set(L, 1);
    batch->bufferSize = maxSprites;
    batch->sprites.reserve(static_cast<size_t>(std::max(0, maxSprites)));
    return 1;
}

Sprite readSprite(lua_State *L, SpriteBatchObj &batch, int idx)
{
    batch.texture.push(L);
    DrawSource src = checkDrawSource(L, lua_gettop(L));
    lua_pop(L, 1);

    Sprite sprite;
    sprite.source = src.source;
    if (QuadObj *quad = luax::testobject<QuadObj>(L, idx, QUAD_TYPE))
    {
        float scaleX = src.texture.width / quad->sw;
        float scaleY = src.texture.height / quad->sh;
        sprite.source = {quad->x * scaleX, quad->y * scaleY, quad->w * scaleX, quad->h * scaleY};
        ++idx;
    }
    if (Matrix *t = luax::testobject<Matrix>(L, idx, math::TRANSFORM_TYPE))
    {
        sprite.transform = *t;
    }
    else
    {
        float x = luax::optfloat(L, idx, 0.0f);
        float y = luax::optfloat(L, idx + 1, 0.0f);
        float r = luax::optfloat(L, idx + 2, 0.0f);
        float sx = luax::optfloat(L, idx + 3, 1.0f);
        float sy = luax::optfloat(L, idx + 4, sx);
        float ox = luax::optfloat(L, idx + 5, 0.0f);
        float oy = luax::optfloat(L, idx + 6, 0.0f);
        float kx = luax::optfloat(L, idx + 7, 0.0f);
        float ky = luax::optfloat(L, idx + 8, 0.0f);
        sprite.transform = makeTransform(x, y, r, sx, sy, ox, oy, kx, ky);
    }
    sprite.color = batch.color;
    sprite.hasColor = batch.hasColor;
    return sprite;
}

int batch_add(lua_State *L)
{
    SpriteBatchObj *batch = checkSpriteBatch(L, 1);
    if (static_cast<int>(batch->sprites.size()) >= batch->bufferSize)
    {
        // Love2D grows dynamic batches automatically.
        batch->bufferSize *= 2;
    }
    batch->sprites.push_back(readSprite(L, *batch, 2));
    lua_pushinteger(L, static_cast<lua_Integer>(batch->sprites.size()));
    return 1;
}

int batch_set(lua_State *L)
{
    SpriteBatchObj *batch = checkSpriteBatch(L, 1);
    lua_Integer id = luaL_checkinteger(L, 2);
    if (id < 1 || id > static_cast<lua_Integer>(batch->sprites.size()))
    {
        return luaL_error(L, "Invalid sprite index: %d", static_cast<int>(id));
    }
    batch->sprites[static_cast<size_t>(id - 1)] = readSprite(L, *batch, 3);
    return 0;
}

int batch_clear(lua_State *L)
{
    checkSpriteBatch(L, 1)->sprites.clear();
    return 0;
}

int batch_flush(lua_State *L)
{
    return 0;
}

int batch_getCount(lua_State *L)
{
    lua_pushinteger(L, static_cast<lua_Integer>(checkSpriteBatch(L, 1)->sprites.size()));
    return 1;
}

int batch_getBufferSize(lua_State *L)
{
    lua_pushinteger(L, checkSpriteBatch(L, 1)->bufferSize);
    return 1;
}

int batch_setBufferSize(lua_State *L)
{
    SpriteBatchObj *batch = checkSpriteBatch(L, 1);
    batch->bufferSize = static_cast<int>(luaL_checkinteger(L, 2));
    if (static_cast<int>(batch->sprites.size()) > batch->bufferSize)
    {
        batch->sprites.resize(static_cast<size_t>(batch->bufferSize));
    }
    return 0;
}

int batch_setColor(lua_State *L)
{
    SpriteBatchObj *batch = checkSpriteBatch(L, 1);
    if (lua_isnoneornil(L, 2))
    {
        batch->hasColor = false;
        batch->color = WHITE;
        return 0;
    }
    float c[4] = {1, 1, 1, 1};
    if (lua_istable(L, 2))
    {
        for (int i = 0; i < 4; ++i)
        {
            lua_rawgeti(L, 2, i + 1);
            if (lua_isnumber(L, -1))
            {
                c[i] = static_cast<float>(lua_tonumber(L, -1));
            }
            lua_pop(L, 1);
        }
    }
    else
    {
        c[0] = luax::checkfloat(L, 2);
        c[1] = luax::checkfloat(L, 3);
        c[2] = luax::checkfloat(L, 4);
        c[3] = luax::optfloat(L, 5, 1.0f);
    }
    batch->hasColor = true;
    batch->color = Color{static_cast<unsigned char>(std::min(1.0f, std::max(0.0f, c[0])) * 255),
                         static_cast<unsigned char>(std::min(1.0f, std::max(0.0f, c[1])) * 255),
                         static_cast<unsigned char>(std::min(1.0f, std::max(0.0f, c[2])) * 255),
                         static_cast<unsigned char>(std::min(1.0f, std::max(0.0f, c[3])) * 255)};
    return 0;
}

int batch_getColor(lua_State *L)
{
    SpriteBatchObj *batch = checkSpriteBatch(L, 1);
    if (!batch->hasColor)
    {
        return 0;
    }
    lua_pushnumber(L, batch->color.r / 255.0);
    lua_pushnumber(L, batch->color.g / 255.0);
    lua_pushnumber(L, batch->color.b / 255.0);
    lua_pushnumber(L, batch->color.a / 255.0);
    return 4;
}

int batch_setTexture(lua_State *L)
{
    SpriteBatchObj *batch = checkSpriteBatch(L, 1);
    checkDrawSource(L, 2);
    batch->texture.set(L, 2);
    return 0;
}

int batch_getTexture(lua_State *L)
{
    checkSpriteBatch(L, 1)->texture.push(L);
    return 1;
}

int batch_setDrawRange(lua_State *L)
{
    SpriteBatchObj *batch = checkSpriteBatch(L, 1);
    if (lua_isnoneornil(L, 2))
    {
        batch->rangeStart = -1;
        batch->rangeCount = -1;
        return 0;
    }
    batch->rangeStart = static_cast<int>(luaL_checkinteger(L, 2)) - 1;
    batch->rangeCount = static_cast<int>(luaL_checkinteger(L, 3));
    return 0;
}

int batch_getDrawRange(lua_State *L)
{
    SpriteBatchObj *batch = checkSpriteBatch(L, 1);
    if (batch->rangeStart < 0)
    {
        return 0;
    }
    lua_pushinteger(L, batch->rangeStart + 1);
    lua_pushinteger(L, batch->rangeCount);
    return 2;
}

int batch_attachAttribute(lua_State *L)
{
    return 0;
}

int batch_tostring(lua_State *L)
{
    SpriteBatchObj *batch = checkSpriteBatch(L, 1);
    lua_pushfstring(L, "SpriteBatch: %d sprites", static_cast<int>(batch->sprites.size()));
    return 1;
}

const luaL_Reg SPRITEBATCH_METHODS[] = {
    {"add", batch_add},
    {"set", batch_set},
    {"clear", batch_clear},
    {"flush", batch_flush},
    {"getCount", batch_getCount},
    {"getBufferSize", batch_getBufferSize},
    {"setBufferSize", batch_setBufferSize},
    {"setColor", batch_setColor},
    {"getColor", batch_getColor},
    {"setTexture", batch_setTexture},
    {"getTexture", batch_getTexture},
    {"setDrawRange", batch_setDrawRange},
    {"getDrawRange", batch_getDrawRange},
    {"attachAttribute", batch_attachAttribute},
    {"__tostring", batch_tostring},
    {nullptr, nullptr},
};

int batch_gc(lua_State *L)
{
    SpriteBatchObj *batch = static_cast<SpriteBatchObj *>(lua_touserdata(L, 1));
    batch->texture.clear(L);
    batch->~SpriteBatchObj();
    return 0;
}

// ---------------------------------------------------------------------------
// Text
// ---------------------------------------------------------------------------

TextObj *checkText(lua_State *L, int idx)
{
    return luax::checkobject<TextObj>(L, idx, TEXT_TYPE);
}

std::string flattenText(lua_State *L, int idx)
{
    if (lua_istable(L, idx))
    {
        std::string out;
        lua_Integer n = luaL_len(L, idx);
        for (lua_Integer i = 1; i <= n; ++i)
        {
            lua_rawgeti(L, idx, i);
            if (lua_type(L, -1) == LUA_TSTRING)
            {
                size_t len = 0;
                const char *s = lua_tolstring(L, -1, &len);
                out.append(s, len);
            }
            lua_pop(L, 1);
        }
        return out;
    }
    size_t len = 0;
    const char *s = luaL_tolstring(L, idx, &len);
    std::string out(s, len);
    lua_pop(L, 1);
    return out;
}

TextLine readTextLine(lua_State *L, int textIdx, int argIdx, bool formatted)
{
    TextLine line;
    line.text = flattenText(L, textIdx);
    if (formatted)
    {
        line.limit = luax::checkfloat(L, argIdx);
        line.align = luaL_optstring(L, argIdx + 1, "left");
        argIdx += 2;
    }
    if (Matrix *t = luax::testobject<Matrix>(L, argIdx, math::TRANSFORM_TYPE))
    {
        line.transform = *t;
        line.hasTransform = true;
    }
    else if (!lua_isnoneornil(L, argIdx))
    {
        float x = luax::optfloat(L, argIdx, 0.0f);
        float y = luax::optfloat(L, argIdx + 1, 0.0f);
        float r = luax::optfloat(L, argIdx + 2, 0.0f);
        float sx = luax::optfloat(L, argIdx + 3, 1.0f);
        float sy = luax::optfloat(L, argIdx + 4, sx);
        float ox = luax::optfloat(L, argIdx + 5, 0.0f);
        float oy = luax::optfloat(L, argIdx + 6, 0.0f);
        float kx = luax::optfloat(L, argIdx + 7, 0.0f);
        float ky = luax::optfloat(L, argIdx + 8, 0.0f);
        line.transform = makeTransform(x, y, r, sx, sy, ox, oy, kx, ky);
        line.hasTransform = true;
    }
    return line;
}

int l_newText(lua_State *L)
{
    checkFont(L, 1);
    bool hasText = !lua_isnoneornil(L, 2);
    TextLine first;
    if (hasText)
    {
        first = readTextLine(L, 2, 3, false);
    }
    TextObj *text = luax::newobject<TextObj>(L, TEXT_TYPE);
    text->font.set(L, 1);
    if (hasText)
    {
        text->lines.push_back(first);
    }
    return 1;
}

int text_set(lua_State *L)
{
    TextObj *text = checkText(L, 1);
    text->lines.clear();
    if (!lua_isnoneornil(L, 2))
    {
        text->lines.push_back(readTextLine(L, 2, 3, false));
    }
    return 0;
}

int text_setf(lua_State *L)
{
    TextObj *text = checkText(L, 1);
    text->lines.clear();
    text->lines.push_back(readTextLine(L, 2, 3, true));
    return 0;
}

int text_add(lua_State *L)
{
    TextObj *text = checkText(L, 1);
    text->lines.push_back(readTextLine(L, 2, 3, false));
    lua_pushinteger(L, static_cast<lua_Integer>(text->lines.size()));
    return 1;
}

int text_addf(lua_State *L)
{
    TextObj *text = checkText(L, 1);
    text->lines.push_back(readTextLine(L, 2, 3, true));
    lua_pushinteger(L, static_cast<lua_Integer>(text->lines.size()));
    return 1;
}

int text_clear(lua_State *L)
{
    checkText(L, 1)->lines.clear();
    return 0;
}

void textBounds(lua_State *L, TextObj &text, float &width, float &height)
{
    text.font.push(L);
    FontObj *font = luax::testobject<FontObj>(L, -1, FONT_TYPE);
    width = 0.0f;
    height = 0.0f;
    if (font == nullptr)
    {
        lua_pop(L, 1);
        return;
    }
    for (const TextLine &line : text.lines)
    {
        std::vector<std::string> lines;
        float w = 0.0f;
        wrapText(*font, line.text, line.limit, lines, w);
        width = std::max(width, line.limit > 0.0f ? line.limit : w);
        height = std::max(height, static_cast<float>(lines.size()) * font->height() * font->lineHeight);
    }
    lua_pop(L, 1);
}

int text_getWidth(lua_State *L)
{
    float w, h;
    textBounds(L, *checkText(L, 1), w, h);
    lua_pushnumber(L, w);
    return 1;
}

int text_getHeight(lua_State *L)
{
    float w, h;
    textBounds(L, *checkText(L, 1), w, h);
    lua_pushnumber(L, h);
    return 1;
}

int text_getDimensions(lua_State *L)
{
    float w, h;
    textBounds(L, *checkText(L, 1), w, h);
    lua_pushnumber(L, w);
    lua_pushnumber(L, h);
    return 2;
}

int text_getFont(lua_State *L)
{
    checkText(L, 1)->font.push(L);
    return 1;
}

int text_setFont(lua_State *L)
{
    TextObj *text = checkText(L, 1);
    checkFont(L, 2);
    text->font.set(L, 2);
    return 0;
}

int text_gc(lua_State *L)
{
    TextObj *text = static_cast<TextObj *>(lua_touserdata(L, 1));
    text->font.clear(L);
    text->~TextObj();
    return 0;
}

const luaL_Reg TEXT_METHODS[] = {
    {"set", text_set},
    {"setf", text_setf},
    {"add", text_add},
    {"addf", text_addf},
    {"clear", text_clear},
    {"getWidth", text_getWidth},
    {"getHeight", text_getHeight},
    {"getDimensions", text_getDimensions},
    {"getFont", text_getFont},
    {"setFont", text_setFont},
    {nullptr, nullptr},
};

} // namespace

// ---------------------------------------------------------------------------
// Draw source
// ---------------------------------------------------------------------------

DrawSource checkDrawSource(lua_State *L, int idx)
{
    if (ImageObj *image = luax::testobject<ImageObj>(L, idx, IMAGE_TYPE))
    {
        return {image->texture, {0, 0, static_cast<float>(image->texture.width), static_cast<float>(image->texture.height)}, false};
    }
    if (CanvasObj *canvas = luax::testobject<CanvasObj>(L, idx, CANVAS_TYPE))
    {
        Texture2D tex = canvas->target.texture;
        return {tex, {0, 0, static_cast<float>(tex.width), static_cast<float>(tex.height)}, true};
    }
    luaL_error(L, "bad argument #%d (Drawable expected, got %s)", idx, luax::typename_(L, idx));
    return {};
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

const luaL_Reg OBJECT_FUNCS[] = {
    {"newImage", l_newImage},
    {"newQuad", l_newQuad},
    {"newCanvas", l_newCanvas},
    {"newFont", l_newFont},
    {"setFont", l_setFont},
    {"getFont", l_getFont},
    {"setNewFont", l_setNewFont},
    {"newSpriteBatch", l_newSpriteBatch},
    {"newText", l_newText},
    {nullptr, nullptr},
};

void registerObjectTypes(lua_State *L)
{
    luax::newtype(L, IMAGE_TYPE, IMAGE_METHODS, luax::gcobject<ImageObj>);
    luax::newtype(L, QUAD_TYPE, QUAD_METHODS, luax::gcobject<QuadObj>);
    luax::newtype(L, CANVAS_TYPE, CANVAS_METHODS, luax::gcobject<CanvasObj>);
    luax::newtype(L, FONT_TYPE, FONT_METHODS, luax::gcobject<FontObj>);
    luax::newtype(L, SPRITEBATCH_TYPE, SPRITEBATCH_METHODS, batch_gc);
    luax::newtype(L, TEXT_TYPE, TEXT_METHODS, text_gc);
    registerShaderType(L);
}

void shutdownObjects()
{
    // Lua state is already closed: drop the reference handle without touching it.
    g_currentFont = luax::Ref();
    for (auto &entry : g_defaultFonts)
    {
        if (entry.second.texture.id != 0)
        {
            UnloadFont(entry.second);
        }
    }
    g_defaultFonts.clear();
}

} // namespace graphics
} // namespace love
