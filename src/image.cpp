// image.cpp - love.image (ImageData backed by a CPU-side raylib Image).
#include "love.hpp"
#include "luax.hpp"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

namespace love
{
namespace
{

const char *IMAGEDATA_TYPE = "ImageData";

Image *checkImageData(lua_State *L, int idx)
{
    return luax::checkobject<Image>(L, idx, IMAGEDATA_TYPE);
}

int imagedata_gc(lua_State *L)
{
    Image *image = static_cast<Image *>(lua_touserdata(L, 1));
    if (image->data != nullptr)
    {
        UnloadImage(*image);
        image->data = nullptr;
    }
    return 0;
}

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

// newImageData(width, height [, format [, rawstring]]) | newImageData(filename | FileData)
int l_newImageData(lua_State *L)
{
    Image *image = nullptr;
    if (lua_isnumber(L, 1))
    {
        int width = static_cast<int>(luaL_checkinteger(L, 1));
        int height = static_cast<int>(luaL_checkinteger(L, 2));
        if (width <= 0 || height <= 0)
        {
            return luaL_error(L, "Invalid image size");
        }
        const char *format = luaL_optstring(L, 3, "rgba8");
        if (std::strcmp(format, "rgba8") != 0)
        {
            return luaL_error(L, "Unsupported ImageData format '%s' (only 'rgba8' is available)", format);
        }
        image = luax::newobject<Image>(L, IMAGEDATA_TYPE);
        *image = GenImageColor(width, height, BLANK);
        if (lua_type(L, 4) == LUA_TSTRING)
        {
            size_t len = 0;
            const char *raw = lua_tolstring(L, 4, &len);
            size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height) * 4;
            if (len != expected)
            {
                return luaL_error(L, "Raw data size (%d) does not match %dx%d rgba8 (%d bytes)",
                                  static_cast<int>(len), width, height, static_cast<int>(expected));
            }
            std::memcpy(image->data, raw, len);
        }
        return 1;
    }

    std::vector<unsigned char> data;
    std::string ext;
    if (lua_type(L, 1) == LUA_TSTRING)
    {
        std::string path = lua_tostring(L, 1);
        if (!filesystem::readFile(path, data))
        {
            return luaL_error(L, "Could not open file %s. Does not exist.", path.c_str());
        }
        ext = fileExtension(path);
    }
    else if (lua_isuserdata(L, 1))
    {
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
        ext = fileExtension(lua_tostring(L, -1) ? lua_tostring(L, -1) : "");
        lua_pop(L, 1);
    }
    else
    {
        return luaL_error(L, "bad argument #1 to 'newImageData' (filename, FileData or dimensions expected)");
    }

    Image loaded = LoadImageFromMemory(ext.c_str(), data.data(), static_cast<int>(data.size()));
    if (loaded.data == nullptr)
    {
        return luaL_error(L, "Could not decode image data");
    }
    ImageFormat(&loaded, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    image = luax::newobject<Image>(L, IMAGEDATA_TYPE);
    *image = loaded;
    return 1;
}

int imagedata_getWidth(lua_State *L)
{
    lua_pushinteger(L, checkImageData(L, 1)->width);
    return 1;
}

int imagedata_getHeight(lua_State *L)
{
    lua_pushinteger(L, checkImageData(L, 1)->height);
    return 1;
}

int imagedata_getDimensions(lua_State *L)
{
    Image *image = checkImageData(L, 1);
    lua_pushinteger(L, image->width);
    lua_pushinteger(L, image->height);
    return 2;
}

bool inside(const Image *image, int x, int y)
{
    return x >= 0 && y >= 0 && x < image->width && y < image->height;
}

int imagedata_getPixel(lua_State *L)
{
    Image *image = checkImageData(L, 1);
    int x = static_cast<int>(luaL_checkinteger(L, 2));
    int y = static_cast<int>(luaL_checkinteger(L, 3));
    if (!inside(image, x, y))
    {
        return luaL_error(L, "Attempt to get out-of-range pixel!");
    }
    Color c = GetImageColor(*image, x, y);
    lua_pushnumber(L, c.r / 255.0);
    lua_pushnumber(L, c.g / 255.0);
    lua_pushnumber(L, c.b / 255.0);
    lua_pushnumber(L, c.a / 255.0);
    return 4;
}

Color readPixelColor(lua_State *L, int idx)
{
    float c[4] = {0, 0, 0, 1};
    if (lua_istable(L, idx))
    {
        for (int i = 0; i < 4; ++i)
        {
            lua_rawgeti(L, idx, i + 1);
            if (lua_isnumber(L, -1))
            {
                c[i] = static_cast<float>(lua_tonumber(L, -1));
            }
            lua_pop(L, 1);
        }
    }
    else
    {
        c[0] = luax::checkfloat(L, idx);
        c[1] = luax::checkfloat(L, idx + 1);
        c[2] = luax::checkfloat(L, idx + 2);
        c[3] = luax::optfloat(L, idx + 3, 1.0f);
    }
    auto clamp = [](float v) { return static_cast<unsigned char>(std::min(1.0f, std::max(0.0f, v)) * 255.0f + 0.5f); };
    return Color{clamp(c[0]), clamp(c[1]), clamp(c[2]), clamp(c[3])};
}

int imagedata_setPixel(lua_State *L)
{
    Image *image = checkImageData(L, 1);
    int x = static_cast<int>(luaL_checkinteger(L, 2));
    int y = static_cast<int>(luaL_checkinteger(L, 3));
    if (!inside(image, x, y))
    {
        return luaL_error(L, "Attempt to set out-of-range pixel!");
    }
    ImageDrawPixel(image, x, y, readPixelColor(L, 4));
    return 0;
}

// mapPixel(fn [, x, y, width, height]) with fn(x, y, r, g, b, a) -> r, g, b, a
int imagedata_mapPixel(lua_State *L)
{
    Image *image = checkImageData(L, 1);
    luaL_checktype(L, 2, LUA_TFUNCTION);
    int x0 = luax::optint(L, 3, 0);
    int y0 = luax::optint(L, 4, 0);
    int w = luax::optint(L, 5, image->width - x0);
    int h = luax::optint(L, 6, image->height - y0);
    if (x0 < 0 || y0 < 0 || x0 + w > image->width || y0 + h > image->height)
    {
        return luaL_error(L, "Invalid rectangle dimensions");
    }
    unsigned char *pixels = static_cast<unsigned char *>(image->data);
    for (int y = y0; y < y0 + h; ++y)
    {
        for (int x = x0; x < x0 + w; ++x)
        {
            unsigned char *p = pixels + (static_cast<size_t>(y) * image->width + x) * 4;
            lua_pushvalue(L, 2);
            lua_pushinteger(L, x);
            lua_pushinteger(L, y);
            lua_pushnumber(L, p[0] / 255.0);
            lua_pushnumber(L, p[1] / 255.0);
            lua_pushnumber(L, p[2] / 255.0);
            lua_pushnumber(L, p[3] / 255.0);
            lua_call(L, 6, 4);
            Color c = readPixelColor(L, -4);
            p[0] = c.r;
            p[1] = c.g;
            p[2] = c.b;
            p[3] = c.a;
            lua_pop(L, 4);
        }
    }
    return 0;
}

// paste(source, dx, dy, sx, sy, sw, sh)
int imagedata_paste(lua_State *L)
{
    Image *dst = checkImageData(L, 1);
    Image *src = checkImageData(L, 2);
    int dx = static_cast<int>(luaL_checkinteger(L, 3));
    int dy = static_cast<int>(luaL_checkinteger(L, 4));
    int sx = luax::optint(L, 5, 0);
    int sy = luax::optint(L, 6, 0);
    int sw = luax::optint(L, 7, src->width);
    int sh = luax::optint(L, 8, src->height);
    Rectangle srcRec = {static_cast<float>(sx), static_cast<float>(sy), static_cast<float>(sw), static_cast<float>(sh)};
    Rectangle dstRec = {static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(sw), static_cast<float>(sh)};
    // ImageDraw blends; Love2D copies. Use a straight pixel copy instead.
    unsigned char *d = static_cast<unsigned char *>(dst->data);
    unsigned char *s = static_cast<unsigned char *>(src->data);
    (void)srcRec;
    (void)dstRec;
    for (int y = 0; y < sh; ++y)
    {
        int ty = dy + y;
        int fy = sy + y;
        if (ty < 0 || ty >= dst->height || fy < 0 || fy >= src->height)
        {
            continue;
        }
        for (int x = 0; x < sw; ++x)
        {
            int tx = dx + x;
            int fx = sx + x;
            if (tx < 0 || tx >= dst->width || fx < 0 || fx >= src->width)
            {
                continue;
            }
            std::memcpy(d + (static_cast<size_t>(ty) * dst->width + tx) * 4, s + (static_cast<size_t>(fy) * src->width + fx) * 4, 4);
        }
    }
    return 0;
}

// encode(format [, filename]) -> FileData (and writes to the save directory)
int imagedata_encode(lua_State *L)
{
    Image *image = checkImageData(L, 1);
    static const char *const names[] = {"png", "tga", "bmp", "jpg"};
    static const int values[] = {0, 1, 2, 3};
    int format = luax::checkenum(L, 2, names, values, "image format");
    const char *ext = (format == 0) ? ".png" : (format == 1) ? ".tga" : (format == 2) ? ".bmp" : ".jpg";
    int size = 0;
    unsigned char *bytes = ExportImageToMemory(*image, ext, &size);
    if (bytes == nullptr)
    {
        return luaL_error(L, "Could not encode image as %s", names[format]);
    }
    if (lua_type(L, 3) == LUA_TSTRING)
    {
        std::string real = filesystem::resolveWrite(lua_tostring(L, 3));
        if (real.empty() || !SaveFileData(real.c_str(), bytes, size))
        {
            RL_FREE(bytes);
            return luaL_error(L, "Could not write %s", lua_tostring(L, 3));
        }
    }
    // Hand the bytes to love.filesystem.newFileData(contents, name).
    lua_getglobal(L, "love");
    lua_getfield(L, -1, "filesystem");
    lua_getfield(L, -1, "newFileData");
    lua_pushlstring(L, reinterpret_cast<const char *>(bytes), static_cast<size_t>(size));
    lua_pushfstring(L, "%s%s", lua_type(L, 3) == LUA_TSTRING ? "" : "image", lua_type(L, 3) == LUA_TSTRING ? lua_tostring(L, 3) : ext);
    lua_call(L, 2, 1);
    RL_FREE(bytes);
    return 1;
}

int imagedata_getFormat(lua_State *L)
{
    lua_pushstring(L, "rgba8");
    return 1;
}

int imagedata_getString(lua_State *L)
{
    Image *image = checkImageData(L, 1);
    size_t size = static_cast<size_t>(image->width) * static_cast<size_t>(image->height) * 4;
    lua_pushlstring(L, static_cast<const char *>(image->data), size);
    return 1;
}

int imagedata_getSize(lua_State *L)
{
    Image *image = checkImageData(L, 1);
    lua_pushinteger(L, static_cast<lua_Integer>(image->width) * image->height * 4);
    return 1;
}

int imagedata_clone(lua_State *L)
{
    Image *image = checkImageData(L, 1);
    Image *copy = luax::newobject<Image>(L, IMAGEDATA_TYPE);
    *copy = ImageCopy(*image);
    return 1;
}

int imagedata_tostring(lua_State *L)
{
    Image *image = checkImageData(L, 1);
    lua_pushfstring(L, "ImageData: %dx%d", image->width, image->height);
    return 1;
}

const luaL_Reg IMAGEDATA_METHODS[] = {
    {"getWidth", imagedata_getWidth},
    {"getHeight", imagedata_getHeight},
    {"getDimensions", imagedata_getDimensions},
    {"getPixel", imagedata_getPixel},
    {"setPixel", imagedata_setPixel},
    {"mapPixel", imagedata_mapPixel},
    {"paste", imagedata_paste},
    {"encode", imagedata_encode},
    {"getFormat", imagedata_getFormat},
    {"getString", imagedata_getString},
    {"getSize", imagedata_getSize},
    {"clone", imagedata_clone},
    {"__tostring", imagedata_tostring},
    {nullptr, nullptr},
};

int l_isCompressed(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

int l_newCompressedData(lua_State *L)
{
    return luaL_error(L, "love.image.newCompressedData: compressed textures are not supported");
}

const luaL_Reg FUNCS[] = {
    {"newImageData", l_newImageData},
    {"isCompressed", l_isCompressed},
    {"newCompressedData", l_newCompressedData},
    {nullptr, nullptr},
};

} // namespace

int open_image(lua_State *L)
{
    luax::newtype(L, IMAGEDATA_TYPE, IMAGEDATA_METHODS, imagedata_gc);
    luaL_newlib(L, FUNCS);
    return 1;
}

} // namespace love
