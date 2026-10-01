// filesystem.cpp - love.filesystem
//
// Love2D exposes a virtual filesystem rooted at the game directory with a
// writable save directory layered on top. LoveRay implements the same model
// directly on the host filesystem (no archive support yet).
#include "love.hpp"
#include "luax.hpp"

#include <sys/stat.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace love
{
namespace filesystem
{

namespace
{

std::string g_executable;
std::string g_source;
std::string g_identity;
bool g_appendIdentity = false;
std::string g_requirePath = "?.lua;?/init.lua";

// Modification times of the files that trigger a restart in hot-reload mode.
struct Watched
{
    std::string path;
    long long modtime;
};
std::vector<Watched> g_watched;

std::string normalize(std::string path)
{
    for (char &c : path)
    {
        if (c == '\\')
        {
            c = '/';
        }
    }
    while (path.size() > 1 && path.back() == '/')
    {
        path.pop_back();
    }
    return path;
}

// Game paths are always relative to the virtual root. Absolute paths and
// ".." components are rejected, as Love2D does.
bool isSafeRelative(const std::string &path)
{
    if (path.empty())
    {
        return true;
    }
    if (path[0] == '/' || (path.size() > 1 && path[1] == ':'))
    {
        return false;
    }
    size_t start = 0;
    while (start <= path.size())
    {
        size_t end = path.find('/', start);
        if (end == std::string::npos)
        {
            end = path.size();
        }
        if (path.compare(start, end - start, "..") == 0)
        {
            return false;
        }
        start = end + 1;
    }
    return true;
}

std::string join(const std::string &base, const std::string &path)
{
    if (path.empty())
    {
        return base;
    }
    if (base.empty())
    {
        return path;
    }
    return base + "/" + path;
}

long long modTimeOf(const std::string &realPath)
{
    struct stat st;
    if (stat(realPath.c_str(), &st) != 0)
    {
        return -1;
    }
    return static_cast<long long>(st.st_mtime);
}

std::string homeDirectory()
{
#if defined(_WIN32)
    const char *profile = std::getenv("USERPROFILE");
    if (profile != nullptr)
    {
        return normalize(profile);
    }
    return "C:/";
#else
    const char *home = std::getenv("HOME");
    return home != nullptr ? std::string(home) : std::string("/tmp");
#endif
}

std::string appdataDirectory()
{
#if defined(_WIN32)
    const char *appdata = std::getenv("APPDATA");
    if (appdata != nullptr)
    {
        return normalize(appdata);
    }
    return homeDirectory();
#elif defined(__APPLE__)
    return homeDirectory() + "/Library/Application Support";
#else
    const char *xdg = std::getenv("XDG_DATA_HOME");
    if (xdg != nullptr && xdg[0] != '\0')
    {
        return std::string(xdg);
    }
    return homeDirectory() + "/.local/share";
#endif
}

struct Info
{
    bool exists = false;
    const char *type = "other";
    long long size = 0;
    long long modtime = -1;
};

Info infoOf(const std::string &realPath)
{
    Info info;
    struct stat st;
    if (realPath.empty() || stat(realPath.c_str(), &st) != 0)
    {
        return info;
    }
    info.exists = true;
    info.modtime = static_cast<long long>(st.st_mtime);
    if (S_ISDIR(st.st_mode))
    {
        info.type = "directory";
    }
    else if (S_ISREG(st.st_mode))
    {
        info.type = "file";
        info.size = static_cast<long long>(st.st_size);
    }
#if !defined(_WIN32)
    else if (S_ISLNK(st.st_mode))
    {
        info.type = "symlink";
    }
#endif
    return info;
}

void pushInfo(lua_State *L, const Info &info)
{
    lua_newtable(L);
    lua_pushstring(L, info.type);
    lua_setfield(L, -2, "type");
    if (std::strcmp(info.type, "file") == 0)
    {
        lua_pushinteger(L, info.size);
        lua_setfield(L, -2, "size");
    }
    if (info.modtime >= 0)
    {
        lua_pushinteger(L, info.modtime);
        lua_setfield(L, -2, "modtime");
    }
}

} // namespace

// ---------------------------------------------------------------------------
// C++ API used by the other modules
// ---------------------------------------------------------------------------

void setSource(const std::string &dir)
{
    g_source = normalize(dir);
    g_watched.clear();
    for (const char *name : {"main.lua", "conf.lua"})
    {
        std::string real = join(g_source, name);
        g_watched.push_back({real, modTimeOf(real)});
    }
}

const std::string &getSource()
{
    return g_source;
}

void setIdentity(const std::string &identity)
{
    g_identity = identity;
}

const std::string &getIdentity()
{
    return g_identity;
}

std::string getSaveDirectory()
{
    std::string base = appdataDirectory();
#if defined(_WIN32) || defined(__APPLE__)
    base += "/LoveRay";
#else
    base += "/loveray";
#endif
    if (!g_identity.empty())
    {
        base += "/" + g_identity;
    }
    return base;
}

std::string resolveRead(const std::string &path)
{
    std::string clean = normalize(path);
    if (!isSafeRelative(clean))
    {
        return "";
    }
    if (!g_identity.empty())
    {
        std::string save = join(getSaveDirectory(), clean);
        if (infoOf(save).exists)
        {
            return save;
        }
    }
    std::string source = join(g_source.empty() ? std::string(".") : g_source, clean);
    if (infoOf(source).exists)
    {
        return source;
    }
    return "";
}

std::string resolveWrite(const std::string &path)
{
    std::string clean = normalize(path);
    if (!isSafeRelative(clean) || clean.empty())
    {
        return "";
    }
    std::string full = join(getSaveDirectory(), clean);
    std::error_code ec;
    fs::create_directories(fs::path(full).parent_path(), ec);
    if (ec)
    {
        return "";
    }
    return full;
}

bool readFile(const std::string &path, std::vector<unsigned char> &out)
{
    std::string real = resolveRead(path);
    if (real.empty())
    {
        return false;
    }
    std::ifstream in(real, std::ios::binary);
    if (!in)
    {
        return false;
    }
    in.seekg(0, std::ios::end);
    std::streamoff size = in.tellg();
    in.seekg(0, std::ios::beg);
    out.resize(static_cast<size_t>(size > 0 ? size : 0));
    if (size > 0)
    {
        in.read(reinterpret_cast<char *>(out.data()), size);
    }
    return static_cast<bool>(in) || size == 0;
}

// ---------------------------------------------------------------------------
// File object
// ---------------------------------------------------------------------------

namespace
{

const char *FILE_TYPE = "File";
const char *FILEDATA_TYPE = "FileData";

struct File
{
    std::string name;
    std::FILE *handle = nullptr;
    char mode = 'c'; // 'r', 'w', 'a' or 'c' (closed)

    ~File()
    {
        close();
    }

    void close()
    {
        if (handle != nullptr)
        {
            std::fclose(handle);
            handle = nullptr;
        }
        mode = 'c';
    }
};

struct FileData
{
    std::string name;
    std::string contents;
};

File *checkFile(lua_State *L, int idx)
{
    return luax::checkobject<File>(L, idx, FILE_TYPE);
}

bool openFile(File &file, char mode, std::string &err)
{
    file.close();
    std::string real;
    const char *fmode = "rb";
    if (mode == 'r')
    {
        real = resolveRead(file.name);
        if (real.empty())
        {
            err = "Could not open file " + file.name + ". Does not exist.";
            return false;
        }
    }
    else
    {
        real = resolveWrite(file.name);
        fmode = (mode == 'w') ? "wb" : "ab";
        if (real.empty())
        {
            err = "Could not open file " + file.name + " for writing.";
            return false;
        }
    }
    file.handle = std::fopen(real.c_str(), fmode);
    if (file.handle == nullptr)
    {
        err = "Could not open file " + file.name + ".";
        return false;
    }
    file.mode = mode;
    return true;
}

char checkMode(lua_State *L, int idx)
{
    static const char *const names[] = {"r", "w", "a", "c"};
    static const int values[] = {'r', 'w', 'a', 'c'};
    return static_cast<char>(luax::checkenum(L, idx, names, values, "file mode"));
}

int file_open(lua_State *L)
{
    File *file = checkFile(L, 1);
    char mode = checkMode(L, 2);
    std::string err;
    if (mode == 'c')
    {
        file->close();
        lua_pushboolean(L, 1);
        return 1;
    }
    if (!openFile(*file, mode, err))
    {
        lua_pushboolean(L, 0);
        lua_pushstring(L, err.c_str());
        return 2;
    }
    lua_pushboolean(L, 1);
    return 1;
}

int file_close(lua_State *L)
{
    File *file = checkFile(L, 1);
    bool wasOpen = file->handle != nullptr;
    file->close();
    lua_pushboolean(L, wasOpen);
    return 1;
}

int file_isOpen(lua_State *L)
{
    lua_pushboolean(L, checkFile(L, 1)->handle != nullptr);
    return 1;
}

int file_getMode(lua_State *L)
{
    char mode[2] = {checkFile(L, 1)->mode, '\0'};
    lua_pushstring(L, mode);
    return 1;
}

int file_getFilename(lua_State *L)
{
    lua_pushstring(L, checkFile(L, 1)->name.c_str());
    return 1;
}

long long fileSize(File &file)
{
    if (file.handle == nullptr)
    {
        std::string real = resolveRead(file.name);
        return infoOf(real).size;
    }
    long pos = std::ftell(file.handle);
    std::fseek(file.handle, 0, SEEK_END);
    long size = std::ftell(file.handle);
    std::fseek(file.handle, pos, SEEK_SET);
    return size;
}

int file_getSize(lua_State *L)
{
    lua_pushinteger(L, fileSize(*checkFile(L, 1)));
    return 1;
}

int file_read(lua_State *L)
{
    File *file = checkFile(L, 1);
    if (file->handle == nullptr || file->mode != 'r')
    {
        lua_pushnil(L);
        lua_pushstring(L, "File is not opened for reading.");
        return 2;
    }
    long long remaining = fileSize(*file) - std::ftell(file->handle);
    long long count = static_cast<long long>(luaL_optinteger(L, 2, remaining));
    if (count < 0 || count > remaining)
    {
        count = remaining;
    }
    std::string buffer(static_cast<size_t>(count), '\0');
    size_t got = count > 0 ? std::fread(&buffer[0], 1, static_cast<size_t>(count), file->handle) : 0;
    lua_pushlstring(L, buffer.data(), got);
    lua_pushinteger(L, static_cast<lua_Integer>(got));
    return 2;
}

int file_write(lua_State *L)
{
    File *file = checkFile(L, 1);
    if (file->handle == nullptr || file->mode == 'r')
    {
        lua_pushboolean(L, 0);
        lua_pushstring(L, "File is not opened for writing.");
        return 2;
    }
    size_t len = 0;
    const char *data;
    if (FileData *fd = luax::testobject<FileData>(L, 2, FILEDATA_TYPE))
    {
        data = fd->contents.data();
        len = fd->contents.size();
    }
    else
    {
        data = luaL_checklstring(L, 2, &len);
    }
    size_t count = static_cast<size_t>(luaL_optinteger(L, 3, static_cast<lua_Integer>(len)));
    if (count > len)
    {
        count = len;
    }
    size_t written = std::fwrite(data, 1, count, file->handle);
    lua_pushboolean(L, written == count);
    return 1;
}

int file_flush(lua_State *L)
{
    File *file = checkFile(L, 1);
    if (file->handle != nullptr)
    {
        std::fflush(file->handle);
    }
    lua_pushboolean(L, file->handle != nullptr);
    return 1;
}

int file_seek(lua_State *L)
{
    File *file = checkFile(L, 1);
    long pos = static_cast<long>(luaL_checkinteger(L, 2));
    lua_pushboolean(L, file->handle != nullptr && std::fseek(file->handle, pos, SEEK_SET) == 0);
    return 1;
}

int file_tell(lua_State *L)
{
    File *file = checkFile(L, 1);
    lua_pushinteger(L, file->handle != nullptr ? std::ftell(file->handle) : -1);
    return 1;
}

int file_isEOF(lua_State *L)
{
    File *file = checkFile(L, 1);
    bool eof = file->handle == nullptr || std::ftell(file->handle) >= fileSize(*file);
    lua_pushboolean(L, eof);
    return 1;
}

int file_lines_iterator(lua_State *L)
{
    File *file = checkFile(L, lua_upvalueindex(1));
    if (file->handle == nullptr)
    {
        return 0;
    }
    std::string line;
    int c;
    bool any = false;
    while ((c = std::fgetc(file->handle)) != EOF)
    {
        any = true;
        if (c == '\n')
        {
            break;
        }
        line.push_back(static_cast<char>(c));
    }
    if (!any)
    {
        if (lua_toboolean(L, lua_upvalueindex(2)))
        {
            file->close();
        }
        return 0;
    }
    if (!line.empty() && line.back() == '\r')
    {
        line.pop_back();
    }
    lua_pushlstring(L, line.data(), line.size());
    return 1;
}

int file_lines(lua_State *L)
{
    File *file = checkFile(L, 1);
    bool autoClose = false;
    if (file->handle == nullptr)
    {
        std::string err;
        if (!openFile(*file, 'r', err))
        {
            return luaL_error(L, "%s", err.c_str());
        }
        autoClose = true;
    }
    lua_pushvalue(L, 1);
    lua_pushboolean(L, autoClose);
    lua_pushcclosure(L, file_lines_iterator, 2);
    return 1;
}

int file_tostring(lua_State *L)
{
    File *file = checkFile(L, 1);
    lua_pushfstring(L, "File: %s", file->name.c_str());
    return 1;
}

const luaL_Reg FILE_METHODS[] = {
    {"open", file_open},
    {"close", file_close},
    {"isOpen", file_isOpen},
    {"getMode", file_getMode},
    {"getFilename", file_getFilename},
    {"getSize", file_getSize},
    {"read", file_read},
    {"write", file_write},
    {"flush", file_flush},
    {"seek", file_seek},
    {"tell", file_tell},
    {"isEOF", file_isEOF},
    {"lines", file_lines},
    {"__tostring", file_tostring},
    {nullptr, nullptr},
};

// FileData ------------------------------------------------------------------

FileData *checkFileData(lua_State *L, int idx)
{
    return luax::checkobject<FileData>(L, idx, FILEDATA_TYPE);
}

int filedata_getString(lua_State *L)
{
    FileData *fd = checkFileData(L, 1);
    lua_pushlstring(L, fd->contents.data(), fd->contents.size());
    return 1;
}

int filedata_getSize(lua_State *L)
{
    lua_pushinteger(L, static_cast<lua_Integer>(checkFileData(L, 1)->contents.size()));
    return 1;
}

int filedata_getFilename(lua_State *L)
{
    lua_pushstring(L, checkFileData(L, 1)->name.c_str());
    return 1;
}

int filedata_getExtension(lua_State *L)
{
    const std::string &name = checkFileData(L, 1)->name;
    size_t dot = name.find_last_of('.');
    if (dot == std::string::npos || dot + 1 >= name.size())
    {
        lua_pushstring(L, "");
    }
    else
    {
        lua_pushstring(L, name.c_str() + dot + 1);
    }
    return 1;
}

const luaL_Reg FILEDATA_METHODS[] = {
    {"getString", filedata_getString},
    {"getSize", filedata_getSize},
    {"getFilename", filedata_getFilename},
    {"getExtension", filedata_getExtension},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// love.filesystem.*
// ---------------------------------------------------------------------------

int l_init(lua_State *L)
{
    g_executable = normalize(luaL_checkstring(L, 1));
    return 0;
}

int l_setSource(lua_State *L)
{
    setSource(luaL_checkstring(L, 1));
    return 0;
}

int l_getSource(lua_State *L)
{
    lua_pushstring(L, g_source.c_str());
    return 1;
}

int l_getSourceBaseDirectory(lua_State *L)
{
    fs::path p(g_source);
    lua_pushstring(L, normalize(p.parent_path().string()).c_str());
    return 1;
}

int l_setIdentity(lua_State *L)
{
    g_identity = luaL_checkstring(L, 1);
    g_appendIdentity = luax::optboolean(L, 2, false);
    return 0;
}

int l_getIdentity(lua_State *L)
{
    lua_pushstring(L, g_identity.c_str());
    return 1;
}

int l_getSaveDirectory(lua_State *L)
{
    lua_pushstring(L, getSaveDirectory().c_str());
    return 1;
}

int l_getWorkingDirectory(lua_State *L)
{
    std::error_code ec;
    lua_pushstring(L, normalize(fs::current_path(ec).string()).c_str());
    return 1;
}

int l_getUserDirectory(lua_State *L)
{
    lua_pushstring(L, homeDirectory().c_str());
    return 1;
}

int l_getAppdataDirectory(lua_State *L)
{
    lua_pushstring(L, appdataDirectory().c_str());
    return 1;
}

int l_getExecutablePath(lua_State *L)
{
    lua_pushstring(L, g_executable.c_str());
    return 1;
}

int l_getRealDirectory(lua_State *L)
{
    std::string path = normalize(luaL_checkstring(L, 1));
    if (!isSafeRelative(path))
    {
        lua_pushnil(L);
        lua_pushstring(L, "Invalid path");
        return 2;
    }
    if (!g_identity.empty() && infoOf(join(getSaveDirectory(), path)).exists)
    {
        lua_pushstring(L, getSaveDirectory().c_str());
        return 1;
    }
    std::string source = g_source.empty() ? std::string(".") : g_source;
    if (infoOf(join(source, path)).exists)
    {
        lua_pushstring(L, source.c_str());
        return 1;
    }
    lua_pushnil(L);
    lua_pushstring(L, "File does not exist");
    return 2;
}

// getInfo(path [, filtertype] [, info table])
int l_getInfo(lua_State *L)
{
    const char *path = luaL_checkstring(L, 1);
    const char *filter = nullptr;
    int tableIndex = 0;
    if (lua_type(L, 2) == LUA_TSTRING)
    {
        filter = lua_tostring(L, 2);
        if (lua_type(L, 3) == LUA_TTABLE)
        {
            tableIndex = 3;
        }
    }
    else if (lua_type(L, 2) == LUA_TTABLE)
    {
        tableIndex = 2;
    }

    Info info = infoOf(resolveRead(path));
    if (!info.exists || (filter != nullptr && std::strcmp(filter, info.type) != 0))
    {
        lua_pushnil(L);
        return 1;
    }
    if (tableIndex != 0)
    {
        lua_pushvalue(L, tableIndex);
        lua_pushstring(L, info.type);
        lua_setfield(L, -2, "type");
        if (std::strcmp(info.type, "file") == 0)
        {
            lua_pushinteger(L, info.size);
            lua_setfield(L, -2, "size");
        }
        lua_pushinteger(L, info.modtime);
        lua_setfield(L, -2, "modtime");
        return 1;
    }
    pushInfo(L, info);
    return 1;
}

// Internal: information about an absolute host path (used by boot.lua).
int l_getRealInfo(lua_State *L)
{
    Info info = infoOf(normalize(luaL_checkstring(L, 1)));
    if (!info.exists)
    {
        lua_pushnil(L);
        return 1;
    }
    pushInfo(L, info);
    return 1;
}

int l_read(lua_State *L)
{
    // read([container,] name [, size])
    int nameIndex = 1;
    bool asData = false;
    if (lua_gettop(L) >= 2 && lua_type(L, 1) == LUA_TSTRING && lua_type(L, 2) == LUA_TSTRING)
    {
        const char *container = lua_tostring(L, 1);
        if (std::strcmp(container, "data") == 0 || std::strcmp(container, "string") == 0)
        {
            asData = std::strcmp(container, "data") == 0;
            nameIndex = 2;
        }
    }
    const char *name = luaL_checkstring(L, nameIndex);
    std::vector<unsigned char> data;
    if (!readFile(name, data))
    {
        lua_pushnil(L);
        lua_pushfstring(L, "Could not open file %s. Does not exist.", name);
        return 2;
    }
    size_t size = data.size();
    if (lua_isinteger(L, nameIndex + 1))
    {
        lua_Integer limit = lua_tointeger(L, nameIndex + 1);
        if (limit >= 0 && static_cast<size_t>(limit) < size)
        {
            size = static_cast<size_t>(limit);
        }
    }
    if (asData)
    {
        FileData *fd = luax::newobject<FileData>(L, FILEDATA_TYPE);
        fd->name = name;
        fd->contents.assign(reinterpret_cast<const char *>(data.data()), size);
    }
    else
    {
        lua_pushlstring(L, reinterpret_cast<const char *>(data.data()), size);
    }
    lua_pushinteger(L, static_cast<lua_Integer>(size));
    return 2;
}

int writeImpl(lua_State *L, const char *fmode)
{
    const char *name = luaL_checkstring(L, 1);
    size_t len = 0;
    const char *data;
    if (FileData *fd = luax::testobject<FileData>(L, 2, FILEDATA_TYPE))
    {
        data = fd->contents.data();
        len = fd->contents.size();
    }
    else
    {
        data = luaL_checklstring(L, 2, &len);
    }
    size_t count = static_cast<size_t>(luaL_optinteger(L, 3, static_cast<lua_Integer>(len)));
    if (count > len)
    {
        count = len;
    }
    std::string real = resolveWrite(name);
    if (real.empty())
    {
        lua_pushboolean(L, 0);
        lua_pushfstring(L, "Could not open file %s for writing.", name);
        return 2;
    }
    std::FILE *f = std::fopen(real.c_str(), fmode);
    if (f == nullptr)
    {
        lua_pushboolean(L, 0);
        lua_pushfstring(L, "Could not open file %s for writing.", name);
        return 2;
    }
    size_t written = std::fwrite(data, 1, count, f);
    std::fclose(f);
    lua_pushboolean(L, written == count);
    return 1;
}

int l_write(lua_State *L)
{
    return writeImpl(L, "wb");
}

int l_append(lua_State *L)
{
    return writeImpl(L, "ab");
}

int l_remove(lua_State *L)
{
    std::string path = normalize(luaL_checkstring(L, 1));
    if (!isSafeRelative(path) || path.empty())
    {
        lua_pushboolean(L, 0);
        return 1;
    }
    std::error_code ec;
    bool ok = fs::remove(join(getSaveDirectory(), path), ec) && !ec;
    lua_pushboolean(L, ok);
    return 1;
}

int l_createDirectory(lua_State *L)
{
    std::string path = normalize(luaL_checkstring(L, 1));
    if (!isSafeRelative(path) || path.empty())
    {
        lua_pushboolean(L, 0);
        return 1;
    }
    std::error_code ec;
    fs::create_directories(join(getSaveDirectory(), path), ec);
    lua_pushboolean(L, !ec);
    return 1;
}

int l_getDirectoryItems(lua_State *L)
{
    std::string path = normalize(luaL_checkstring(L, 1));
    lua_newtable(L);
    if (!isSafeRelative(path))
    {
        return 1;
    }
    std::vector<std::string> roots;
    if (!g_identity.empty())
    {
        roots.push_back(join(getSaveDirectory(), path));
    }
    roots.push_back(join(g_source.empty() ? std::string(".") : g_source, path));

    // Items present in both locations are reported once.
    lua_newtable(L); // seen set
    int n = 0;
    for (const std::string &root : roots)
    {
        std::error_code ec;
        if (!fs::is_directory(root, ec))
        {
            continue;
        }
        for (const auto &entry : fs::directory_iterator(root, ec))
        {
            std::string name = entry.path().filename().string();
            lua_getfield(L, -1, name.c_str());
            bool seen = !lua_isnil(L, -1);
            lua_pop(L, 1);
            if (seen)
            {
                continue;
            }
            lua_pushboolean(L, 1);
            lua_setfield(L, -2, name.c_str());
            lua_pushstring(L, name.c_str());
            lua_rawseti(L, -3, ++n);
        }
    }
    lua_pop(L, 1);

    // Stable, deterministic order.
    lua_getglobal(L, "table");
    lua_getfield(L, -1, "sort");
    lua_pushvalue(L, -3);
    lua_call(L, 1, 0);
    lua_pop(L, 1);
    return 1;
}

int l_lines_iterator(lua_State *L)
{
    return file_lines_iterator(L);
}

int l_lines(lua_State *L)
{
    const char *name = luaL_checkstring(L, 1);
    File *file = luax::newobject<File>(L, FILE_TYPE);
    file->name = name;
    std::string err;
    if (!openFile(*file, 'r', err))
    {
        return luaL_error(L, "%s", err.c_str());
    }
    lua_pushboolean(L, 1);
    lua_pushcclosure(L, l_lines_iterator, 2);
    return 1;
}

int l_load(lua_State *L)
{
    const char *name = luaL_checkstring(L, 1);
    std::vector<unsigned char> data;
    if (!readFile(name, data))
    {
        lua_pushnil(L);
        lua_pushfstring(L, "Could not open file %s. Does not exist.", name);
        return 2;
    }
    std::string chunkname = std::string("@") + name;
    int status = luaL_loadbufferx(L, reinterpret_cast<const char *>(data.data()), data.size(), chunkname.c_str(), nullptr);
    if (status != LUA_OK)
    {
        lua_pushnil(L);
        lua_insert(L, -2);
        return 2;
    }
    return 1;
}

int l_exists(lua_State *L)
{
    lua_pushboolean(L, infoOf(resolveRead(luaL_checkstring(L, 1))).exists);
    return 1;
}

int l_isFile(lua_State *L)
{
    Info info = infoOf(resolveRead(luaL_checkstring(L, 1)));
    lua_pushboolean(L, info.exists && std::strcmp(info.type, "file") == 0);
    return 1;
}

int l_isDirectory(lua_State *L)
{
    Info info = infoOf(resolveRead(luaL_checkstring(L, 1)));
    lua_pushboolean(L, info.exists && std::strcmp(info.type, "directory") == 0);
    return 1;
}

int l_isFused(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

int l_mount(lua_State *L)
{
    // Archives are not supported yet; directories inside the source already
    // are visible, so mounting them is a no-op that reports success.
    std::string archive = normalize(luaL_checkstring(L, 1));
    Info info = infoOf(archive);
    lua_pushboolean(L, info.exists && std::strcmp(info.type, "directory") == 0);
    return 1;
}

int l_unmount(lua_State *L)
{
    lua_pushboolean(L, 1);
    return 1;
}

int l_newFile(lua_State *L)
{
    const char *name = luaL_checkstring(L, 1);
    bool hasMode = !lua_isnoneornil(L, 2);
    char mode = hasMode ? checkMode(L, 2) : 'c';
    File *file = luax::newobject<File>(L, FILE_TYPE);
    file->name = name;
    if (hasMode)
    {
        std::string err;
        if (mode != 'c' && !openFile(*file, mode, err))
        {
            lua_pop(L, 1);
            lua_pushnil(L);
            lua_pushstring(L, err.c_str());
            return 2;
        }
    }
    return 1;
}

int l_newFileData(lua_State *L)
{
    // newFileData(contents, name) or newFileData(filepath)
    if (lua_gettop(L) == 1 || lua_isnoneornil(L, 2))
    {
        const char *name = luaL_checkstring(L, 1);
        std::vector<unsigned char> data;
        if (!readFile(name, data))
        {
            lua_pushnil(L);
            lua_pushfstring(L, "Could not open file %s. Does not exist.", name);
            return 2;
        }
        FileData *fd = luax::newobject<FileData>(L, FILEDATA_TYPE);
        fd->name = name;
        fd->contents.assign(reinterpret_cast<const char *>(data.data()), data.size());
        return 1;
    }
    size_t len = 0;
    const char *contents = luaL_checklstring(L, 1, &len);
    const char *name = luaL_checkstring(L, 2);
    FileData *fd = luax::newobject<FileData>(L, FILEDATA_TYPE);
    fd->name = name;
    fd->contents.assign(contents, len);
    return 1;
}

int l_setRequirePath(lua_State *L)
{
    g_requirePath = luaL_checkstring(L, 1);
    return 0;
}

int l_getRequirePath(lua_State *L)
{
    lua_pushstring(L, g_requirePath.c_str());
    return 1;
}

int l_setCRequirePath(lua_State *L)
{
    return 0;
}

int l_getCRequirePath(lua_State *L)
{
    lua_pushstring(L, "");
    return 1;
}

int l_setSymlinksEnabled(lua_State *L)
{
    return 0;
}

int l_areSymlinksEnabled(lua_State *L)
{
    lua_pushboolean(L, 1);
    return 1;
}

// Internal: true when main.lua or conf.lua changed on disk since boot.
int l_sourceChanged(lua_State *L)
{
    bool changed = false;
    for (Watched &w : g_watched)
    {
        long long now = modTimeOf(w.path);
        if (now != w.modtime)
        {
            w.modtime = now;
            changed = true;
        }
    }
    lua_pushboolean(L, changed);
    return 1;
}

// package.searchers entry resolving modules through love.filesystem.
int l_searcher(lua_State *L)
{
    std::string module = luaL_checkstring(L, 1);
    for (char &c : module)
    {
        if (c == '.')
        {
            c = '/';
        }
    }
    std::string tried;
    size_t start = 0;
    while (start <= g_requirePath.size())
    {
        size_t end = g_requirePath.find(';', start);
        if (end == std::string::npos)
        {
            end = g_requirePath.size();
        }
        std::string pattern = g_requirePath.substr(start, end - start);
        start = end + 1;
        if (pattern.empty())
        {
            continue;
        }
        std::string candidate;
        for (char c : pattern)
        {
            if (c == '?')
            {
                candidate += module;
            }
            else
            {
                candidate.push_back(c);
            }
        }
        std::vector<unsigned char> data;
        if (readFile(candidate, data))
        {
            std::string chunkname = "@" + candidate;
            if (luaL_loadbuffer(L, reinterpret_cast<const char *>(data.data()), data.size(), chunkname.c_str()) != LUA_OK)
            {
                return luaL_error(L, "error loading module '%s' from file '%s':\n\t%s",
                                  lua_tostring(L, 1), candidate.c_str(), lua_tostring(L, -1));
            }
            lua_pushstring(L, candidate.c_str());
            return 2;
        }
        tried += "\n\tno file '" + candidate + "' in LOVE game directories.";
    }
    lua_pushstring(L, tried.c_str());
    return 1;
}

const luaL_Reg FUNCS[] = {
    {"init", l_init},
    {"setSource", l_setSource},
    {"getSource", l_getSource},
    {"getSourceBaseDirectory", l_getSourceBaseDirectory},
    {"setIdentity", l_setIdentity},
    {"getIdentity", l_getIdentity},
    {"getSaveDirectory", l_getSaveDirectory},
    {"getWorkingDirectory", l_getWorkingDirectory},
    {"getUserDirectory", l_getUserDirectory},
    {"getAppdataDirectory", l_getAppdataDirectory},
    {"getExecutablePath", l_getExecutablePath},
    {"getRealDirectory", l_getRealDirectory},
    {"getRealInfo", l_getRealInfo},
    {"getInfo", l_getInfo},
    {"read", l_read},
    {"write", l_write},
    {"append", l_append},
    {"remove", l_remove},
    {"createDirectory", l_createDirectory},
    {"getDirectoryItems", l_getDirectoryItems},
    {"lines", l_lines},
    {"load", l_load},
    {"exists", l_exists},
    {"isFile", l_isFile},
    {"isDirectory", l_isDirectory},
    {"isFused", l_isFused},
    {"mount", l_mount},
    {"unmount", l_unmount},
    {"newFile", l_newFile},
    {"newFileData", l_newFileData},
    {"setRequirePath", l_setRequirePath},
    {"getRequirePath", l_getRequirePath},
    {"setCRequirePath", l_setCRequirePath},
    {"getCRequirePath", l_getCRequirePath},
    {"setSymlinksEnabled", l_setSymlinksEnabled},
    {"areSymlinksEnabled", l_areSymlinksEnabled},
    {"_sourceChanged", l_sourceChanged},
    {nullptr, nullptr},
};

} // namespace
} // namespace filesystem

int open_filesystem(lua_State *L)
{
    using namespace filesystem;
    luax::newtype(L, FILE_TYPE, FILE_METHODS, luax::gcobject<File>);
    luax::newtype(L, FILEDATA_TYPE, FILEDATA_METHODS, luax::gcobject<FileData>);
    luaL_newlib(L, FUNCS);

    // Install the game-directory searcher right after the preload searcher,
    // so `require "player"` finds <game>/player.lua before any system path.
    lua_getglobal(L, "package");
    lua_getfield(L, -1, "searchers");
    lua_Integer count = luaL_len(L, -1);
    for (lua_Integer i = count; i >= 2; --i)
    {
        lua_rawgeti(L, -1, i);
        lua_rawseti(L, -2, i + 1);
    }
    lua_pushcfunction(L, l_searcher);
    lua_rawseti(L, -2, 2);
    lua_pop(L, 2);
    return 1;
}

} // namespace love
