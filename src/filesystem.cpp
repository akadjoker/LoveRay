// filesystem.cpp - love.filesystem
//
// Love2D exposes a virtual filesystem rooted at the game directory with a
// writable save directory layered on top. The save directory is served from
// the host filesystem and everything else from the virtual filesystem in
// vfs.cpp (directories and zip archives, including a zip appended to the
// executable).
#include "love.hpp"
#include "luax.hpp"
#include "vfs.hpp"

#include <sys/stat.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#if defined(_WIN32)
extern "C" __declspec(dllimport) unsigned long __stdcall GetModuleFileNameA(void *module, char *file, unsigned long size);
#elif defined(__linux__)
#include <unistd.h>
#endif

namespace fs = std::filesystem;

namespace love
{
namespace filesystem
{

namespace
{

std::string g_executable;
bool g_fused = false;
bool g_sourceIsArchive = false;
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

void setSource(const std::string &path)
{
    vfs::clear();
    g_source = normalize(path);
    g_watched.clear();
    std::string error;
    Info info = infoOf(g_source);
    g_sourceIsArchive = std::strcmp(info.type, "file") == 0;
    if (g_sourceIsArchive)
    {
        if (!vfs::mountArchiveFile(g_source, "", true, error))
        {
            log(LogLevel::Error, "Could not open %s: %s", g_source.c_str(), error.c_str());
        }
        g_watched.push_back({g_source, modTimeOf(g_source)});
        return;
    }
    if (!vfs::mountDirectory(g_source, "", true, error))
    {
        log(LogLevel::Error, "%s", error.c_str());
    }
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

namespace
{

Info infoVirtual(const std::string &path)
{
    Info info;
    std::string clean = normalize(path);
    if (!isSafeRelative(clean))
    {
        return info;
    }
    if (!g_identity.empty())
    {
        Info saved = infoOf(join(getSaveDirectory(), clean));
        if (saved.exists)
        {
            return saved;
        }
    }
    vfs::Stat stat = vfs::stat(clean);
    if (stat.exists)
    {
        info.exists = true;
        info.type = stat.isDirectory ? "directory" : "file";
        info.size = stat.size;
        info.modtime = stat.modtime;
    }
    return info;
}

} // namespace

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
    return vfs::realFile(clean);
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

bool readFile(const std::string &path, std::vector<unsigned char> &out, std::string *error)
{
    std::string clean = normalize(path);
    if (!isSafeRelative(clean))
    {
        return false;
    }
    std::string real = g_identity.empty() ? std::string() : join(getSaveDirectory(), clean);
    if (!real.empty() && std::strcmp(infoOf(real).type, "file") == 0)
    {
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
    return vfs::readFile(clean, out, error);
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
    std::FILE *handle = nullptr;  // write and append modes
    std::vector<unsigned char> data; // read mode keeps the whole file in memory
    size_t position = 0;
    char mode = 'c'; // 'r', 'w', 'a' or 'c' (closed)

    ~File()
    {
        close();
    }

    bool isOpen() const
    {
        return mode != 'c';
    }

    void close()
    {
        if (handle != nullptr)
        {
            std::fclose(handle);
            handle = nullptr;
        }
        data.clear();
        data.shrink_to_fit();
        position = 0;
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
    if (mode == 'r')
    {
        if (!readFile(file.name, file.data))
        {
            err = "Could not open file " + file.name + ". Does not exist.";
            return false;
        }
        file.mode = 'r';
        return true;
    }
    std::string real = resolveWrite(file.name);
    if (real.empty())
    {
        err = "Could not open file " + file.name + " for writing.";
        return false;
    }
    file.handle = std::fopen(real.c_str(), mode == 'w' ? "wb" : "ab");
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
    bool wasOpen = file->isOpen();
    file->close();
    lua_pushboolean(L, wasOpen);
    return 1;
}

int file_isOpen(lua_State *L)
{
    lua_pushboolean(L, checkFile(L, 1)->isOpen());
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
    if (file.mode == 'r')
    {
        return static_cast<long long>(file.data.size());
    }
    if (file.handle != nullptr)
    {
        long pos = std::ftell(file.handle);
        std::fseek(file.handle, 0, SEEK_END);
        long size = std::ftell(file.handle);
        std::fseek(file.handle, pos, SEEK_SET);
        return size;
    }
    std::vector<unsigned char> data;
    if (!readFile(file.name, data))
    {
        return 0;
    }
    return static_cast<long long>(data.size());
}

int file_getSize(lua_State *L)
{
    lua_pushinteger(L, fileSize(*checkFile(L, 1)));
    return 1;
}

int file_read(lua_State *L)
{
    File *file = checkFile(L, 1);
    if (file->mode != 'r')
    {
        lua_pushnil(L);
        lua_pushstring(L, "File is not opened for reading.");
        return 2;
    }
    long long remaining = static_cast<long long>(file->data.size() - file->position);
    long long count = static_cast<long long>(luaL_optinteger(L, 2, remaining));
    if (count < 0 || count > remaining)
    {
        count = remaining;
    }
    lua_pushlstring(L, reinterpret_cast<const char *>(file->data.data()) + file->position, static_cast<size_t>(count));
    file->position += static_cast<size_t>(count);
    lua_pushinteger(L, static_cast<lua_Integer>(count));
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
    long long pos = static_cast<long long>(luaL_checkinteger(L, 2));
    if (file->mode == 'r')
    {
        bool ok = pos >= 0 && pos <= static_cast<long long>(file->data.size());
        if (ok)
        {
            file->position = static_cast<size_t>(pos);
        }
        lua_pushboolean(L, ok);
        return 1;
    }
    lua_pushboolean(L, file->handle != nullptr && std::fseek(file->handle, static_cast<long>(pos), SEEK_SET) == 0);
    return 1;
}

int file_tell(lua_State *L)
{
    File *file = checkFile(L, 1);
    if (file->mode == 'r')
    {
        lua_pushinteger(L, static_cast<lua_Integer>(file->position));
    }
    else
    {
        lua_pushinteger(L, file->handle != nullptr ? std::ftell(file->handle) : -1);
    }
    return 1;
}

int file_isEOF(lua_State *L)
{
    File *file = checkFile(L, 1);
    bool eof = true;
    if (file->mode == 'r')
    {
        eof = file->position >= file->data.size();
    }
    else if (file->handle != nullptr)
    {
        eof = std::ftell(file->handle) >= fileSize(*file);
    }
    lua_pushboolean(L, eof);
    return 1;
}

int file_lines_iterator(lua_State *L)
{
    File *file = checkFile(L, lua_upvalueindex(1));
    if (file->mode != 'r')
    {
        return 0;
    }
    if (file->position >= file->data.size())
    {
        if (lua_toboolean(L, lua_upvalueindex(2)))
        {
            file->close();
        }
        return 0;
    }
    const char *begin = reinterpret_cast<const char *>(file->data.data()) + file->position;
    size_t available = file->data.size() - file->position;
    const char *newline = static_cast<const char *>(std::memchr(begin, '\n', available));
    size_t length = newline != nullptr ? static_cast<size_t>(newline - begin) : available;
    file->position += length + (newline != nullptr ? 1 : 0);
    if (length > 0 && begin[length - 1] == '\r')
    {
        --length;
    }
    lua_pushlstring(L, begin, length);
    return 1;
}

int file_lines(lua_State *L)
{
    File *file = checkFile(L, 1);
    bool autoClose = false;
    if (!file->isOpen())
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

std::string realExecutable()
{
#if defined(__linux__)
    char buffer[4096];
    ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length > 0)
    {
        buffer[length] = '\0';
        return buffer;
    }
#elif defined(_WIN32)
    char buffer[4096];
    unsigned long length = GetModuleFileNameA(nullptr, buffer, sizeof(buffer));
    if (length > 0 && length < sizeof(buffer))
    {
        return normalize(std::string(buffer, length));
    }
#endif
    return g_executable;
}

int l_init(lua_State *L)
{
    g_executable = normalize(luaL_checkstring(L, 1));
    std::string real = realExecutable();
    if (vfs::mountFused(real))
    {
        g_fused = true;
        g_sourceIsArchive = true;
        g_source = real;
        g_watched.clear();
        g_watched.push_back({real, modTimeOf(real)});
    }
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
    lua_pushstring(L, realExecutable().c_str());
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
    std::string origin = vfs::origin(path);
    if (!origin.empty())
    {
        lua_pushstring(L, origin.c_str());
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

    Info info = infoVirtual(path);
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
    std::string error;
    if (!readFile(name, data, &error))
    {
        lua_pushnil(L);
        if (error.empty() || error == "Does not exist")
        {
            lua_pushfstring(L, "Could not open file %s. Does not exist.", name);
        }
        else
        {
            lua_pushfstring(L, "Could not read file %s: %s", name, error.c_str());
        }
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
    std::vector<std::string> names;
    if (!g_identity.empty())
    {
        std::error_code ec;
        std::string saved = join(getSaveDirectory(), path);
        if (fs::is_directory(saved, ec))
        {
            for (const auto &entry : fs::directory_iterator(saved, ec))
            {
                names.push_back(entry.path().filename().string());
            }
        }
    }
    vfs::list(path, names);
    std::sort(names.begin(), names.end());
    names.erase(std::unique(names.begin(), names.end()), names.end());
    int n = 0;
    for (const std::string &name : names)
    {
        lua_pushstring(L, name.c_str());
        lua_rawseti(L, -2, ++n);
    }
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
    lua_pushboolean(L, infoVirtual(luaL_checkstring(L, 1)).exists);
    return 1;
}

int l_isFile(lua_State *L)
{
    Info info = infoVirtual(luaL_checkstring(L, 1));
    lua_pushboolean(L, info.exists && std::strcmp(info.type, "file") == 0);
    return 1;
}

int l_isDirectory(lua_State *L)
{
    Info info = infoVirtual(luaL_checkstring(L, 1));
    lua_pushboolean(L, info.exists && std::strcmp(info.type, "directory") == 0);
    return 1;
}

int l_isFused(lua_State *L)
{
    lua_pushboolean(L, g_fused);
    return 1;
}

bool mountPath(const std::string &given, const std::string &point, bool append, std::string &error)
{
    std::string clean = normalize(given);
    if (isSafeRelative(clean) && !clean.empty())
    {
        std::string saved = g_identity.empty() ? std::string() : join(getSaveDirectory(), clean);
        Info info = saved.empty() ? Info() : infoOf(saved);
        if (info.exists)
        {
            return std::strcmp(info.type, "directory") == 0 ? vfs::mountDirectory(saved, point, append, error)
                                                             : vfs::mountArchiveFile(saved, point, append, error);
        }
        vfs::Stat stat = vfs::stat(clean);
        if (stat.exists && stat.isDirectory)
        {
            std::string real = vfs::realFile(clean);
            if (!real.empty())
            {
                return vfs::mountDirectory(real, point, append, error);
            }
        }
        else if (stat.exists)
        {
            std::vector<unsigned char> bytes;
            std::string readError;
            if (vfs::readFile(clean, bytes, &readError))
            {
                return vfs::mountArchiveMemory(std::move(bytes), clean, point, append, error);
            }
        }
    }
    Info host = infoOf(normalize(given));
    if (host.exists)
    {
        return std::strcmp(host.type, "directory") == 0 ? vfs::mountDirectory(normalize(given), point, append, error)
                                                         : vfs::mountArchiveFile(normalize(given), point, append, error);
    }
    error = "Could not open " + given + ". Does not exist.";
    return false;
}

// mount(archive, mountpoint [, appendToPath]) | mount(filedata, name, mountpoint [, appendToPath])
int l_mount(lua_State *L)
{
    std::string error;
    bool ok;
    if (FileData *data = luax::testobject<FileData>(L, 1, FILEDATA_TYPE))
    {
        const char *name = luaL_checkstring(L, 2);
        std::string point = luaL_optstring(L, 3, "/");
        bool append = luax::optboolean(L, 4, false);
        std::vector<unsigned char> bytes(data->contents.begin(), data->contents.end());
        ok = vfs::mountArchiveMemory(std::move(bytes), name, point, append, error);
    }
    else
    {
        const char *archive = luaL_checkstring(L, 1);
        std::string point = luaL_optstring(L, 2, "/");
        bool append = luax::optboolean(L, 3, false);
        ok = mountPath(archive, point, append, error);
    }
    lua_pushboolean(L, ok);
    if (ok)
    {
        return 1;
    }
    lua_pushstring(L, error.c_str());
    return 2;
}

int l_unmount(lua_State *L)
{
    std::string name = luaL_checkstring(L, 1);
    bool ok = vfs::unmount(name) || vfs::unmount(normalize(name));
    if (!ok && !g_identity.empty())
    {
        ok = vfs::unmount(join(getSaveDirectory(), normalize(name)));
    }
    lua_pushboolean(L, ok);
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
