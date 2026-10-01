#include "vfs.hpp"

#include <sys/stat.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>

extern "C" int sinflate(void *out, int cap, const void *in, int size);

namespace fs = std::filesystem;

namespace love
{
namespace vfs
{

namespace
{

struct ZipEntry
{
    uint32_t method = 0;
    uint32_t crc = 0;
    uint64_t compressedSize = 0;
    uint64_t size = 0;
    uint64_t localOffset = 0;
    bool directory = false;
    long long modtime = -1;
};

struct Archive
{
    std::string path;
    std::vector<unsigned char> memory;
    bool inMemory = false;
    uint64_t base = 0;
    std::unordered_map<std::string, ZipEntry> entries;
    std::map<std::string, std::set<std::string>> children;

    bool read(uint64_t offset, uint64_t count, std::vector<unsigned char> &out) const
    {
        out.resize(static_cast<size_t>(count));
        if (count == 0)
        {
            return true;
        }
        if (inMemory)
        {
            if (offset + count > memory.size())
            {
                return false;
            }
            std::memcpy(out.data(), memory.data() + offset, static_cast<size_t>(count));
            return true;
        }
        std::ifstream in(path, std::ios::binary);
        if (!in)
        {
            return false;
        }
        in.seekg(static_cast<std::streamoff>(offset));
        in.read(reinterpret_cast<char *>(out.data()), static_cast<std::streamsize>(count));
        return static_cast<uint64_t>(in.gcount()) == count;
    }

    uint64_t totalSize() const
    {
        if (inMemory)
        {
            return memory.size();
        }
        std::error_code ec;
        auto size = fs::file_size(path, ec);
        return ec ? 0 : static_cast<uint64_t>(size);
    }
};

struct Mount
{
    std::string name;
    std::string point;
    std::string directory;
    std::shared_ptr<Archive> archive;
};

std::vector<Mount> g_mounts;

uint16_t u16(const unsigned char *p)
{
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t u32(const unsigned char *p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

uint32_t crc32(const unsigned char *data, size_t size)
{
    static uint32_t table[256];
    static bool ready = false;
    if (!ready)
    {
        for (uint32_t i = 0; i < 256; ++i)
        {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k)
            {
                c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            table[i] = c;
        }
        ready = true;
    }
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < size; ++i)
    {
        crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

long long dosTime(uint16_t time, uint16_t date)
{
    std::tm tm = {};
    tm.tm_year = ((date >> 9) & 0x7F) + 80;
    tm.tm_mon = ((date >> 5) & 0x0F) - 1;
    tm.tm_mday = date & 0x1F;
    tm.tm_hour = (time >> 11) & 0x1F;
    tm.tm_min = (time >> 5) & 0x3F;
    tm.tm_sec = (time & 0x1F) * 2;
    tm.tm_isdst = -1;
    std::time_t t = std::mktime(&tm);
    return t == static_cast<std::time_t>(-1) ? -1 : static_cast<long long>(t);
}

std::string clean(std::string path)
{
    for (char &c : path)
    {
        if (c == '\\')
        {
            c = '/';
        }
    }
    while (!path.empty() && path.front() == '/')
    {
        path.erase(path.begin());
    }
    while (!path.empty() && path.back() == '/')
    {
        path.pop_back();
    }
    return path;
}

bool safeName(const std::string &name)
{
    size_t start = 0;
    while (start <= name.size())
    {
        size_t end = name.find('/', start);
        if (end == std::string::npos)
        {
            end = name.size();
        }
        if (name.compare(start, end - start, "..") == 0)
        {
            return false;
        }
        start = end + 1;
    }
    return true;
}

void addChild(Archive &archive, const std::string &path)
{
    size_t slash = path.find_last_of('/');
    std::string parent = slash == std::string::npos ? "" : path.substr(0, slash);
    std::string leaf = slash == std::string::npos ? path : path.substr(slash + 1);
    archive.children[parent].insert(leaf);
    if (!parent.empty() && archive.entries.find(parent) == archive.entries.end())
    {
        ZipEntry dir;
        dir.directory = true;
        archive.entries[parent] = dir;
        addChild(archive, parent);
    }
}

bool parseArchive(Archive &archive, std::string &error)
{
    uint64_t total = archive.totalSize();
    if (total < 22)
    {
        error = "Not a zip archive";
        return false;
    }
    uint64_t tailSize = std::min<uint64_t>(total, 22 + 65535);
    std::vector<unsigned char> tail;
    if (!archive.read(total - tailSize, tailSize, tail))
    {
        error = "Could not read the archive";
        return false;
    }

    for (size_t i = tail.size() - 22 + 1; i-- > 0;)
    {
        if (u32(&tail[i]) != 0x06054b50u)
        {
            continue;
        }
        uint32_t entryCount = u16(&tail[i + 10]);
        uint32_t cdSize = u32(&tail[i + 12]);
        uint32_t cdOffset = u32(&tail[i + 16]);
        if (entryCount == 0xFFFF || cdSize == 0xFFFFFFFFu || cdOffset == 0xFFFFFFFFu)
        {
            error = "ZIP64 archives are not supported";
            return false;
        }
        uint64_t eocdPosition = total - tailSize + i;
        if (eocdPosition < static_cast<uint64_t>(cdSize) + cdOffset)
        {
            continue;
        }
        archive.base = eocdPosition - cdSize - cdOffset;

        std::vector<unsigned char> directory;
        if (!archive.read(archive.base + cdOffset, cdSize, directory))
        {
            continue;
        }
        archive.entries.clear();
        archive.children.clear();

        size_t pos = 0;
        bool valid = true;
        for (uint32_t n = 0; n < entryCount; ++n)
        {
            if (pos + 46 > directory.size() || u32(&directory[pos]) != 0x02014b50u)
            {
                valid = false;
                break;
            }
            const unsigned char *h = &directory[pos];
            uint16_t flags = u16(h + 8);
            ZipEntry entry;
            entry.method = u16(h + 10);
            entry.modtime = dosTime(u16(h + 12), u16(h + 14));
            entry.crc = u32(h + 16);
            entry.compressedSize = u32(h + 20);
            entry.size = u32(h + 24);
            uint16_t nameLength = u16(h + 28);
            uint16_t extraLength = u16(h + 30);
            uint16_t commentLength = u16(h + 32);
            entry.localOffset = u32(h + 42);
            if (pos + 46 + nameLength > directory.size())
            {
                valid = false;
                break;
            }
            std::string raw(reinterpret_cast<const char *>(h + 46), nameLength);
            pos += 46u + nameLength + extraLength + commentLength;

            if (flags & 1)
            {
                error = "Encrypted archives are not supported";
                return false;
            }
            std::string name = clean(raw);
            entry.directory = !raw.empty() && (raw.back() == '/' || raw.back() == '\\');
            if (name.empty() || !safeName(name))
            {
                continue;
            }
            archive.entries[name] = entry;
            addChild(archive, name);
        }
        if (!valid)
        {
            continue;
        }
        return true;
    }
    error = "Not a zip archive";
    return false;
}

std::shared_ptr<Archive> openFileArchive(const std::string &file, std::string &error)
{
    auto archive = std::make_shared<Archive>();
    archive->path = file;
    if (!parseArchive(*archive, error))
    {
        return nullptr;
    }
    return archive;
}

bool extract(const Archive &archive, const ZipEntry &entry, std::vector<unsigned char> &out, std::string *error)
{
    auto fail = [&](const char *message) {
        if (error != nullptr)
        {
            *error = message;
        }
        return false;
    };
    if (entry.directory)
    {
        return fail("Is a directory");
    }
    std::vector<unsigned char> header;
    if (!archive.read(archive.base + entry.localOffset, 30, header) || u32(header.data()) != 0x04034b50u)
    {
        return fail("Corrupt zip entry header");
    }
    uint64_t dataOffset = archive.base + entry.localOffset + 30 + u16(&header[26]) + u16(&header[28]);
    std::vector<unsigned char> stored;
    if (!archive.read(dataOffset, entry.compressedSize, stored))
    {
        return fail("Truncated zip entry");
    }
    if (entry.method == 0)
    {
        out = std::move(stored);
    }
    else if (entry.method == 8)
    {
        out.assign(static_cast<size_t>(entry.size), 0);
        int written = entry.size == 0 ? 0 : sinflate(out.data(), static_cast<int>(entry.size), stored.data(), static_cast<int>(stored.size()));
        if (written < 0 || static_cast<uint64_t>(written) != entry.size)
        {
            return fail("Corrupt compressed data");
        }
    }
    else
    {
        return fail("Unsupported zip compression method");
    }
    if (out.size() != entry.size || crc32(out.data(), out.size()) != entry.crc)
    {
        return fail("Checksum mismatch in zip entry");
    }
    return true;
}

// Strips the mount point from `path`; false when `path` is outside it.
bool relativeTo(const Mount &mount, const std::string &path, std::string &rest)
{
    if (mount.point.empty())
    {
        rest = path;
        return true;
    }
    if (path == mount.point)
    {
        rest.clear();
        return true;
    }
    if (path.size() > mount.point.size() && path.compare(0, mount.point.size(), mount.point) == 0 &&
        path[mount.point.size()] == '/')
    {
        rest = path.substr(mount.point.size() + 1);
        return true;
    }
    return false;
}

std::string joinPath(const std::string &base, const std::string &rest)
{
    if (rest.empty())
    {
        return base;
    }
    if (base.empty())
    {
        return rest;
    }
    return base + "/" + rest;
}

void insertMount(Mount mount, bool append)
{
    if (append)
    {
        g_mounts.push_back(std::move(mount));
    }
    else
    {
        g_mounts.insert(g_mounts.begin(), std::move(mount));
    }
}

} // namespace

void clear()
{
    g_mounts.clear();
}

bool mountDirectory(const std::string &dir, const std::string &mountpoint, bool append, std::string &error)
{
    std::error_code ec;
    if (!fs::is_directory(dir, ec))
    {
        error = "Not a directory: " + dir;
        return false;
    }
    Mount mount;
    mount.name = dir;
    mount.point = clean(mountpoint);
    mount.directory = dir;
    insertMount(std::move(mount), append);
    return true;
}

bool mountArchiveFile(const std::string &file, const std::string &mountpoint, bool append, std::string &error)
{
    auto archive = openFileArchive(file, error);
    if (!archive)
    {
        return false;
    }
    Mount mount;
    mount.name = file;
    mount.point = clean(mountpoint);
    mount.archive = archive;
    insertMount(std::move(mount), append);
    return true;
}

bool mountArchiveMemory(std::vector<unsigned char> data, const std::string &name, const std::string &mountpoint,
                        bool append, std::string &error)
{
    auto archive = std::make_shared<Archive>();
    archive->inMemory = true;
    archive->memory = std::move(data);
    archive->path = name;
    if (!parseArchive(*archive, error))
    {
        return false;
    }
    Mount mount;
    mount.name = name;
    mount.point = clean(mountpoint);
    mount.archive = archive;
    insertMount(std::move(mount), append);
    return true;
}

bool mountFused(const std::string &executable)
{
    std::string error;
    auto archive = openFileArchive(executable, error);
    if (!archive || archive->entries.empty())
    {
        return false;
    }
    Mount mount;
    mount.name = executable;
    mount.archive = archive;
    insertMount(std::move(mount), true);
    return true;
}

bool unmount(const std::string &nameOrPath)
{
    for (auto it = g_mounts.begin(); it != g_mounts.end(); ++it)
    {
        if (it->name == nameOrPath || (it->archive && it->archive->path == nameOrPath) || it->directory == nameOrPath)
        {
            g_mounts.erase(it);
            return true;
        }
    }
    return false;
}

bool readFile(const std::string &path, std::vector<unsigned char> &out, std::string *error)
{
    for (const Mount &mount : g_mounts)
    {
        std::string rest;
        if (!relativeTo(mount, path, rest))
        {
            continue;
        }
        if (mount.archive)
        {
            auto it = mount.archive->entries.find(rest);
            if (it != mount.archive->entries.end() && !it->second.directory)
            {
                return extract(*mount.archive, it->second, out, error);
            }
        }
        else
        {
            std::string real = joinPath(mount.directory, rest);
            std::error_code ec;
            if (fs::is_regular_file(real, ec))
            {
                std::ifstream in(real, std::ios::binary);
                if (!in)
                {
                    continue;
                }
                in.seekg(0, std::ios::end);
                std::streamoff size = in.tellg();
                in.seekg(0, std::ios::beg);
                out.resize(static_cast<size_t>(size > 0 ? size : 0));
                if (size > 0)
                {
                    in.read(reinterpret_cast<char *>(out.data()), size);
                }
                return true;
            }
        }
    }
    if (error != nullptr && error->empty())
    {
        *error = "Does not exist";
    }
    return false;
}

Stat stat(const std::string &path)
{
    Stat result;
    for (const Mount &mount : g_mounts)
    {
        std::string rest;
        if (relativeTo(mount, path, rest))
        {
            if (mount.archive)
            {
                if (rest.empty())
                {
                    result.exists = result.isDirectory = true;
                    return result;
                }
                auto it = mount.archive->entries.find(rest);
                if (it != mount.archive->entries.end())
                {
                    result.exists = true;
                    result.isDirectory = it->second.directory;
                    result.size = static_cast<long long>(it->second.size);
                    result.modtime = it->second.modtime;
                    return result;
                }
            }
            else
            {
                struct stat st;
                std::string real = joinPath(mount.directory, rest);
                if (::stat(real.c_str(), &st) == 0)
                {
                    result.exists = true;
                    result.isDirectory = S_ISDIR(st.st_mode);
                    result.size = result.isDirectory ? 0 : static_cast<long long>(st.st_size);
                    result.modtime = static_cast<long long>(st.st_mtime);
                    return result;
                }
            }
        }
        // A mount point below `path` makes `path` a virtual directory.
        if (!mount.point.empty() && mount.point.size() > path.size() &&
            (path.empty() || (mount.point.compare(0, path.size(), path) == 0 && mount.point[path.size()] == '/')))
        {
            result.exists = result.isDirectory = true;
            return result;
        }
    }
    return result;
}

void list(const std::string &path, std::vector<std::string> &names)
{
    std::set<std::string> seen(names.begin(), names.end());
    auto add = [&](const std::string &name) {
        if (seen.insert(name).second)
        {
            names.push_back(name);
        }
    };
    for (const Mount &mount : g_mounts)
    {
        std::string rest;
        if (relativeTo(mount, path, rest))
        {
            if (mount.archive)
            {
                auto it = mount.archive->children.find(rest);
                if (it != mount.archive->children.end())
                {
                    for (const std::string &name : it->second)
                    {
                        add(name);
                    }
                }
            }
            else
            {
                std::error_code ec;
                std::string real = joinPath(mount.directory, rest);
                if (fs::is_directory(real, ec))
                {
                    for (const auto &entry : fs::directory_iterator(real, ec))
                    {
                        add(entry.path().filename().string());
                    }
                }
            }
        }
        if (!mount.point.empty() && mount.point.size() > path.size() &&
            (path.empty() || (mount.point.compare(0, path.size(), path) == 0 && mount.point[path.size()] == '/')))
        {
            size_t from = path.empty() ? 0 : path.size() + 1;
            size_t slash = mount.point.find('/', from);
            add(mount.point.substr(from, slash == std::string::npos ? std::string::npos : slash - from));
        }
    }
}

std::string origin(const std::string &path)
{
    for (const Mount &mount : g_mounts)
    {
        std::string rest;
        if (!relativeTo(mount, path, rest))
        {
            continue;
        }
        if (mount.archive)
        {
            if (rest.empty() || mount.archive->entries.count(rest) != 0)
            {
                return mount.archive->path;
            }
        }
        else
        {
            std::error_code ec;
            if (fs::exists(joinPath(mount.directory, rest), ec))
            {
                return mount.directory;
            }
        }
    }
    return "";
}

std::string realFile(const std::string &path)
{
    for (const Mount &mount : g_mounts)
    {
        std::string rest;
        if (mount.archive || !relativeTo(mount, path, rest))
        {
            continue;
        }
        std::string real = joinPath(mount.directory, rest);
        std::error_code ec;
        if (fs::exists(real, ec))
        {
            return real;
        }
    }
    return "";
}

} // namespace vfs
} // namespace love
