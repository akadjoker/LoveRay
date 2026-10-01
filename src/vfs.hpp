// vfs.hpp - read-only virtual filesystem: directories and zip archives mounted
// at a path. love.filesystem layers the writable save directory on top.
#pragma once

#include <string>
#include <vector>

namespace love
{
namespace vfs
{

struct Stat
{
    bool exists = false;
    bool isDirectory = false;
    long long size = 0;
    long long modtime = -1;
};

// All paths are relative to the virtual root, with '/' separators and no
// leading slash. The empty string is the root.

void clear();

bool mountDirectory(const std::string &dir, const std::string &mountpoint, bool append, std::string &error);
// `name` identifies the mount for unmount(); for files it is the path.
bool mountArchiveFile(const std::string &file, const std::string &mountpoint, bool append, std::string &error);
bool mountArchiveMemory(std::vector<unsigned char> data, const std::string &name, const std::string &mountpoint,
                        bool append, std::string &error);
// Mounts a zip appended to an executable. Returns false when there is none.
bool mountFused(const std::string &executable);
bool unmount(const std::string &nameOrPath);

bool readFile(const std::string &path, std::vector<unsigned char> &out, std::string *error = nullptr);
Stat stat(const std::string &path);
void list(const std::string &path, std::vector<std::string> &names);

// Path of the directory or archive that provides `path`; empty when missing.
std::string origin(const std::string &path);
// Real file path when `path` is served by a mounted directory, else empty.
std::string realFile(const std::string &path);

} // namespace vfs
} // namespace love
