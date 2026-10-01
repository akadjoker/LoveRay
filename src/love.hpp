// love.hpp - shared engine services used across the LoveRay modules.
#pragma once

#include <lua.hpp>
#include <raylib.h>

#include <string>
#include <vector>

#ifndef LOVERAY_VERSION
#define LOVERAY_VERSION "0.1.0"
#endif

// The Love2D API level this runtime follows.
#define LOVE_API_VERSION_MAJOR 11
#define LOVE_API_VERSION_MINOR 5
#define LOVE_API_VERSION_STRING "11.5"

namespace love
{

enum class LogLevel
{
    Info,
    Warning,
    Error
};

void log(LogLevel level, const char *fmt, ...);

// Embedded resources (boot scripts, default font). Returns nullptr when
// `name` is unknown.
const unsigned char *resource(const char *name, unsigned int *length);

// Module entry points. Each one pushes its module table and returns 1.
int open_filesystem(lua_State *L);
int open_window(lua_State *L);
int open_graphics(lua_State *L);
int open_event(lua_State *L);
int open_keyboard(lua_State *L);
int open_mouse(lua_State *L);
int open_joystick(lua_State *L);
int open_timer(lua_State *L);
int open_math(lua_State *L);
int open_audio(lua_State *L);
int open_system(lua_State *L);
int open_image(lua_State *L);
int open_physics(lua_State *L);

// ---------------------------------------------------------------------------
// Cross-module services
// ---------------------------------------------------------------------------

namespace filesystem
{
// Directory that contains the game's main.lua (empty when running nogame).
void setSource(const std::string &dir);
const std::string &getSource();

// Save directory identity (love.filesystem.setIdentity).
void setIdentity(const std::string &identity);
const std::string &getIdentity();
std::string getSaveDirectory();

// Resolve a game-relative path for reading: the save directory wins over the
// source directory, like in Love2D. Returns "" when the file does not exist.
std::string resolveRead(const std::string &path);

// Resolve a game-relative path for writing inside the save directory,
// creating the intermediate directories. Returns "" on failure.
std::string resolveWrite(const std::string &path);

bool readFile(const std::string &path, std::vector<unsigned char> &out);
} // namespace filesystem

namespace window
{
bool isOpen();
// Opens an invisible default window when a module needs the GL context before
// love.window.setMode ran (for instance when conf.lua disables the window).
void ensureOpen();
void shutdown();
} // namespace window

namespace graphics
{
// Starts the raylib frame on demand; every drawing call goes through this so
// user code never has to worry about BeginDrawing/EndDrawing pairing.
void ensureFrame();
bool frameActive();
// Ends the frame (EndDrawing), which is also where raylib polls input.
void present();
void shutdown();
} // namespace graphics

namespace event
{
// Reads raylib's input state and converts changes into queued Love2D events.
void pump(lua_State *L);
} // namespace event

namespace audio
{
void ensureDevice();
void update();
void shutdown();
} // namespace audio

namespace keyboard
{
// Love2D KeyConstant <-> raylib key code. Return KEY_NULL / nullptr when unknown.
int keyFromName(const char *name);
const char *nameFromKey(int key);
bool keyRepeatEnabled();
bool textInputEnabled();
// All raylib key codes that have a Love2D name, used by the event pump.
const std::vector<int> &allKeys();
} // namespace keyboard

namespace mouse
{
// Love2D uses 1-based buttons (1 = left, 2 = right, 3 = middle).
int raylibButton(int loveButton);
int loveButton(int raylibButton);
} // namespace mouse

namespace joystick
{
// Pushes the Joystick object for raylib gamepad `index` (creating it once).
void pushJoystick(lua_State *L, int index);
const char *gamepadButtonName(int raylibButton);
const char *gamepadAxisName(int raylibAxis);
} // namespace joystick

namespace math
{
constexpr const char *TRANSFORM_TYPE = "Transform";
// Transform userdata payload is a raylib Matrix (column-major 4x4).
Matrix *checkTransform(lua_State *L, int idx);
void pushTransform(lua_State *L, const Matrix &m);
// Polygon helpers shared with love.graphics (flat x,y list).
bool isConvex(const std::vector<float> &points);
// Ear-clipping triangulation; `indices` receives vertex indices in triples.
bool triangulate(const std::vector<float> &points, std::vector<int> &indices);
} // namespace math

namespace timer
{
// Called once per love.timer.step(); also keeps the FPS counter.
double step();
double getDelta();
double getAverageDelta();
int getFPS();
} // namespace timer

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

struct BootResult
{
    int exitCode = 0;
    bool restart = false;
};

// Creates a Lua state, installs the `love` table and runs boot.lua.
BootResult boot(int argc, char **argv);

} // namespace love
