// math.cpp - love.math: random generators, noise, color conversion,
// polygon helpers, Transform and BezierCurve.
#include "love.hpp"
#include "luax.hpp"

#include <raymath.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>

namespace love
{
namespace math
{

namespace
{

const char *RNG_TYPE = "RandomGenerator";
const char *BEZIER_TYPE = "BezierCurve";

// ---------------------------------------------------------------------------
// Random numbers (xoshiro256** like Love2D's 64-bit generator family)
// ---------------------------------------------------------------------------

struct RandomGenerator
{
    uint64_t s[4];
    uint64_t seed;
    double lastNormal = 0.0;
    bool hasNormal = false;

    explicit RandomGenerator(uint64_t seedValue = 0x0139408DCBBF7A44ULL)
    {
        setSeed(seedValue);
    }

    static uint64_t splitmix(uint64_t &x)
    {
        uint64_t z = (x += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    void setSeed(uint64_t seedValue)
    {
        seed = seedValue;
        uint64_t x = seedValue;
        for (uint64_t &v : s)
        {
            v = splitmix(x);
        }
        hasNormal = false;
    }

    static uint64_t rotl(uint64_t x, int k)
    {
        return (x << k) | (x >> (64 - k));
    }

    uint64_t next()
    {
        uint64_t result = rotl(s[1] * 5, 7) * 9;
        uint64_t t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = rotl(s[3], 45);
        return result;
    }

    double random()
    {
        return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0);
    }

    double randomNormal(double stddev, double mean)
    {
        if (hasNormal)
        {
            hasNormal = false;
            return lastNormal * stddev + mean;
        }
        double u1, u2, r;
        do
        {
            u1 = 2.0 * random() - 1.0;
            u2 = 2.0 * random() - 1.0;
            r = u1 * u1 + u2 * u2;
        } while (r >= 1.0 || r == 0.0);
        double f = std::sqrt(-2.0 * std::log(r) / r);
        lastNormal = u2 * f;
        hasNormal = true;
        return u1 * f * stddev + mean;
    }

    std::string getState() const
    {
        char buffer[80];
        std::snprintf(buffer, sizeof(buffer), "0x%016llx%016llx%016llx%016llx",
                      static_cast<unsigned long long>(s[0]), static_cast<unsigned long long>(s[1]),
                      static_cast<unsigned long long>(s[2]), static_cast<unsigned long long>(s[3]));
        return buffer;
    }

    bool setState(const std::string &state)
    {
        if (state.size() != 66 || state.compare(0, 2, "0x") != 0)
        {
            return false;
        }
        for (int i = 0; i < 4; ++i)
        {
            s[i] = std::strtoull(state.substr(2 + i * 16, 16).c_str(), nullptr, 16);
        }
        hasNormal = false;
        return true;
    }
};

RandomGenerator g_rng;

RandomGenerator *checkRng(lua_State *L, int idx)
{
    return luax::checkobject<RandomGenerator>(L, idx, RNG_TYPE);
}

// random() | random(max) | random(min, max)
int randomImpl(lua_State *L, RandomGenerator &rng, int idx)
{
    int n = lua_gettop(L) - idx + 1;
    if (n <= 0)
    {
        lua_pushnumber(L, rng.random());
        return 1;
    }
    lua_Integer lo = 1;
    lua_Integer hi;
    if (n == 1)
    {
        hi = luaL_checkinteger(L, idx);
    }
    else
    {
        lo = luaL_checkinteger(L, idx);
        hi = luaL_checkinteger(L, idx + 1);
    }
    if (lo > hi)
    {
        return luaL_error(L, "bad argument to 'random' (interval is empty)");
    }
    double range = static_cast<double>(hi - lo) + 1.0;
    lua_pushinteger(L, lo + static_cast<lua_Integer>(std::floor(rng.random() * range)));
    return 1;
}

uint64_t seedFromArgs(lua_State *L, int idx)
{
    lua_Number low = luaL_checknumber(L, idx);
    if (lua_isnoneornil(L, idx + 1))
    {
        if (std::floor(low) == low)
        {
            return static_cast<uint64_t>(static_cast<int64_t>(low));
        }
        uint64_t bits;
        double d = low;
        std::memcpy(&bits, &d, sizeof(bits));
        return bits;
    }
    uint64_t lowBits = static_cast<uint32_t>(static_cast<int64_t>(low));
    uint64_t highBits = static_cast<uint32_t>(static_cast<int64_t>(luaL_checknumber(L, idx + 1)));
    return (highBits << 32) | lowBits;
}

void pushSeed(lua_State *L, uint64_t seed)
{
    lua_pushinteger(L, static_cast<lua_Integer>(seed & 0xFFFFFFFFULL));
    lua_pushinteger(L, static_cast<lua_Integer>(seed >> 32));
}

int rng_random(lua_State *L)
{
    return randomImpl(L, *checkRng(L, 1), 2);
}

int rng_randomNormal(lua_State *L)
{
    RandomGenerator *rng = checkRng(L, 1);
    double stddev = luaL_optnumber(L, 2, 1.0);
    double mean = luaL_optnumber(L, 3, 0.0);
    lua_pushnumber(L, rng->randomNormal(stddev, mean));
    return 1;
}

int rng_setSeed(lua_State *L)
{
    checkRng(L, 1)->setSeed(seedFromArgs(L, 2));
    return 0;
}

int rng_getSeed(lua_State *L)
{
    pushSeed(L, checkRng(L, 1)->seed);
    return 2;
}

int rng_setState(lua_State *L)
{
    if (!checkRng(L, 1)->setState(luaL_checkstring(L, 2)))
    {
        return luaL_error(L, "Invalid random state");
    }
    return 0;
}

int rng_getState(lua_State *L)
{
    lua_pushstring(L, checkRng(L, 1)->getState().c_str());
    return 1;
}

const luaL_Reg RNG_METHODS[] = {
    {"random", rng_random},
    {"randomNormal", rng_randomNormal},
    {"setSeed", rng_setSeed},
    {"getSeed", rng_getSeed},
    {"setState", rng_setState},
    {"getState", rng_getState},
    {nullptr, nullptr},
};

int l_newRandomGenerator(lua_State *L)
{
    uint64_t seed;
    if (!lua_isnoneornil(L, 1))
    {
        seed = seedFromArgs(L, 1);
    }
    else
    {
        seed = static_cast<uint64_t>(std::random_device{}()) << 32 | std::random_device{}();
    }
    RandomGenerator *rng = luax::newobject<RandomGenerator>(L, RNG_TYPE);
    rng->setSeed(seed);
    return 1;
}

int l_random(lua_State *L)
{
    return randomImpl(L, g_rng, 1);
}

int l_randomNormal(lua_State *L)
{
    double stddev = luaL_optnumber(L, 1, 1.0);
    double mean = luaL_optnumber(L, 2, 0.0);
    lua_pushnumber(L, g_rng.randomNormal(stddev, mean));
    return 1;
}

int l_setRandomSeed(lua_State *L)
{
    g_rng.setSeed(seedFromArgs(L, 1));
    return 0;
}

int l_getRandomSeed(lua_State *L)
{
    pushSeed(L, g_rng.seed);
    return 2;
}

int l_setRandomState(lua_State *L)
{
    if (!g_rng.setState(luaL_checkstring(L, 1)))
    {
        return luaL_error(L, "Invalid random state");
    }
    return 0;
}

int l_getRandomState(lua_State *L)
{
    lua_pushstring(L, g_rng.getState().c_str());
    return 1;
}

// ---------------------------------------------------------------------------
// Simplex noise (Stefan Gustavson's reference implementation), output in [0, 1]
// ---------------------------------------------------------------------------

const int GRAD3[12][3] = {{1, 1, 0}, {-1, 1, 0}, {1, -1, 0}, {-1, -1, 0}, {1, 0, 1}, {-1, 0, 1},
                          {1, 0, -1}, {-1, 0, -1}, {0, 1, 1}, {0, -1, 1}, {0, 1, -1}, {0, -1, -1}};

const int GRAD4[32][4] = {{0, 1, 1, 1}, {0, 1, 1, -1}, {0, 1, -1, 1}, {0, 1, -1, -1}, {0, -1, 1, 1}, {0, -1, 1, -1},
                          {0, -1, -1, 1}, {0, -1, -1, -1}, {1, 0, 1, 1}, {1, 0, 1, -1}, {1, 0, -1, 1}, {1, 0, -1, -1},
                          {-1, 0, 1, 1}, {-1, 0, 1, -1}, {-1, 0, -1, 1}, {-1, 0, -1, -1}, {1, 1, 0, 1}, {1, 1, 0, -1},
                          {1, -1, 0, 1}, {1, -1, 0, -1}, {-1, 1, 0, 1}, {-1, 1, 0, -1}, {-1, -1, 0, 1}, {-1, -1, 0, -1},
                          {1, 1, 1, 0}, {1, 1, -1, 0}, {1, -1, 1, 0}, {1, -1, -1, 0}, {-1, 1, 1, 0}, {-1, 1, -1, 0},
                          {-1, -1, 1, 0}, {-1, -1, -1, 0}};

const unsigned char PERM_BASE[256] = {
    151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10,
    23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87,
    174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211,
    133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208,
    89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5,
    202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119,
    248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232,
    178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249,
    14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205,
    93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180};

const unsigned char *perm()
{
    static unsigned char table[512];
    static bool ready = false;
    if (!ready)
    {
        for (int i = 0; i < 512; ++i)
        {
            table[i] = PERM_BASE[i & 255];
        }
        ready = true;
    }
    return table;
}

int fastFloor(double x)
{
    return x > 0 ? static_cast<int>(x) : static_cast<int>(x) - 1;
}

double dot(const int *g, double x, double y)
{
    return g[0] * x + g[1] * y;
}

double dot(const int *g, double x, double y, double z)
{
    return g[0] * x + g[1] * y + g[2] * z;
}

double dot(const int *g, double x, double y, double z, double w)
{
    return g[0] * x + g[1] * y + g[2] * z + g[3] * w;
}

double simplex2(double xin, double yin)
{
    const unsigned char *p = perm();
    const double F2 = 0.5 * (std::sqrt(3.0) - 1.0);
    const double G2 = (3.0 - std::sqrt(3.0)) / 6.0;
    double s = (xin + yin) * F2;
    int i = fastFloor(xin + s);
    int j = fastFloor(yin + s);
    double t = (i + j) * G2;
    double x0 = xin - (i - t);
    double y0 = yin - (j - t);
    int i1 = x0 > y0 ? 1 : 0;
    int j1 = x0 > y0 ? 0 : 1;
    double x1 = x0 - i1 + G2;
    double y1 = y0 - j1 + G2;
    double x2 = x0 - 1.0 + 2.0 * G2;
    double y2 = y0 - 1.0 + 2.0 * G2;
    int ii = i & 255;
    int jj = j & 255;
    int gi0 = p[ii + p[jj]] % 12;
    int gi1 = p[ii + i1 + p[jj + j1]] % 12;
    int gi2 = p[ii + 1 + p[jj + 1]] % 12;
    double n0 = 0, n1 = 0, n2 = 0;
    double t0 = 0.5 - x0 * x0 - y0 * y0;
    if (t0 >= 0)
    {
        t0 *= t0;
        n0 = t0 * t0 * dot(GRAD3[gi0], x0, y0);
    }
    double t1 = 0.5 - x1 * x1 - y1 * y1;
    if (t1 >= 0)
    {
        t1 *= t1;
        n1 = t1 * t1 * dot(GRAD3[gi1], x1, y1);
    }
    double t2 = 0.5 - x2 * x2 - y2 * y2;
    if (t2 >= 0)
    {
        t2 *= t2;
        n2 = t2 * t2 * dot(GRAD3[gi2], x2, y2);
    }
    return 70.0 * (n0 + n1 + n2);
}

double simplex3(double xin, double yin, double zin)
{
    const unsigned char *p = perm();
    const double F3 = 1.0 / 3.0;
    const double G3 = 1.0 / 6.0;
    double s = (xin + yin + zin) * F3;
    int i = fastFloor(xin + s);
    int j = fastFloor(yin + s);
    int k = fastFloor(zin + s);
    double t = (i + j + k) * G3;
    double x0 = xin - (i - t);
    double y0 = yin - (j - t);
    double z0 = zin - (k - t);
    int i1, j1, k1, i2, j2, k2;
    if (x0 >= y0)
    {
        if (y0 >= z0)
        {
            i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 1; k2 = 0;
        }
        else if (x0 >= z0)
        {
            i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 0; k2 = 1;
        }
        else
        {
            i1 = 0; j1 = 0; k1 = 1; i2 = 1; j2 = 0; k2 = 1;
        }
    }
    else
    {
        if (y0 < z0)
        {
            i1 = 0; j1 = 0; k1 = 1; i2 = 0; j2 = 1; k2 = 1;
        }
        else if (x0 < z0)
        {
            i1 = 0; j1 = 1; k1 = 0; i2 = 0; j2 = 1; k2 = 1;
        }
        else
        {
            i1 = 0; j1 = 1; k1 = 0; i2 = 1; j2 = 1; k2 = 0;
        }
    }
    double x1 = x0 - i1 + G3, y1 = y0 - j1 + G3, z1 = z0 - k1 + G3;
    double x2 = x0 - i2 + 2.0 * G3, y2 = y0 - j2 + 2.0 * G3, z2 = z0 - k2 + 2.0 * G3;
    double x3 = x0 - 1.0 + 3.0 * G3, y3 = y0 - 1.0 + 3.0 * G3, z3 = z0 - 1.0 + 3.0 * G3;
    int ii = i & 255, jj = j & 255, kk = k & 255;
    int gi0 = p[ii + p[jj + p[kk]]] % 12;
    int gi1 = p[ii + i1 + p[jj + j1 + p[kk + k1]]] % 12;
    int gi2 = p[ii + i2 + p[jj + j2 + p[kk + k2]]] % 12;
    int gi3 = p[ii + 1 + p[jj + 1 + p[kk + 1]]] % 12;
    double n[4] = {0, 0, 0, 0};
    double corners[4][3] = {{x0, y0, z0}, {x1, y1, z1}, {x2, y2, z2}, {x3, y3, z3}};
    int gi[4] = {gi0, gi1, gi2, gi3};
    for (int c = 0; c < 4; ++c)
    {
        double tt = 0.6 - corners[c][0] * corners[c][0] - corners[c][1] * corners[c][1] - corners[c][2] * corners[c][2];
        if (tt >= 0)
        {
            tt *= tt;
            n[c] = tt * tt * dot(GRAD3[gi[c]], corners[c][0], corners[c][1], corners[c][2]);
        }
    }
    return 32.0 * (n[0] + n[1] + n[2] + n[3]);
}

double simplex4(double x, double y, double z, double w)
{
    const unsigned char *p = perm();
    const double F4 = (std::sqrt(5.0) - 1.0) / 4.0;
    const double G4 = (5.0 - std::sqrt(5.0)) / 20.0;
    double s = (x + y + z + w) * F4;
    int i = fastFloor(x + s), j = fastFloor(y + s), k = fastFloor(z + s), l = fastFloor(w + s);
    double t = (i + j + k + l) * G4;
    double x0 = x - (i - t), y0 = y - (j - t), z0 = z - (k - t), w0 = w - (l - t);
    int rankx = 0, ranky = 0, rankz = 0, rankw = 0;
    if (x0 > y0) rankx++; else ranky++;
    if (x0 > z0) rankx++; else rankz++;
    if (x0 > w0) rankx++; else rankw++;
    if (y0 > z0) ranky++; else rankz++;
    if (y0 > w0) ranky++; else rankw++;
    if (z0 > w0) rankz++; else rankw++;
    int i1 = rankx >= 3 ? 1 : 0, j1 = ranky >= 3 ? 1 : 0, k1 = rankz >= 3 ? 1 : 0, l1 = rankw >= 3 ? 1 : 0;
    int i2 = rankx >= 2 ? 1 : 0, j2 = ranky >= 2 ? 1 : 0, k2 = rankz >= 2 ? 1 : 0, l2 = rankw >= 2 ? 1 : 0;
    int i3 = rankx >= 1 ? 1 : 0, j3 = ranky >= 1 ? 1 : 0, k3 = rankz >= 1 ? 1 : 0, l3 = rankw >= 1 ? 1 : 0;
    double c[5][4] = {
        {x0, y0, z0, w0},
        {x0 - i1 + G4, y0 - j1 + G4, z0 - k1 + G4, w0 - l1 + G4},
        {x0 - i2 + 2 * G4, y0 - j2 + 2 * G4, z0 - k2 + 2 * G4, w0 - l2 + 2 * G4},
        {x0 - i3 + 3 * G4, y0 - j3 + 3 * G4, z0 - k3 + 3 * G4, w0 - l3 + 3 * G4},
        {x0 - 1 + 4 * G4, y0 - 1 + 4 * G4, z0 - 1 + 4 * G4, w0 - 1 + 4 * G4},
    };
    int ii = i & 255, jj = j & 255, kk = k & 255, ll = l & 255;
    int gi[5] = {
        p[ii + p[jj + p[kk + p[ll]]]] % 32,
        p[ii + i1 + p[jj + j1 + p[kk + k1 + p[ll + l1]]]] % 32,
        p[ii + i2 + p[jj + j2 + p[kk + k2 + p[ll + l2]]]] % 32,
        p[ii + i3 + p[jj + j3 + p[kk + k3 + p[ll + l3]]]] % 32,
        p[ii + 1 + p[jj + 1 + p[kk + 1 + p[ll + 1]]]] % 32,
    };
    double total = 0.0;
    for (int n = 0; n < 5; ++n)
    {
        double tt = 0.6 - c[n][0] * c[n][0] - c[n][1] * c[n][1] - c[n][2] * c[n][2] - c[n][3] * c[n][3];
        if (tt >= 0)
        {
            tt *= tt;
            total += tt * tt * dot(GRAD4[gi[n]], c[n][0], c[n][1], c[n][2], c[n][3]);
        }
    }
    return 27.0 * total;
}

int l_noise(lua_State *L)
{
    int n = lua_gettop(L);
    double value;
    switch (n)
    {
    case 1:
        value = simplex2(luaL_checknumber(L, 1), 0.0);
        break;
    case 2:
        value = simplex2(luaL_checknumber(L, 1), luaL_checknumber(L, 2));
        break;
    case 3:
        value = simplex3(luaL_checknumber(L, 1), luaL_checknumber(L, 2), luaL_checknumber(L, 3));
        break;
    case 4:
        value = simplex4(luaL_checknumber(L, 1), luaL_checknumber(L, 2), luaL_checknumber(L, 3), luaL_checknumber(L, 4));
        break;
    default:
        return luaL_error(L, "love.math.noise expects 1 to 4 numbers");
    }
    lua_pushnumber(L, std::min(1.0, std::max(0.0, value * 0.5 + 0.5)));
    return 1;
}

// ---------------------------------------------------------------------------
// Colors
// ---------------------------------------------------------------------------

int l_colorFromBytes(lua_State *L)
{
    int n;
    double c[4];
    if (lua_istable(L, 1))
    {
        n = static_cast<int>(luaL_len(L, 1));
        n = std::min(n, 4);
        for (int i = 0; i < n; ++i)
        {
            lua_rawgeti(L, 1, i + 1);
            c[i] = luaL_checknumber(L, -1);
            lua_pop(L, 1);
        }
    }
    else
    {
        n = std::min(lua_gettop(L), 4);
        for (int i = 0; i < n; ++i)
        {
            c[i] = luaL_checknumber(L, i + 1);
        }
    }
    for (int i = 0; i < n; ++i)
    {
        lua_pushnumber(L, std::min(1.0, std::max(0.0, c[i] / 255.0)));
    }
    return n;
}

int l_colorToBytes(lua_State *L)
{
    int n;
    double c[4];
    if (lua_istable(L, 1))
    {
        n = static_cast<int>(luaL_len(L, 1));
        n = std::min(n, 4);
        for (int i = 0; i < n; ++i)
        {
            lua_rawgeti(L, 1, i + 1);
            c[i] = luaL_checknumber(L, -1);
            lua_pop(L, 1);
        }
    }
    else
    {
        n = std::min(lua_gettop(L), 4);
        for (int i = 0; i < n; ++i)
        {
            c[i] = luaL_checknumber(L, i + 1);
        }
    }
    for (int i = 0; i < n; ++i)
    {
        lua_pushinteger(L, static_cast<lua_Integer>(std::floor(std::min(1.0, std::max(0.0, c[i])) * 255.0 + 0.5)));
    }
    return n;
}

double gammaToLinear(double c)
{
    if (c <= 0.04045)
    {
        return c / 12.92;
    }
    return std::pow((c + 0.055) / 1.055, 2.4);
}

double linearToGamma(double c)
{
    if (c <= 0.0031308)
    {
        return c * 12.92;
    }
    return 1.055 * std::pow(c, 1.0 / 2.4) - 0.055;
}

template <double (*Fn)(double)>
int convertColor(lua_State *L)
{
    int n;
    double c[4];
    if (lua_istable(L, 1))
    {
        n = std::min(static_cast<int>(luaL_len(L, 1)), 4);
        for (int i = 0; i < n; ++i)
        {
            lua_rawgeti(L, 1, i + 1);
            c[i] = luaL_checknumber(L, -1);
            lua_pop(L, 1);
        }
    }
    else
    {
        n = std::min(lua_gettop(L), 4);
        for (int i = 0; i < n; ++i)
        {
            c[i] = luaL_checknumber(L, i + 1);
        }
    }
    for (int i = 0; i < n; ++i)
    {
        lua_pushnumber(L, i == 3 ? c[i] : Fn(std::min(1.0, std::max(0.0, c[i]))));
    }
    return n;
}

// ---------------------------------------------------------------------------
// Polygons
// ---------------------------------------------------------------------------

void readVertices(lua_State *L, int idx, std::vector<float> &out)
{
    out.clear();
    if (lua_istable(L, idx))
    {
        lua_Integer n = luaL_len(L, idx);
        for (lua_Integer i = 1; i <= n; ++i)
        {
            lua_rawgeti(L, idx, i);
            out.push_back(static_cast<float>(luaL_checknumber(L, -1)));
            lua_pop(L, 1);
        }
    }
    else
    {
        int n = lua_gettop(L);
        for (int i = idx; i <= n; ++i)
        {
            out.push_back(static_cast<float>(luaL_checknumber(L, i)));
        }
    }
    if (out.size() % 2 != 0)
    {
        luaL_error(L, "Number of vertex components must be a multiple of two");
    }
}

int l_isConvex(lua_State *L)
{
    std::vector<float> verts;
    readVertices(L, 1, verts);
    lua_pushboolean(L, isConvex(verts));
    return 1;
}

int l_triangulate(lua_State *L)
{
    std::vector<float> verts;
    readVertices(L, 1, verts);
    if (verts.size() < 6)
    {
        return luaL_error(L, "Need at least 3 vertices to triangulate");
    }
    std::vector<int> indices;
    if (!triangulate(verts, indices))
    {
        return luaL_error(L, "Cannot triangulate polygon");
    }
    lua_newtable(L);
    int count = 0;
    for (size_t i = 0; i + 2 < indices.size(); i += 3)
    {
        lua_newtable(L);
        for (int k = 0; k < 3; ++k)
        {
            int v = indices[i + k];
            lua_pushnumber(L, verts[v * 2]);
            lua_rawseti(L, -2, k * 2 + 1);
            lua_pushnumber(L, verts[v * 2 + 1]);
            lua_rawseti(L, -2, k * 2 + 2);
        }
        lua_rawseti(L, -2, ++count);
    }
    return 1;
}

// ---------------------------------------------------------------------------
// Transform
// ---------------------------------------------------------------------------

Matrix mul(const Matrix &a, const Matrix &b)
{
    const float *fa = reinterpret_cast<const float *>(&a);
    const float *fb = reinterpret_cast<const float *>(&b);
    Matrix r;
    float *fr = reinterpret_cast<float *>(&r);
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

Matrix transformation(float x, float y, float angle, float sx, float sy, float ox, float oy, float kx, float ky)
{
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

int tf_clone(lua_State *L)
{
    pushTransform(L, *checkTransform(L, 1));
    return 1;
}

int tf_inverse(lua_State *L)
{
    pushTransform(L, MatrixInvert(*checkTransform(L, 1)));
    return 1;
}

int tf_apply(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    Matrix *other = checkTransform(L, 2);
    *m = mul(*m, *other);
    lua_pushvalue(L, 1);
    return 1;
}

int tf_isAffine2DTransform(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    bool affine = m->m2 == 0 && m->m3 == 0 && m->m6 == 0 && m->m7 == 0 && m->m8 == 0 && m->m9 == 0 &&
                  m->m10 == 1 && m->m11 == 0 && m->m14 == 0 && m->m15 == 1;
    lua_pushboolean(L, affine);
    return 1;
}

int tf_reset(lua_State *L)
{
    *checkTransform(L, 1) = MatrixIdentity();
    lua_pushvalue(L, 1);
    return 1;
}

int tf_translate(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    *m = mul(*m, MatrixTranslate(luax::checkfloat(L, 2), luax::checkfloat(L, 3), 0.0f));
    lua_pushvalue(L, 1);
    return 1;
}

int tf_rotate(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    *m = mul(*m, MatrixRotateZ(luax::checkfloat(L, 2)));
    lua_pushvalue(L, 1);
    return 1;
}

int tf_scale(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    float sx = luax::checkfloat(L, 2);
    float sy = luax::optfloat(L, 3, sx);
    *m = mul(*m, MatrixScale(sx, sy, 1.0f));
    lua_pushvalue(L, 1);
    return 1;
}

int tf_shear(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    Matrix shear = MatrixIdentity();
    shear.m4 = luax::checkfloat(L, 2);
    shear.m1 = luax::checkfloat(L, 3);
    *m = mul(*m, shear);
    lua_pushvalue(L, 1);
    return 1;
}

int tf_setTransformation(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    float x = luax::optfloat(L, 2, 0.0f);
    float y = luax::optfloat(L, 3, 0.0f);
    float r = luax::optfloat(L, 4, 0.0f);
    float sx = luax::optfloat(L, 5, 1.0f);
    float sy = luax::optfloat(L, 6, sx);
    float ox = luax::optfloat(L, 7, 0.0f);
    float oy = luax::optfloat(L, 8, 0.0f);
    float kx = luax::optfloat(L, 9, 0.0f);
    float ky = luax::optfloat(L, 10, 0.0f);
    *m = transformation(x, y, r, sx, sy, ox, oy, kx, ky);
    lua_pushvalue(L, 1);
    return 1;
}

int tf_transformPoint(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    Vector3 p = Vector3Transform({luax::checkfloat(L, 2), luax::checkfloat(L, 3), 0.0f}, *m);
    lua_pushnumber(L, p.x);
    lua_pushnumber(L, p.y);
    return 2;
}

int tf_inverseTransformPoint(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    Vector3 p = Vector3Transform({luax::checkfloat(L, 2), luax::checkfloat(L, 3), 0.0f}, MatrixInvert(*m));
    lua_pushnumber(L, p.x);
    lua_pushnumber(L, p.y);
    return 2;
}

// Love2D exposes the matrix row-major: e11, e12, e13, e14, e21, ...
int tf_getMatrix(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    const float *f = reinterpret_cast<const float *>(m);
    for (int i = 0; i < 16; ++i)
    {
        lua_pushnumber(L, f[i]);
    }
    return 16;
}

int tf_setMatrix(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    int idx = 2;
    bool columnMajor = false;
    if (lua_type(L, 2) == LUA_TSTRING)
    {
        static const char *const names[] = {"row", "column"};
        static const int values[] = {0, 1};
        columnMajor = luax::checkenum(L, 2, names, values, "matrix layout") == 1;
        idx = 3;
    }
    float values[16];
    if (lua_istable(L, idx))
    {
        for (int i = 0; i < 16; ++i)
        {
            lua_rawgeti(L, idx, i + 1);
            values[i] = static_cast<float>(luaL_checknumber(L, -1));
            lua_pop(L, 1);
        }
    }
    else
    {
        for (int i = 0; i < 16; ++i)
        {
            values[i] = luax::checkfloat(L, idx + i);
        }
    }
    float *f = reinterpret_cast<float *>(m);
    for (int i = 0; i < 16; ++i)
    {
        int row = columnMajor ? i % 4 : i / 4;
        int col = columnMajor ? i / 4 : i % 4;
        f[row * 4 + col] = values[i];
    }
    lua_pushvalue(L, 1);
    return 1;
}

int tf_mul(lua_State *L)
{
    Matrix *a = checkTransform(L, 1);
    Matrix *b = checkTransform(L, 2);
    pushTransform(L, mul(*a, *b));
    return 1;
}

int tf_tostring(lua_State *L)
{
    Matrix *m = checkTransform(L, 1);
    lua_pushfstring(L, "Transform: [%f %f %f | %f %f %f]", m->m0, m->m4, m->m12, m->m1, m->m5, m->m13);
    return 1;
}

const luaL_Reg TRANSFORM_METHODS[] = {
    {"clone", tf_clone},
    {"inverse", tf_inverse},
    {"apply", tf_apply},
    {"isAffine2DTransform", tf_isAffine2DTransform},
    {"reset", tf_reset},
    {"translate", tf_translate},
    {"rotate", tf_rotate},
    {"scale", tf_scale},
    {"shear", tf_shear},
    {"setTransformation", tf_setTransformation},
    {"transformPoint", tf_transformPoint},
    {"inverseTransformPoint", tf_inverseTransformPoint},
    {"getMatrix", tf_getMatrix},
    {"setMatrix", tf_setMatrix},
    {"__mul", tf_mul},
    {"__tostring", tf_tostring},
    {nullptr, nullptr},
};

int l_newTransform(lua_State *L)
{
    if (lua_gettop(L) == 0)
    {
        pushTransform(L, MatrixIdentity());
        return 1;
    }
    float x = luax::optfloat(L, 1, 0.0f);
    float y = luax::optfloat(L, 2, 0.0f);
    float r = luax::optfloat(L, 3, 0.0f);
    float sx = luax::optfloat(L, 4, 1.0f);
    float sy = luax::optfloat(L, 5, sx);
    float ox = luax::optfloat(L, 6, 0.0f);
    float oy = luax::optfloat(L, 7, 0.0f);
    float kx = luax::optfloat(L, 8, 0.0f);
    float ky = luax::optfloat(L, 9, 0.0f);
    pushTransform(L, transformation(x, y, r, sx, sy, ox, oy, kx, ky));
    return 1;
}

// ---------------------------------------------------------------------------
// BezierCurve
// ---------------------------------------------------------------------------

struct BezierCurve
{
    std::vector<Vector2> points;

    Vector2 evaluate(double t) const
    {
        std::vector<Vector2> tmp = points;
        for (size_t step = 1; step < tmp.size(); ++step)
        {
            for (size_t i = 0; i + step < tmp.size(); ++i)
            {
                tmp[i].x = static_cast<float>((1.0 - t) * tmp[i].x + t * tmp[i + 1].x);
                tmp[i].y = static_cast<float>((1.0 - t) * tmp[i].y + t * tmp[i + 1].y);
            }
        }
        return tmp.empty() ? Vector2{0, 0} : tmp[0];
    }

    // de Casteljau subdivision, same scheme as Love2D's BezierCurve::render.
    void subdivide(std::vector<Vector2> &out, int depth) const
    {
        if (depth <= 0 || points.size() < 2)
        {
            return;
        }
        std::vector<Vector2> result = points;
        for (int k = 0; k < depth; ++k)
        {
            std::vector<Vector2> next;
            next.reserve(result.size() * 2);
            for (size_t i = 0; i + 1 < result.size(); ++i)
            {
                next.push_back(result[i]);
                next.push_back({(result[i].x + result[i + 1].x) * 0.5f, (result[i].y + result[i + 1].y) * 0.5f});
            }
            next.push_back(result.back());
            result.swap(next);
        }
        out = result;
    }
};

BezierCurve *checkBezier(lua_State *L, int idx)
{
    return luax::checkobject<BezierCurve>(L, idx, BEZIER_TYPE);
}

int bz_evaluate(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    double t = luaL_checknumber(L, 2);
    if (t < 0.0 || t > 1.0)
    {
        return luaL_error(L, "Invalid evaluation parameter: must be between 0 and 1");
    }
    Vector2 p = curve->evaluate(t);
    lua_pushnumber(L, p.x);
    lua_pushnumber(L, p.y);
    return 2;
}

int bz_getControlPoint(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    lua_Integer i = luaL_checkinteger(L, 2);
    lua_Integer n = static_cast<lua_Integer>(curve->points.size());
    if (i < 0)
    {
        i += n + 1;
    }
    if (i < 1 || i > n)
    {
        return luaL_error(L, "Invalid control point index");
    }
    lua_pushnumber(L, curve->points[static_cast<size_t>(i - 1)].x);
    lua_pushnumber(L, curve->points[static_cast<size_t>(i - 1)].y);
    return 2;
}

int bz_setControlPoint(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    lua_Integer i = luaL_checkinteger(L, 2);
    lua_Integer n = static_cast<lua_Integer>(curve->points.size());
    if (i < 0)
    {
        i += n + 1;
    }
    if (i < 1 || i > n)
    {
        return luaL_error(L, "Invalid control point index");
    }
    curve->points[static_cast<size_t>(i - 1)] = {luax::checkfloat(L, 3), luax::checkfloat(L, 4)};
    return 0;
}

int bz_insertControlPoint(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    Vector2 p = {luax::checkfloat(L, 2), luax::checkfloat(L, 3)};
    lua_Integer i = luaL_optinteger(L, 4, -1);
    lua_Integer n = static_cast<lua_Integer>(curve->points.size());
    if (i < 0)
    {
        i += n + 2;
    }
    if (i < 1 || i > n + 1)
    {
        return luaL_error(L, "Invalid control point index");
    }
    curve->points.insert(curve->points.begin() + (i - 1), p);
    return 0;
}

int bz_removeControlPoint(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    lua_Integer i = luaL_checkinteger(L, 2);
    lua_Integer n = static_cast<lua_Integer>(curve->points.size());
    if (i < 0)
    {
        i += n + 1;
    }
    if (i < 1 || i > n)
    {
        return luaL_error(L, "Invalid control point index");
    }
    curve->points.erase(curve->points.begin() + (i - 1));
    return 0;
}

int bz_getControlPointCount(lua_State *L)
{
    lua_pushinteger(L, static_cast<lua_Integer>(checkBezier(L, 1)->points.size()));
    return 1;
}

int bz_getDegree(lua_State *L)
{
    lua_pushinteger(L, static_cast<lua_Integer>(checkBezier(L, 1)->points.size()) - 1);
    return 1;
}

int bz_getDerivative(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    BezierCurve *d = luax::newobject<BezierCurve>(L, BEZIER_TYPE);
    size_t n = curve->points.size();
    if (n < 2)
    {
        return luaL_error(L, "Cannot derive a curve with less than two control points");
    }
    float degree = static_cast<float>(n - 1);
    for (size_t i = 0; i + 1 < n; ++i)
    {
        d->points.push_back({degree * (curve->points[i + 1].x - curve->points[i].x),
                             degree * (curve->points[i + 1].y - curve->points[i].y)});
    }
    return 1;
}

void pushPointList(lua_State *L, const std::vector<Vector2> &pts)
{
    lua_createtable(L, static_cast<int>(pts.size() * 2), 0);
    int n = 0;
    for (const Vector2 &p : pts)
    {
        lua_pushnumber(L, p.x);
        lua_rawseti(L, -2, ++n);
        lua_pushnumber(L, p.y);
        lua_rawseti(L, -2, ++n);
    }
}

int bz_render(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    int depth = luax::optint(L, 2, 5);
    if (curve->points.size() < 2)
    {
        return luaL_error(L, "Invalid Bezier curve: Not enough control points.");
    }
    std::vector<Vector2> pts;
    curve->subdivide(pts, depth);
    pushPointList(L, pts);
    return 1;
}

int bz_renderSegment(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    double start = luaL_checknumber(L, 2);
    double end = luaL_checknumber(L, 3);
    int depth = luax::optint(L, 4, 5);
    if (curve->points.size() < 2)
    {
        return luaL_error(L, "Invalid Bezier curve: Not enough control points.");
    }
    std::vector<Vector2> pts;
    curve->subdivide(pts, depth);
    size_t from = static_cast<size_t>(std::max(0.0, std::min(1.0, start)) * (pts.size() - 1));
    size_t to = static_cast<size_t>(std::max(0.0, std::min(1.0, end)) * (pts.size() - 1));
    std::vector<Vector2> segment(pts.begin() + from, pts.begin() + to + 1);
    pushPointList(L, segment);
    return 1;
}

int bz_translate(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    float dx = luax::checkfloat(L, 2);
    float dy = luax::checkfloat(L, 3);
    for (Vector2 &p : curve->points)
    {
        p.x += dx;
        p.y += dy;
    }
    return 0;
}

int bz_rotate(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    float angle = luax::checkfloat(L, 2);
    float ox = luax::optfloat(L, 3, 0.0f);
    float oy = luax::optfloat(L, 4, 0.0f);
    float c = std::cos(angle);
    float s = std::sin(angle);
    for (Vector2 &p : curve->points)
    {
        float x = p.x - ox;
        float y = p.y - oy;
        p.x = c * x - s * y + ox;
        p.y = s * x + c * y + oy;
    }
    return 0;
}

int bz_scale(lua_State *L)
{
    BezierCurve *curve = checkBezier(L, 1);
    float scale = luax::checkfloat(L, 2);
    float ox = luax::optfloat(L, 3, 0.0f);
    float oy = luax::optfloat(L, 4, 0.0f);
    for (Vector2 &p : curve->points)
    {
        p.x = (p.x - ox) * scale + ox;
        p.y = (p.y - oy) * scale + oy;
    }
    return 0;
}

const luaL_Reg BEZIER_METHODS[] = {
    {"evaluate", bz_evaluate},
    {"getControlPoint", bz_getControlPoint},
    {"setControlPoint", bz_setControlPoint},
    {"insertControlPoint", bz_insertControlPoint},
    {"removeControlPoint", bz_removeControlPoint},
    {"getControlPointCount", bz_getControlPointCount},
    {"getDegree", bz_getDegree},
    {"getDerivative", bz_getDerivative},
    {"render", bz_render},
    {"renderSegment", bz_renderSegment},
    {"translate", bz_translate},
    {"rotate", bz_rotate},
    {"scale", bz_scale},
    {nullptr, nullptr},
};

int l_newBezierCurve(lua_State *L)
{
    std::vector<float> verts;
    readVertices(L, 1, verts);
    BezierCurve *curve = luax::newobject<BezierCurve>(L, BEZIER_TYPE);
    for (size_t i = 0; i + 1 < verts.size(); i += 2)
    {
        curve->points.push_back({verts[i], verts[i + 1]});
    }
    return 1;
}

int l_compress(lua_State *L)
{
    return luaL_error(L, "love.math.compress is not supported (use love.data in Love2D 11; not available in LoveRay)");
}

const luaL_Reg FUNCS[] = {
    {"random", l_random},
    {"randomNormal", l_randomNormal},
    {"setRandomSeed", l_setRandomSeed},
    {"getRandomSeed", l_getRandomSeed},
    {"setRandomState", l_setRandomState},
    {"getRandomState", l_getRandomState},
    {"newRandomGenerator", l_newRandomGenerator},
    {"noise", l_noise},
    {"colorFromBytes", l_colorFromBytes},
    {"colorToBytes", l_colorToBytes},
    {"gammaToLinear", convertColor<gammaToLinear>},
    {"linearToGamma", convertColor<linearToGamma>},
    {"isConvex", l_isConvex},
    {"triangulate", l_triangulate},
    {"newTransform", l_newTransform},
    {"newBezierCurve", l_newBezierCurve},
    {"compress", l_compress},
    {"decompress", l_compress},
    {nullptr, nullptr},
};

} // namespace

// ---------------------------------------------------------------------------
// Public helpers
// ---------------------------------------------------------------------------

Matrix *checkTransform(lua_State *L, int idx)
{
    return luax::checkobject<Matrix>(L, idx, TRANSFORM_TYPE);
}

void pushTransform(lua_State *L, const Matrix &m)
{
    Matrix *t = luax::newobject<Matrix>(L, TRANSFORM_TYPE);
    *t = m;
}

bool isConvex(const std::vector<float> &pts)
{
    size_t n = pts.size() / 2;
    if (n < 3)
    {
        return false;
    }
    int sign = 0;
    for (size_t i = 0; i < n; ++i)
    {
        size_t j = (i + 1) % n;
        size_t k = (i + 2) % n;
        float ax = pts[j * 2] - pts[i * 2];
        float ay = pts[j * 2 + 1] - pts[i * 2 + 1];
        float bx = pts[k * 2] - pts[j * 2];
        float by = pts[k * 2 + 1] - pts[j * 2 + 1];
        float cross = ax * by - ay * bx;
        if (std::fabs(cross) < 1e-9f)
        {
            continue;
        }
        int s = cross > 0 ? 1 : -1;
        if (sign == 0)
        {
            sign = s;
        }
        else if (s != sign)
        {
            return false;
        }
    }
    return true;
}

namespace
{

float signedArea(const std::vector<float> &pts, const std::vector<int> &order)
{
    float area = 0.0f;
    for (size_t i = 0; i < order.size(); ++i)
    {
        int a = order[i];
        int b = order[(i + 1) % order.size()];
        area += pts[a * 2] * pts[b * 2 + 1] - pts[b * 2] * pts[a * 2 + 1];
    }
    return area * 0.5f;
}

bool pointInTriangle(float px, float py, float ax, float ay, float bx, float by, float cx, float cy)
{
    float d1 = (px - bx) * (ay - by) - (ax - bx) * (py - by);
    float d2 = (px - cx) * (by - cy) - (bx - cx) * (py - cy);
    float d3 = (px - ax) * (cy - ay) - (cx - ax) * (py - ay);
    bool hasNeg = d1 < 0 || d2 < 0 || d3 < 0;
    bool hasPos = d1 > 0 || d2 > 0 || d3 > 0;
    return !(hasNeg && hasPos);
}

} // namespace

bool triangulate(const std::vector<float> &pts, std::vector<int> &indices)
{
    indices.clear();
    int n = static_cast<int>(pts.size() / 2);
    if (n < 3)
    {
        return false;
    }
    std::vector<int> order(n);
    for (int i = 0; i < n; ++i)
    {
        order[i] = i;
    }
    // Work in counter-clockwise order so "convex" has one meaning below.
    if (signedArea(pts, order) < 0)
    {
        std::reverse(order.begin(), order.end());
    }

    int guard = 0;
    while (order.size() > 3 && guard < n * n)
    {
        bool clipped = false;
        for (size_t i = 0; i < order.size(); ++i)
        {
            int ia = order[(i + order.size() - 1) % order.size()];
            int ib = order[i];
            int ic = order[(i + 1) % order.size()];
            float ax = pts[ia * 2], ay = pts[ia * 2 + 1];
            float bx = pts[ib * 2], by = pts[ib * 2 + 1];
            float cx = pts[ic * 2], cy = pts[ic * 2 + 1];
            float cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
            if (cross <= 0)
            {
                continue; // reflex vertex
            }
            bool contains = false;
            for (int other : order)
            {
                if (other == ia || other == ib || other == ic)
                {
                    continue;
                }
                if (pointInTriangle(pts[other * 2], pts[other * 2 + 1], ax, ay, bx, by, cx, cy))
                {
                    contains = true;
                    break;
                }
            }
            if (contains)
            {
                continue;
            }
            indices.push_back(ia);
            indices.push_back(ib);
            indices.push_back(ic);
            order.erase(order.begin() + static_cast<long>(i));
            clipped = true;
            break;
        }
        if (!clipped)
        {
            return false;
        }
        ++guard;
    }
    if (order.size() == 3)
    {
        indices.push_back(order[0]);
        indices.push_back(order[1]);
        indices.push_back(order[2]);
    }
    return true;
}

} // namespace math

int open_math(lua_State *L)
{
    using namespace math;
    luax::newtype(L, RNG_TYPE, RNG_METHODS, luax::gcobject<RandomGenerator>);
    luax::newtype(L, TRANSFORM_TYPE, TRANSFORM_METHODS);
    luax::newtype(L, BEZIER_TYPE, BEZIER_METHODS, luax::gcobject<BezierCurve>);
    g_rng.setSeed(static_cast<uint64_t>(std::random_device{}()) << 32 | std::random_device{}());
    luaL_newlib(L, FUNCS);
    return 1;
}

} // namespace love
