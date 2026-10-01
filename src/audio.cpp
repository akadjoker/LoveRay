// audio.cpp - love.audio (Sources on top of raylib's raudio).
#include "love.hpp"
#include "luax.hpp"

#include <algorithm>
#include <cstring>
#include <set>
#include <string>
#include <vector>

namespace love
{
namespace audio
{

namespace
{

const char *SOURCE_TYPE = "Source";

bool g_deviceReady = false;
float g_masterVolume = 1.0f;
float g_listenerPosition[3] = {0, 0, 0};
float g_listenerVelocity[3] = {0, 0, 0};
float g_listenerOrientation[6] = {0, 0, -1, 0, 1, 0};
std::string g_distanceModel = "inverseclamped";
float g_dopplerScale = 1.0f;

struct Source
{
    bool isStream = false;
    Sound sound = {};
    Music music = {};
    std::vector<unsigned char> data; // kept alive for streams decoded from memory
    bool looping = false;
    bool paused = false;
    bool playing = false;
    float volume = 1.0f;
    float pitch = 1.0f;
    float minVolume = 0.0f;
    float maxVolume = 1.0f;
    float position[3] = {0, 0, 0};
    float velocity[3] = {0, 0, 0};
    float direction[3] = {0, 0, 0};
    bool relative = false;
    float referenceDistance = 1.0f;
    float maxDistance = 1e9f;
    float rolloff = 1.0f;
    float coneInner = 6.283185f;
    float coneOuter = 6.283185f;
    float coneOuterVolume = 0.0f;
    float airAbsorption = 0.0f;
    std::string filename;

    ~Source()
    {
        unload();
    }

    void unload()
    {
        if (isStream)
        {
            if (music.ctxData != nullptr)
            {
                UnloadMusicStream(music);
                music.ctxData = nullptr;
            }
        }
        else if (sound.stream.buffer != nullptr)
        {
            UnloadSound(sound);
            sound.stream.buffer = nullptr;
        }
    }

    bool valid() const
    {
        return isStream ? IsMusicValid(music) : IsSoundValid(sound);
    }

    bool isPlaying() const
    {
        if (!valid() || paused)
        {
            return false;
        }
        return isStream ? IsMusicStreamPlaying(music) : IsSoundPlaying(sound);
    }

    void play()
    {
        if (!valid())
        {
            return;
        }
        if (paused)
        {
            paused = false;
            if (isStream)
            {
                ResumeMusicStream(music);
            }
            else
            {
                ResumeSound(sound);
            }
        }
        else if (!isPlaying())
        {
            if (isStream)
            {
                PlayMusicStream(music);
            }
            else
            {
                PlaySound(sound);
            }
        }
        playing = true;
    }

    void stop()
    {
        if (!valid())
        {
            return;
        }
        if (isStream)
        {
            StopMusicStream(music);
        }
        else
        {
            StopSound(sound);
        }
        paused = false;
        playing = false;
    }

    void pause()
    {
        if (!valid() || !isPlaying())
        {
            return;
        }
        if (isStream)
        {
            PauseMusicStream(music);
        }
        else
        {
            PauseSound(sound);
        }
        paused = true;
    }

    void applyVolume()
    {
        float v = std::min(maxVolume, std::max(minVolume, volume));
        if (isStream)
        {
            SetMusicVolume(music, v);
        }
        else
        {
            SetSoundVolume(sound, v);
        }
    }

    void applyPitch()
    {
        if (isStream)
        {
            SetMusicPitch(music, pitch);
        }
        else
        {
            SetSoundPitch(sound, pitch);
        }
    }

    double duration() const
    {
        if (isStream)
        {
            return GetMusicTimeLength(music);
        }
        if (sound.stream.sampleRate == 0)
        {
            return 0.0;
        }
        return static_cast<double>(sound.frameCount) / sound.stream.sampleRate;
    }
};

std::set<Source *> g_sources;

Source *checkSource(lua_State *L, int idx)
{
    return luax::checkobject<Source>(L, idx, SOURCE_TYPE);
}

int source_gc(lua_State *L)
{
    Source *source = static_cast<Source *>(lua_touserdata(L, 1));
    g_sources.erase(source);
    source->~Source();
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

bool loadSource(Source &source, const std::string &ext, std::vector<unsigned char> &data, bool stream)
{
    ensureDevice();
    source.isStream = stream;
    if (stream)
    {
        source.data = std::move(data);
        source.music = LoadMusicStreamFromMemory(ext.c_str(), source.data.data(), static_cast<int>(source.data.size()));
        source.music.looping = false;
        return IsMusicValid(source.music);
    }
    Wave wave = LoadWaveFromMemory(ext.c_str(), data.data(), static_cast<int>(data.size()));
    if (wave.data == nullptr)
    {
        return false;
    }
    source.sound = LoadSoundFromWave(wave);
    UnloadWave(wave);
    return IsSoundValid(source.sound);
}

// newSource(filename | FileData, type)
int l_newSource(lua_State *L)
{
    std::string filename;
    std::vector<unsigned char> data;
    std::string ext;
    if (lua_type(L, 1) == LUA_TSTRING)
    {
        filename = lua_tostring(L, 1);
        if (!filesystem::readFile(filename, data))
        {
            return luaL_error(L, "Could not open file %s. Does not exist.", filename.c_str());
        }
        ext = fileExtension(filename);
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
        filename = lua_tostring(L, -1) ? lua_tostring(L, -1) : "";
        ext = fileExtension(filename);
        lua_pop(L, 1);
    }
    else
    {
        return luaL_error(L, "bad argument #1 to 'newSource' (filename or FileData expected)");
    }

    static const char *const names[] = {"static", "stream", "queue"};
    static const int values[] = {0, 1, 2};
    int type = luax::checkenum(L, 2, names, values, "source type");
    if (type == 2)
    {
        return luaL_error(L, "Queueable sources are not supported");
    }

    Source *source = luax::newobject<Source>(L, SOURCE_TYPE);
    source->filename = filename;
    if (!loadSource(*source, ext, data, type == 1))
    {
        return luaL_error(L, "Could not decode audio file '%s'", filename.c_str());
    }
    source->applyVolume();
    g_sources.insert(source);
    return 1;
}

int src_play(lua_State *L)
{
    Source *source = checkSource(L, 1);
    source->play();
    lua_pushboolean(L, source->valid());
    return 1;
}

int src_stop(lua_State *L)
{
    checkSource(L, 1)->stop();
    return 0;
}

int src_pause(lua_State *L)
{
    checkSource(L, 1)->pause();
    return 0;
}

int src_isPlaying(lua_State *L)
{
    lua_pushboolean(L, checkSource(L, 1)->isPlaying());
    return 1;
}

int src_setLooping(lua_State *L)
{
    Source *source = checkSource(L, 1);
    source->looping = luax::checkboolean(L, 2);
    if (source->isStream)
    {
        source->music.looping = source->looping;
    }
    return 0;
}

int src_isLooping(lua_State *L)
{
    lua_pushboolean(L, checkSource(L, 1)->looping);
    return 1;
}

int src_setVolume(lua_State *L)
{
    Source *source = checkSource(L, 1);
    source->volume = std::min(1.0f, std::max(0.0f, luax::checkfloat(L, 2)));
    source->applyVolume();
    return 0;
}

int src_getVolume(lua_State *L)
{
    lua_pushnumber(L, checkSource(L, 1)->volume);
    return 1;
}

int src_setPitch(lua_State *L)
{
    Source *source = checkSource(L, 1);
    float pitch = luax::checkfloat(L, 2);
    if (pitch <= 0.0f)
    {
        return luaL_error(L, "Pitch must be greater than zero");
    }
    source->pitch = pitch;
    source->applyPitch();
    return 0;
}

int src_getPitch(lua_State *L)
{
    lua_pushnumber(L, checkSource(L, 1)->pitch);
    return 1;
}

int src_setVolumeLimits(lua_State *L)
{
    Source *source = checkSource(L, 1);
    source->minVolume = luax::checkfloat(L, 2);
    source->maxVolume = luax::checkfloat(L, 3);
    source->applyVolume();
    return 0;
}

int src_getVolumeLimits(lua_State *L)
{
    Source *source = checkSource(L, 1);
    lua_pushnumber(L, source->minVolume);
    lua_pushnumber(L, source->maxVolume);
    return 2;
}

double unitFactor(lua_State *L, int idx, const Source &source)
{
    const char *unit = luaL_optstring(L, idx, "seconds");
    if (std::strcmp(unit, "samples") == 0)
    {
        unsigned int rate = source.isStream ? source.music.stream.sampleRate : source.sound.stream.sampleRate;
        return rate == 0 ? 1.0 : static_cast<double>(rate);
    }
    if (std::strcmp(unit, "seconds") != 0)
    {
        luaL_error(L, "Invalid time unit '%s' (expected 'seconds' or 'samples')", unit);
    }
    return 1.0;
}

int src_seek(lua_State *L)
{
    Source *source = checkSource(L, 1);
    double offset = luaL_checknumber(L, 2) / unitFactor(L, 3, *source);
    if (source->isStream)
    {
        SeekMusicStream(source->music, static_cast<float>(offset));
    }
    return 0;
}

int src_tell(lua_State *L)
{
    Source *source = checkSource(L, 1);
    double seconds = source->isStream ? GetMusicTimePlayed(source->music) : 0.0;
    lua_pushnumber(L, seconds * unitFactor(L, 2, *source));
    return 1;
}

int src_getDuration(lua_State *L)
{
    Source *source = checkSource(L, 1);
    lua_pushnumber(L, source->duration() * unitFactor(L, 2, *source));
    return 1;
}

int src_getType(lua_State *L)
{
    lua_pushstring(L, checkSource(L, 1)->isStream ? "stream" : "static");
    return 1;
}

int src_getChannelCount(lua_State *L)
{
    Source *source = checkSource(L, 1);
    lua_pushinteger(L, source->isStream ? source->music.stream.channels : source->sound.stream.channels);
    return 1;
}

int src_getSampleRate(lua_State *L)
{
    Source *source = checkSource(L, 1);
    lua_pushinteger(L, source->isStream ? source->music.stream.sampleRate : source->sound.stream.sampleRate);
    return 1;
}

int src_clone(lua_State *L)
{
    Source *source = checkSource(L, 1);
    Source *copy = luax::newobject<Source>(L, SOURCE_TYPE);
    copy->filename = source->filename;
    copy->looping = source->looping;
    copy->volume = source->volume;
    copy->pitch = source->pitch;
    copy->isStream = source->isStream;
    if (source->isStream)
    {
        copy->data = source->data;
        copy->music = LoadMusicStreamFromMemory(fileExtension(source->filename).c_str(), copy->data.data(),
                                                static_cast<int>(copy->data.size()));
        copy->music.looping = copy->looping;
    }
    else
    {
        copy->sound = LoadSoundAlias(source->sound);
        // Aliases share sample data; release only the alias on collection.
    }
    copy->applyVolume();
    copy->applyPitch();
    g_sources.insert(copy);
    return 1;
}

int src_setPosition(lua_State *L)
{
    Source *source = checkSource(L, 1);
    for (int i = 0; i < 3; ++i)
    {
        source->position[i] = luax::optfloat(L, 2 + i, 0.0f);
    }
    return 0;
}

int src_getPosition(lua_State *L)
{
    Source *source = checkSource(L, 1);
    for (float v : source->position)
    {
        lua_pushnumber(L, v);
    }
    return 3;
}

int src_setVelocity(lua_State *L)
{
    Source *source = checkSource(L, 1);
    for (int i = 0; i < 3; ++i)
    {
        source->velocity[i] = luax::optfloat(L, 2 + i, 0.0f);
    }
    return 0;
}

int src_getVelocity(lua_State *L)
{
    Source *source = checkSource(L, 1);
    for (float v : source->velocity)
    {
        lua_pushnumber(L, v);
    }
    return 3;
}

int src_setDirection(lua_State *L)
{
    Source *source = checkSource(L, 1);
    for (int i = 0; i < 3; ++i)
    {
        source->direction[i] = luax::optfloat(L, 2 + i, 0.0f);
    }
    return 0;
}

int src_getDirection(lua_State *L)
{
    Source *source = checkSource(L, 1);
    for (float v : source->direction)
    {
        lua_pushnumber(L, v);
    }
    return 3;
}

int src_setRelative(lua_State *L)
{
    checkSource(L, 1)->relative = luax::checkboolean(L, 2);
    return 0;
}

int src_isRelative(lua_State *L)
{
    lua_pushboolean(L, checkSource(L, 1)->relative);
    return 1;
}

int src_setAttenuationDistances(lua_State *L)
{
    Source *source = checkSource(L, 1);
    source->referenceDistance = luax::checkfloat(L, 2);
    source->maxDistance = luax::checkfloat(L, 3);
    return 0;
}

int src_getAttenuationDistances(lua_State *L)
{
    Source *source = checkSource(L, 1);
    lua_pushnumber(L, source->referenceDistance);
    lua_pushnumber(L, source->maxDistance);
    return 2;
}

int src_setRolloff(lua_State *L)
{
    checkSource(L, 1)->rolloff = luax::checkfloat(L, 2);
    return 0;
}

int src_getRolloff(lua_State *L)
{
    lua_pushnumber(L, checkSource(L, 1)->rolloff);
    return 1;
}

int src_setCone(lua_State *L)
{
    Source *source = checkSource(L, 1);
    source->coneInner = luax::checkfloat(L, 2);
    source->coneOuter = luax::checkfloat(L, 3);
    source->coneOuterVolume = luax::optfloat(L, 4, 0.0f);
    return 0;
}

int src_getCone(lua_State *L)
{
    Source *source = checkSource(L, 1);
    lua_pushnumber(L, source->coneInner);
    lua_pushnumber(L, source->coneOuter);
    lua_pushnumber(L, source->coneOuterVolume);
    return 3;
}

int src_setAirAbsorption(lua_State *L)
{
    checkSource(L, 1)->airAbsorption = luax::checkfloat(L, 2);
    return 0;
}

int src_getAirAbsorption(lua_State *L)
{
    lua_pushnumber(L, checkSource(L, 1)->airAbsorption);
    return 1;
}

int src_setFilter(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

int src_getFilter(lua_State *L)
{
    lua_pushnil(L);
    return 1;
}

int src_setEffect(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

int src_getEffect(lua_State *L)
{
    lua_pushnil(L);
    return 1;
}

int src_getActiveEffects(lua_State *L)
{
    lua_newtable(L);
    return 1;
}

int src_getFreeBufferCount(lua_State *L)
{
    lua_pushinteger(L, 0);
    return 1;
}

int src_queue(lua_State *L)
{
    return luaL_error(L, "Source:queue is only available on queueable sources, which are not supported");
}

int src_tostring(lua_State *L)
{
    Source *source = checkSource(L, 1);
    lua_pushfstring(L, "Source: %s (%s)", source->filename.c_str(), source->isStream ? "stream" : "static");
    return 1;
}

const luaL_Reg SOURCE_METHODS[] = {
    {"play", src_play},
    {"stop", src_stop},
    {"pause", src_pause},
    {"isPlaying", src_isPlaying},
    {"setLooping", src_setLooping},
    {"isLooping", src_isLooping},
    {"setVolume", src_setVolume},
    {"getVolume", src_getVolume},
    {"setPitch", src_setPitch},
    {"getPitch", src_getPitch},
    {"setVolumeLimits", src_setVolumeLimits},
    {"getVolumeLimits", src_getVolumeLimits},
    {"seek", src_seek},
    {"tell", src_tell},
    {"getDuration", src_getDuration},
    {"getType", src_getType},
    {"getChannelCount", src_getChannelCount},
    {"getSampleRate", src_getSampleRate},
    {"clone", src_clone},
    {"setPosition", src_setPosition},
    {"getPosition", src_getPosition},
    {"setVelocity", src_setVelocity},
    {"getVelocity", src_getVelocity},
    {"setDirection", src_setDirection},
    {"getDirection", src_getDirection},
    {"setRelative", src_setRelative},
    {"isRelative", src_isRelative},
    {"setAttenuationDistances", src_setAttenuationDistances},
    {"getAttenuationDistances", src_getAttenuationDistances},
    {"setRolloff", src_setRolloff},
    {"getRolloff", src_getRolloff},
    {"setCone", src_setCone},
    {"getCone", src_getCone},
    {"setAirAbsorption", src_setAirAbsorption},
    {"getAirAbsorption", src_getAirAbsorption},
    {"setFilter", src_setFilter},
    {"getFilter", src_getFilter},
    {"setEffect", src_setEffect},
    {"getEffect", src_getEffect},
    {"getActiveEffects", src_getActiveEffects},
    {"getFreeBufferCount", src_getFreeBufferCount},
    {"queue", src_queue},
    {"__tostring", src_tostring},
    {nullptr, nullptr},
};

// Applies `fn` to every Source passed as arguments (or inside a table).
template <class Fn>
void forEachSourceArg(lua_State *L, Fn fn)
{
    int n = lua_gettop(L);
    for (int i = 1; i <= n; ++i)
    {
        if (lua_istable(L, i))
        {
            lua_Integer count = luaL_len(L, i);
            for (lua_Integer k = 1; k <= count; ++k)
            {
                lua_rawgeti(L, i, k);
                fn(checkSource(L, -1));
                lua_pop(L, 1);
            }
        }
        else
        {
            fn(checkSource(L, i));
        }
    }
}

int l_play(lua_State *L)
{
    bool all = true;
    forEachSourceArg(L, [&](Source *s) {
        s->play();
        all = all && s->valid();
    });
    lua_pushboolean(L, all);
    return 1;
}

int l_stop(lua_State *L)
{
    if (lua_gettop(L) == 0)
    {
        for (Source *s : g_sources)
        {
            s->stop();
        }
        return 0;
    }
    forEachSourceArg(L, [](Source *s) { s->stop(); });
    return 0;
}

int l_pause(lua_State *L)
{
    if (lua_gettop(L) == 0)
    {
        lua_newtable(L);
        int n = 0;
        for (Source *s : g_sources)
        {
            if (s->isPlaying())
            {
                s->pause();
                if (luax::pushregistered(L, s))
                {
                    lua_rawseti(L, -2, ++n);
                }
            }
        }
        return 1;
    }
    forEachSourceArg(L, [](Source *s) { s->pause(); });
    return 0;
}

int l_setVolume(lua_State *L)
{
    g_masterVolume = std::min(1.0f, std::max(0.0f, luax::checkfloat(L, 1)));
    ensureDevice();
    SetMasterVolume(g_masterVolume);
    return 0;
}

int l_getVolume(lua_State *L)
{
    lua_pushnumber(L, g_masterVolume);
    return 1;
}

int l_getActiveSourceCount(lua_State *L)
{
    int count = 0;
    for (Source *s : g_sources)
    {
        if (s->isPlaying())
        {
            ++count;
        }
    }
    lua_pushinteger(L, count);
    return 1;
}

int l_getSourceCount(lua_State *L)
{
    return l_getActiveSourceCount(L);
}

int l_setPosition(lua_State *L)
{
    for (int i = 0; i < 3; ++i)
    {
        g_listenerPosition[i] = luax::optfloat(L, 1 + i, 0.0f);
    }
    return 0;
}

int l_getPosition(lua_State *L)
{
    for (float v : g_listenerPosition)
    {
        lua_pushnumber(L, v);
    }
    return 3;
}

int l_setVelocity(lua_State *L)
{
    for (int i = 0; i < 3; ++i)
    {
        g_listenerVelocity[i] = luax::optfloat(L, 1 + i, 0.0f);
    }
    return 0;
}

int l_getVelocity(lua_State *L)
{
    for (float v : g_listenerVelocity)
    {
        lua_pushnumber(L, v);
    }
    return 3;
}

int l_setOrientation(lua_State *L)
{
    for (int i = 0; i < 6; ++i)
    {
        g_listenerOrientation[i] = luax::optfloat(L, 1 + i, 0.0f);
    }
    return 0;
}

int l_getOrientation(lua_State *L)
{
    for (float v : g_listenerOrientation)
    {
        lua_pushnumber(L, v);
    }
    return 6;
}

int l_setDistanceModel(lua_State *L)
{
    g_distanceModel = luaL_checkstring(L, 1);
    return 0;
}

int l_getDistanceModel(lua_State *L)
{
    lua_pushstring(L, g_distanceModel.c_str());
    return 1;
}

int l_setDopplerScale(lua_State *L)
{
    g_dopplerScale = luax::checkfloat(L, 1);
    return 0;
}

int l_getDopplerScale(lua_State *L)
{
    lua_pushnumber(L, g_dopplerScale);
    return 1;
}

int l_getRecordingDevices(lua_State *L)
{
    lua_newtable(L);
    return 1;
}

int l_setMixWithSystem(lua_State *L)
{
    lua_pushboolean(L, 1);
    return 1;
}

int l_isEffectsSupported(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

int l_setEffect(lua_State *L)
{
    lua_pushboolean(L, 0);
    return 1;
}

int l_getEffect(lua_State *L)
{
    lua_pushnil(L);
    return 1;
}

int l_getActiveEffects(lua_State *L)
{
    lua_newtable(L);
    return 1;
}

int l_getMaxSceneEffects(lua_State *L)
{
    lua_pushinteger(L, 0);
    return 1;
}

int l_getMaxSourceEffects(lua_State *L)
{
    lua_pushinteger(L, 0);
    return 1;
}

const luaL_Reg FUNCS[] = {
    {"newSource", l_newSource},
    {"play", l_play},
    {"stop", l_stop},
    {"pause", l_pause},
    {"setVolume", l_setVolume},
    {"getVolume", l_getVolume},
    {"getActiveSourceCount", l_getActiveSourceCount},
    {"getSourceCount", l_getSourceCount},
    {"setPosition", l_setPosition},
    {"getPosition", l_getPosition},
    {"setVelocity", l_setVelocity},
    {"getVelocity", l_getVelocity},
    {"setOrientation", l_setOrientation},
    {"getOrientation", l_getOrientation},
    {"setDistanceModel", l_setDistanceModel},
    {"getDistanceModel", l_getDistanceModel},
    {"setDopplerScale", l_setDopplerScale},
    {"getDopplerScale", l_getDopplerScale},
    {"getRecordingDevices", l_getRecordingDevices},
    {"setMixWithSystem", l_setMixWithSystem},
    {"isEffectsSupported", l_isEffectsSupported},
    {"setEffect", l_setEffect},
    {"getEffect", l_getEffect},
    {"getActiveEffects", l_getActiveEffects},
    {"getMaxSceneEffects", l_getMaxSceneEffects},
    {"getMaxSourceEffects", l_getMaxSourceEffects},
    {nullptr, nullptr},
};

} // namespace

void ensureDevice()
{
    if (g_deviceReady)
    {
        return;
    }
    SetTraceLogLevel(LOG_WARNING);
    InitAudioDevice();
    g_deviceReady = true;
    if (IsAudioDeviceReady())
    {
        SetMasterVolume(g_masterVolume);
    }
}

void update()
{
    if (!g_deviceReady)
    {
        return;
    }
    for (Source *s : g_sources)
    {
        if (!s->valid())
        {
            continue;
        }
        if (s->isStream)
        {
            UpdateMusicStream(s->music);
            if (s->playing && !s->paused && !IsMusicStreamPlaying(s->music))
            {
                s->playing = false;
            }
        }
        else if (s->playing && !s->paused && !IsSoundPlaying(s->sound))
        {
            if (s->looping)
            {
                PlaySound(s->sound);
            }
            else
            {
                s->playing = false;
            }
        }
    }
}

void shutdown()
{
    g_sources.clear();
    if (g_deviceReady)
    {
        CloseAudioDevice();
        g_deviceReady = false;
    }
}

} // namespace audio

int open_audio(lua_State *L)
{
    luax::newtype(L, audio::SOURCE_TYPE, audio::SOURCE_METHODS, audio::source_gc);
    luaL_newlib(L, audio::FUNCS);
    return 1;
}

} // namespace love
