// particles.cpp - love.graphics.newParticleSystem
#include "graphics_internal.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace love
{
namespace graphics
{

namespace
{

constexpr int MAX_SIZES = 8;
constexpr int MAX_COLORS = 8;
constexpr int MAX_BUFFER = 1048576;
constexpr float PI_F = 3.14159265358979f;

enum class InsertMode
{
    Top,
    Bottom,
    Random
};

enum class AreaDistribution
{
    None,
    Uniform,
    Normal,
    Ellipse,
    BorderEllipse,
    BorderRectangle
};

const char *const INSERT_NAMES[] = {"top", "bottom", "random"};
const int INSERT_VALUES[] = {static_cast<int>(InsertMode::Top), static_cast<int>(InsertMode::Bottom),
                             static_cast<int>(InsertMode::Random)};

const char *const AREA_NAMES[] = {"none", "uniform", "normal", "ellipse", "borderellipse", "borderrectangle"};
const int AREA_VALUES[] = {static_cast<int>(AreaDistribution::None),          static_cast<int>(AreaDistribution::Uniform),
                           static_cast<int>(AreaDistribution::Normal),        static_cast<int>(AreaDistribution::Ellipse),
                           static_cast<int>(AreaDistribution::BorderEllipse), static_cast<int>(AreaDistribution::BorderRectangle)};

struct RGBA
{
    float r = 1, g = 1, b = 1, a = 1;
};

struct Particle
{
    float life = 0;
    float lifetime = 0;
    Vector2 position = {0, 0};
    Vector2 origin = {0, 0};
    Vector2 velocity = {0, 0};
    Vector2 linearAcceleration = {0, 0};
    float radialAcceleration = 0;
    float tangentialAcceleration = 0;
    float linearDamping = 0;
    float size = 1;
    float sizeOffset = 0;
    float spinStart = 0;
    float spinEnd = 0;
    float rotation = 0;
    float angle = 0;
    RGBA color;
    int quad = 0;
};

} // namespace

struct ParticleSystemObj
{
    luax::Ref texture;
    int textureWidth = 1;
    int textureHeight = 1;
    std::vector<QuadObj> quads;
    std::vector<Particle> particles;
    int bufferSize = 1000;

    bool active = true;
    bool paused = false;
    float emissionRate = 0;
    float emitCounter = 0;
    float life = 0;
    float emitterLifetime = -1;
    float lifetimeMin = 0;
    float lifetimeMax = 0;
    Vector2 position = {0, 0};
    Vector2 previousPosition = {0, 0};
    float direction = 0;
    float spread = 0;
    float speedMin = 0, speedMax = 0;
    float accelXMin = 0, accelYMin = 0, accelXMax = 0, accelYMax = 0;
    float dampingMin = 0, dampingMax = 0;
    float radialMin = 0, radialMax = 0;
    float tangentialMin = 0, tangentialMax = 0;
    std::vector<float> sizes = {1.0f};
    float sizeVariation = 0;
    float rotationMin = 0, rotationMax = 0;
    float spinMin = 0, spinMax = 0, spinVariation = 0;
    Vector2 offset = {0, 0};
    std::vector<RGBA> colors = {RGBA()};
    bool relativeRotation = false;
    InsertMode insertMode = InsertMode::Top;
    AreaDistribution area = AreaDistribution::None;
    Vector2 areaSpread = {0, 0};
    float areaAngle = 0;
    bool areaRelativeToCenter = false;
    std::mt19937 rng{std::random_device{}()};

    float random()
    {
        return std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
    }

    float random(float a, float b)
    {
        return a == b ? a : a + (b - a) * random();
    }

    float randomNormal(float deviation)
    {
        return deviation <= 0.0f ? 0.0f : std::normal_distribution<float>(0.0f, deviation)(rng);
    }

    float variation(float inner, float outer, float amount)
    {
        float low = inner - (outer * 0.5f) * amount;
        float high = inner + (outer * 0.5f) * amount;
        return random(low, high);
    }

    static float lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    // Size, color and quad depend only on the fraction of life that has passed.
    void applyAge(Particle &p)
    {
        float t = p.lifetime > 0.0f ? 1.0f - p.life / p.lifetime : 0.0f;
        t = std::min(1.0f, std::max(0.0f, t));

        float s = t * static_cast<float>(sizes.size() - 1);
        size_t i = static_cast<size_t>(s);
        size_t k = std::min(i + 1, sizes.size() - 1);
        p.size = lerp(sizes[i], sizes[k], s - static_cast<float>(i)) * (1.0f - p.sizeOffset);

        float c = t * static_cast<float>(colors.size() - 1);
        size_t ci = static_cast<size_t>(c);
        size_t ck = std::min(ci + 1, colors.size() - 1);
        float f = c - static_cast<float>(ci);
        p.color = {lerp(colors[ci].r, colors[ck].r, f), lerp(colors[ci].g, colors[ck].g, f),
                   lerp(colors[ci].b, colors[ck].b, f), lerp(colors[ci].a, colors[ck].a, f)};

        if (!quads.empty())
        {
            p.quad = std::min(static_cast<int>(quads.size()) - 1, static_cast<int>(t * static_cast<float>(quads.size())));
        }
    }

    Vector2 areaOffset()
    {
        Vector2 off = {0, 0};
        switch (area)
        {
        case AreaDistribution::None:
            return off;
        case AreaDistribution::Uniform:
            off = {random(-areaSpread.x, areaSpread.x), random(-areaSpread.y, areaSpread.y)};
            break;
        case AreaDistribution::Normal:
            off = {randomNormal(areaSpread.x), randomNormal(areaSpread.y)};
            break;
        case AreaDistribution::Ellipse:
        {
            float angle = random(0.0f, 2.0f * PI_F);
            float radius = std::sqrt(random());
            off = {std::cos(angle) * radius * areaSpread.x, std::sin(angle) * radius * areaSpread.y};
            break;
        }
        case AreaDistribution::BorderEllipse:
        {
            float angle = random(0.0f, 2.0f * PI_F);
            off = {std::cos(angle) * areaSpread.x, std::sin(angle) * areaSpread.y};
            break;
        }
        case AreaDistribution::BorderRectangle:
        {
            float w = areaSpread.x * 2.0f;
            float h = areaSpread.y * 2.0f;
            float perimeter = 2.0f * (w + h);
            float d = random(0.0f, perimeter);
            if (perimeter <= 0.0f)
            {
                break;
            }
            if (d < w)
            {
                off = {-areaSpread.x + d, -areaSpread.y};
            }
            else if (d < w + h)
            {
                off = {areaSpread.x, -areaSpread.y + (d - w)};
            }
            else if (d < 2.0f * w + h)
            {
                off = {areaSpread.x - (d - w - h), areaSpread.y};
            }
            else
            {
                off = {-areaSpread.x, areaSpread.y - (d - 2.0f * w - h)};
            }
            break;
        }
        }
        float c = std::cos(areaAngle);
        float s = std::sin(areaAngle);
        return {off.x * c - off.y * s, off.x * s + off.y * c};
    }

    void emitParticle(float t)
    {
        if (static_cast<int>(particles.size()) >= bufferSize)
        {
            return;
        }
        Particle p;
        p.lifetime = random(lifetimeMin, lifetimeMax);
        p.life = p.lifetime;

        Vector2 base = {lerp(previousPosition.x, position.x, t), lerp(previousPosition.y, position.y, t)};
        Vector2 off = areaOffset();
        p.position = {base.x + off.x, base.y + off.y};
        p.origin = base;

        float dir = random(direction - spread * 0.5f, direction + spread * 0.5f);
        if (area != AreaDistribution::None && areaRelativeToCenter && (off.x != 0.0f || off.y != 0.0f))
        {
            dir += std::atan2(off.y, off.x);
        }
        float speed = random(speedMin, speedMax);
        p.velocity = {std::cos(dir) * speed, std::sin(dir) * speed};

        p.linearAcceleration = {random(accelXMin, accelXMax), random(accelYMin, accelYMax)};
        p.radialAcceleration = random(radialMin, radialMax);
        p.tangentialAcceleration = random(tangentialMin, tangentialMax);
        p.linearDamping = random(dampingMin, dampingMax);
        p.sizeOffset = random(0.0f, sizeVariation);
        p.spinStart = variation(spinMin, spinMax, spinVariation);
        p.spinEnd = variation(spinMax, spinMin, spinVariation);
        p.rotation = random(rotationMin, rotationMax);
        p.angle = p.rotation;
        if (relativeRotation)
        {
            p.angle += std::atan2(p.velocity.y, p.velocity.x);
        }
        applyAge(p);

        switch (insertMode)
        {
        case InsertMode::Top:
            particles.push_back(p);
            break;
        case InsertMode::Bottom:
            particles.insert(particles.begin(), p);
            break;
        case InsertMode::Random:
        {
            size_t at = static_cast<size_t>(random() * static_cast<float>(particles.size() + 1));
            particles.insert(particles.begin() + static_cast<long>(std::min(at, particles.size())), p);
            break;
        }
        }
    }

    void update(float dt)
    {
        if (dt == 0.0f || paused)
        {
            return;
        }

        size_t keep = 0;
        for (size_t i = 0; i < particles.size(); ++i)
        {
            Particle &p = particles[i];
            p.life -= dt;
            if (p.life <= 0.0f)
            {
                continue;
            }

            Vector2 radial = {p.position.x - p.origin.x, p.position.y - p.origin.y};
            float length = std::sqrt(radial.x * radial.x + radial.y * radial.y);
            if (length > 0.0f)
            {
                radial = {radial.x / length, radial.y / length};
            }
            Vector2 tangential = {-radial.y, radial.x};

            float damping = 1.0f / (1.0f + p.linearDamping * dt);
            p.velocity.x *= damping;
            p.velocity.y *= damping;
            p.velocity.x += (radial.x * p.radialAcceleration + tangential.x * p.tangentialAcceleration + p.linearAcceleration.x) * dt;
            p.velocity.y += (radial.y * p.radialAcceleration + tangential.y * p.tangentialAcceleration + p.linearAcceleration.y) * dt;
            p.position.x += p.velocity.x * dt;
            p.position.y += p.velocity.y * dt;

            float age = 1.0f - p.life / p.lifetime;
            p.rotation += lerp(p.spinStart, p.spinEnd, age) * dt;
            p.angle = p.rotation;
            if (relativeRotation)
            {
                p.angle += std::atan2(p.velocity.y, p.velocity.x);
            }
            applyAge(p);

            if (keep != i)
            {
                particles[keep] = p;
            }
            ++keep;
        }
        particles.resize(keep);

        if (active && emissionRate > 0.0f)
        {
            float rate = 1.0f / emissionRate;
            emitCounter += dt;
            float total = emitCounter - rate;
            while (emitCounter > rate)
            {
                emitParticle(total > 0.0f ? 1.0f - (emitCounter - rate) / total : 1.0f);
                emitCounter -= rate;
            }
        }
        if (active)
        {
            life += dt;
            if (emitterLifetime >= 0.0f && life >= emitterLifetime)
            {
                stop();
            }
        }
        previousPosition = position;
    }

    void stop()
    {
        active = false;
        life = 0.0f;
        emitCounter = 0.0f;
    }

    void reset()
    {
        particles.clear();
        life = 0.0f;
        emitCounter = 0.0f;
    }
};

namespace
{

ParticleSystemObj *check(lua_State *L, int idx)
{
    return luax::checkobject<ParticleSystemObj>(L, idx, PARTICLES_TYPE);
}

int ps_gc(lua_State *L)
{
    ParticleSystemObj *ps = static_cast<ParticleSystemObj *>(lua_touserdata(L, 1));
    ps->texture.clear(L);
    ps->~ParticleSystemObj();
    return 0;
}

void setTextureFrom(lua_State *L, ParticleSystemObj &ps, int idx)
{
    DrawSource src = checkDrawSource(L, idx);
    ps.texture.set(L, idx);
    ps.textureWidth = src.texture.width;
    ps.textureHeight = src.texture.height;
}

int l_newParticleSystem(lua_State *L)
{
    DrawSource src = checkDrawSource(L, 1);
    int buffer = luax::optint(L, 2, 1000);
    if (buffer < 1 || buffer > MAX_BUFFER)
    {
        return luaL_error(L, "Invalid buffer size (1 to %d)", MAX_BUFFER);
    }
    ParticleSystemObj *ps = luax::newobject<ParticleSystemObj>(L, PARTICLES_TYPE);
    ps->texture.set(L, 1);
    ps->textureWidth = src.texture.width;
    ps->textureHeight = src.texture.height;
    ps->bufferSize = buffer;
    ps->offset = {static_cast<float>(src.texture.width) * 0.5f, static_cast<float>(src.texture.height) * 0.5f};
    ps->particles.reserve(static_cast<size_t>(std::min(buffer, 4096)));
    return 1;
}

int ps_clone(lua_State *L)
{
    ParticleSystemObj *source = check(L, 1);
    ParticleSystemObj *copy = luax::newobject<ParticleSystemObj>(L, PARTICLES_TYPE);
    copy->quads = source->quads;
    copy->bufferSize = source->bufferSize;
    copy->emissionRate = source->emissionRate;
    copy->emitterLifetime = source->emitterLifetime;
    copy->lifetimeMin = source->lifetimeMin;
    copy->lifetimeMax = source->lifetimeMax;
    copy->position = copy->previousPosition = source->position;
    copy->direction = source->direction;
    copy->spread = source->spread;
    copy->speedMin = source->speedMin;
    copy->speedMax = source->speedMax;
    copy->accelXMin = source->accelXMin;
    copy->accelYMin = source->accelYMin;
    copy->accelXMax = source->accelXMax;
    copy->accelYMax = source->accelYMax;
    copy->dampingMin = source->dampingMin;
    copy->dampingMax = source->dampingMax;
    copy->radialMin = source->radialMin;
    copy->radialMax = source->radialMax;
    copy->tangentialMin = source->tangentialMin;
    copy->tangentialMax = source->tangentialMax;
    copy->sizes = source->sizes;
    copy->sizeVariation = source->sizeVariation;
    copy->rotationMin = source->rotationMin;
    copy->rotationMax = source->rotationMax;
    copy->spinMin = source->spinMin;
    copy->spinMax = source->spinMax;
    copy->spinVariation = source->spinVariation;
    copy->offset = source->offset;
    copy->colors = source->colors;
    copy->relativeRotation = source->relativeRotation;
    copy->insertMode = source->insertMode;
    copy->area = source->area;
    copy->areaSpread = source->areaSpread;
    copy->areaAngle = source->areaAngle;
    copy->areaRelativeToCenter = source->areaRelativeToCenter;
    copy->textureWidth = source->textureWidth;
    copy->textureHeight = source->textureHeight;
    source->texture.push(L);
    copy->texture.set(L, -1);
    lua_pop(L, 1);
    copy->active = false;
    return 1;
}

// Emission --------------------------------------------------------------------

int ps_emit(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    int count = static_cast<int>(luaL_checkinteger(L, 2));
    for (int i = 0; i < count; ++i)
    {
        ps->emitParticle(1.0f);
    }
    return 0;
}

int ps_start(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->active = true;
    ps->paused = false;
    return 0;
}

int ps_stop(lua_State *L)
{
    check(L, 1)->stop();
    return 0;
}

int ps_pause(lua_State *L)
{
    check(L, 1)->paused = true;
    return 0;
}

int ps_reset(lua_State *L)
{
    check(L, 1)->reset();
    return 0;
}

int ps_update(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->update(luax::checkfloat(L, 2));
    return 0;
}

int ps_isActive(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushboolean(L, ps->active && !ps->paused);
    return 1;
}

int ps_isPaused(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushboolean(L, ps->active && ps->paused);
    return 1;
}

int ps_isStopped(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushboolean(L, !ps->active && ps->life == 0.0f);
    return 1;
}

int ps_getCount(lua_State *L)
{
    lua_pushinteger(L, static_cast<lua_Integer>(check(L, 1)->particles.size()));
    return 1;
}

int ps_setBufferSize(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_Integer size = luaL_checkinteger(L, 2);
    if (size < 1 || size > MAX_BUFFER)
    {
        return luaL_error(L, "Invalid buffer size (1 to %d)", MAX_BUFFER);
    }
    ps->bufferSize = static_cast<int>(size);
    if (static_cast<int>(ps->particles.size()) > ps->bufferSize)
    {
        ps->particles.resize(static_cast<size_t>(ps->bufferSize));
    }
    return 0;
}

int ps_getBufferSize(lua_State *L)
{
    lua_pushinteger(L, check(L, 1)->bufferSize);
    return 1;
}

int ps_setEmissionRate(lua_State *L)
{
    float rate = luax::checkfloat(L, 2);
    if (rate < 0.0f)
    {
        return luaL_error(L, "Invalid emission rate (must be >= 0)");
    }
    check(L, 1)->emissionRate = rate;
    return 0;
}

int ps_getEmissionRate(lua_State *L)
{
    lua_pushnumber(L, check(L, 1)->emissionRate);
    return 1;
}

int ps_setEmitterLifetime(lua_State *L)
{
    check(L, 1)->emitterLifetime = luax::checkfloat(L, 2);
    return 0;
}

int ps_getEmitterLifetime(lua_State *L)
{
    lua_pushnumber(L, check(L, 1)->emitterLifetime);
    return 1;
}

int ps_setParticleLifetime(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->lifetimeMin = luax::checkfloat(L, 2);
    ps->lifetimeMax = luax::optfloat(L, 3, ps->lifetimeMin);
    return 0;
}

int ps_getParticleLifetime(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushnumber(L, ps->lifetimeMin);
    lua_pushnumber(L, ps->lifetimeMax);
    return 2;
}

int ps_setPosition(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->position = {luax::checkfloat(L, 2), luax::checkfloat(L, 3)};
    ps->previousPosition = ps->position;
    return 0;
}

int ps_moveTo(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->previousPosition = ps->position;
    ps->position = {luax::checkfloat(L, 2), luax::checkfloat(L, 3)};
    return 0;
}

int ps_getPosition(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushnumber(L, ps->position.x);
    lua_pushnumber(L, ps->position.y);
    return 2;
}

int ps_setDirection(lua_State *L)
{
    check(L, 1)->direction = luax::checkfloat(L, 2);
    return 0;
}

int ps_getDirection(lua_State *L)
{
    lua_pushnumber(L, check(L, 1)->direction);
    return 1;
}

int ps_setSpread(lua_State *L)
{
    check(L, 1)->spread = luax::checkfloat(L, 2);
    return 0;
}

int ps_getSpread(lua_State *L)
{
    lua_pushnumber(L, check(L, 1)->spread);
    return 1;
}

int ps_setSpeed(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->speedMin = luax::checkfloat(L, 2);
    ps->speedMax = luax::optfloat(L, 3, ps->speedMin);
    return 0;
}

int ps_getSpeed(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushnumber(L, ps->speedMin);
    lua_pushnumber(L, ps->speedMax);
    return 2;
}

int ps_setLinearAcceleration(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->accelXMin = luax::checkfloat(L, 2);
    ps->accelYMin = luax::checkfloat(L, 3);
    ps->accelXMax = luax::optfloat(L, 4, ps->accelXMin);
    ps->accelYMax = luax::optfloat(L, 5, ps->accelYMin);
    return 0;
}

int ps_getLinearAcceleration(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushnumber(L, ps->accelXMin);
    lua_pushnumber(L, ps->accelYMin);
    lua_pushnumber(L, ps->accelXMax);
    lua_pushnumber(L, ps->accelYMax);
    return 4;
}

int ps_setLinearDamping(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->dampingMin = luax::checkfloat(L, 2);
    ps->dampingMax = luax::optfloat(L, 3, ps->dampingMin);
    return 0;
}

int ps_getLinearDamping(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushnumber(L, ps->dampingMin);
    lua_pushnumber(L, ps->dampingMax);
    return 2;
}

int ps_setRadialAcceleration(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->radialMin = luax::checkfloat(L, 2);
    ps->radialMax = luax::optfloat(L, 3, ps->radialMin);
    return 0;
}

int ps_getRadialAcceleration(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushnumber(L, ps->radialMin);
    lua_pushnumber(L, ps->radialMax);
    return 2;
}

int ps_setTangentialAcceleration(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->tangentialMin = luax::checkfloat(L, 2);
    ps->tangentialMax = luax::optfloat(L, 3, ps->tangentialMin);
    return 0;
}

int ps_getTangentialAcceleration(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushnumber(L, ps->tangentialMin);
    lua_pushnumber(L, ps->tangentialMax);
    return 2;
}

int ps_setSizes(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    int count = lua_gettop(L) - 1;
    if (count < 1 || count > MAX_SIZES)
    {
        return luaL_error(L, "Invalid number of particle sizes (1 to %d)", MAX_SIZES);
    }
    ps->sizes.clear();
    for (int i = 0; i < count; ++i)
    {
        ps->sizes.push_back(luax::checkfloat(L, 2 + i));
    }
    return 0;
}

int ps_getSizes(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    for (float size : ps->sizes)
    {
        lua_pushnumber(L, size);
    }
    return static_cast<int>(ps->sizes.size());
}

int ps_setSizeVariation(lua_State *L)
{
    float variation = luax::checkfloat(L, 2);
    if (variation < 0.0f || variation > 1.0f)
    {
        return luaL_error(L, "Size variation has to be between 0 and 1, inclusive");
    }
    check(L, 1)->sizeVariation = variation;
    return 0;
}

int ps_getSizeVariation(lua_State *L)
{
    lua_pushnumber(L, check(L, 1)->sizeVariation);
    return 1;
}

int ps_setRotation(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->rotationMin = luax::checkfloat(L, 2);
    ps->rotationMax = luax::optfloat(L, 3, ps->rotationMin);
    return 0;
}

int ps_getRotation(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushnumber(L, ps->rotationMin);
    lua_pushnumber(L, ps->rotationMax);
    return 2;
}

int ps_setSpin(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->spinMin = luax::checkfloat(L, 2);
    ps->spinMax = luax::optfloat(L, 3, ps->spinMin);
    ps->spinVariation = luax::optfloat(L, 4, ps->spinVariation);
    return 0;
}

int ps_getSpin(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushnumber(L, ps->spinMin);
    lua_pushnumber(L, ps->spinMax);
    lua_pushnumber(L, ps->spinVariation);
    return 3;
}

int ps_setSpinVariation(lua_State *L)
{
    float variation = luax::checkfloat(L, 2);
    if (variation < 0.0f || variation > 1.0f)
    {
        return luaL_error(L, "Spin variation has to be between 0 and 1, inclusive");
    }
    check(L, 1)->spinVariation = variation;
    return 0;
}

int ps_getSpinVariation(lua_State *L)
{
    lua_pushnumber(L, check(L, 1)->spinVariation);
    return 1;
}

int ps_setOffset(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    ps->offset = {luax::checkfloat(L, 2), luax::checkfloat(L, 3)};
    return 0;
}

int ps_getOffset(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushnumber(L, ps->offset.x);
    lua_pushnumber(L, ps->offset.y);
    return 2;
}

int ps_setRelativeRotation(lua_State *L)
{
    check(L, 1)->relativeRotation = luax::checkboolean(L, 2);
    return 0;
}

int ps_hasRelativeRotation(lua_State *L)
{
    lua_pushboolean(L, check(L, 1)->relativeRotation);
    return 1;
}

int ps_setInsertMode(lua_State *L)
{
    check(L, 1)->insertMode = static_cast<InsertMode>(luax::checkenum(L, 2, INSERT_NAMES, INSERT_VALUES, "insert mode"));
    return 0;
}

int ps_getInsertMode(lua_State *L)
{
    lua_pushstring(L, luax::enumname(INSERT_NAMES, INSERT_VALUES, static_cast<int>(check(L, 1)->insertMode)));
    return 1;
}

int ps_setEmissionArea(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    AreaDistribution area = static_cast<AreaDistribution>(luax::checkenum(L, 2, AREA_NAMES, AREA_VALUES, "distribution"));
    float dx = area == AreaDistribution::None ? 0.0f : luax::checkfloat(L, 3);
    float dy = area == AreaDistribution::None ? 0.0f : luax::checkfloat(L, 4);
    if (dx < 0.0f || dy < 0.0f)
    {
        return luaL_error(L, "Invalid emission area dimensions (must be >= 0)");
    }
    ps->area = area;
    ps->areaSpread = {dx, dy};
    ps->areaAngle = luax::optfloat(L, 5, 0.0f);
    ps->areaRelativeToCenter = luax::optboolean(L, 6, false);
    return 0;
}

int ps_getEmissionArea(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_pushstring(L, luax::enumname(AREA_NAMES, AREA_VALUES, static_cast<int>(ps->area)));
    lua_pushnumber(L, ps->areaSpread.x);
    lua_pushnumber(L, ps->areaSpread.y);
    lua_pushnumber(L, ps->areaAngle);
    lua_pushboolean(L, ps->areaRelativeToCenter);
    return 5;
}

float clamp01(float v)
{
    return std::min(1.0f, std::max(0.0f, v));
}

int ps_setColors(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    std::vector<RGBA> colors;
    int n = lua_gettop(L) - 1;
    if (n >= 1 && lua_istable(L, 2))
    {
        for (int i = 2; i <= lua_gettop(L); ++i)
        {
            luaL_checktype(L, i, LUA_TTABLE);
            float c[4] = {1, 1, 1, 1};
            for (int k = 0; k < 4; ++k)
            {
                lua_rawgeti(L, i, k + 1);
                if (lua_isnumber(L, -1))
                {
                    c[k] = static_cast<float>(lua_tonumber(L, -1));
                }
                else if (k < 3)
                {
                    lua_pop(L, 1);
                    return luaL_error(L, "Each color table needs at least three numbers");
                }
                lua_pop(L, 1);
            }
            colors.push_back({clamp01(c[0]), clamp01(c[1]), clamp01(c[2]), clamp01(c[3])});
        }
    }
    else
    {
        if (n < 4 || n % 4 != 0)
        {
            return luaL_error(L, "Colors must be given as groups of four numbers (r, g, b, a) or as tables");
        }
        for (int i = 0; i < n; i += 4)
        {
            colors.push_back({clamp01(luax::checkfloat(L, 2 + i)), clamp01(luax::checkfloat(L, 3 + i)),
                              clamp01(luax::checkfloat(L, 4 + i)), clamp01(luax::checkfloat(L, 5 + i))});
        }
    }
    if (colors.empty() || colors.size() > MAX_COLORS)
    {
        return luaL_error(L, "Invalid number of colors (1 to %d)", MAX_COLORS);
    }
    ps->colors = colors;
    return 0;
}

int ps_getColors(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    for (const RGBA &c : ps->colors)
    {
        lua_pushnumber(L, c.r);
        lua_pushnumber(L, c.g);
        lua_pushnumber(L, c.b);
        lua_pushnumber(L, c.a);
    }
    return static_cast<int>(ps->colors.size()) * 4;
}

int ps_setQuads(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    std::vector<QuadObj> quads;
    if (lua_istable(L, 2))
    {
        lua_Integer n = luaL_len(L, 2);
        for (lua_Integer i = 1; i <= n; ++i)
        {
            lua_rawgeti(L, 2, i);
            quads.push_back(*luax::checkobject<QuadObj>(L, -1, QUAD_TYPE));
            lua_pop(L, 1);
        }
    }
    else
    {
        for (int i = 2; i <= lua_gettop(L); ++i)
        {
            quads.push_back(*luax::checkobject<QuadObj>(L, i, QUAD_TYPE));
        }
    }
    ps->quads = quads;
    return 0;
}

int ps_getQuads(lua_State *L)
{
    ParticleSystemObj *ps = check(L, 1);
    lua_createtable(L, static_cast<int>(ps->quads.size()), 0);
    int n = 0;
    for (const QuadObj &q : ps->quads)
    {
        QuadObj *copy = luax::newobject<QuadObj>(L, QUAD_TYPE);
        *copy = q;
        lua_rawseti(L, -2, ++n);
    }
    return 1;
}

int ps_setTexture(lua_State *L)
{
    setTextureFrom(L, *check(L, 1), 2);
    return 0;
}

int ps_getTexture(lua_State *L)
{
    check(L, 1)->texture.push(L);
    return 1;
}

int ps_tostring(lua_State *L)
{
    lua_pushfstring(L, "ParticleSystem: %d particles", static_cast<int>(check(L, 1)->particles.size()));
    return 1;
}

const luaL_Reg PARTICLE_METHODS[] = {
    {"clone", ps_clone},
    {"emit", ps_emit},
    {"start", ps_start},
    {"stop", ps_stop},
    {"pause", ps_pause},
    {"reset", ps_reset},
    {"update", ps_update},
    {"isActive", ps_isActive},
    {"isPaused", ps_isPaused},
    {"isStopped", ps_isStopped},
    {"getCount", ps_getCount},
    {"setBufferSize", ps_setBufferSize},
    {"getBufferSize", ps_getBufferSize},
    {"setEmissionRate", ps_setEmissionRate},
    {"getEmissionRate", ps_getEmissionRate},
    {"setEmitterLifetime", ps_setEmitterLifetime},
    {"getEmitterLifetime", ps_getEmitterLifetime},
    {"setParticleLifetime", ps_setParticleLifetime},
    {"getParticleLifetime", ps_getParticleLifetime},
    {"setPosition", ps_setPosition},
    {"getPosition", ps_getPosition},
    {"moveTo", ps_moveTo},
    {"setDirection", ps_setDirection},
    {"getDirection", ps_getDirection},
    {"setSpread", ps_setSpread},
    {"getSpread", ps_getSpread},
    {"setSpeed", ps_setSpeed},
    {"getSpeed", ps_getSpeed},
    {"setLinearAcceleration", ps_setLinearAcceleration},
    {"getLinearAcceleration", ps_getLinearAcceleration},
    {"setLinearDamping", ps_setLinearDamping},
    {"getLinearDamping", ps_getLinearDamping},
    {"setRadialAcceleration", ps_setRadialAcceleration},
    {"getRadialAcceleration", ps_getRadialAcceleration},
    {"setTangentialAcceleration", ps_setTangentialAcceleration},
    {"getTangentialAcceleration", ps_getTangentialAcceleration},
    {"setSizes", ps_setSizes},
    {"getSizes", ps_getSizes},
    {"setSizeVariation", ps_setSizeVariation},
    {"getSizeVariation", ps_getSizeVariation},
    {"setRotation", ps_setRotation},
    {"getRotation", ps_getRotation},
    {"setSpin", ps_setSpin},
    {"getSpin", ps_getSpin},
    {"setSpinVariation", ps_setSpinVariation},
    {"getSpinVariation", ps_getSpinVariation},
    {"setOffset", ps_setOffset},
    {"getOffset", ps_getOffset},
    {"setRelativeRotation", ps_setRelativeRotation},
    {"hasRelativeRotation", ps_hasRelativeRotation},
    {"setInsertMode", ps_setInsertMode},
    {"getInsertMode", ps_getInsertMode},
    {"setEmissionArea", ps_setEmissionArea},
    {"getEmissionArea", ps_getEmissionArea},
    {"setColors", ps_setColors},
    {"getColors", ps_getColors},
    {"setQuads", ps_setQuads},
    {"getQuads", ps_getQuads},
    {"setTexture", ps_setTexture},
    {"getTexture", ps_getTexture},
    {"__tostring", ps_tostring},
    {nullptr, nullptr},
};

} // namespace

void drawParticleSystem(lua_State *L, ParticleSystemObj &ps, const Matrix &transform)
{
    if (ps.particles.empty())
    {
        return;
    }
    ps.texture.push(L);
    DrawSource base = checkDrawSource(L, lua_gettop(L));
    lua_pop(L, 1);

    Color tint = currentColor();
    for (const Particle &p : ps.particles)
    {
        DrawSource part = base;
        if (!ps.quads.empty())
        {
            const QuadObj &q = ps.quads[static_cast<size_t>(p.quad)];
            float sx = static_cast<float>(base.texture.width) / q.sw;
            float sy = static_cast<float>(base.texture.height) / q.sh;
            part.source = {q.x * sx, q.y * sy, q.w * sx, q.h * sy};
        }
        Color color = {static_cast<unsigned char>(p.color.r * tint.r + 0.5f), static_cast<unsigned char>(p.color.g * tint.g + 0.5f),
                       static_cast<unsigned char>(p.color.b * tint.b + 0.5f), static_cast<unsigned char>(p.color.a * tint.a + 0.5f)};
        Matrix local = makeTransform(p.position.x, p.position.y, p.angle, p.size, p.size, ps.offset.x, ps.offset.y, 0.0f, 0.0f);
        drawTextured(part, multiply(transform, local), color);
    }
}

void registerParticleType(lua_State *L)
{
    luax::newtype(L, PARTICLES_TYPE, PARTICLE_METHODS, ps_gc);
}

const luaL_Reg PARTICLE_FUNCS[] = {
    {"newParticleSystem", l_newParticleSystem},
    {nullptr, nullptr},
};

} // namespace graphics
} // namespace love
