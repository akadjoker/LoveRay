// physics.cpp - love.physics: World, Body, Shape, Fixture and Contact.
#include "physics_internal.hpp"

#include <box2d/b2_distance.h>
#include <rlgl.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace love
{
namespace physics
{

namespace
{

float g_meter = 30.0f;

} // namespace

float getMeter()
{
    return g_meter;
}

float scaleDown(float v)
{
    return v / g_meter;
}

float scaleUp(float v)
{
    return v * g_meter;
}

b2Vec2 scaleDown(const b2Vec2 &v)
{
    return b2Vec2(v.x / g_meter, v.y / g_meter);
}

b2Vec2 scaleUp(const b2Vec2 &v)
{
    return b2Vec2(v.x * g_meter, v.y * g_meter);
}

// ---------------------------------------------------------------------------
// Object lookup
// ---------------------------------------------------------------------------

WorldObj *checkWorld(lua_State *L, int idx)
{
    WorldObj *world = luax::checkobject<WorldObj>(L, idx, WORLD_TYPE);
    if (world->destroyed)
    {
        luaL_error(L, "Attempt to use destroyed World");
    }
    return world;
}

BodyObj *checkBody(lua_State *L, int idx)
{
    BodyObj *body = luax::checkobject<BodyObj>(L, idx, BODY_TYPE);
    if (body->body == nullptr)
    {
        luaL_error(L, "Attempt to use destroyed Body");
    }
    return body;
}

FixtureObj *checkFixture(lua_State *L, int idx)
{
    FixtureObj *fixture = luax::checkobject<FixtureObj>(L, idx, FIXTURE_TYPE);
    if (fixture->fixture == nullptr)
    {
        luaL_error(L, "Attempt to use destroyed Fixture");
    }
    return fixture;
}

ShapeObj *checkShape(lua_State *L, int idx)
{
    ShapeObj *shape = static_cast<ShapeObj *>(lua_touserdata(L, idx));
    const char *name = luax::typename_(L, idx);
    bool isShape = shape != nullptr && (std::strcmp(name, CIRCLE_TYPE) == 0 || std::strcmp(name, POLYGON_TYPE) == 0 ||
                                        std::strcmp(name, EDGE_TYPE) == 0 || std::strcmp(name, CHAIN_TYPE) == 0);
    if (!isShape)
    {
        luaL_error(L, "bad argument #%d (Shape expected, got %s)", idx, name);
    }
    if (shape->shape == nullptr)
    {
        luaL_error(L, "Attempt to use destroyed Shape");
    }
    return shape;
}

const char *shapeTypeName(b2Shape::Type type)
{
    switch (type)
    {
    case b2Shape::e_circle:
        return CIRCLE_TYPE;
    case b2Shape::e_polygon:
        return POLYGON_TYPE;
    case b2Shape::e_edge:
        return EDGE_TYPE;
    case b2Shape::e_chain:
        return CHAIN_TYPE;
    default:
        return POLYGON_TYPE;
    }
}

void pushBody(lua_State *L, b2Body *body, WorldObj *world)
{
    if (body == nullptr)
    {
        lua_pushnil(L);
        return;
    }
    if (luax::pushregistered(L, body))
    {
        return;
    }
    BodyObj *obj = luax::newobject<BodyObj>(L, BODY_TYPE);
    obj->body = body;
    obj->world = world;
    body->GetUserData().pointer = reinterpret_cast<uintptr_t>(world);
    luax::registerobject(L, body, -1, true);
}

WorldObj *worldOf(b2Body *body)
{
    return reinterpret_cast<WorldObj *>(body->GetUserData().pointer);
}

void pushFixture(lua_State *L, b2Fixture *fixture)
{
    if (fixture == nullptr)
    {
        lua_pushnil(L);
        return;
    }
    if (luax::pushregistered(L, fixture))
    {
        return;
    }
    FixtureObj *obj = luax::newobject<FixtureObj>(L, FIXTURE_TYPE);
    obj->fixture = fixture;
    obj->world = worldOf(fixture->GetBody());
    luax::registerobject(L, fixture, -1, true);
}

void pushFixtureShape(lua_State *L, b2Fixture *fixture)
{
    b2Shape *shape = fixture->GetShape();
    if (luax::pushregistered(L, shape))
    {
        return;
    }
    ShapeObj *obj = luax::newobject<ShapeObj>(L, shapeTypeName(shape->GetType()));
    obj->shape = shape;
    obj->owned = false;
    luax::registerobject(L, shape, -1, true);
}

void pushContact(lua_State *L, WorldObj *world, b2Contact *contact)
{
    ContactObj *obj = luax::newobject<ContactObj>(L, CONTACT_TYPE);
    obj->contact = contact;
    world->liveContacts.push_back(obj);
}

// ---------------------------------------------------------------------------
// Destruction
// ---------------------------------------------------------------------------

void destroyFixtureNow(lua_State *L, b2Fixture *fixture)
{
    if (luax::pushregistered(L, fixture))
    {
        static_cast<FixtureObj *>(lua_touserdata(L, -1))->fixture = nullptr;
        lua_pop(L, 1);
        luax::unregisterobject(L, fixture);
    }
    b2Shape *shape = fixture->GetShape();
    if (luax::pushregistered(L, shape))
    {
        static_cast<ShapeObj *>(lua_touserdata(L, -1))->shape = nullptr;
        lua_pop(L, 1);
        luax::unregisterobject(L, shape);
    }
    fixture->GetBody()->DestroyFixture(fixture);
}

void destroyJointNow(lua_State *L, WorldObj *world, b2Joint *joint)
{
    if (luax::pushregistered(L, joint))
    {
        static_cast<JointObj *>(lua_touserdata(L, -1))->joint = nullptr;
        lua_pop(L, 1);
        luax::unregisterobject(L, joint);
    }
    world->world->DestroyJoint(joint);
}

void destroyBodyNow(lua_State *L, WorldObj *world, b2Body *body)
{
    // Box2D destroys attached joints and fixtures; invalidate their handles first.
    for (b2JointEdge *edge = body->GetJointList(); edge != nullptr;)
    {
        b2Joint *joint = edge->joint;
        edge = edge->next;
        if (luax::pushregistered(L, joint))
        {
            static_cast<JointObj *>(lua_touserdata(L, -1))->joint = nullptr;
            lua_pop(L, 1);
            luax::unregisterobject(L, joint);
        }
    }
    for (b2Fixture *fixture = body->GetFixtureList(); fixture != nullptr; fixture = fixture->GetNext())
    {
        if (luax::pushregistered(L, fixture))
        {
            static_cast<FixtureObj *>(lua_touserdata(L, -1))->fixture = nullptr;
            lua_pop(L, 1);
            luax::unregisterobject(L, fixture);
        }
        b2Shape *shape = fixture->GetShape();
        if (luax::pushregistered(L, shape))
        {
            static_cast<ShapeObj *>(lua_touserdata(L, -1))->shape = nullptr;
            lua_pop(L, 1);
            luax::unregisterobject(L, shape);
        }
    }
    if (luax::pushregistered(L, body))
    {
        static_cast<BodyObj *>(lua_touserdata(L, -1))->body = nullptr;
        lua_pop(L, 1);
        luax::unregisterobject(L, body);
    }
    world->world->DestroyBody(body);
}

namespace
{

void flushDestroyQueue(lua_State *L, WorldObj *world)
{
    for (b2Fixture *fixture : world->destroyFixtures)
    {
        destroyFixtureNow(L, fixture);
    }
    world->destroyFixtures.clear();
    for (b2Joint *joint : world->destroyJoints)
    {
        destroyJointNow(L, world, joint);
    }
    world->destroyJoints.clear();
    for (b2Body *body : world->destroyBodies)
    {
        destroyBodyNow(L, world, body);
    }
    world->destroyBodies.clear();
}

// ---------------------------------------------------------------------------
// Callbacks into Lua from inside Box2D
//
// Errors cannot propagate through Box2D's C++ frames, so they are stored and
// raised once World:update returns.
// ---------------------------------------------------------------------------

void invalidateContacts(WorldObj *world)
{
    for (ContactObj *c : world->liveContacts)
    {
        c->contact = nullptr;
    }
    world->liveContacts.clear();
}

} // namespace

class ContactListener : public b2ContactListener
{
public:
    explicit ContactListener(WorldObj *world) : world_(world) {}

    void BeginContact(b2Contact *contact) override
    {
        dispatch(0, contact, nullptr);
    }

    void EndContact(b2Contact *contact) override
    {
        dispatch(1, contact, nullptr);
    }

    void PreSolve(b2Contact *contact, const b2Manifold *) override
    {
        dispatch(2, contact, nullptr);
    }

    void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) override
    {
        dispatch(3, contact, impulse);
    }

private:
    void dispatch(int index, b2Contact *contact, const b2ContactImpulse *impulse)
    {
        lua_State *L = world_->L;
        if (!world_->callbacks[index].valid() || !world_->pendingError.empty())
        {
            return;
        }
        int top = lua_gettop(L);
        world_->callbacks[index].push(L);
        pushFixture(L, contact->GetFixtureA());
        pushFixture(L, contact->GetFixtureB());
        pushContact(L, world_, contact);
        ContactObj *contactObj = static_cast<ContactObj *>(lua_touserdata(L, -1));
        int nargs = 3;
        if (impulse != nullptr)
        {
            for (int i = 0; i < impulse->count && i < 2; ++i)
            {
                lua_pushnumber(L, impulse->normalImpulses[i]);
                lua_pushnumber(L, impulse->tangentImpulses[i]);
                nargs += 2;
            }
        }
        if (lua_pcall(L, nargs, 0, 0) != LUA_OK)
        {
            world_->pendingError = lua_tostring(L, -1) ? lua_tostring(L, -1) : "unknown error";
        }
        contactObj->contact = nullptr;
        lua_settop(L, top);
    }

    WorldObj *world_;
};

class ContactFilter : public b2ContactFilter
{
public:
    explicit ContactFilter(WorldObj *world) : world_(world) {}

    bool ShouldCollide(b2Fixture *a, b2Fixture *b) override
    {
        if (!b2ContactFilter::ShouldCollide(a, b))
        {
            return false;
        }
        if (!world_->filter.valid() || !world_->pendingError.empty())
        {
            return true;
        }
        lua_State *L = world_->L;
        int top = lua_gettop(L);
        world_->filter.push(L);
        pushFixture(L, a);
        pushFixture(L, b);
        bool result = true;
        if (lua_pcall(L, 2, 1, 0) != LUA_OK)
        {
            world_->pendingError = lua_tostring(L, -1) ? lua_tostring(L, -1) : "unknown error";
        }
        else
        {
            result = lua_toboolean(L, -1) != 0;
        }
        lua_settop(L, top);
        return result;
    }

private:
    WorldObj *world_;
};

namespace
{

class DebugDraw : public b2Draw
{
public:
    void DrawPolygon(const b2Vec2 *vertices, int32 count, const b2Color &color) override
    {
        Color c = toColor(color);
        for (int32 i = 0; i < count; ++i)
        {
            b2Vec2 a = scaleUp(vertices[i]);
            b2Vec2 b = scaleUp(vertices[(i + 1) % count]);
            DrawLineV({a.x, a.y}, {b.x, b.y}, c);
        }
    }

    void DrawSolidPolygon(const b2Vec2 *vertices, int32 count, const b2Color &color) override
    {
        Color fill = toColor(color);
        fill.a = 128;
        rlBegin(RL_TRIANGLES);
        rlColor4ub(fill.r, fill.g, fill.b, fill.a);
        for (int32 i = 1; i + 1 < count; ++i)
        {
            b2Vec2 a = scaleUp(vertices[0]);
            b2Vec2 b = scaleUp(vertices[i]);
            b2Vec2 c = scaleUp(vertices[i + 1]);
            rlVertex2f(a.x, a.y);
            rlVertex2f(c.x, c.y);
            rlVertex2f(b.x, b.y);
        }
        rlEnd();
        DrawPolygon(vertices, count, color);
    }

    void DrawCircle(const b2Vec2 &center, float radius, const b2Color &color) override
    {
        b2Vec2 c = scaleUp(center);
        DrawCircleLines(static_cast<int>(c.x), static_cast<int>(c.y), scaleUp(radius), toColor(color));
    }

    void DrawSolidCircle(const b2Vec2 &center, float radius, const b2Vec2 &axis, const b2Color &color) override
    {
        b2Vec2 c = scaleUp(center);
        float r = scaleUp(radius);
        Color fill = toColor(color);
        fill.a = 128;
        DrawCircleV({c.x, c.y}, r, fill);
        DrawCircleLines(static_cast<int>(c.x), static_cast<int>(c.y), r, toColor(color));
        DrawLineV({c.x, c.y}, {c.x + axis.x * r, c.y + axis.y * r}, toColor(color));
    }

    void DrawSegment(const b2Vec2 &p1, const b2Vec2 &p2, const b2Color &color) override
    {
        b2Vec2 a = scaleUp(p1);
        b2Vec2 b = scaleUp(p2);
        DrawLineV({a.x, a.y}, {b.x, b.y}, toColor(color));
    }

    void DrawTransform(const b2Transform &xf) override
    {
        b2Vec2 p = scaleUp(xf.p);
        b2Vec2 px = scaleUp(xf.p + 0.4f * xf.q.GetXAxis());
        b2Vec2 py = scaleUp(xf.p + 0.4f * xf.q.GetYAxis());
        DrawLineV({p.x, p.y}, {px.x, px.y}, RED);
        DrawLineV({p.x, p.y}, {py.x, py.y}, GREEN);
    }

    void DrawPoint(const b2Vec2 &p, float size, const b2Color &color) override
    {
        b2Vec2 c = scaleUp(p);
        DrawRectangleV({c.x - size * 0.5f, c.y - size * 0.5f}, {size, size}, toColor(color));
    }

private:
    static Color toColor(const b2Color &c)
    {
        return Color{static_cast<unsigned char>(c.r * 255), static_cast<unsigned char>(c.g * 255),
                     static_cast<unsigned char>(c.b * 255), static_cast<unsigned char>(c.a * 255)};
    }
};

DebugDraw g_debugDraw;

// ---------------------------------------------------------------------------
// World
// ---------------------------------------------------------------------------

int world_gc(lua_State *L)
{
    WorldObj *world = static_cast<WorldObj *>(lua_touserdata(L, 1));
    if (!world->destroyed && world->world != nullptr)
    {
        for (b2Body *body = world->world->GetBodyList(); body != nullptr;)
        {
            b2Body *next = body->GetNext();
            destroyBodyNow(L, world, body);
            body = next;
        }
        delete world->world;
        world->world = nullptr;
        world->destroyed = true;
    }
    for (luax::Ref &ref : world->callbacks)
    {
        ref.clear(L);
    }
    world->filter.clear(L);
    delete world->listener;
    delete world->contactFilter;
    world->~WorldObj();
    return 0;
}

int l_newWorld(lua_State *L)
{
    float gx = luax::optfloat(L, 1, 0.0f);
    float gy = luax::optfloat(L, 2, 0.0f);
    bool sleep = luax::optboolean(L, 3, true);
    WorldObj *world = luax::newobject<WorldObj>(L, WORLD_TYPE);
    world->L = L;
    world->world = new b2World(b2Vec2(scaleDown(gx), scaleDown(gy)));
    world->world->SetAllowSleeping(sleep);
    world->listener = new ContactListener(world);
    world->contactFilter = new ContactFilter(world);
    world->world->SetContactListener(world->listener);
    world->world->SetContactFilter(world->contactFilter);
    world->world->SetDebugDraw(&g_debugDraw);
    b2BodyDef groundDef;
    world->groundBody = world->world->CreateBody(&groundDef);
    world->groundBody->GetUserData().pointer = reinterpret_cast<uintptr_t>(world);
    luax::registerobject(L, world->world, -1);
    return 1;
}

int world_update(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    float dt = luax::checkfloat(L, 2);
    int velocityIterations = luax::optint(L, 3, 8);
    int positionIterations = luax::optint(L, 4, 3);
    world->L = L;
    world->pendingError.clear();
    world->world->Step(dt, velocityIterations, positionIterations);
    invalidateContacts(world);
    flushDestroyQueue(L, world);
    if (!world->pendingError.empty())
    {
        std::string err = world->pendingError;
        world->pendingError.clear();
        return luaL_error(L, "%s", err.c_str());
    }
    return 0;
}

int world_destroy(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    if (world->world->IsLocked())
    {
        return luaL_error(L, "Cannot destroy the World during World:update");
    }
    for (b2Body *body = world->world->GetBodyList(); body != nullptr;)
    {
        b2Body *next = body->GetNext();
        destroyBodyNow(L, world, body);
        body = next;
    }
    delete world->world;
    world->world = nullptr;
    world->destroyed = true;
    return 0;
}

int world_isDestroyed(lua_State *L)
{
    WorldObj *world = luax::checkobject<WorldObj>(L, 1, WORLD_TYPE);
    lua_pushboolean(L, world->destroyed);
    return 1;
}

int world_setGravity(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    world->world->SetGravity(b2Vec2(scaleDown(luax::checkfloat(L, 2)), scaleDown(luax::checkfloat(L, 3))));
    return 0;
}

int world_getGravity(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    b2Vec2 g = scaleUp(world->world->GetGravity());
    lua_pushnumber(L, g.x);
    lua_pushnumber(L, g.y);
    return 2;
}

int world_setSleepingAllowed(lua_State *L)
{
    checkWorld(L, 1)->world->SetAllowSleeping(luax::checkboolean(L, 2));
    return 0;
}

int world_isSleepingAllowed(lua_State *L)
{
    lua_pushboolean(L, checkWorld(L, 1)->world->GetAllowSleeping());
    return 1;
}

int world_isLocked(lua_State *L)
{
    lua_pushboolean(L, checkWorld(L, 1)->world->IsLocked());
    return 1;
}

int world_translateOrigin(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    world->world->ShiftOrigin(b2Vec2(scaleDown(luax::checkfloat(L, 2)), scaleDown(luax::checkfloat(L, 3))));
    return 0;
}

int world_setCallbacks(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    for (int i = 0; i < 4; ++i)
    {
        int idx = 2 + i;
        if (lua_isnoneornil(L, idx))
        {
            world->callbacks[i].clear(L);
        }
        else
        {
            luaL_checktype(L, idx, LUA_TFUNCTION);
            world->callbacks[i].set(L, idx);
        }
    }
    return 0;
}

int world_getCallbacks(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    for (luax::Ref &ref : world->callbacks)
    {
        ref.push(L);
    }
    return 4;
}

int world_setContactFilter(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    if (lua_isnoneornil(L, 2))
    {
        world->filter.clear(L);
    }
    else
    {
        luaL_checktype(L, 2, LUA_TFUNCTION);
        world->filter.set(L, 2);
    }
    return 0;
}

int world_getContactFilter(lua_State *L)
{
    checkWorld(L, 1)->filter.push(L);
    return 1;
}

int world_getBodies(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    lua_newtable(L);
    int n = 0;
    for (b2Body *body = world->world->GetBodyList(); body != nullptr; body = body->GetNext())
    {
        if (body == world->groundBody)
        {
            continue;
        }
        pushBody(L, body, world);
        lua_rawseti(L, -2, ++n);
    }
    return 1;
}

int world_getBodyCount(lua_State *L)
{
    lua_pushinteger(L, checkWorld(L, 1)->world->GetBodyCount() - 1);
    return 1;
}

int world_getJoints(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    lua_newtable(L);
    int n = 0;
    for (b2Joint *joint = world->world->GetJointList(); joint != nullptr; joint = joint->GetNext())
    {
        pushJoint(L, joint, world);
        lua_rawseti(L, -2, ++n);
    }
    return 1;
}

int world_getJointCount(lua_State *L)
{
    lua_pushinteger(L, checkWorld(L, 1)->world->GetJointCount());
    return 1;
}

int world_getContacts(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    lua_newtable(L);
    int n = 0;
    for (b2Contact *contact = world->world->GetContactList(); contact != nullptr; contact = contact->GetNext())
    {
        pushContact(L, world, contact);
        lua_rawseti(L, -2, ++n);
    }
    return 1;
}

int world_getContactCount(lua_State *L)
{
    lua_pushinteger(L, checkWorld(L, 1)->world->GetContactCount());
    return 1;
}

class QueryCallback : public b2QueryCallback
{
public:
    QueryCallback(lua_State *L, int fnIndex) : L_(L), fnIndex_(fnIndex) {}

    bool ReportFixture(b2Fixture *fixture) override
    {
        if (!error.empty())
        {
            return false;
        }
        lua_pushvalue(L_, fnIndex_);
        pushFixture(L_, fixture);
        if (lua_pcall(L_, 1, 1, 0) != LUA_OK)
        {
            error = lua_tostring(L_, -1);
            lua_pop(L_, 1);
            return false;
        }
        bool cont = lua_toboolean(L_, -1) != 0;
        lua_pop(L_, 1);
        return cont;
    }

    std::string error;

private:
    lua_State *L_;
    int fnIndex_;
};

int world_queryBoundingBox(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    b2AABB aabb;
    aabb.lowerBound = b2Vec2(scaleDown(luax::checkfloat(L, 2)), scaleDown(luax::checkfloat(L, 3)));
    aabb.upperBound = b2Vec2(scaleDown(luax::checkfloat(L, 4)), scaleDown(luax::checkfloat(L, 5)));
    luaL_checktype(L, 6, LUA_TFUNCTION);
    QueryCallback callback(L, 6);
    world->world->QueryAABB(&callback, aabb);
    if (!callback.error.empty())
    {
        return luaL_error(L, "%s", callback.error.c_str());
    }
    return 0;
}

class RayCastCallback : public b2RayCastCallback
{
public:
    RayCastCallback(lua_State *L, int fnIndex) : L_(L), fnIndex_(fnIndex) {}

    float ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) override
    {
        if (!error.empty())
        {
            return 0.0f;
        }
        lua_pushvalue(L_, fnIndex_);
        pushFixture(L_, fixture);
        b2Vec2 p = scaleUp(point);
        lua_pushnumber(L_, p.x);
        lua_pushnumber(L_, p.y);
        lua_pushnumber(L_, normal.x);
        lua_pushnumber(L_, normal.y);
        lua_pushnumber(L_, fraction);
        if (lua_pcall(L_, 6, 1, 0) != LUA_OK)
        {
            error = lua_tostring(L_, -1);
            lua_pop(L_, 1);
            return 0.0f;
        }
        float control = static_cast<float>(luaL_optnumber(L_, -1, 1.0));
        lua_pop(L_, 1);
        return control;
    }

    std::string error;

private:
    lua_State *L_;
    int fnIndex_;
};

int world_rayCast(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    b2Vec2 p1(scaleDown(luax::checkfloat(L, 2)), scaleDown(luax::checkfloat(L, 3)));
    b2Vec2 p2(scaleDown(luax::checkfloat(L, 4)), scaleDown(luax::checkfloat(L, 5)));
    luaL_checktype(L, 6, LUA_TFUNCTION);
    RayCastCallback callback(L, 6);
    world->world->RayCast(&callback, p1, p2);
    if (!callback.error.empty())
    {
        return luaL_error(L, "%s", callback.error.c_str());
    }
    return 0;
}

// LoveRay extension: Box2D debug rendering with the current transform.
int world_draw(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    graphics::ensureFrame();
    uint32 flags = b2Draw::e_shapeBit | b2Draw::e_jointBit;
    if (luax::optboolean(L, 2, false))
    {
        flags |= b2Draw::e_aabbBit | b2Draw::e_centerOfMassBit;
    }
    g_debugDraw.SetFlags(flags);
    world->world->DebugDraw();
    return 0;
}

int world_tostring(lua_State *L)
{
    WorldObj *world = luax::checkobject<WorldObj>(L, 1, WORLD_TYPE);
    lua_pushfstring(L, "World: %d bodies", world->destroyed ? 0 : world->world->GetBodyCount() - 1);
    return 1;
}

const luaL_Reg WORLD_METHODS[] = {
    {"update", world_update},
    {"destroy", world_destroy},
    {"isDestroyed", world_isDestroyed},
    {"setGravity", world_setGravity},
    {"getGravity", world_getGravity},
    {"setSleepingAllowed", world_setSleepingAllowed},
    {"isSleepingAllowed", world_isSleepingAllowed},
    {"isLocked", world_isLocked},
    {"translateOrigin", world_translateOrigin},
    {"setCallbacks", world_setCallbacks},
    {"getCallbacks", world_getCallbacks},
    {"setContactFilter", world_setContactFilter},
    {"getContactFilter", world_getContactFilter},
    {"getBodies", world_getBodies},
    {"getBodyCount", world_getBodyCount},
    {"getJoints", world_getJoints},
    {"getJointCount", world_getJointCount},
    {"getContacts", world_getContacts},
    {"getContactCount", world_getContactCount},
    {"queryBoundingBox", world_queryBoundingBox},
    {"rayCast", world_rayCast},
    {"draw", world_draw},
    {"__tostring", world_tostring},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// Body
// ---------------------------------------------------------------------------

const char *const BODY_TYPE_NAMES[] = {"static", "dynamic", "kinematic"};
const int BODY_TYPE_VALUES[] = {b2_staticBody, b2_dynamicBody, b2_kinematicBody};

int l_newBody(lua_State *L)
{
    WorldObj *world = checkWorld(L, 1);
    b2BodyDef def;
    def.position = b2Vec2(scaleDown(luax::optfloat(L, 2, 0.0f)), scaleDown(luax::optfloat(L, 3, 0.0f)));
    def.type = static_cast<b2BodyType>(lua_isnoneornil(L, 4) ? b2_staticBody
                                                              : luax::checkenum(L, 4, BODY_TYPE_NAMES, BODY_TYPE_VALUES, "body type"));
    if (world->world->IsLocked())
    {
        return luaL_error(L, "Cannot create a Body during World:update");
    }
    b2Body *body = world->world->CreateBody(&def);
    pushBody(L, body, world);
    return 1;
}

b2Vec2 readPoint(lua_State *L, int idx)
{
    return b2Vec2(scaleDown(luax::checkfloat(L, idx)), scaleDown(luax::checkfloat(L, idx + 1)));
}

void pushPoint(lua_State *L, const b2Vec2 &p)
{
    b2Vec2 s = scaleUp(p);
    lua_pushnumber(L, s.x);
    lua_pushnumber(L, s.y);
}

int body_destroy(lua_State *L)
{
    BodyObj *body = checkBody(L, 1);
    if (body->world->world->IsLocked())
    {
        body->world->destroyBodies.push_back(body->body);
        body->body = nullptr;
        return 0;
    }
    destroyBodyNow(L, body->world, body->body);
    return 0;
}

int body_isDestroyed(lua_State *L)
{
    BodyObj *body = luax::checkobject<BodyObj>(L, 1, BODY_TYPE);
    lua_pushboolean(L, body->body == nullptr);
    return 1;
}

int body_getWorld(lua_State *L)
{
    BodyObj *body = checkBody(L, 1);
    if (!luax::pushregistered(L, body->world->world))
    {
        lua_pushnil(L);
    }
    return 1;
}

int body_getPosition(lua_State *L)
{
    pushPoint(L, checkBody(L, 1)->body->GetPosition());
    return 2;
}

int body_getX(lua_State *L)
{
    lua_pushnumber(L, scaleUp(checkBody(L, 1)->body->GetPosition().x));
    return 1;
}

int body_getY(lua_State *L)
{
    lua_pushnumber(L, scaleUp(checkBody(L, 1)->body->GetPosition().y));
    return 1;
}

int body_setPosition(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    body->SetTransform(readPoint(L, 2), body->GetAngle());
    return 0;
}

int body_setX(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    body->SetTransform(b2Vec2(scaleDown(luax::checkfloat(L, 2)), body->GetPosition().y), body->GetAngle());
    return 0;
}

int body_setY(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    body->SetTransform(b2Vec2(body->GetPosition().x, scaleDown(luax::checkfloat(L, 2))), body->GetAngle());
    return 0;
}

int body_getAngle(lua_State *L)
{
    lua_pushnumber(L, checkBody(L, 1)->body->GetAngle());
    return 1;
}

int body_setAngle(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    body->SetTransform(body->GetPosition(), luax::checkfloat(L, 2));
    return 0;
}

int body_getTransform(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    pushPoint(L, body->GetPosition());
    lua_pushnumber(L, body->GetAngle());
    return 3;
}

int body_setTransform(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    body->SetTransform(readPoint(L, 2), luax::checkfloat(L, 4));
    return 0;
}

int body_getLinearVelocity(lua_State *L)
{
    pushPoint(L, checkBody(L, 1)->body->GetLinearVelocity());
    return 2;
}

int body_setLinearVelocity(lua_State *L)
{
    checkBody(L, 1)->body->SetLinearVelocity(readPoint(L, 2));
    return 0;
}

int body_getAngularVelocity(lua_State *L)
{
    lua_pushnumber(L, checkBody(L, 1)->body->GetAngularVelocity());
    return 1;
}

int body_setAngularVelocity(lua_State *L)
{
    checkBody(L, 1)->body->SetAngularVelocity(luax::checkfloat(L, 2));
    return 0;
}

int body_getMass(lua_State *L)
{
    lua_pushnumber(L, checkBody(L, 1)->body->GetMass());
    return 1;
}

int body_setMass(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    b2MassData data = body->GetMassData();
    data.mass = luax::checkfloat(L, 2);
    body->SetMassData(&data);
    return 0;
}

int body_getInertia(lua_State *L)
{
    lua_pushnumber(L, scaleUp(scaleUp(checkBody(L, 1)->body->GetInertia())));
    return 1;
}

int body_setInertia(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    b2MassData data = body->GetMassData();
    data.I = scaleDown(scaleDown(luax::checkfloat(L, 2)));
    body->SetMassData(&data);
    return 0;
}

int body_getMassData(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    b2MassData data = body->GetMassData();
    pushPoint(L, data.center);
    lua_pushnumber(L, data.mass);
    lua_pushnumber(L, scaleUp(scaleUp(data.I)));
    return 4;
}

int body_setMassData(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    b2MassData data;
    data.center = readPoint(L, 2);
    data.mass = luax::checkfloat(L, 4);
    data.I = scaleDown(scaleDown(luax::checkfloat(L, 5)));
    body->SetMassData(&data);
    return 0;
}

int body_resetMassData(lua_State *L)
{
    checkBody(L, 1)->body->ResetMassData();
    return 0;
}

int body_getLocalCenter(lua_State *L)
{
    pushPoint(L, checkBody(L, 1)->body->GetLocalCenter());
    return 2;
}

int body_getWorldCenter(lua_State *L)
{
    pushPoint(L, checkBody(L, 1)->body->GetWorldCenter());
    return 2;
}

int body_applyForce(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    b2Vec2 force = readPoint(L, 2);
    if (lua_gettop(L) >= 5)
    {
        body->ApplyForce(force, readPoint(L, 4), true);
    }
    else
    {
        body->ApplyForceToCenter(force, true);
    }
    return 0;
}

int body_applyTorque(lua_State *L)
{
    checkBody(L, 1)->body->ApplyTorque(scaleDown(scaleDown(luax::checkfloat(L, 2))), true);
    return 0;
}

int body_applyLinearImpulse(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    b2Vec2 impulse = readPoint(L, 2);
    if (lua_gettop(L) >= 5)
    {
        body->ApplyLinearImpulse(impulse, readPoint(L, 4), true);
    }
    else
    {
        body->ApplyLinearImpulseToCenter(impulse, true);
    }
    return 0;
}

int body_applyAngularImpulse(lua_State *L)
{
    checkBody(L, 1)->body->ApplyAngularImpulse(scaleDown(scaleDown(luax::checkfloat(L, 2))), true);
    return 0;
}

int body_getLinearDamping(lua_State *L)
{
    lua_pushnumber(L, checkBody(L, 1)->body->GetLinearDamping());
    return 1;
}

int body_setLinearDamping(lua_State *L)
{
    checkBody(L, 1)->body->SetLinearDamping(luax::checkfloat(L, 2));
    return 0;
}

int body_getAngularDamping(lua_State *L)
{
    lua_pushnumber(L, checkBody(L, 1)->body->GetAngularDamping());
    return 1;
}

int body_setAngularDamping(lua_State *L)
{
    checkBody(L, 1)->body->SetAngularDamping(luax::checkfloat(L, 2));
    return 0;
}

int body_getGravityScale(lua_State *L)
{
    lua_pushnumber(L, checkBody(L, 1)->body->GetGravityScale());
    return 1;
}

int body_setGravityScale(lua_State *L)
{
    checkBody(L, 1)->body->SetGravityScale(luax::checkfloat(L, 2));
    return 0;
}

int body_getType(lua_State *L)
{
    lua_pushstring(L, luax::enumname(BODY_TYPE_NAMES, BODY_TYPE_VALUES, checkBody(L, 1)->body->GetType()));
    return 1;
}

int body_setType(lua_State *L)
{
    BodyObj *body = checkBody(L, 1);
    int type = luax::checkenum(L, 2, BODY_TYPE_NAMES, BODY_TYPE_VALUES, "body type");
    if (body->world->world->IsLocked())
    {
        return luaL_error(L, "Cannot change the body type during World:update");
    }
    body->body->SetType(static_cast<b2BodyType>(type));
    return 0;
}

int body_isActive(lua_State *L)
{
    lua_pushboolean(L, checkBody(L, 1)->body->IsEnabled());
    return 1;
}

int body_setActive(lua_State *L)
{
    BodyObj *body = checkBody(L, 1);
    if (body->world->world->IsLocked())
    {
        return luaL_error(L, "Cannot change the active state during World:update");
    }
    body->body->SetEnabled(luax::checkboolean(L, 2));
    return 0;
}

int body_isAwake(lua_State *L)
{
    lua_pushboolean(L, checkBody(L, 1)->body->IsAwake());
    return 1;
}

int body_setAwake(lua_State *L)
{
    checkBody(L, 1)->body->SetAwake(luax::checkboolean(L, 2));
    return 0;
}

int body_isBullet(lua_State *L)
{
    lua_pushboolean(L, checkBody(L, 1)->body->IsBullet());
    return 1;
}

int body_setBullet(lua_State *L)
{
    checkBody(L, 1)->body->SetBullet(luax::checkboolean(L, 2));
    return 0;
}

int body_isFixedRotation(lua_State *L)
{
    lua_pushboolean(L, checkBody(L, 1)->body->IsFixedRotation());
    return 1;
}

int body_setFixedRotation(lua_State *L)
{
    checkBody(L, 1)->body->SetFixedRotation(luax::checkboolean(L, 2));
    return 0;
}

int body_isSleepingAllowed(lua_State *L)
{
    lua_pushboolean(L, checkBody(L, 1)->body->IsSleepingAllowed());
    return 1;
}

int body_setSleepingAllowed(lua_State *L)
{
    checkBody(L, 1)->body->SetSleepingAllowed(luax::checkboolean(L, 2));
    return 0;
}

int body_isTouching(lua_State *L)
{
    b2Body *a = checkBody(L, 1)->body;
    b2Body *b = checkBody(L, 2)->body;
    for (b2ContactEdge *edge = a->GetContactList(); edge != nullptr; edge = edge->next)
    {
        if (edge->other == b && edge->contact->IsTouching())
        {
            lua_pushboolean(L, 1);
            return 1;
        }
    }
    lua_pushboolean(L, 0);
    return 1;
}

int body_getFixtures(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    lua_newtable(L);
    int n = 0;
    for (b2Fixture *fixture = body->GetFixtureList(); fixture != nullptr; fixture = fixture->GetNext())
    {
        pushFixture(L, fixture);
        lua_rawseti(L, -2, ++n);
    }
    return 1;
}

int body_getJoints(lua_State *L)
{
    BodyObj *body = checkBody(L, 1);
    lua_newtable(L);
    int n = 0;
    for (b2JointEdge *edge = body->body->GetJointList(); edge != nullptr; edge = edge->next)
    {
        pushJoint(L, edge->joint, body->world);
        lua_rawseti(L, -2, ++n);
    }
    return 1;
}

int body_getContacts(lua_State *L)
{
    BodyObj *body = checkBody(L, 1);
    lua_newtable(L);
    int n = 0;
    for (b2ContactEdge *edge = body->body->GetContactList(); edge != nullptr; edge = edge->next)
    {
        pushContact(L, body->world, edge->contact);
        lua_rawseti(L, -2, ++n);
    }
    return 1;
}

int body_getWorldPoint(lua_State *L)
{
    pushPoint(L, checkBody(L, 1)->body->GetWorldPoint(readPoint(L, 2)));
    return 2;
}

int body_getWorldVector(lua_State *L)
{
    pushPoint(L, checkBody(L, 1)->body->GetWorldVector(readPoint(L, 2)));
    return 2;
}

int body_getWorldPoints(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    int n = lua_gettop(L) - 1;
    if (n % 2 != 0)
    {
        return luaL_error(L, "Number of vertex components must be a multiple of two");
    }
    for (int i = 0; i < n; i += 2)
    {
        b2Vec2 p = body->GetWorldPoint(readPoint(L, 2 + i));
        pushPoint(L, p);
    }
    return n;
}

int body_getLocalPoint(lua_State *L)
{
    pushPoint(L, checkBody(L, 1)->body->GetLocalPoint(readPoint(L, 2)));
    return 2;
}

int body_getLocalVector(lua_State *L)
{
    pushPoint(L, checkBody(L, 1)->body->GetLocalVector(readPoint(L, 2)));
    return 2;
}

int body_getLocalPoints(lua_State *L)
{
    b2Body *body = checkBody(L, 1)->body;
    int n = lua_gettop(L) - 1;
    if (n % 2 != 0)
    {
        return luaL_error(L, "Number of vertex components must be a multiple of two");
    }
    for (int i = 0; i < n; i += 2)
    {
        pushPoint(L, body->GetLocalPoint(readPoint(L, 2 + i)));
    }
    return n;
}

int body_getLinearVelocityFromWorldPoint(lua_State *L)
{
    pushPoint(L, checkBody(L, 1)->body->GetLinearVelocityFromWorldPoint(readPoint(L, 2)));
    return 2;
}

int body_getLinearVelocityFromLocalPoint(lua_State *L)
{
    pushPoint(L, checkBody(L, 1)->body->GetLinearVelocityFromLocalPoint(readPoint(L, 2)));
    return 2;
}

int body_setUserData(lua_State *L)
{
    checkBody(L, 1);
    lua_settop(L, 2);
    lua_setiuservalue(L, 1, 1);
    return 0;
}

int body_getUserData(lua_State *L)
{
    checkBody(L, 1);
    lua_getiuservalue(L, 1, 1);
    return 1;
}

int body_tostring(lua_State *L)
{
    BodyObj *body = luax::checkobject<BodyObj>(L, 1, BODY_TYPE);
    if (body->body == nullptr)
    {
        lua_pushstring(L, "Body: destroyed");
        return 1;
    }
    b2Vec2 p = scaleUp(body->body->GetPosition());
    lua_pushfstring(L, "Body: %s at %f,%f", luax::enumname(BODY_TYPE_NAMES, BODY_TYPE_VALUES, body->body->GetType()), p.x, p.y);
    return 1;
}

const luaL_Reg BODY_METHODS[] = {
    {"destroy", body_destroy},
    {"isDestroyed", body_isDestroyed},
    {"getWorld", body_getWorld},
    {"getPosition", body_getPosition},
    {"getX", body_getX},
    {"getY", body_getY},
    {"setPosition", body_setPosition},
    {"setX", body_setX},
    {"setY", body_setY},
    {"getAngle", body_getAngle},
    {"setAngle", body_setAngle},
    {"getTransform", body_getTransform},
    {"setTransform", body_setTransform},
    {"getLinearVelocity", body_getLinearVelocity},
    {"setLinearVelocity", body_setLinearVelocity},
    {"getAngularVelocity", body_getAngularVelocity},
    {"setAngularVelocity", body_setAngularVelocity},
    {"getMass", body_getMass},
    {"setMass", body_setMass},
    {"getInertia", body_getInertia},
    {"setInertia", body_setInertia},
    {"getMassData", body_getMassData},
    {"setMassData", body_setMassData},
    {"resetMassData", body_resetMassData},
    {"getLocalCenter", body_getLocalCenter},
    {"getWorldCenter", body_getWorldCenter},
    {"applyForce", body_applyForce},
    {"applyTorque", body_applyTorque},
    {"applyLinearImpulse", body_applyLinearImpulse},
    {"applyAngularImpulse", body_applyAngularImpulse},
    {"getLinearDamping", body_getLinearDamping},
    {"setLinearDamping", body_setLinearDamping},
    {"getAngularDamping", body_getAngularDamping},
    {"setAngularDamping", body_setAngularDamping},
    {"getGravityScale", body_getGravityScale},
    {"setGravityScale", body_setGravityScale},
    {"getType", body_getType},
    {"setType", body_setType},
    {"isActive", body_isActive},
    {"setActive", body_setActive},
    {"isAwake", body_isAwake},
    {"setAwake", body_setAwake},
    {"isBullet", body_isBullet},
    {"setBullet", body_setBullet},
    {"isFixedRotation", body_isFixedRotation},
    {"setFixedRotation", body_setFixedRotation},
    {"isSleepingAllowed", body_isSleepingAllowed},
    {"setSleepingAllowed", body_setSleepingAllowed},
    {"isTouching", body_isTouching},
    {"getFixtures", body_getFixtures},
    {"getJoints", body_getJoints},
    {"getContacts", body_getContacts},
    {"getWorldPoint", body_getWorldPoint},
    {"getWorldVector", body_getWorldVector},
    {"getWorldPoints", body_getWorldPoints},
    {"getLocalPoint", body_getLocalPoint},
    {"getLocalVector", body_getLocalVector},
    {"getLocalPoints", body_getLocalPoints},
    {"getLinearVelocityFromWorldPoint", body_getLinearVelocityFromWorldPoint},
    {"getLinearVelocityFromLocalPoint", body_getLinearVelocityFromLocalPoint},
    {"setUserData", body_setUserData},
    {"getUserData", body_getUserData},
    {"__tostring", body_tostring},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// Shapes
// ---------------------------------------------------------------------------

int shape_gc(lua_State *L)
{
    ShapeObj *shape = static_cast<ShapeObj *>(lua_touserdata(L, 1));
    if (shape->owned && shape->shape != nullptr)
    {
        delete shape->shape;
    }
    shape->shape = nullptr;
    return 0;
}

int l_newCircleShape(lua_State *L)
{
    b2CircleShape *circle = new b2CircleShape();
    if (lua_gettop(L) >= 3)
    {
        circle->m_p = readPoint(L, 1);
        circle->m_radius = scaleDown(luax::checkfloat(L, 3));
    }
    else
    {
        circle->m_radius = scaleDown(luax::checkfloat(L, 1));
    }
    ShapeObj *shape = luax::newobject<ShapeObj>(L, CIRCLE_TYPE);
    shape->shape = circle;
    shape->owned = true;
    return 1;
}

int l_newRectangleShape(lua_State *L)
{
    b2PolygonShape *polygon = new b2PolygonShape();
    if (lua_gettop(L) >= 4)
    {
        b2Vec2 center = readPoint(L, 1);
        float hw = scaleDown(luax::checkfloat(L, 3)) * 0.5f;
        float hh = scaleDown(luax::checkfloat(L, 4)) * 0.5f;
        polygon->SetAsBox(hw, hh, center, luax::optfloat(L, 5, 0.0f));
    }
    else
    {
        float hw = scaleDown(luax::checkfloat(L, 1)) * 0.5f;
        float hh = scaleDown(luax::checkfloat(L, 2)) * 0.5f;
        polygon->SetAsBox(hw, hh);
    }
    ShapeObj *shape = luax::newobject<ShapeObj>(L, POLYGON_TYPE);
    shape->shape = polygon;
    shape->owned = true;
    return 1;
}

void readVertexList(lua_State *L, int idx, std::vector<b2Vec2> &out)
{
    out.clear();
    if (lua_istable(L, idx))
    {
        lua_Integer n = luaL_len(L, idx);
        if (n % 2 != 0)
        {
            luaL_error(L, "Number of vertex components must be a multiple of two");
        }
        for (lua_Integer i = 1; i <= n; i += 2)
        {
            lua_rawgeti(L, idx, i);
            lua_rawgeti(L, idx, i + 1);
            out.push_back(b2Vec2(scaleDown(luax::checkfloat(L, -2)), scaleDown(luax::checkfloat(L, -1))));
            lua_pop(L, 2);
        }
        return;
    }
    int n = lua_gettop(L) - idx + 1;
    if (n % 2 != 0)
    {
        luaL_error(L, "Number of vertex components must be a multiple of two");
    }
    for (int i = idx; i < idx + n; i += 2)
    {
        out.push_back(readPoint(L, i));
    }
}

int l_newPolygonShape(lua_State *L)
{
    std::vector<b2Vec2> vertices;
    readVertexList(L, 1, vertices);
    if (vertices.size() < 3)
    {
        return luaL_error(L, "Need at least 3 vertices to create a PolygonShape");
    }
    if (vertices.size() > b2_maxPolygonVertices)
    {
        return luaL_error(L, "Too many vertices (maximum %d)", b2_maxPolygonVertices);
    }
    b2PolygonShape *polygon = new b2PolygonShape();
    if (!polygon->Set(vertices.data(), static_cast<int32>(vertices.size())))
    {
        delete polygon;
        return luaL_error(L, "Polygon is degenerate");
    }
    ShapeObj *shape = luax::newobject<ShapeObj>(L, POLYGON_TYPE);
    shape->shape = polygon;
    shape->owned = true;
    return 1;
}

int l_newEdgeShape(lua_State *L)
{
    b2EdgeShape *edge = new b2EdgeShape();
    edge->SetTwoSided(readPoint(L, 1), readPoint(L, 3));
    ShapeObj *shape = luax::newobject<ShapeObj>(L, EDGE_TYPE);
    shape->shape = edge;
    shape->owned = true;
    return 1;
}

int l_newChainShape(lua_State *L)
{
    bool loop = luax::checkboolean(L, 1);
    std::vector<b2Vec2> vertices;
    readVertexList(L, 2, vertices);
    if (vertices.size() < 2)
    {
        return luaL_error(L, "Need at least 2 vertices to create a ChainShape");
    }
    b2ChainShape *chain = new b2ChainShape();
    if (loop)
    {
        chain->CreateLoop(vertices.data(), static_cast<int32>(vertices.size()));
    }
    else
    {
        b2Vec2 prev = vertices.front() + (vertices.front() - vertices[1]);
        b2Vec2 next = vertices.back() + (vertices.back() - vertices[vertices.size() - 2]);
        chain->CreateChain(vertices.data(), static_cast<int32>(vertices.size()), prev, next);
    }
    ShapeObj *shape = luax::newobject<ShapeObj>(L, CHAIN_TYPE);
    shape->shape = chain;
    shape->owned = true;
    return 1;
}

int shape_getType(lua_State *L)
{
    ShapeObj *shape = checkShape(L, 1);
    switch (shape->shape->GetType())
    {
    case b2Shape::e_circle:
        lua_pushstring(L, "circle");
        break;
    case b2Shape::e_polygon:
        lua_pushstring(L, "polygon");
        break;
    case b2Shape::e_edge:
        lua_pushstring(L, "edge");
        break;
    case b2Shape::e_chain:
        lua_pushstring(L, "chain");
        break;
    default:
        lua_pushnil(L);
        break;
    }
    return 1;
}

int shape_getRadius(lua_State *L)
{
    lua_pushnumber(L, scaleUp(checkShape(L, 1)->shape->m_radius));
    return 1;
}

int shape_getChildCount(lua_State *L)
{
    lua_pushinteger(L, checkShape(L, 1)->shape->GetChildCount());
    return 1;
}

int shape_computeAABB(lua_State *L)
{
    ShapeObj *shape = checkShape(L, 1);
    b2Transform xf(readPoint(L, 2), b2Rot(luax::checkfloat(L, 4)));
    int child = luax::optint(L, 5, 1) - 1;
    b2AABB aabb;
    shape->shape->ComputeAABB(&aabb, xf, child);
    pushPoint(L, aabb.lowerBound);
    pushPoint(L, aabb.upperBound);
    return 4;
}

int shape_computeMass(lua_State *L)
{
    ShapeObj *shape = checkShape(L, 1);
    b2MassData data;
    shape->shape->ComputeMass(&data, luax::checkfloat(L, 2));
    pushPoint(L, data.center);
    lua_pushnumber(L, data.mass);
    lua_pushnumber(L, scaleUp(scaleUp(data.I)));
    return 4;
}

int shape_testPoint(lua_State *L)
{
    ShapeObj *shape = checkShape(L, 1);
    b2Transform xf(readPoint(L, 2), b2Rot(luax::checkfloat(L, 4)));
    lua_pushboolean(L, shape->shape->TestPoint(xf, readPoint(L, 5)));
    return 1;
}

int shape_rayCast(lua_State *L)
{
    ShapeObj *shape = checkShape(L, 1);
    b2RayCastInput input;
    input.p1 = readPoint(L, 2);
    input.p2 = input.p1 + luax::checkfloat(L, 6) * (readPoint(L, 4) - input.p1);
    input.maxFraction = 1.0f;
    b2Transform xf(readPoint(L, 7), b2Rot(luax::checkfloat(L, 9)));
    int child = luax::optint(L, 10, 1) - 1;
    b2RayCastOutput output;
    if (!shape->shape->RayCast(&output, input, xf, child))
    {
        return 0;
    }
    lua_pushnumber(L, output.normal.x);
    lua_pushnumber(L, output.normal.y);
    lua_pushnumber(L, output.fraction);
    return 3;
}

int circle_getPoint(lua_State *L)
{
    pushPoint(L, static_cast<b2CircleShape *>(checkShape(L, 1)->shape)->m_p);
    return 2;
}

int circle_setPoint(lua_State *L)
{
    static_cast<b2CircleShape *>(checkShape(L, 1)->shape)->m_p = readPoint(L, 2);
    return 0;
}

int circle_setRadius(lua_State *L)
{
    checkShape(L, 1)->shape->m_radius = scaleDown(luax::checkfloat(L, 2));
    return 0;
}

int polygon_getPoints(lua_State *L)
{
    b2PolygonShape *polygon = static_cast<b2PolygonShape *>(checkShape(L, 1)->shape);
    for (int32 i = 0; i < polygon->m_count; ++i)
    {
        pushPoint(L, polygon->m_vertices[i]);
    }
    return polygon->m_count * 2;
}

int polygon_validate(lua_State *L)
{
    lua_pushboolean(L, static_cast<b2PolygonShape *>(checkShape(L, 1)->shape)->Validate());
    return 1;
}

int edge_getPoints(lua_State *L)
{
    b2EdgeShape *edge = static_cast<b2EdgeShape *>(checkShape(L, 1)->shape);
    pushPoint(L, edge->m_vertex1);
    pushPoint(L, edge->m_vertex2);
    return 4;
}

int edge_setNextVertex(lua_State *L)
{
    b2EdgeShape *edge = static_cast<b2EdgeShape *>(checkShape(L, 1)->shape);
    edge->m_vertex3 = readPoint(L, 2);
    edge->m_oneSided = true;
    return 0;
}

int edge_setPreviousVertex(lua_State *L)
{
    b2EdgeShape *edge = static_cast<b2EdgeShape *>(checkShape(L, 1)->shape);
    edge->m_vertex0 = readPoint(L, 2);
    edge->m_oneSided = true;
    return 0;
}

int edge_getNextVertex(lua_State *L)
{
    b2EdgeShape *edge = static_cast<b2EdgeShape *>(checkShape(L, 1)->shape);
    if (!edge->m_oneSided)
    {
        return 0;
    }
    pushPoint(L, edge->m_vertex3);
    return 2;
}

int edge_getPreviousVertex(lua_State *L)
{
    b2EdgeShape *edge = static_cast<b2EdgeShape *>(checkShape(L, 1)->shape);
    if (!edge->m_oneSided)
    {
        return 0;
    }
    pushPoint(L, edge->m_vertex0);
    return 2;
}

int chain_getPoints(lua_State *L)
{
    b2ChainShape *chain = static_cast<b2ChainShape *>(checkShape(L, 1)->shape);
    for (int32 i = 0; i < chain->m_count; ++i)
    {
        pushPoint(L, chain->m_vertices[i]);
    }
    return chain->m_count * 2;
}

int chain_getPoint(lua_State *L)
{
    b2ChainShape *chain = static_cast<b2ChainShape *>(checkShape(L, 1)->shape);
    int index = static_cast<int>(luaL_checkinteger(L, 2)) - 1;
    if (index < 0 || index >= chain->m_count)
    {
        return luaL_error(L, "Invalid vertex index");
    }
    pushPoint(L, chain->m_vertices[index]);
    return 2;
}

int chain_getVertexCount(lua_State *L)
{
    lua_pushinteger(L, static_cast<b2ChainShape *>(checkShape(L, 1)->shape)->m_count);
    return 1;
}

int chain_getChildEdge(lua_State *L)
{
    b2ChainShape *chain = static_cast<b2ChainShape *>(checkShape(L, 1)->shape);
    int index = static_cast<int>(luaL_checkinteger(L, 2)) - 1;
    if (index < 0 || index >= chain->GetChildCount())
    {
        return luaL_error(L, "Invalid child index");
    }
    b2EdgeShape *edge = new b2EdgeShape();
    chain->GetChildEdge(edge, index);
    ShapeObj *shape = luax::newobject<ShapeObj>(L, EDGE_TYPE);
    shape->shape = edge;
    shape->owned = true;
    return 1;
}

int chain_setNextVertex(lua_State *L)
{
    static_cast<b2ChainShape *>(checkShape(L, 1)->shape)->m_nextVertex = readPoint(L, 2);
    return 0;
}

int chain_setPreviousVertex(lua_State *L)
{
    static_cast<b2ChainShape *>(checkShape(L, 1)->shape)->m_prevVertex = readPoint(L, 2);
    return 0;
}

int chain_getNextVertex(lua_State *L)
{
    pushPoint(L, static_cast<b2ChainShape *>(checkShape(L, 1)->shape)->m_nextVertex);
    return 2;
}

int chain_getPreviousVertex(lua_State *L)
{
    pushPoint(L, static_cast<b2ChainShape *>(checkShape(L, 1)->shape)->m_prevVertex);
    return 2;
}

int shape_tostring(lua_State *L)
{
    lua_pushfstring(L, "%s", luax::typename_(L, 1));
    return 1;
}

#define SHAPE_COMMON_METHODS                                                                                     \
    {"getType", shape_getType}, {"getRadius", shape_getRadius}, {"getChildCount", shape_getChildCount},         \
        {"computeAABB", shape_computeAABB}, {"computeMass", shape_computeMass}, {"testPoint", shape_testPoint}, \
        {"rayCast", shape_rayCast}, {"__tostring", shape_tostring}

const luaL_Reg CIRCLE_METHODS[] = {
    SHAPE_COMMON_METHODS,
    {"getPoint", circle_getPoint},
    {"setPoint", circle_setPoint},
    {"setRadius", circle_setRadius},
    {nullptr, nullptr},
};

const luaL_Reg POLYGON_METHODS[] = {
    SHAPE_COMMON_METHODS,
    {"getPoints", polygon_getPoints},
    {"validate", polygon_validate},
    {nullptr, nullptr},
};

const luaL_Reg EDGE_METHODS[] = {
    SHAPE_COMMON_METHODS,
    {"getPoints", edge_getPoints},
    {"setNextVertex", edge_setNextVertex},
    {"setPreviousVertex", edge_setPreviousVertex},
    {"getNextVertex", edge_getNextVertex},
    {"getPreviousVertex", edge_getPreviousVertex},
    {nullptr, nullptr},
};

const luaL_Reg CHAIN_METHODS[] = {
    SHAPE_COMMON_METHODS,
    {"getPoints", chain_getPoints},
    {"getPoint", chain_getPoint},
    {"getVertexCount", chain_getVertexCount},
    {"getChildEdge", chain_getChildEdge},
    {"setNextVertex", chain_setNextVertex},
    {"setPreviousVertex", chain_setPreviousVertex},
    {"getNextVertex", chain_getNextVertex},
    {"getPreviousVertex", chain_getPreviousVertex},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

int l_newFixture(lua_State *L)
{
    BodyObj *body = checkBody(L, 1);
    ShapeObj *shape = checkShape(L, 2);
    float density = luax::optfloat(L, 3, 1.0f);
    if (body->world->world->IsLocked())
    {
        return luaL_error(L, "Cannot create a Fixture during World:update");
    }
    b2FixtureDef def;
    def.shape = shape->shape;
    def.density = density;
    b2Fixture *fixture = body->body->CreateFixture(&def);
    pushFixture(L, fixture);
    return 1;
}

int fixture_destroy(lua_State *L)
{
    FixtureObj *fixture = checkFixture(L, 1);
    if (fixture->world->world->IsLocked())
    {
        fixture->world->destroyFixtures.push_back(fixture->fixture);
        fixture->fixture = nullptr;
        return 0;
    }
    destroyFixtureNow(L, fixture->fixture);
    return 0;
}

int fixture_isDestroyed(lua_State *L)
{
    FixtureObj *fixture = luax::checkobject<FixtureObj>(L, 1, FIXTURE_TYPE);
    lua_pushboolean(L, fixture->fixture == nullptr);
    return 1;
}

int fixture_getBody(lua_State *L)
{
    FixtureObj *fixture = checkFixture(L, 1);
    pushBody(L, fixture->fixture->GetBody(), fixture->world);
    return 1;
}

int fixture_getShape(lua_State *L)
{
    pushFixtureShape(L, checkFixture(L, 1)->fixture);
    return 1;
}

int fixture_getType(lua_State *L)
{
    FixtureObj *fixture = checkFixture(L, 1);
    switch (fixture->fixture->GetType())
    {
    case b2Shape::e_circle:
        lua_pushstring(L, "circle");
        break;
    case b2Shape::e_polygon:
        lua_pushstring(L, "polygon");
        break;
    case b2Shape::e_edge:
        lua_pushstring(L, "edge");
        break;
    default:
        lua_pushstring(L, "chain");
        break;
    }
    return 1;
}

int fixture_getDensity(lua_State *L)
{
    lua_pushnumber(L, checkFixture(L, 1)->fixture->GetDensity());
    return 1;
}

int fixture_setDensity(lua_State *L)
{
    FixtureObj *fixture = checkFixture(L, 1);
    fixture->fixture->SetDensity(luax::checkfloat(L, 2));
    return 0;
}

int fixture_getFriction(lua_State *L)
{
    lua_pushnumber(L, checkFixture(L, 1)->fixture->GetFriction());
    return 1;
}

int fixture_setFriction(lua_State *L)
{
    checkFixture(L, 1)->fixture->SetFriction(luax::checkfloat(L, 2));
    return 0;
}

int fixture_getRestitution(lua_State *L)
{
    lua_pushnumber(L, checkFixture(L, 1)->fixture->GetRestitution());
    return 1;
}

int fixture_setRestitution(lua_State *L)
{
    checkFixture(L, 1)->fixture->SetRestitution(luax::checkfloat(L, 2));
    return 0;
}

int fixture_isSensor(lua_State *L)
{
    lua_pushboolean(L, checkFixture(L, 1)->fixture->IsSensor());
    return 1;
}

int fixture_setSensor(lua_State *L)
{
    checkFixture(L, 1)->fixture->SetSensor(luax::checkboolean(L, 2));
    return 0;
}

uint16 readBits(lua_State *L, int first)
{
    uint16 bits = 0;
    int n = lua_gettop(L);
    for (int i = first; i <= n; ++i)
    {
        int bit = static_cast<int>(luaL_checkinteger(L, i));
        if (bit < 1 || bit > 16)
        {
            luaL_error(L, "Values must be in the range 1-16");
        }
        bits |= static_cast<uint16>(1u << (bit - 1));
    }
    return bits;
}

int pushBits(lua_State *L, uint16 bits)
{
    int n = 0;
    for (int bit = 0; bit < 16; ++bit)
    {
        if (bits & (1u << bit))
        {
            lua_pushinteger(L, bit + 1);
            ++n;
        }
    }
    return n;
}

int fixture_setCategory(lua_State *L)
{
    FixtureObj *fixture = checkFixture(L, 1);
    b2Filter filter = fixture->fixture->GetFilterData();
    filter.categoryBits = readBits(L, 2);
    fixture->fixture->SetFilterData(filter);
    return 0;
}

int fixture_getCategory(lua_State *L)
{
    return pushBits(L, checkFixture(L, 1)->fixture->GetFilterData().categoryBits);
}

int fixture_setMask(lua_State *L)
{
    FixtureObj *fixture = checkFixture(L, 1);
    b2Filter filter = fixture->fixture->GetFilterData();
    filter.maskBits = static_cast<uint16>(~readBits(L, 2));
    fixture->fixture->SetFilterData(filter);
    return 0;
}

int fixture_getMask(lua_State *L)
{
    return pushBits(L, static_cast<uint16>(~checkFixture(L, 1)->fixture->GetFilterData().maskBits));
}

int fixture_setGroupIndex(lua_State *L)
{
    FixtureObj *fixture = checkFixture(L, 1);
    b2Filter filter = fixture->fixture->GetFilterData();
    filter.groupIndex = static_cast<int16>(luaL_checkinteger(L, 2));
    fixture->fixture->SetFilterData(filter);
    return 0;
}

int fixture_getGroupIndex(lua_State *L)
{
    lua_pushinteger(L, checkFixture(L, 1)->fixture->GetFilterData().groupIndex);
    return 1;
}

int fixture_setFilterData(lua_State *L)
{
    FixtureObj *fixture = checkFixture(L, 1);
    b2Filter filter;
    filter.categoryBits = static_cast<uint16>(luaL_checkinteger(L, 2));
    filter.maskBits = static_cast<uint16>(luaL_checkinteger(L, 3));
    filter.groupIndex = static_cast<int16>(luaL_checkinteger(L, 4));
    fixture->fixture->SetFilterData(filter);
    return 0;
}

int fixture_getFilterData(lua_State *L)
{
    b2Filter filter = checkFixture(L, 1)->fixture->GetFilterData();
    lua_pushinteger(L, filter.categoryBits);
    lua_pushinteger(L, filter.maskBits);
    lua_pushinteger(L, filter.groupIndex);
    return 3;
}

int fixture_getBoundingBox(lua_State *L)
{
    FixtureObj *fixture = checkFixture(L, 1);
    int child = luax::optint(L, 2, 1) - 1;
    if (child < 0 || child >= fixture->fixture->GetShape()->GetChildCount())
    {
        return luaL_error(L, "Invalid child index");
    }
    b2AABB aabb = fixture->fixture->GetAABB(child);
    pushPoint(L, aabb.lowerBound);
    pushPoint(L, aabb.upperBound);
    return 4;
}

int fixture_getMassData(lua_State *L)
{
    b2MassData data;
    checkFixture(L, 1)->fixture->GetMassData(&data);
    pushPoint(L, data.center);
    lua_pushnumber(L, data.mass);
    lua_pushnumber(L, scaleUp(scaleUp(data.I)));
    return 4;
}

int fixture_testPoint(lua_State *L)
{
    lua_pushboolean(L, checkFixture(L, 1)->fixture->TestPoint(readPoint(L, 2)));
    return 1;
}

int fixture_rayCast(lua_State *L)
{
    FixtureObj *fixture = checkFixture(L, 1);
    b2RayCastInput input;
    input.p1 = readPoint(L, 2);
    input.p2 = input.p1 + luax::checkfloat(L, 6) * (readPoint(L, 4) - input.p1);
    input.maxFraction = 1.0f;
    int child = luax::optint(L, 7, 1) - 1;
    b2RayCastOutput output;
    if (!fixture->fixture->RayCast(&output, input, child))
    {
        return 0;
    }
    lua_pushnumber(L, output.normal.x);
    lua_pushnumber(L, output.normal.y);
    lua_pushnumber(L, output.fraction);
    return 3;
}

int fixture_setUserData(lua_State *L)
{
    checkFixture(L, 1);
    lua_settop(L, 2);
    lua_setiuservalue(L, 1, 1);
    return 0;
}

int fixture_getUserData(lua_State *L)
{
    checkFixture(L, 1);
    lua_getiuservalue(L, 1, 1);
    return 1;
}

int fixture_tostring(lua_State *L)
{
    FixtureObj *fixture = luax::checkobject<FixtureObj>(L, 1, FIXTURE_TYPE);
    lua_pushstring(L, fixture->fixture == nullptr ? "Fixture: destroyed" : "Fixture");
    return 1;
}

const luaL_Reg FIXTURE_METHODS[] = {
    {"destroy", fixture_destroy},
    {"isDestroyed", fixture_isDestroyed},
    {"getBody", fixture_getBody},
    {"getShape", fixture_getShape},
    {"getType", fixture_getType},
    {"getDensity", fixture_getDensity},
    {"setDensity", fixture_setDensity},
    {"getFriction", fixture_getFriction},
    {"setFriction", fixture_setFriction},
    {"getRestitution", fixture_getRestitution},
    {"setRestitution", fixture_setRestitution},
    {"isSensor", fixture_isSensor},
    {"setSensor", fixture_setSensor},
    {"setCategory", fixture_setCategory},
    {"getCategory", fixture_getCategory},
    {"setMask", fixture_setMask},
    {"getMask", fixture_getMask},
    {"setGroupIndex", fixture_setGroupIndex},
    {"getGroupIndex", fixture_getGroupIndex},
    {"setFilterData", fixture_setFilterData},
    {"getFilterData", fixture_getFilterData},
    {"getBoundingBox", fixture_getBoundingBox},
    {"getMassData", fixture_getMassData},
    {"testPoint", fixture_testPoint},
    {"rayCast", fixture_rayCast},
    {"setUserData", fixture_setUserData},
    {"getUserData", fixture_getUserData},
    {"__tostring", fixture_tostring},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// Contact
// ---------------------------------------------------------------------------

ContactObj *checkContact(lua_State *L, int idx)
{
    ContactObj *contact = luax::checkobject<ContactObj>(L, idx, CONTACT_TYPE);
    if (contact->contact == nullptr)
    {
        luaL_error(L, "Attempt to use a Contact outside of the callback or update it belongs to");
    }
    return contact;
}

int contact_getFixtures(lua_State *L)
{
    ContactObj *contact = checkContact(L, 1);
    pushFixture(L, contact->contact->GetFixtureA());
    pushFixture(L, contact->contact->GetFixtureB());
    return 2;
}

int contact_getFriction(lua_State *L)
{
    lua_pushnumber(L, checkContact(L, 1)->contact->GetFriction());
    return 1;
}

int contact_setFriction(lua_State *L)
{
    checkContact(L, 1)->contact->SetFriction(luax::checkfloat(L, 2));
    return 0;
}

int contact_resetFriction(lua_State *L)
{
    checkContact(L, 1)->contact->ResetFriction();
    return 0;
}

int contact_getRestitution(lua_State *L)
{
    lua_pushnumber(L, checkContact(L, 1)->contact->GetRestitution());
    return 1;
}

int contact_setRestitution(lua_State *L)
{
    checkContact(L, 1)->contact->SetRestitution(luax::checkfloat(L, 2));
    return 0;
}

int contact_resetRestitution(lua_State *L)
{
    checkContact(L, 1)->contact->ResetRestitution();
    return 0;
}

int contact_isEnabled(lua_State *L)
{
    lua_pushboolean(L, checkContact(L, 1)->contact->IsEnabled());
    return 1;
}

int contact_setEnabled(lua_State *L)
{
    checkContact(L, 1)->contact->SetEnabled(luax::checkboolean(L, 2));
    return 0;
}

int contact_isTouching(lua_State *L)
{
    lua_pushboolean(L, checkContact(L, 1)->contact->IsTouching());
    return 1;
}

int contact_getNormal(lua_State *L)
{
    b2WorldManifold manifold;
    checkContact(L, 1)->contact->GetWorldManifold(&manifold);
    lua_pushnumber(L, manifold.normal.x);
    lua_pushnumber(L, manifold.normal.y);
    return 2;
}

int contact_getPositions(lua_State *L)
{
    ContactObj *contact = checkContact(L, 1);
    b2WorldManifold manifold;
    contact->contact->GetWorldManifold(&manifold);
    int count = contact->contact->GetManifold()->pointCount;
    for (int i = 0; i < count; ++i)
    {
        pushPoint(L, manifold.points[i]);
    }
    return count * 2;
}

int contact_getChildren(lua_State *L)
{
    ContactObj *contact = checkContact(L, 1);
    lua_pushinteger(L, contact->contact->GetChildIndexA() + 1);
    lua_pushinteger(L, contact->contact->GetChildIndexB() + 1);
    return 2;
}

int contact_setTangentSpeed(lua_State *L)
{
    checkContact(L, 1)->contact->SetTangentSpeed(scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

int contact_getTangentSpeed(lua_State *L)
{
    lua_pushnumber(L, scaleUp(checkContact(L, 1)->contact->GetTangentSpeed()));
    return 1;
}

int contact_isDestroyed(lua_State *L)
{
    ContactObj *contact = luax::checkobject<ContactObj>(L, 1, CONTACT_TYPE);
    lua_pushboolean(L, contact->contact == nullptr);
    return 1;
}

const luaL_Reg CONTACT_METHODS[] = {
    {"getFixtures", contact_getFixtures},
    {"getFriction", contact_getFriction},
    {"setFriction", contact_setFriction},
    {"resetFriction", contact_resetFriction},
    {"getRestitution", contact_getRestitution},
    {"setRestitution", contact_setRestitution},
    {"resetRestitution", contact_resetRestitution},
    {"isEnabled", contact_isEnabled},
    {"setEnabled", contact_setEnabled},
    {"isTouching", contact_isTouching},
    {"getNormal", contact_getNormal},
    {"getPositions", contact_getPositions},
    {"getChildren", contact_getChildren},
    {"setTangentSpeed", contact_setTangentSpeed},
    {"getTangentSpeed", contact_getTangentSpeed},
    {"isDestroyed", contact_isDestroyed},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// Module
// ---------------------------------------------------------------------------

int l_setMeter(lua_State *L)
{
    float meter = luax::checkfloat(L, 1);
    if (meter < 1.0f)
    {
        return luaL_error(L, "Physics error: love.physics.setMeter does not support meter values less than 1");
    }
    g_meter = meter;
    return 0;
}

int l_getMeter(lua_State *L)
{
    lua_pushnumber(L, g_meter);
    return 1;
}

int l_getDistance(lua_State *L)
{
    FixtureObj *a = checkFixture(L, 1);
    FixtureObj *b = checkFixture(L, 2);
    b2DistanceProxy proxyA;
    b2DistanceProxy proxyB;
    proxyA.Set(a->fixture->GetShape(), 0);
    proxyB.Set(b->fixture->GetShape(), 0);
    b2DistanceInput input;
    input.proxyA = proxyA;
    input.proxyB = proxyB;
    input.transformA = a->fixture->GetBody()->GetTransform();
    input.transformB = b->fixture->GetBody()->GetTransform();
    input.useRadii = true;
    b2SimplexCache cache;
    cache.count = 0;
    b2DistanceOutput output;
    b2Distance(&output, &cache, &input);
    lua_pushnumber(L, scaleUp(output.distance));
    pushPoint(L, output.pointA);
    pushPoint(L, output.pointB);
    return 5;
}

const luaL_Reg FUNCS[] = {
    {"newWorld", l_newWorld},
    {"newBody", l_newBody},
    {"newCircleShape", l_newCircleShape},
    {"newRectangleShape", l_newRectangleShape},
    {"newPolygonShape", l_newPolygonShape},
    {"newEdgeShape", l_newEdgeShape},
    {"newChainShape", l_newChainShape},
    {"newFixture", l_newFixture},
    {"setMeter", l_setMeter},
    {"getMeter", l_getMeter},
    {"getDistance", l_getDistance},
    {nullptr, nullptr},
};

} // namespace

void checkBodyPair(lua_State *L, int idx, BodyObj *&a, BodyObj *&b)
{
    a = checkBody(L, idx);
    b = checkBody(L, idx + 1);
    if (a->world != b->world)
    {
        luaL_error(L, "Bodies must belong to the same World");
    }
    if (a->world->world->IsLocked())
    {
        luaL_error(L, "Cannot create a Joint during World:update");
    }
}

bool readCollideConnected(lua_State *L, int idx, bool def)
{
    return luax::optboolean(L, idx, def);
}

} // namespace physics

int open_physics(lua_State *L)
{
    using namespace physics;
    luax::newtype(L, WORLD_TYPE, WORLD_METHODS, world_gc);
    luax::newtype(L, BODY_TYPE, BODY_METHODS);
    luax::newtype(L, FIXTURE_TYPE, FIXTURE_METHODS);
    luax::newtype(L, CONTACT_TYPE, CONTACT_METHODS);
    luax::newtype(L, CIRCLE_TYPE, CIRCLE_METHODS, shape_gc);
    luax::newtype(L, POLYGON_TYPE, POLYGON_METHODS, shape_gc);
    luax::newtype(L, EDGE_TYPE, EDGE_METHODS, shape_gc);
    luax::newtype(L, CHAIN_TYPE, CHAIN_METHODS, shape_gc);
    registerJointTypes(L);
    luaL_newlib(L, FUNCS);
    luaL_setfuncs(L, JOINT_FUNCS, 0);
    return 1;
}

} // namespace love
