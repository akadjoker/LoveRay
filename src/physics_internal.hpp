#pragma once

#include "love.hpp"
#include "luax.hpp"

#include <box2d/box2d.h>

#include <string>
#include <vector>

namespace love
{
namespace physics
{

constexpr const char *WORLD_TYPE = "World";
constexpr const char *BODY_TYPE = "Body";
constexpr const char *FIXTURE_TYPE = "Fixture";
constexpr const char *CONTACT_TYPE = "Contact";
constexpr const char *CIRCLE_TYPE = "CircleShape";
constexpr const char *POLYGON_TYPE = "PolygonShape";
constexpr const char *EDGE_TYPE = "EdgeShape";
constexpr const char *CHAIN_TYPE = "ChainShape";

float getMeter();
float scaleDown(float v);
float scaleUp(float v);
b2Vec2 scaleDown(const b2Vec2 &v);
b2Vec2 scaleUp(const b2Vec2 &v);

struct WorldObj;

struct BodyObj
{
    b2Body *body = nullptr;
    WorldObj *world = nullptr;
};

struct FixtureObj
{
    b2Fixture *fixture = nullptr;
    WorldObj *world = nullptr;
};

struct JointObj
{
    b2Joint *joint = nullptr;
    WorldObj *world = nullptr;
};

struct ShapeObj
{
    b2Shape *shape = nullptr;
    bool owned = false;
};

struct ContactObj
{
    b2Contact *contact = nullptr;
};

class ContactListener;
class ContactFilter;

struct WorldObj
{
    b2World *world = nullptr;
    b2Body *groundBody = nullptr;
    lua_State *L = nullptr;
    luax::Ref callbacks[4]; // begin, end, presolve, postsolve
    luax::Ref filter;
    ContactListener *listener = nullptr;
    ContactFilter *contactFilter = nullptr;
    std::vector<b2Body *> destroyBodies;
    std::vector<b2Joint *> destroyJoints;
    std::vector<b2Fixture *> destroyFixtures;
    std::vector<ContactObj *> liveContacts;
    std::string pendingError;
    bool destroyed = false;
};

WorldObj *checkWorld(lua_State *L, int idx);
BodyObj *checkBody(lua_State *L, int idx);
FixtureObj *checkFixture(lua_State *L, int idx);
ShapeObj *checkShape(lua_State *L, int idx);
JointObj *checkJoint(lua_State *L, int idx);

void pushBody(lua_State *L, b2Body *body, WorldObj *world);
void pushFixture(lua_State *L, b2Fixture *fixture);
void pushJoint(lua_State *L, b2Joint *joint, WorldObj *world);
void pushContact(lua_State *L, WorldObj *world, b2Contact *contact);
// Pushes the Shape wrapper for a fixture's live shape (not owned by Lua).
void pushFixtureShape(lua_State *L, b2Fixture *fixture);

void destroyJointNow(lua_State *L, WorldObj *world, b2Joint *joint);
void destroyBodyNow(lua_State *L, WorldObj *world, b2Body *body);
void destroyFixtureNow(lua_State *L, b2Fixture *fixture);

// Reads a body pair (bodyA, bodyB) at idx/idx+1; both must share a world.
void checkBodyPair(lua_State *L, int idx, BodyObj *&a, BodyObj *&b);
bool readCollideConnected(lua_State *L, int idx, bool def = false);

void registerJointTypes(lua_State *L);
extern const luaL_Reg JOINT_FUNCS[];

} // namespace physics
} // namespace love
