// physics_joints.cpp - love.physics joints.
#include "physics_internal.hpp"

#include <cmath>
#include <cstring>

namespace love
{
namespace physics
{

namespace
{

constexpr const char *DISTANCE_TYPE = "DistanceJoint";
constexpr const char *REVOLUTE_TYPE = "RevoluteJoint";
constexpr const char *PRISMATIC_TYPE = "PrismaticJoint";
constexpr const char *WHEEL_TYPE = "WheelJoint";
constexpr const char *WELD_TYPE = "WeldJoint";
constexpr const char *MOUSE_TYPE = "MouseJoint";
constexpr const char *MOTOR_TYPE = "MotorJoint";
constexpr const char *FRICTION_TYPE = "FrictionJoint";
constexpr const char *ROPE_TYPE = "RopeJoint";
constexpr const char *PULLEY_TYPE = "PulleyJoint";
constexpr const char *GEAR_TYPE = "GearJoint";

// Rope joints are distance joints with a zero minimum length (Box2D 2.4).
std::vector<b2Joint *> g_ropeJoints;

bool isRope(b2Joint *joint)
{
    for (b2Joint *j : g_ropeJoints)
    {
        if (j == joint)
        {
            return true;
        }
    }
    return false;
}

const char *jointTypeName(b2Joint *joint)
{
    switch (joint->GetType())
    {
    case e_distanceJoint:
        return isRope(joint) ? ROPE_TYPE : DISTANCE_TYPE;
    case e_revoluteJoint:
        return REVOLUTE_TYPE;
    case e_prismaticJoint:
        return PRISMATIC_TYPE;
    case e_wheelJoint:
        return WHEEL_TYPE;
    case e_weldJoint:
        return WELD_TYPE;
    case e_mouseJoint:
        return MOUSE_TYPE;
    case e_motorJoint:
        return MOTOR_TYPE;
    case e_frictionJoint:
        return FRICTION_TYPE;
    case e_pulleyJoint:
        return PULLEY_TYPE;
    case e_gearJoint:
        return GEAR_TYPE;
    default:
        return DISTANCE_TYPE;
    }
}

const char *jointTypeString(b2Joint *joint)
{
    switch (joint->GetType())
    {
    case e_distanceJoint:
        return isRope(joint) ? "rope" : "distance";
    case e_revoluteJoint:
        return "revolute";
    case e_prismaticJoint:
        return "prismatic";
    case e_wheelJoint:
        return "wheel";
    case e_weldJoint:
        return "weld";
    case e_mouseJoint:
        return "mouse";
    case e_motorJoint:
        return "motor";
    case e_frictionJoint:
        return "friction";
    case e_pulleyJoint:
        return "pulley";
    case e_gearJoint:
        return "gear";
    default:
        return "unknown";
    }
}

// Same effective mass Box2D uses in b2LinearStiffness / b2AngularStiffness.
float reducedMass(b2Joint *j)
{
    float a = j->GetBodyA()->GetMass();
    float b = j->GetBodyB()->GetMass();
    if (a > 0.0f && b > 0.0f)
    {
        return a * b / (a + b);
    }
    return a > 0.0f ? a : b;
}

float reducedInertia(b2Joint *j)
{
    float a = j->GetBodyA()->GetInertia();
    float b = j->GetBodyB()->GetInertia();
    if (a > 0.0f && b > 0.0f)
    {
        return a * b / (a + b);
    }
    return a > 0.0f ? a : b;
}

float frequencyOf(b2Joint *, float stiffness, float mass)
{
    return mass > 0.0f ? std::sqrt(stiffness / mass) / (2.0f * b2_pi) : 0.0f;
}

float dampingFor(float ratio, float stiffness, float mass)
{
    float omega = mass > 0.0f ? std::sqrt(stiffness / mass) : 0.0f;
    return 2.0f * mass * ratio * omega;
}

float ratioOf(float damping, float stiffness, float mass)
{
    float omega = mass > 0.0f ? std::sqrt(stiffness / mass) : 0.0f;
    return (mass > 0.0f && omega > 0.0f) ? damping / (2.0f * mass * omega) : 0.0f;
}

b2Vec2 point(lua_State *L, int idx)
{
    return b2Vec2(scaleDown(luax::checkfloat(L, idx)), scaleDown(luax::checkfloat(L, idx + 1)));
}

void pushVec(lua_State *L, const b2Vec2 &v)
{
    b2Vec2 s = scaleUp(v);
    lua_pushnumber(L, s.x);
    lua_pushnumber(L, s.y);
}

template <class T>
T *joint(lua_State *L, int idx, const char *tname)
{
    JointObj *obj = luax::testobject<JointObj>(L, idx, tname);
    if (obj == nullptr)
    {
        luaL_error(L, "bad argument #%d (%s expected, got %s)", idx, tname, luax::typename_(L, idx));
    }
    if (obj->joint == nullptr)
    {
        luaL_error(L, "Attempt to use destroyed %s", tname);
    }
    return static_cast<T *>(obj->joint);
}

JointObj *create(lua_State *L, WorldObj *world, const b2JointDef &def)
{
    b2Joint *j = world->world->CreateJoint(&def);
    pushJoint(L, j, world);
    return static_cast<JointObj *>(lua_touserdata(L, -1));
}

// ---------------------------------------------------------------------------
// Common Joint methods
// ---------------------------------------------------------------------------

int joint_destroy(lua_State *L)
{
    JointObj *obj = checkJoint(L, 1);
    if (obj->world->world->IsLocked())
    {
        obj->world->destroyJoints.push_back(obj->joint);
        obj->joint = nullptr;
        return 0;
    }
    destroyJointNow(L, obj->world, obj->joint);
    return 0;
}

int joint_isDestroyed(lua_State *L)
{
    JointObj *obj = static_cast<JointObj *>(lua_touserdata(L, 1));
    lua_pushboolean(L, obj == nullptr || obj->joint == nullptr);
    return 1;
}

int joint_getType(lua_State *L)
{
    lua_pushstring(L, jointTypeString(checkJoint(L, 1)->joint));
    return 1;
}

int joint_getBodies(lua_State *L)
{
    JointObj *obj = checkJoint(L, 1);
    pushBody(L, obj->joint->GetBodyA(), obj->world);
    pushBody(L, obj->joint->GetBodyB(), obj->world);
    return 2;
}

int joint_getAnchors(lua_State *L)
{
    b2Joint *j = checkJoint(L, 1)->joint;
    pushVec(L, j->GetAnchorA());
    pushVec(L, j->GetAnchorB());
    return 4;
}

int joint_getCollideConnected(lua_State *L)
{
    lua_pushboolean(L, checkJoint(L, 1)->joint->GetCollideConnected());
    return 1;
}

int joint_getReactionForce(lua_State *L)
{
    b2Joint *j = checkJoint(L, 1)->joint;
    pushVec(L, j->GetReactionForce(luax::checkfloat(L, 2)));
    return 2;
}

int joint_getReactionTorque(lua_State *L)
{
    b2Joint *j = checkJoint(L, 1)->joint;
    lua_pushnumber(L, scaleUp(scaleUp(j->GetReactionTorque(luax::checkfloat(L, 2)))));
    return 1;
}

int joint_setUserData(lua_State *L)
{
    checkJoint(L, 1);
    lua_settop(L, 2);
    lua_setiuservalue(L, 1, 1);
    return 0;
}

int joint_getUserData(lua_State *L)
{
    checkJoint(L, 1);
    lua_getiuservalue(L, 1, 1);
    return 1;
}

int joint_tostring(lua_State *L)
{
    JointObj *obj = static_cast<JointObj *>(lua_touserdata(L, 1));
    if (obj == nullptr || obj->joint == nullptr)
    {
        lua_pushstring(L, "Joint: destroyed");
        return 1;
    }
    lua_pushstring(L, jointTypeName(obj->joint));
    return 1;
}

#define JOINT_COMMON_METHODS                                                                                      \
    {"destroy", joint_destroy}, {"isDestroyed", joint_isDestroyed}, {"getType", joint_getType},                   \
        {"getBodies", joint_getBodies}, {"getAnchors", joint_getAnchors},                                         \
        {"getCollideConnected", joint_getCollideConnected}, {"getReactionForce", joint_getReactionForce},         \
        {"getReactionTorque", joint_getReactionTorque}, {"setUserData", joint_setUserData},                       \
        {"getUserData", joint_getUserData}, {"__tostring", joint_tostring}

// ---------------------------------------------------------------------------
// DistanceJoint / RopeJoint
// ---------------------------------------------------------------------------

int l_newDistanceJoint(lua_State *L)
{
    BodyObj *a;
    BodyObj *b;
    checkBodyPair(L, 1, a, b);
    b2DistanceJointDef def;
    def.Initialize(a->body, b->body, point(L, 3), point(L, 5));
    def.collideConnected = readCollideConnected(L, 7);
    create(L, a->world, def);
    return 1;
}

int l_newRopeJoint(lua_State *L)
{
    BodyObj *a;
    BodyObj *b;
    checkBodyPair(L, 1, a, b);
    b2DistanceJointDef def;
    def.Initialize(a->body, b->body, point(L, 3), point(L, 5));
    def.minLength = 0.0f;
    def.maxLength = scaleDown(luax::checkfloat(L, 7));
    def.length = def.maxLength;
    def.stiffness = 0.0f;
    def.damping = 0.0f;
    def.collideConnected = readCollideConnected(L, 8);
    JointObj *obj = create(L, a->world, def);
    g_ropeJoints.push_back(obj->joint);
    lua_pop(L, 1);
    luax::unregisterobject(L, obj->joint);
    pushJoint(L, obj->joint, a->world);
    return 1;
}

int distance_setLength(lua_State *L)
{
    joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE)->SetLength(scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

int distance_getLength(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE)->GetLength()));
    return 1;
}

int distance_setMinLength(lua_State *L)
{
    joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE)->SetMinLength(scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

int distance_getMinLength(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE)->GetMinLength()));
    return 1;
}

int distance_setMaxLength(lua_State *L)
{
    joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE)->SetMaxLength(scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

int distance_getMaxLength(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE)->GetMaxLength()));
    return 1;
}

int distance_setStiffness(lua_State *L)
{
    joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE)->SetStiffness(luax::checkfloat(L, 2));
    return 0;
}

int distance_getStiffness(lua_State *L)
{
    lua_pushnumber(L, joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE)->GetStiffness());
    return 1;
}

int distance_setDamping(lua_State *L)
{
    joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE)->SetDamping(luax::checkfloat(L, 2));
    return 0;
}

int distance_getDamping(lua_State *L)
{
    lua_pushnumber(L, joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE)->GetDamping());
    return 1;
}

// Love2D 11 frequency/damping-ratio setters map onto Box2D 2.4 stiffness.
int distance_setFrequency(lua_State *L)
{
    b2DistanceJoint *j = joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE);
    float ratio = ratioOf(j->GetDamping(), j->GetStiffness(), reducedMass(j));
    float stiffness, damping;
    b2LinearStiffness(stiffness, damping, luax::checkfloat(L, 2), ratio, j->GetBodyA(), j->GetBodyB());
    j->SetStiffness(stiffness);
    j->SetDamping(damping);
    return 0;
}

int distance_getFrequency(lua_State *L)
{
    b2DistanceJoint *j = joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE);
    lua_pushnumber(L, frequencyOf(j, j->GetStiffness(), reducedMass(j)));
    return 1;
}

int distance_setDampingRatio(lua_State *L)
{
    b2DistanceJoint *j = joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE);
    j->SetDamping(dampingFor(luax::checkfloat(L, 2), j->GetStiffness(), reducedMass(j)));
    return 0;
}

int distance_getDampingRatio(lua_State *L)
{
    b2DistanceJoint *j = joint<b2DistanceJoint>(L, 1, DISTANCE_TYPE);
    lua_pushnumber(L, ratioOf(j->GetDamping(), j->GetStiffness(), reducedMass(j)));
    return 1;
}

const luaL_Reg DISTANCE_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"setLength", distance_setLength},
    {"getLength", distance_getLength},
    {"setMinLength", distance_setMinLength},
    {"getMinLength", distance_getMinLength},
    {"setMaxLength", distance_setMaxLength},
    {"getMaxLength", distance_getMaxLength},
    {"setStiffness", distance_setStiffness},
    {"getStiffness", distance_getStiffness},
    {"setDamping", distance_setDamping},
    {"getDamping", distance_getDamping},
    {"setFrequency", distance_setFrequency},
    {"getFrequency", distance_getFrequency},
    {"setDampingRatio", distance_setDampingRatio},
    {"getDampingRatio", distance_getDampingRatio},
    {nullptr, nullptr},
};

int rope_getMaxLength(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2DistanceJoint>(L, 1, ROPE_TYPE)->GetMaxLength()));
    return 1;
}

int rope_setMaxLength(lua_State *L)
{
    joint<b2DistanceJoint>(L, 1, ROPE_TYPE)->SetMaxLength(scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

const luaL_Reg ROPE_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"getMaxLength", rope_getMaxLength},
    {"setMaxLength", rope_setMaxLength},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// RevoluteJoint
// ---------------------------------------------------------------------------

int l_newRevoluteJoint(lua_State *L)
{
    BodyObj *a;
    BodyObj *b;
    checkBodyPair(L, 1, a, b);
    b2RevoluteJointDef def;
    if (lua_gettop(L) >= 6 && lua_isnumber(L, 5) && lua_isnumber(L, 6))
    {
        def.bodyA = a->body;
        def.bodyB = b->body;
        def.localAnchorA = a->body->GetLocalPoint(point(L, 3));
        def.localAnchorB = b->body->GetLocalPoint(point(L, 5));
        def.collideConnected = readCollideConnected(L, 7);
        def.referenceAngle = luax::optfloat(L, 8, b->body->GetAngle() - a->body->GetAngle());
    }
    else
    {
        def.Initialize(a->body, b->body, point(L, 3));
        def.collideConnected = readCollideConnected(L, 5);
    }
    create(L, a->world, def);
    return 1;
}

int revolute_getJointAngle(lua_State *L)
{
    lua_pushnumber(L, joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->GetJointAngle());
    return 1;
}

int revolute_getJointSpeed(lua_State *L)
{
    lua_pushnumber(L, joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->GetJointSpeed());
    return 1;
}

int revolute_setMotorEnabled(lua_State *L)
{
    joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->EnableMotor(luax::checkboolean(L, 2));
    return 0;
}

int revolute_isMotorEnabled(lua_State *L)
{
    lua_pushboolean(L, joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->IsMotorEnabled());
    return 1;
}

int revolute_setMaxMotorTorque(lua_State *L)
{
    joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->SetMaxMotorTorque(scaleDown(scaleDown(luax::checkfloat(L, 2))));
    return 0;
}

int revolute_getMaxMotorTorque(lua_State *L)
{
    lua_pushnumber(L, scaleUp(scaleUp(joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->GetMaxMotorTorque())));
    return 1;
}

int revolute_setMotorSpeed(lua_State *L)
{
    joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->SetMotorSpeed(luax::checkfloat(L, 2));
    return 0;
}

int revolute_getMotorSpeed(lua_State *L)
{
    lua_pushnumber(L, joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->GetMotorSpeed());
    return 1;
}

int revolute_getMotorTorque(lua_State *L)
{
    lua_pushnumber(L, scaleUp(scaleUp(joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->GetMotorTorque(luax::checkfloat(L, 2)))));
    return 1;
}

int revolute_setLimitsEnabled(lua_State *L)
{
    joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->EnableLimit(luax::checkboolean(L, 2));
    return 0;
}

int revolute_areLimitsEnabled(lua_State *L)
{
    lua_pushboolean(L, joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->IsLimitEnabled());
    return 1;
}

int revolute_setLimits(lua_State *L)
{
    joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->SetLimits(luax::checkfloat(L, 2), luax::checkfloat(L, 3));
    return 0;
}

int revolute_getLimits(lua_State *L)
{
    b2RevoluteJoint *j = joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE);
    lua_pushnumber(L, j->GetLowerLimit());
    lua_pushnumber(L, j->GetUpperLimit());
    return 2;
}

int revolute_setLowerLimit(lua_State *L)
{
    b2RevoluteJoint *j = joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE);
    j->SetLimits(luax::checkfloat(L, 2), j->GetUpperLimit());
    return 0;
}

int revolute_setUpperLimit(lua_State *L)
{
    b2RevoluteJoint *j = joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE);
    j->SetLimits(j->GetLowerLimit(), luax::checkfloat(L, 2));
    return 0;
}

int revolute_getLowerLimit(lua_State *L)
{
    lua_pushnumber(L, joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->GetLowerLimit());
    return 1;
}

int revolute_getUpperLimit(lua_State *L)
{
    lua_pushnumber(L, joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->GetUpperLimit());
    return 1;
}

int revolute_getReferenceAngle(lua_State *L)
{
    lua_pushnumber(L, joint<b2RevoluteJoint>(L, 1, REVOLUTE_TYPE)->GetReferenceAngle());
    return 1;
}

const luaL_Reg REVOLUTE_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"getJointAngle", revolute_getJointAngle},
    {"getJointSpeed", revolute_getJointSpeed},
    {"setMotorEnabled", revolute_setMotorEnabled},
    {"isMotorEnabled", revolute_isMotorEnabled},
    {"setMaxMotorTorque", revolute_setMaxMotorTorque},
    {"getMaxMotorTorque", revolute_getMaxMotorTorque},
    {"setMotorSpeed", revolute_setMotorSpeed},
    {"getMotorSpeed", revolute_getMotorSpeed},
    {"getMotorTorque", revolute_getMotorTorque},
    {"setLimitsEnabled", revolute_setLimitsEnabled},
    {"areLimitsEnabled", revolute_areLimitsEnabled},
    {"hasLimitsEnabled", revolute_areLimitsEnabled},
    {"setLimits", revolute_setLimits},
    {"getLimits", revolute_getLimits},
    {"setLowerLimit", revolute_setLowerLimit},
    {"setUpperLimit", revolute_setUpperLimit},
    {"getLowerLimit", revolute_getLowerLimit},
    {"getUpperLimit", revolute_getUpperLimit},
    {"getReferenceAngle", revolute_getReferenceAngle},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// PrismaticJoint
// ---------------------------------------------------------------------------

int l_newPrismaticJoint(lua_State *L)
{
    BodyObj *a;
    BodyObj *b;
    checkBodyPair(L, 1, a, b);
    b2PrismaticJointDef def;
    if (lua_gettop(L) >= 8 && lua_isnumber(L, 7) && lua_isnumber(L, 8))
    {
        b2Vec2 axis(luax::checkfloat(L, 7), luax::checkfloat(L, 8));
        axis.Normalize();
        def.bodyA = a->body;
        def.bodyB = b->body;
        def.localAnchorA = a->body->GetLocalPoint(point(L, 3));
        def.localAnchorB = b->body->GetLocalPoint(point(L, 5));
        def.localAxisA = a->body->GetLocalVector(axis);
        def.collideConnected = readCollideConnected(L, 9);
        def.referenceAngle = luax::optfloat(L, 10, b->body->GetAngle() - a->body->GetAngle());
    }
    else
    {
        b2Vec2 axis(luax::checkfloat(L, 5), luax::checkfloat(L, 6));
        axis.Normalize();
        def.Initialize(a->body, b->body, point(L, 3), axis);
        def.collideConnected = readCollideConnected(L, 7);
    }
    create(L, a->world, def);
    return 1;
}

int prismatic_getJointTranslation(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->GetJointTranslation()));
    return 1;
}

int prismatic_getJointSpeed(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->GetJointSpeed()));
    return 1;
}

int prismatic_setMotorEnabled(lua_State *L)
{
    joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->EnableMotor(luax::checkboolean(L, 2));
    return 0;
}

int prismatic_isMotorEnabled(lua_State *L)
{
    lua_pushboolean(L, joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->IsMotorEnabled());
    return 1;
}

int prismatic_setMaxMotorForce(lua_State *L)
{
    joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->SetMaxMotorForce(scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

int prismatic_getMaxMotorForce(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->GetMaxMotorForce()));
    return 1;
}

int prismatic_setMotorSpeed(lua_State *L)
{
    joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->SetMotorSpeed(scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

int prismatic_getMotorSpeed(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->GetMotorSpeed()));
    return 1;
}

int prismatic_getMotorForce(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->GetMotorForce(luax::checkfloat(L, 2))));
    return 1;
}

int prismatic_setLimitsEnabled(lua_State *L)
{
    joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->EnableLimit(luax::checkboolean(L, 2));
    return 0;
}

int prismatic_areLimitsEnabled(lua_State *L)
{
    lua_pushboolean(L, joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->IsLimitEnabled());
    return 1;
}

int prismatic_setLimits(lua_State *L)
{
    joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->SetLimits(scaleDown(luax::checkfloat(L, 2)), scaleDown(luax::checkfloat(L, 3)));
    return 0;
}

int prismatic_getLimits(lua_State *L)
{
    b2PrismaticJoint *j = joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE);
    lua_pushnumber(L, scaleUp(j->GetLowerLimit()));
    lua_pushnumber(L, scaleUp(j->GetUpperLimit()));
    return 2;
}

int prismatic_setLowerLimit(lua_State *L)
{
    b2PrismaticJoint *j = joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE);
    j->SetLimits(scaleDown(luax::checkfloat(L, 2)), j->GetUpperLimit());
    return 0;
}

int prismatic_setUpperLimit(lua_State *L)
{
    b2PrismaticJoint *j = joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE);
    j->SetLimits(j->GetLowerLimit(), scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

int prismatic_getLowerLimit(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->GetLowerLimit()));
    return 1;
}

int prismatic_getUpperLimit(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->GetUpperLimit()));
    return 1;
}

int prismatic_getAxis(lua_State *L)
{
    b2PrismaticJoint *j = joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE);
    b2Vec2 axis = j->GetBodyA()->GetWorldVector(j->GetLocalAxisA());
    lua_pushnumber(L, axis.x);
    lua_pushnumber(L, axis.y);
    return 2;
}

int prismatic_getReferenceAngle(lua_State *L)
{
    lua_pushnumber(L, joint<b2PrismaticJoint>(L, 1, PRISMATIC_TYPE)->GetReferenceAngle());
    return 1;
}

const luaL_Reg PRISMATIC_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"getJointTranslation", prismatic_getJointTranslation},
    {"getJointSpeed", prismatic_getJointSpeed},
    {"setMotorEnabled", prismatic_setMotorEnabled},
    {"isMotorEnabled", prismatic_isMotorEnabled},
    {"setMaxMotorForce", prismatic_setMaxMotorForce},
    {"getMaxMotorForce", prismatic_getMaxMotorForce},
    {"setMotorSpeed", prismatic_setMotorSpeed},
    {"getMotorSpeed", prismatic_getMotorSpeed},
    {"getMotorForce", prismatic_getMotorForce},
    {"setLimitsEnabled", prismatic_setLimitsEnabled},
    {"areLimitsEnabled", prismatic_areLimitsEnabled},
    {"hasLimitsEnabled", prismatic_areLimitsEnabled},
    {"setLimits", prismatic_setLimits},
    {"getLimits", prismatic_getLimits},
    {"setLowerLimit", prismatic_setLowerLimit},
    {"setUpperLimit", prismatic_setUpperLimit},
    {"getLowerLimit", prismatic_getLowerLimit},
    {"getUpperLimit", prismatic_getUpperLimit},
    {"getAxis", prismatic_getAxis},
    {"getReferenceAngle", prismatic_getReferenceAngle},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// WheelJoint
// ---------------------------------------------------------------------------

int l_newWheelJoint(lua_State *L)
{
    BodyObj *a;
    BodyObj *b;
    checkBodyPair(L, 1, a, b);
    b2WheelJointDef def;
    if (lua_gettop(L) >= 8 && lua_isnumber(L, 7) && lua_isnumber(L, 8))
    {
        b2Vec2 axis(luax::checkfloat(L, 7), luax::checkfloat(L, 8));
        axis.Normalize();
        def.bodyA = a->body;
        def.bodyB = b->body;
        def.localAnchorA = a->body->GetLocalPoint(point(L, 3));
        def.localAnchorB = b->body->GetLocalPoint(point(L, 5));
        def.localAxisA = a->body->GetLocalVector(axis);
        def.collideConnected = readCollideConnected(L, 9);
    }
    else
    {
        b2Vec2 axis(luax::checkfloat(L, 5), luax::checkfloat(L, 6));
        axis.Normalize();
        def.Initialize(a->body, b->body, point(L, 3), axis);
        def.collideConnected = readCollideConnected(L, 7);
    }
    b2LinearStiffness(def.stiffness, def.damping, 2.0f, 0.7f, a->body, b->body);
    create(L, a->world, def);
    return 1;
}

int wheel_getJointTranslation(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->GetJointTranslation()));
    return 1;
}

int wheel_getJointSpeed(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->GetJointLinearSpeed()));
    return 1;
}

int wheel_setMotorEnabled(lua_State *L)
{
    joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->EnableMotor(luax::checkboolean(L, 2));
    return 0;
}

int wheel_isMotorEnabled(lua_State *L)
{
    lua_pushboolean(L, joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->IsMotorEnabled());
    return 1;
}

int wheel_setMotorSpeed(lua_State *L)
{
    joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->SetMotorSpeed(luax::checkfloat(L, 2));
    return 0;
}

int wheel_getMotorSpeed(lua_State *L)
{
    lua_pushnumber(L, joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->GetMotorSpeed());
    return 1;
}

int wheel_setMaxMotorTorque(lua_State *L)
{
    joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->SetMaxMotorTorque(scaleDown(scaleDown(luax::checkfloat(L, 2))));
    return 0;
}

int wheel_getMaxMotorTorque(lua_State *L)
{
    lua_pushnumber(L, scaleUp(scaleUp(joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->GetMaxMotorTorque())));
    return 1;
}

int wheel_getMotorTorque(lua_State *L)
{
    lua_pushnumber(L, scaleUp(scaleUp(joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->GetMotorTorque(luax::checkfloat(L, 2)))));
    return 1;
}

int wheel_setStiffness(lua_State *L)
{
    joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->SetStiffness(luax::checkfloat(L, 2));
    return 0;
}

int wheel_getStiffness(lua_State *L)
{
    lua_pushnumber(L, joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->GetStiffness());
    return 1;
}

int wheel_setDamping(lua_State *L)
{
    joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->SetDamping(luax::checkfloat(L, 2));
    return 0;
}

int wheel_getDamping(lua_State *L)
{
    lua_pushnumber(L, joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->GetDamping());
    return 1;
}

int wheel_setSpringFrequency(lua_State *L)
{
    b2WheelJoint *j = joint<b2WheelJoint>(L, 1, WHEEL_TYPE);
    float ratio = ratioOf(j->GetDamping(), j->GetStiffness(), reducedMass(j));
    float stiffness, damping;
    b2LinearStiffness(stiffness, damping, luax::checkfloat(L, 2), ratio, j->GetBodyA(), j->GetBodyB());
    j->SetStiffness(stiffness);
    j->SetDamping(damping);
    return 0;
}

int wheel_getSpringFrequency(lua_State *L)
{
    b2WheelJoint *j = joint<b2WheelJoint>(L, 1, WHEEL_TYPE);
    lua_pushnumber(L, frequencyOf(j, j->GetStiffness(), reducedMass(j)));
    return 1;
}

int wheel_setSpringDampingRatio(lua_State *L)
{
    b2WheelJoint *j = joint<b2WheelJoint>(L, 1, WHEEL_TYPE);
    j->SetDamping(dampingFor(luax::checkfloat(L, 2), j->GetStiffness(), reducedMass(j)));
    return 0;
}

int wheel_getSpringDampingRatio(lua_State *L)
{
    b2WheelJoint *j = joint<b2WheelJoint>(L, 1, WHEEL_TYPE);
    lua_pushnumber(L, ratioOf(j->GetDamping(), j->GetStiffness(), reducedMass(j)));
    return 1;
}

int wheel_setLimitsEnabled(lua_State *L)
{
    joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->EnableLimit(luax::checkboolean(L, 2));
    return 0;
}

int wheel_areLimitsEnabled(lua_State *L)
{
    lua_pushboolean(L, joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->IsLimitEnabled());
    return 1;
}

int wheel_setLimits(lua_State *L)
{
    joint<b2WheelJoint>(L, 1, WHEEL_TYPE)->SetLimits(scaleDown(luax::checkfloat(L, 2)), scaleDown(luax::checkfloat(L, 3)));
    return 0;
}

int wheel_getLimits(lua_State *L)
{
    b2WheelJoint *j = joint<b2WheelJoint>(L, 1, WHEEL_TYPE);
    lua_pushnumber(L, scaleUp(j->GetLowerLimit()));
    lua_pushnumber(L, scaleUp(j->GetUpperLimit()));
    return 2;
}

int wheel_getAxis(lua_State *L)
{
    b2WheelJoint *j = joint<b2WheelJoint>(L, 1, WHEEL_TYPE);
    b2Vec2 axis = j->GetBodyA()->GetWorldVector(j->GetLocalAxisA());
    lua_pushnumber(L, axis.x);
    lua_pushnumber(L, axis.y);
    return 2;
}

const luaL_Reg WHEEL_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"getJointTranslation", wheel_getJointTranslation},
    {"getJointSpeed", wheel_getJointSpeed},
    {"setMotorEnabled", wheel_setMotorEnabled},
    {"isMotorEnabled", wheel_isMotorEnabled},
    {"setMotorSpeed", wheel_setMotorSpeed},
    {"getMotorSpeed", wheel_getMotorSpeed},
    {"setMaxMotorTorque", wheel_setMaxMotorTorque},
    {"getMaxMotorTorque", wheel_getMaxMotorTorque},
    {"getMotorTorque", wheel_getMotorTorque},
    {"setStiffness", wheel_setStiffness},
    {"getStiffness", wheel_getStiffness},
    {"setDamping", wheel_setDamping},
    {"getDamping", wheel_getDamping},
    {"setSpringFrequency", wheel_setSpringFrequency},
    {"getSpringFrequency", wheel_getSpringFrequency},
    {"setSpringDampingRatio", wheel_setSpringDampingRatio},
    {"getSpringDampingRatio", wheel_getSpringDampingRatio},
    {"setLimitsEnabled", wheel_setLimitsEnabled},
    {"areLimitsEnabled", wheel_areLimitsEnabled},
    {"setLimits", wheel_setLimits},
    {"getLimits", wheel_getLimits},
    {"getAxis", wheel_getAxis},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// WeldJoint
// ---------------------------------------------------------------------------

int l_newWeldJoint(lua_State *L)
{
    BodyObj *a;
    BodyObj *b;
    checkBodyPair(L, 1, a, b);
    b2WeldJointDef def;
    if (lua_gettop(L) >= 6 && lua_isnumber(L, 5) && lua_isnumber(L, 6))
    {
        def.bodyA = a->body;
        def.bodyB = b->body;
        def.localAnchorA = a->body->GetLocalPoint(point(L, 3));
        def.localAnchorB = b->body->GetLocalPoint(point(L, 5));
        def.collideConnected = readCollideConnected(L, 7);
        def.referenceAngle = luax::optfloat(L, 8, b->body->GetAngle() - a->body->GetAngle());
    }
    else
    {
        def.Initialize(a->body, b->body, point(L, 3));
        def.collideConnected = readCollideConnected(L, 5);
    }
    create(L, a->world, def);
    return 1;
}

int weld_setStiffness(lua_State *L)
{
    joint<b2WeldJoint>(L, 1, WELD_TYPE)->SetStiffness(luax::checkfloat(L, 2));
    return 0;
}

int weld_getStiffness(lua_State *L)
{
    lua_pushnumber(L, joint<b2WeldJoint>(L, 1, WELD_TYPE)->GetStiffness());
    return 1;
}

int weld_setDamping(lua_State *L)
{
    joint<b2WeldJoint>(L, 1, WELD_TYPE)->SetDamping(luax::checkfloat(L, 2));
    return 0;
}

int weld_getDamping(lua_State *L)
{
    lua_pushnumber(L, joint<b2WeldJoint>(L, 1, WELD_TYPE)->GetDamping());
    return 1;
}

int weld_setFrequency(lua_State *L)
{
    b2WeldJoint *j = joint<b2WeldJoint>(L, 1, WELD_TYPE);
    float ratio = ratioOf(j->GetDamping(), j->GetStiffness(), reducedInertia(j));
    float stiffness, damping;
    b2AngularStiffness(stiffness, damping, luax::checkfloat(L, 2), ratio, j->GetBodyA(), j->GetBodyB());
    j->SetStiffness(stiffness);
    j->SetDamping(damping);
    return 0;
}

int weld_getFrequency(lua_State *L)
{
    b2WeldJoint *j = joint<b2WeldJoint>(L, 1, WELD_TYPE);
    lua_pushnumber(L, frequencyOf(j, j->GetStiffness(), reducedInertia(j)));
    return 1;
}

int weld_setDampingRatio(lua_State *L)
{
    b2WeldJoint *j = joint<b2WeldJoint>(L, 1, WELD_TYPE);
    j->SetDamping(dampingFor(luax::checkfloat(L, 2), j->GetStiffness(), reducedInertia(j)));
    return 0;
}

int weld_getDampingRatio(lua_State *L)
{
    b2WeldJoint *j = joint<b2WeldJoint>(L, 1, WELD_TYPE);
    lua_pushnumber(L, ratioOf(j->GetDamping(), j->GetStiffness(), reducedInertia(j)));
    return 1;
}

int weld_getReferenceAngle(lua_State *L)
{
    lua_pushnumber(L, joint<b2WeldJoint>(L, 1, WELD_TYPE)->GetReferenceAngle());
    return 1;
}

const luaL_Reg WELD_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"setStiffness", weld_setStiffness},
    {"getStiffness", weld_getStiffness},
    {"setDamping", weld_setDamping},
    {"getDamping", weld_getDamping},
    {"setFrequency", weld_setFrequency},
    {"getFrequency", weld_getFrequency},
    {"setDampingRatio", weld_setDampingRatio},
    {"getDampingRatio", weld_getDampingRatio},
    {"getReferenceAngle", weld_getReferenceAngle},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// MouseJoint
// ---------------------------------------------------------------------------

int l_newMouseJoint(lua_State *L)
{
    BodyObj *body = checkBody(L, 1);
    if (body->world->world->IsLocked())
    {
        return luaL_error(L, "Cannot create a Joint during World:update");
    }
    b2MouseJointDef def;
    def.bodyA = body->world->groundBody;
    def.bodyB = body->body;
    def.target = point(L, 2);
    def.maxForce = 1000.0f * body->body->GetMass();
    b2LinearStiffness(def.stiffness, def.damping, 5.0f, 0.7f, def.bodyA, def.bodyB);
    create(L, body->world, def);
    return 1;
}

int mouse_setTarget(lua_State *L)
{
    joint<b2MouseJoint>(L, 1, MOUSE_TYPE)->SetTarget(point(L, 2));
    return 0;
}

int mouse_getTarget(lua_State *L)
{
    pushVec(L, joint<b2MouseJoint>(L, 1, MOUSE_TYPE)->GetTarget());
    return 2;
}

int mouse_setMaxForce(lua_State *L)
{
    joint<b2MouseJoint>(L, 1, MOUSE_TYPE)->SetMaxForce(scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

int mouse_getMaxForce(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2MouseJoint>(L, 1, MOUSE_TYPE)->GetMaxForce()));
    return 1;
}

int mouse_setStiffness(lua_State *L)
{
    joint<b2MouseJoint>(L, 1, MOUSE_TYPE)->SetStiffness(luax::checkfloat(L, 2));
    return 0;
}

int mouse_getStiffness(lua_State *L)
{
    lua_pushnumber(L, joint<b2MouseJoint>(L, 1, MOUSE_TYPE)->GetStiffness());
    return 1;
}

int mouse_setDamping(lua_State *L)
{
    joint<b2MouseJoint>(L, 1, MOUSE_TYPE)->SetDamping(luax::checkfloat(L, 2));
    return 0;
}

int mouse_getDamping(lua_State *L)
{
    lua_pushnumber(L, joint<b2MouseJoint>(L, 1, MOUSE_TYPE)->GetDamping());
    return 1;
}

int mouse_setFrequency(lua_State *L)
{
    b2MouseJoint *j = joint<b2MouseJoint>(L, 1, MOUSE_TYPE);
    float ratio = ratioOf(j->GetDamping(), j->GetStiffness(), reducedMass(j));
    float stiffness, damping;
    b2LinearStiffness(stiffness, damping, luax::checkfloat(L, 2), ratio, j->GetBodyA(), j->GetBodyB());
    j->SetStiffness(stiffness);
    j->SetDamping(damping);
    return 0;
}

int mouse_getFrequency(lua_State *L)
{
    b2MouseJoint *j = joint<b2MouseJoint>(L, 1, MOUSE_TYPE);
    lua_pushnumber(L, frequencyOf(j, j->GetStiffness(), reducedMass(j)));
    return 1;
}

int mouse_setDampingRatio(lua_State *L)
{
    b2MouseJoint *j = joint<b2MouseJoint>(L, 1, MOUSE_TYPE);
    j->SetDamping(dampingFor(luax::checkfloat(L, 2), j->GetStiffness(), reducedMass(j)));
    return 0;
}

int mouse_getDampingRatio(lua_State *L)
{
    b2MouseJoint *j = joint<b2MouseJoint>(L, 1, MOUSE_TYPE);
    lua_pushnumber(L, ratioOf(j->GetDamping(), j->GetStiffness(), reducedMass(j)));
    return 1;
}

const luaL_Reg MOUSE_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"setTarget", mouse_setTarget},
    {"getTarget", mouse_getTarget},
    {"setMaxForce", mouse_setMaxForce},
    {"getMaxForce", mouse_getMaxForce},
    {"setStiffness", mouse_setStiffness},
    {"getStiffness", mouse_getStiffness},
    {"setDamping", mouse_setDamping},
    {"getDamping", mouse_getDamping},
    {"setFrequency", mouse_setFrequency},
    {"getFrequency", mouse_getFrequency},
    {"setDampingRatio", mouse_setDampingRatio},
    {"getDampingRatio", mouse_getDampingRatio},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// MotorJoint
// ---------------------------------------------------------------------------

int l_newMotorJoint(lua_State *L)
{
    BodyObj *a;
    BodyObj *b;
    checkBodyPair(L, 1, a, b);
    b2MotorJointDef def;
    def.Initialize(a->body, b->body);
    def.correctionFactor = luax::optfloat(L, 3, 0.3f);
    def.collideConnected = readCollideConnected(L, 4);
    create(L, a->world, def);
    return 1;
}

int motor_setLinearOffset(lua_State *L)
{
    joint<b2MotorJoint>(L, 1, MOTOR_TYPE)->SetLinearOffset(point(L, 2));
    return 0;
}

int motor_getLinearOffset(lua_State *L)
{
    pushVec(L, joint<b2MotorJoint>(L, 1, MOTOR_TYPE)->GetLinearOffset());
    return 2;
}

int motor_setAngularOffset(lua_State *L)
{
    joint<b2MotorJoint>(L, 1, MOTOR_TYPE)->SetAngularOffset(luax::checkfloat(L, 2));
    return 0;
}

int motor_getAngularOffset(lua_State *L)
{
    lua_pushnumber(L, joint<b2MotorJoint>(L, 1, MOTOR_TYPE)->GetAngularOffset());
    return 1;
}

int motor_setMaxForce(lua_State *L)
{
    joint<b2MotorJoint>(L, 1, MOTOR_TYPE)->SetMaxForce(scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

int motor_getMaxForce(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2MotorJoint>(L, 1, MOTOR_TYPE)->GetMaxForce()));
    return 1;
}

int motor_setMaxTorque(lua_State *L)
{
    joint<b2MotorJoint>(L, 1, MOTOR_TYPE)->SetMaxTorque(scaleDown(scaleDown(luax::checkfloat(L, 2))));
    return 0;
}

int motor_getMaxTorque(lua_State *L)
{
    lua_pushnumber(L, scaleUp(scaleUp(joint<b2MotorJoint>(L, 1, MOTOR_TYPE)->GetMaxTorque())));
    return 1;
}

int motor_setCorrectionFactor(lua_State *L)
{
    joint<b2MotorJoint>(L, 1, MOTOR_TYPE)->SetCorrectionFactor(luax::checkfloat(L, 2));
    return 0;
}

int motor_getCorrectionFactor(lua_State *L)
{
    lua_pushnumber(L, joint<b2MotorJoint>(L, 1, MOTOR_TYPE)->GetCorrectionFactor());
    return 1;
}

const luaL_Reg MOTOR_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"setLinearOffset", motor_setLinearOffset},
    {"getLinearOffset", motor_getLinearOffset},
    {"setAngularOffset", motor_setAngularOffset},
    {"getAngularOffset", motor_getAngularOffset},
    {"setMaxForce", motor_setMaxForce},
    {"getMaxForce", motor_getMaxForce},
    {"setMaxTorque", motor_setMaxTorque},
    {"getMaxTorque", motor_getMaxTorque},
    {"setCorrectionFactor", motor_setCorrectionFactor},
    {"getCorrectionFactor", motor_getCorrectionFactor},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// FrictionJoint
// ---------------------------------------------------------------------------

int l_newFrictionJoint(lua_State *L)
{
    BodyObj *a;
    BodyObj *b;
    checkBodyPair(L, 1, a, b);
    b2FrictionJointDef def;
    if (lua_gettop(L) >= 6 && lua_isnumber(L, 5) && lua_isnumber(L, 6))
    {
        def.bodyA = a->body;
        def.bodyB = b->body;
        def.localAnchorA = a->body->GetLocalPoint(point(L, 3));
        def.localAnchorB = b->body->GetLocalPoint(point(L, 5));
        def.collideConnected = readCollideConnected(L, 7);
    }
    else
    {
        def.Initialize(a->body, b->body, point(L, 3));
        def.collideConnected = readCollideConnected(L, 5);
    }
    create(L, a->world, def);
    return 1;
}

int friction_setMaxForce(lua_State *L)
{
    joint<b2FrictionJoint>(L, 1, FRICTION_TYPE)->SetMaxForce(scaleDown(luax::checkfloat(L, 2)));
    return 0;
}

int friction_getMaxForce(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2FrictionJoint>(L, 1, FRICTION_TYPE)->GetMaxForce()));
    return 1;
}

int friction_setMaxTorque(lua_State *L)
{
    joint<b2FrictionJoint>(L, 1, FRICTION_TYPE)->SetMaxTorque(scaleDown(scaleDown(luax::checkfloat(L, 2))));
    return 0;
}

int friction_getMaxTorque(lua_State *L)
{
    lua_pushnumber(L, scaleUp(scaleUp(joint<b2FrictionJoint>(L, 1, FRICTION_TYPE)->GetMaxTorque())));
    return 1;
}

const luaL_Reg FRICTION_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"setMaxForce", friction_setMaxForce},
    {"getMaxForce", friction_getMaxForce},
    {"setMaxTorque", friction_setMaxTorque},
    {"getMaxTorque", friction_getMaxTorque},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// PulleyJoint
// ---------------------------------------------------------------------------

int l_newPulleyJoint(lua_State *L)
{
    BodyObj *a;
    BodyObj *b;
    checkBodyPair(L, 1, a, b);
    b2PulleyJointDef def;
    float ratio = luax::optfloat(L, 11, 1.0f);
    def.Initialize(a->body, b->body, point(L, 3), point(L, 5), point(L, 7), point(L, 9), ratio);
    def.collideConnected = readCollideConnected(L, 12, true);
    create(L, a->world, def);
    return 1;
}

int pulley_getGroundAnchors(lua_State *L)
{
    b2PulleyJoint *j = joint<b2PulleyJoint>(L, 1, PULLEY_TYPE);
    pushVec(L, j->GetGroundAnchorA());
    pushVec(L, j->GetGroundAnchorB());
    return 4;
}

int pulley_getLengthA(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2PulleyJoint>(L, 1, PULLEY_TYPE)->GetCurrentLengthA()));
    return 1;
}

int pulley_getLengthB(lua_State *L)
{
    lua_pushnumber(L, scaleUp(joint<b2PulleyJoint>(L, 1, PULLEY_TYPE)->GetCurrentLengthB()));
    return 1;
}

int pulley_getMaxLengths(lua_State *L)
{
    b2PulleyJoint *j = joint<b2PulleyJoint>(L, 1, PULLEY_TYPE);
    lua_pushnumber(L, scaleUp(j->GetLengthA()));
    lua_pushnumber(L, scaleUp(j->GetLengthB()));
    return 2;
}

int pulley_getRatio(lua_State *L)
{
    lua_pushnumber(L, joint<b2PulleyJoint>(L, 1, PULLEY_TYPE)->GetRatio());
    return 1;
}

int pulley_getConstant(lua_State *L)
{
    b2PulleyJoint *j = joint<b2PulleyJoint>(L, 1, PULLEY_TYPE);
    lua_pushnumber(L, scaleUp(j->GetLengthA() + j->GetRatio() * j->GetLengthB()));
    return 1;
}

const luaL_Reg PULLEY_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"getGroundAnchors", pulley_getGroundAnchors},
    {"getLengthA", pulley_getLengthA},
    {"getLengthB", pulley_getLengthB},
    {"getMaxLengths", pulley_getMaxLengths},
    {"getRatio", pulley_getRatio},
    {"getConstant", pulley_getConstant},
    {nullptr, nullptr},
};

// ---------------------------------------------------------------------------
// GearJoint
// ---------------------------------------------------------------------------

int l_newGearJoint(lua_State *L)
{
    JointObj *a = checkJoint(L, 1);
    JointObj *b = checkJoint(L, 2);
    if (a->world != b->world)
    {
        return luaL_error(L, "Joints must belong to the same World");
    }
    b2JointType ta = a->joint->GetType();
    b2JointType tb = b->joint->GetType();
    if ((ta != e_revoluteJoint && ta != e_prismaticJoint) || (tb != e_revoluteJoint && tb != e_prismaticJoint))
    {
        return luaL_error(L, "Gear joints require revolute or prismatic joints");
    }
    b2GearJointDef def;
    def.joint1 = a->joint;
    def.joint2 = b->joint;
    def.bodyA = a->joint->GetBodyB();
    def.bodyB = b->joint->GetBodyB();
    def.ratio = luax::optfloat(L, 3, 1.0f);
    def.collideConnected = readCollideConnected(L, 4);
    create(L, a->world, def);
    return 1;
}

int gear_setRatio(lua_State *L)
{
    joint<b2GearJoint>(L, 1, GEAR_TYPE)->SetRatio(luax::checkfloat(L, 2));
    return 0;
}

int gear_getRatio(lua_State *L)
{
    lua_pushnumber(L, joint<b2GearJoint>(L, 1, GEAR_TYPE)->GetRatio());
    return 1;
}

int gear_getJoints(lua_State *L)
{
    JointObj *obj = checkJoint(L, 1);
    b2GearJoint *j = static_cast<b2GearJoint *>(obj->joint);
    pushJoint(L, j->GetJoint1(), obj->world);
    pushJoint(L, j->GetJoint2(), obj->world);
    return 2;
}

const luaL_Reg GEAR_METHODS[] = {
    JOINT_COMMON_METHODS,
    {"setRatio", gear_setRatio},
    {"getRatio", gear_getRatio},
    {"getJoints", gear_getJoints},
    {nullptr, nullptr},
};

} // namespace

JointObj *checkJoint(lua_State *L, int idx)
{
    JointObj *obj = static_cast<JointObj *>(lua_touserdata(L, idx));
    bool ok = false;
    if (obj != nullptr && lua_getmetatable(L, idx))
    {
        lua_getfield(L, -1, "__name");
        const char *name = lua_tostring(L, -1);
        ok = name != nullptr && std::strstr(name, "Joint") != nullptr;
        lua_pop(L, 2);
    }
    if (!ok)
    {
        luaL_error(L, "bad argument #%d (Joint expected, got %s)", idx, luax::typename_(L, idx));
    }
    if (obj->joint == nullptr)
    {
        luaL_error(L, "Attempt to use destroyed Joint");
    }
    return obj;
}

void pushJoint(lua_State *L, b2Joint *joint, WorldObj *world)
{
    if (joint == nullptr)
    {
        lua_pushnil(L);
        return;
    }
    if (luax::pushregistered(L, joint))
    {
        return;
    }
    JointObj *obj = luax::newobject<JointObj>(L, jointTypeName(joint));
    obj->joint = joint;
    obj->world = world;
    luax::registerobject(L, joint, -1, true);
}

void registerJointTypes(lua_State *L)
{
    luax::newtype(L, DISTANCE_TYPE, DISTANCE_METHODS);
    luax::newtype(L, ROPE_TYPE, ROPE_METHODS);
    luax::newtype(L, REVOLUTE_TYPE, REVOLUTE_METHODS);
    luax::newtype(L, PRISMATIC_TYPE, PRISMATIC_METHODS);
    luax::newtype(L, WHEEL_TYPE, WHEEL_METHODS);
    luax::newtype(L, WELD_TYPE, WELD_METHODS);
    luax::newtype(L, MOUSE_TYPE, MOUSE_METHODS);
    luax::newtype(L, MOTOR_TYPE, MOTOR_METHODS);
    luax::newtype(L, FRICTION_TYPE, FRICTION_METHODS);
    luax::newtype(L, PULLEY_TYPE, PULLEY_METHODS);
    luax::newtype(L, GEAR_TYPE, GEAR_METHODS);
}

const luaL_Reg JOINT_FUNCS[] = {
    {"newDistanceJoint", l_newDistanceJoint},
    {"newRopeJoint", l_newRopeJoint},
    {"newRevoluteJoint", l_newRevoluteJoint},
    {"newPrismaticJoint", l_newPrismaticJoint},
    {"newWheelJoint", l_newWheelJoint},
    {"newWeldJoint", l_newWeldJoint},
    {"newMouseJoint", l_newMouseJoint},
    {"newMotorJoint", l_newMotorJoint},
    {"newFrictionJoint", l_newFrictionJoint},
    {"newPulleyJoint", l_newPulleyJoint},
    {"newGearJoint", l_newGearJoint},
    {nullptr, nullptr},
};

} // namespace physics
} // namespace love
