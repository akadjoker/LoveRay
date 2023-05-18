#ifndef BOXPHYSICS_H
#define BOXPHYSICS_H

#include <vector>
#include <lua.hpp>
#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>
#include "box2d.h"

int luaopen_box2d(lua_State *L);
int luaclass_box2d(lua_State *L);

void init_box2d();
void free_box2d();
void clear_box2d();

#endif

//***********************************************************************************
//***********************************************************************************
//                                BINDING
//***********************************************************************************
//***********************************************************************************
#ifdef BOX_PHYSICS_IMPLEMENTATION

// Defina a escala da simulação (30 pixels por metro)
static float SCALE = 30.0f;

// Converta uma posição de pixel para uma posição de mundo
b2Vec2 pixelToWorld(float x, float y)
{
    return b2Vec2(x / SCALE, y / SCALE);
}

b2Vec2 vectorToWorld(const Vector2 &v)
{
    return b2Vec2(v.x / SCALE, v.y / SCALE);
}
// Converta uma posição de mundo para uma posição de pixel
Vector2 worldToPixel(const b2Vec2 &pos)
{
    Vector2 v;
    v.x = pos.x * SCALE;
    v.y = pos.y * SCALE;
    return v;
}

float worldToPixel(float value)
{
    return value * SCALE;
}

float pixelToWorld(float value)
{
    return value / SCALE;
}

float degreesToRadians(float degrees)
{
    return degrees * b2_pi / 180.0f;
}

namespace nRay
{

    Color getColor(const b2Color &color)
    {
        Color c;
        c.r = color.r * 255;
        c.g = color.g * 255;
        c.b = color.b * 255;
        c.a = color.a * 255;
        return c;
    }

    void rDrawCircle(const b2Vec2 &center, float radius, const b2Color &color)
    {
        Vector2 pos = worldToPixel(center);
        float r = worldToPixel(radius);
        DrawCircleLines(pos.x, pos.y, r, getColor(color));
    }
    void rDrawSolidCircle(const b2Vec2 &center, float radius, const b2Color &color)
    {
        Vector2 pos = worldToPixel(center);
        float r = worldToPixel(radius);
        DrawCircle(pos.x, pos.y, r, getColor(color));
    }

}

static bool drawFilled = false;

class bDebugDraw : public b2Draw
{
public:
    bDebugDraw() {}
    ~bDebugDraw() {}

    void DrawPolygon(const b2Vec2 *vertices, int32 vertexCount, const b2Color &color)
    {

        for (int i = 0; i < vertexCount; i++)
        {
            int j = (i + 1) % vertexCount;
            Vector2 p1 = worldToPixel(vertices[i]);
            Vector2 p2 = worldToPixel(vertices[j]);
            DrawLine(p1.x, p1.y, p2.x, p2.y, raylibColor(color));
        }
    }

    void DrawSolidPolygon(const b2Vec2 *vertices, int32 vertexCount, const b2Color &color)
    {

        if (vertexCount >= 3)
        {

            // Vector2 v0 = worldToPixel(vertices[2]);
            // Vector2 v1 = worldToPixel(vertices[1]);
            // Vector2 v2 = worldToPixel(vertices[0]);
            // DrawTriangle(v0, v1, v2, raylibColor(color));

            rlBegin(RL_QUADS);

            Color fill = raylibColor(color);
            rlColor4ub(fill.r, fill.g, fill.b, fill.a);

            for (int i = 0; i < vertexCount - 1; i++)
            {
                Vector2 v0 = worldToPixel(vertices[0]);
                Vector2 v1 = worldToPixel(vertices[i + 1]);
                Vector2 v2 = worldToPixel(vertices[i + 1]);
                Vector2 v3 = worldToPixel(vertices[i]);
                rlVertex2f(v0.x, v0.y);
                rlVertex2f(v1.x, v1.y);
                rlVertex2f(v2.x, v2.y);
                rlVertex2f(v3.x, v3.y);
            }
            rlEnd();
        }
    }

    void DrawCircle(const b2Vec2 &center, float radius, const b2Color &color)
    {
        nRay::rDrawCircle(center, radius, color);
    }

    void DrawSolidCircle(const b2Vec2 &center, float radius, const b2Vec2 &axis, const b2Color &color)
    {
        nRay::rDrawSolidCircle(center, radius, color);
        Vector2 pos = worldToPixel(center);
        DrawLine(pos.x, pos.y, pos.x + axis.x * radius * SCALE, pos.y + axis.y * radius * SCALE, RED);
    }

    void DrawSegment(const b2Vec2 &p1, const b2Vec2 &p2, const b2Color &color)
    {
        Vector2 pos1 = worldToPixel(p1);
        Vector2 pos2 = worldToPixel(p2);
        DrawLine(pos1.x, pos1.y, pos2.x, pos2.y, raylibColor(color));
    }

    void DrawTransform(const b2Transform &xf)
    {
        Vector2 p1 = worldToPixel(xf.p);
        Vector2 p2 = worldToPixel(b2Mul(xf, b2Vec2(0.5f, 0.0f)));
        //   DrawLine(p1.x, p1.y, p2.x, p2.y, GREEN);
    }
    void DrawPoint(const b2Vec2 &p, float size, const b2Color &color)
    {
        nRay::rDrawCircle(p, 0.1f, color);
    }

private:
    Color raylibColor(const b2Color &color)
    {
        return {(unsigned char)(color.r * 255), (unsigned char)(color.g * 255), (unsigned char)(color.b * 255), (unsigned char)(color.a * 255)};
    }
};

static b2World *world;
static b2Body *groundBody;
static int32 velocityIterations = 6;
static int32 positionIterations = 2;
bDebugDraw debugDraw;
std::vector<b2Joint *> jointsScheduledForRemoval;
std::vector<b2Body *> bodysScheduledForRemoval;

bool isConvex(b2Vec2 prev, b2Vec2 curr, b2Vec2 next)
{
    float cross = (curr.x - prev.x) * (next.y - prev.y) - (curr.y - prev.y) * (next.x - prev.x);
    return cross < 0.0f;
}

bool isInside(b2Vec2 a, b2Vec2 b, b2Vec2 c, b2Vec2 p)
{
    float ax = a.x, ay = a.y;
    float bx = b.x, by = b.y;
    float cx = c.x, cy = c.y;
    float px = p.x, py = p.y;
    float apx = px - ax, apy = py - ay;
    float bpx = px - bx, bpy = py - by;
    float cpx = px - cx, cpy = py - cy;
    float abx = bx - ax, aby = by - ay;
    float bcy = cy - by, bcx = cx - bx;
    float cax = ax - cx, cay = ay - cy;
    float aCROSSbp = ax * by - ay * bx + bx * py - by * px + px * ay - py * ax;
    float cCROSSap = cx * ay - cy * ax + ax * py - ay * px + px * cy - py * cx;
    float bCROSScp = bx * cy - by * cx + cx * py - cy * px + px * by - py * bx;
    return ((aCROSSbp >= 0.0f && cCROSSap >= 0.0f && bCROSScp >= 0.0f) ||
            (aCROSSbp <= 0.0f && cCROSSap <= 0.0f && bCROSScp <= 0.0f));
}

bool isEar(b2Vec2 prev, b2Vec2 curr, b2Vec2 next, const std::vector<b2Vec2> &vertices)
{
    if (isConvex(prev, curr, next))
    {
        for (int i = 0; i < vertices.size(); i++)
        {
            if (vertices[i] != prev && vertices[i] != curr && vertices[i] != next)
            {
                if (isInside(prev, curr, next, vertices[i]))
                {
                    return false;
                }
            }
        }
        return true;
    }
    return false;
}

/*
// Cria o vetor de vértices do polígono
std::vector<b2Vec2> vertices = {...}; // substitua por seus próprios vértices

// Triangula o polígono
std::vector<b2Vec2> triangles = triangulate(vertices);

// Cria um corpo físico Box2D
b2BodyDef bodyDef;
b2Body* body = world->CreateBody(&bodyDef);

// Cria uma forma de polígono Box2D para cada triângulo e adiciona ao corpo físico
for (int i = 0; i < triangles.size(); i += 3) {
    std::vector<b2Vec2> triangleVertices;
    triangleVertices.push_back(triangles[i]);
    triangleVertices.push_back(triangles[i + 1]);
    triangleVertices.push_back(triangles[i + 2]);
    b2PolygonShape triangleShape;
    triangleShape.Set(triangleVertices.data(), triangleVertices.size());
    body->CreateFixture(&triangleShape, 1.0f);
}
*/
std::vector<b2Vec2> triangulate(std::vector<b2Vec2> vertices)
{
    std::vector<b2Vec2> triangles;
    while (vertices.size() > 2)
    {
        for (int i = 0; i < vertices.size(); i++)
        {
            b2Vec2 prev = vertices[(i + vertices.size() - 1) % vertices.size()];
            b2Vec2 curr = vertices[i];
            b2Vec2 next = vertices[(i + 1) % vertices.size()];
            if (isEar(prev, curr, next, vertices))
            {
                triangles.push_back(prev);
                triangles.push_back(curr);
                triangles.push_back(next);
                vertices.erase(vertices.begin() + i);
                break;
            }
        }
    }
    return triangles;
}

b2Body *getBody(lua_State *L)
{
    b2Body **body_ptr = (b2Body **)luaL_checkudata(L, 1, "Body");
    if (body_ptr == nullptr || *body_ptr == nullptr)
    {
        luaL_error(L, "Invalid Body object");
    }
    return *body_ptr;
}

b2Body *getBodyAt(lua_State *L, int index)
{
    b2Body **body_ptr = (b2Body **)luaL_checkudata(L, index, "Body");
    if (body_ptr == nullptr || *body_ptr == nullptr)
    {
        luaL_error(L, "Invalid Body object");
    }
    return *body_ptr;
}

b2Fixture *getShape(lua_State *L)
{
    b2Fixture **shape_ptr = (b2Fixture **)luaL_checkudata(L, 1, "Shape");
    if (shape_ptr == nullptr || *shape_ptr == nullptr)
    {
        luaL_error(L, "Invalid Shape object");
    }
    return *shape_ptr;
}

b2Fixture *getShapeAt(lua_State *L, int index)
{
    b2Fixture **shape_ptr = (b2Fixture **)luaL_checkudata(L, index, "Shape");
    if (shape_ptr == nullptr || *shape_ptr == nullptr)
    {
        luaL_error(L, "Invalid Shape object");
    }
    return *shape_ptr;
}

namespace nBody
{

    static int body_gc(lua_State *L)
    {
        b2Body **body_ptr = (b2Body **)luaL_checkudata(L, 1, "Body");
        if (body_ptr == nullptr || *body_ptr == nullptr)
        {
            luaL_error(L, "Invalid Body object");
        }
        b2Body *body = *body_ptr;
        Log(LOG_INFO, "Body free");
        world->DestroyBody(body);
        *body_ptr = nullptr;
        return 0;
    }

    static int body_remove(lua_State *L)
    {
        b2Body **body_ptr = (b2Body **)luaL_checkudata(L, 1, "Body");
        if (body_ptr == nullptr || *body_ptr == nullptr)
        {
            luaL_error(L, "Invalid Body object");
        }
        b2Body *body = *body_ptr;
        Log(LOG_INFO, "Body Remove");
        bodysScheduledForRemoval.push_back(body);
        // world->DestroyBody(body);
        *body_ptr = nullptr;
        return 0;
    }

    static int body_add_box(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }

        b2BodyType type = body->GetType();

        b2PolygonShape shape;
        b2FixtureDef FixtureDef;

        if (lua_gettop(L) == 3)
        {
            float w = luaL_checknumber(L, 2) / 2;
            float h = luaL_checknumber(L, 3) / 2;

            shape.SetAsBox(pixelToWorld(w), pixelToWorld(h));
        }
        else if (lua_gettop(L) == 6)
        {
            float w = luaL_checknumber(L, 2) / 2;
            float h = luaL_checknumber(L, 3) / 2;
            float x = luaL_checknumber(L, 4);
            float y = luaL_checknumber(L, 5);
            float angle = luaL_checknumber(L, 6);

            shape.SetAsBox(pixelToWorld(w), pixelToWorld(h), b2Vec2(pixelToWorld(x), pixelToWorld(y)), angle);
        }

        if (body->GetType() == b2_dynamicBody)
        {
            FixtureDef.density = 1.f;
            FixtureDef.friction = 0.7f;
        }

        FixtureDef.shape = &shape;
        b2Fixture *m_shape = body->CreateFixture(&FixtureDef);
        b2Fixture **userdata = (b2Fixture **)lua_newuserdata(L, sizeof(b2Fixture *));
        *userdata = m_shape;
        luaL_getmetatable(L, "Shape");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int body_add_circle(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }

        b2CircleShape shape;
        b2FixtureDef FixtureDef;

        if (body->GetType() == b2_dynamicBody)
        {
            FixtureDef.density = 1.f;
            FixtureDef.friction = 0.7f;
        }

        shape.m_radius = luaL_checknumber(L, 2) / SCALE;
        if (lua_gettop(L) == 4)
        {

            float x = luaL_checknumber(L, 3);
            float y = luaL_checknumber(L, 4);
            shape.m_p = b2Vec2(pixelToWorld(x), pixelToWorld(y));
        }

        FixtureDef.shape = &shape;
        b2Fixture *m_shape = body->CreateFixture(&FixtureDef);
        b2Fixture **userdata = (b2Fixture **)lua_newuserdata(L, sizeof(b2Fixture *));
        *userdata = m_shape;
        luaL_getmetatable(L, "Shape");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int body_add_line(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }

        b2EdgeShape shape;
        b2FixtureDef FixtureDef;

        if (body->GetType() == b2_dynamicBody)
        {
            FixtureDef.density = 1.f;
            FixtureDef.friction = 0.7f;
        }

        float x0 = luaL_checknumber(L, 2);
        float y0 = luaL_checknumber(L, 3);
        float x1 = luaL_checknumber(L, 4);
        float y1 = luaL_checknumber(L, 5);

        b2Vec2 vertices[2];
        vertices[0].Set(pixelToWorld(x0), pixelToWorld(y0));
        vertices[1].Set(pixelToWorld(x1), pixelToWorld(y1));
        shape.SetTwoSided(vertices[0], vertices[1]);

        FixtureDef.shape = &shape;
        b2Fixture *m_shape = body->CreateFixture(&FixtureDef);
        b2Fixture **userdata = (b2Fixture **)lua_newuserdata(L, sizeof(b2Fixture *));
        *userdata = m_shape;
        luaL_getmetatable(L, "Shape");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int body_get_mass(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }
        lua_pushnumber(L, body->GetMass());
        return 1;
    }

    static int body_get_inertia(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }
        lua_pushnumber(L, body->GetInertia());
        return 1;
    }

    static int body_get_linearVelocity(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }
        b2Vec2 velocity = body->GetLinearVelocity();
        lua_pushnumber(L, velocity.x);
        lua_pushnumber(L, velocity.y);
        return 2;
    }

    static int body_get_angularVelocity(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }
        lua_pushnumber(L, body->GetAngularVelocity());
        return 1;
    }

    static int body_get_position(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }
        Vector2 position = worldToPixel(body->GetPosition());
        lua_pushnumber(L, position.x);
        lua_pushnumber(L, position.y);
        return 2;
    }

    static int body_get_angle(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }
        lua_pushnumber(L, body->GetAngle());
        return 1;
    }

    static int body_get_worldCenter(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }
        b2Vec2 position = body->GetWorldCenter();
        lua_pushnumber(L, position.x);
        lua_pushnumber(L, position.y);
        return 2;
    }

    static int body_get_localCenter(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body object");
            return 1;
        }
        b2Vec2 position = body->GetLocalCenter();
        lua_pushnumber(L, position.x);
        lua_pushnumber(L, position.y);
        return 2;
    }

    static int body_get_linearDamping(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            return 1;
        }
        lua_pushnumber(L, body->GetLinearDamping());
        return 1;
    }

    static int body_get_angularDamping(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            return 1;
        }
        lua_pushnumber(L, body->GetAngularDamping());
        return 1;
    }

    static int body_get_gravityScale(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            return 1;
        }
        lua_pushnumber(L, body->GetGravityScale());
        return 1;
    }

    static int body_get_type(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            return 1;
        }
        switch (body->GetType())
        {
        case b2_staticBody:
            lua_pushstring(L, "static");
            break;
        case b2_kinematicBody:
            lua_pushstring(L, "kinematic");
            break;
        case b2_dynamicBody:
            lua_pushstring(L, "dynamic");
            break;
        default:
            lua_pushnil(L);
            break;
        }
        return 1;
    }

    static int body_get_bullet(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            return 1;
        }
        lua_pushboolean(L, body->IsBullet());
        return 1;
    }

    static int body_get_sleepingAllowed(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            return 1;
        }
        lua_pushboolean(L, body->IsSleepingAllowed());
        return 1;
    }

    static int body_get_awake(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            return 1;
        }
        lua_pushboolean(L, body->IsAwake());
        return 1;
    }

    static int body_get_fixedRotation(lua_State *L)
    {
        b2Body *body = getBody(L);
        if (!body)
        {
            lua_pushnil(L);
            return 1;
        }
        lua_pushboolean(L, body->IsFixedRotation());
        return 1;
    }

    void register_box2d_Body(lua_State *L)
    {

        luaL_newmetatable(L, "Body");

        lua_pushcfunction(L, body_gc);
        lua_setfield(L, -2, "__gc");

        lua_newtable(L);

        lua_pushcfunction(L, body_remove);
        lua_setfield(L, -2, "remove");

        lua_pushcfunction(L, body_add_box);
        lua_setfield(L, -2, "addBox");

        lua_pushcfunction(L, body_add_circle);
        lua_setfield(L, -2, "addCircle");

        lua_pushcfunction(L, body_add_line);
        lua_setfield(L, -2, "addLine");

        lua_pushcfunction(L, body_get_mass);
        lua_setfield(L, -2, "getMass");

        lua_pushcfunction(L, body_get_inertia);
        lua_setfield(L, -2, "getInertia");

        lua_pushcfunction(L, body_get_linearVelocity);
        lua_setfield(L, -2, "getLinearVelocity");

        lua_pushcfunction(L, body_get_angularVelocity);
        lua_setfield(L, -2, "getAngularVelocity");

        lua_pushcfunction(L, body_get_position);
        lua_setfield(L, -2, "getPosition");

        lua_pushcfunction(L, body_get_angle);
        lua_setfield(L, -2, "getAngle");

        lua_pushcfunction(L, body_get_localCenter);
        lua_setfield(L, -2, "getLocalCenter");

        lua_pushcfunction(L, body_get_linearDamping);
        lua_setfield(L, -2, "getLinearDamping");

        lua_pushcfunction(L, body_get_angularDamping);
        lua_setfield(L, -2, "getAngularDamping");

        lua_pushcfunction(L, body_get_gravityScale);
        lua_setfield(L, -2, "getGravityScale");

        lua_pushcfunction(L, body_get_type);
        lua_setfield(L, -2, "getType");

        lua_pushcfunction(L, body_get_bullet);
        lua_setfield(L, -2, "getBullet");

        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);
    }
}

namespace nShape
{

    static int shape_remove(lua_State *L)
    {
        // b2Shape *shape = (b2Shape *)lua_touserdata(L, 1);
        // b2Body *body = shape->GetBody();
        // body->DestroyFixture(shape);
        return 0;
    }

    static int shape_set_friction(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[setFriction] Invalid Shape object");
            return 1;
        }

        float friction = luaL_checknumber(L, 2);
        shape->SetFriction(friction);
        return 0;
    }
    static int shape_get_friction(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[getFriction] Invalid Shape object");
            return 1;
        }

        float friction = shape->GetFriction();
        lua_pushnumber(L, friction);
        return 1;
    }

    static int shape_set_density(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[setDensity] Invalid Shape object");
            return 1;
        }

        float density = luaL_checknumber(L, 2);
        shape->SetDensity(density);
        return 0;
    }

    static int shape_get_density(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[getDensity] Invalid Shape object");
            return 1;
        }
        float density = luaL_checknumber(L, 2);
        shape->SetDensity(density);
        return 0;
    }

    static int shape_set_restitution(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[setRestitution] Invalid Shape object");
            return 1;
        }

        float restitution = luaL_checknumber(L, 2);
        shape->SetRestitution(restitution);
        return 0;
    }
    static int shape_get_restitution(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[getRestitution] Invalid Shape object");
            return 1;
        }

        float restitution = shape->GetRestitution();
        lua_pushnumber(L, restitution);
        return 1;
    }

    static int shape_set_sensor(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[setSensor] Invalid Shape object");
            return 1;
        }

        bool sensor = lua_toboolean(L, 2);
        shape->SetSensor(sensor);
        return 0;
    }

    static int shape_get_sensor(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[getSensor] Invalid Shape object");
            return 1;
        }

        bool sensor = shape->IsSensor();
        lua_pushboolean(L, sensor);
        return 1;
    }

    static int shape_set_category(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[setCategory] Invalid Shape object");
            return 1;
        }

        b2Filter filter = shape->GetFilterData();
        filter.categoryBits = luaL_checknumber(L, 2);
        shape->SetFilterData(filter);
        return 0;
    }
    static int shape_get_category(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[getCategory] Invalid Shape object");
            return 1;
        }
        b2Filter filter = shape->GetFilterData();
        int category = filter.categoryBits;
        lua_pushnumber(L, category);
        return 1;
    }

    static int shape_set_mask(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[setMask] Invalid Shape object");
            return 1;
        }

        b2Filter filter = shape->GetFilterData();
        filter.maskBits = luaL_checknumber(L, 2);

        shape->SetFilterData(filter);
        return 0;
    }

    static int shape_get_mask(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[getMask] Invalid Shape object");
            return 1;
        }

        int mask = shape->GetFilterData().maskBits;
        lua_pushnumber(L, mask);
        return 1;
    }

    static int shape_set_group(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[serGroup] Invalid Shape object");
            return 1;
        }
        b2Filter filter = shape->GetFilterData();
        filter.groupIndex = luaL_checknumber(L, 2);
        shape->SetFilterData(filter);
        return 0;
    }

    static int shape_get_group(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[getGroup] Invalid Shape object");
            return 1;
        }
        b2Filter filter = shape->GetFilterData();
        int group = filter.groupIndex;
        lua_pushnumber(L, group);
        return 1;
    }

    static int shap_get_filter(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[getFilter] Invalid Shape object");
            return 1;
        }
        b2Filter filter = shape->GetFilterData();
        lua_pushnumber(L, filter.categoryBits);
        lua_pushnumber(L, filter.maskBits);
        lua_pushnumber(L, filter.groupIndex);
        return 3;
    }

    static int shpe_set_filter(lua_State *L)
    {
        b2Fixture *shape = getShape(L);
        if (!shape)
        {
            lua_pushboolean(L, false);
            luaL_error(L, "[setFilter] Invalid Shape object");
            return 1;
        }
        b2Filter filter = shape->GetFilterData();
        filter.categoryBits = luaL_checknumber(L, 2);
        filter.maskBits = luaL_checknumber(L, 3);
        filter.groupIndex = luaL_checknumber(L, 4);
        shape->SetFilterData(filter);
        return 0;
    }

    void register_box2d_Shape(lua_State *L)
    {

        luaL_newmetatable(L, "Shape");

        lua_newtable(L);

        lua_pushcfunction(L, shape_remove);
        lua_setfield(L, -2, "remove");

        lua_pushcfunction(L, shape_set_friction);
        lua_setfield(L, -2, "setFriction");

        lua_pushcfunction(L, shape_set_density);
        lua_setfield(L, -2, "setDensity");

        lua_pushcfunction(L, shape_set_restitution);
        lua_setfield(L, -2, "setRestitution");

        lua_pushcfunction(L, shape_set_sensor);
        lua_setfield(L, -2, "setSensor");

        lua_pushcfunction(L, shape_set_category);
        lua_setfield(L, -2, "setCategory");

        lua_pushcfunction(L, shape_set_mask);
        lua_setfield(L, -2, "setMask");

        lua_pushcfunction(L, shape_set_group);
        lua_setfield(L, -2, "setGroup");

        lua_pushcfunction(L, shape_get_friction);
        lua_setfield(L, -2, "getFriction");

        lua_pushcfunction(L, shape_get_density);
        lua_setfield(L, -2, "getDensity");

        lua_pushcfunction(L, shape_get_restitution);
        lua_setfield(L, -2, "getRestitution");

        lua_pushcfunction(L, shape_get_sensor);
        lua_setfield(L, -2, "getSensor");

        lua_pushcfunction(L, shape_get_category);
        lua_setfield(L, -2, "getCategory");

        lua_pushcfunction(L, shape_get_mask);
        lua_setfield(L, -2, "getMask");

        lua_pushcfunction(L, shape_get_group);
        lua_setfield(L, -2, "getGroup");

        lua_pushcfunction(L, shap_get_filter);
        lua_setfield(L, -2, "getFilter");

        lua_pushcfunction(L, shpe_set_filter);
        lua_setfield(L, -2, "setFilter");

        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);
    }
}

namespace nJoint
{

    //*********************************************************************************************************************
    //*********************************************************************************************************************
    //*****************************************       DISTANCE JOINT       ************************************************
    //*********************************************************************************************************************
    //*********************************************************************************************************************

    static int distance_joint_gc(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "DistanceJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid DistanceJoint object");
        }
        b2Joint *joint = *joint_ptr;
        Log(LOG_INFO, "Joint free");
        world->DestroyJoint(joint);
        return 0;
    }
    static int distance_joint_remove(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "DistanceJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid DistanceJoint object");
        }
        b2Joint *joint = *joint_ptr;
        jointsScheduledForRemoval.push_back(joint);
        Log(LOG_INFO, "Joint remove");
        *joint_ptr = nullptr;
        return 0;
    }

    void register_distance_joint(lua_State *L)
    {

        luaL_newmetatable(L, "DistanceJoint");
        lua_pushcfunction(L, distance_joint_gc);
        lua_setfield(L, -2, "__gc");
        lua_newtable(L);

        lua_pushcfunction(L, distance_joint_remove);
        lua_setfield(L, -2, "remove");

        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);
    }

    //*********************************************************************************************************************
    //*********************************************************************************************************************
    //*****************************************       REVOLT JOINT         ************************************************
    //*********************************************************************************************************************
    //*********************************************************************************************************************

    static int revolt_joint_gc(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "RevoltJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Revolt Joint object");
        }
        b2Joint *joint = *joint_ptr;
        TraceLog(LOG_INFO, "Joint free");
        world->DestroyJoint(joint);
        return 0;
    }
    static int revolt_joint_remove(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "RevoltJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Revolt Joint object");
        }
        b2Joint *joint = *joint_ptr;
        jointsScheduledForRemoval.push_back(joint);
        *joint_ptr = nullptr;
        return 0;
    }

    void register_revolt_joint(lua_State *L)
    {

        luaL_newmetatable(L, "RevoltJoint");
        lua_pushcfunction(L, revolt_joint_gc);
        lua_setfield(L, -2, "__gc");
        lua_newtable(L);

        lua_pushcfunction(L, revolt_joint_remove);
        lua_setfield(L, -2, "remove");

        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);
    }

    //*********************************************************************************************************************
    //*********************************************************************************************************************
    //*****************************************       WHELL JOINT         ************************************************
    //*********************************************************************************************************************
    //*********************************************************************************************************************

    static int whell_joint_set_motor_speed(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        uJoint->SetMotorSpeed(luaL_checknumber(L, 2));
        return 0;
    }

    static int whell_joint_get_motor_speed(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        lua_pushnumber(L, uJoint->GetMotorSpeed());
        return 1;
    }

    static int whell_joint_set_motor(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        uJoint->EnableMotor(lua_toboolean(L, 2));
        return 0;
    }
    static int whell_joint_get_max_torque(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }
        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        lua_pushnumber(L, uJoint->GetMaxMotorTorque());
        return 1;
    }
    static int whell_joint_get_torque(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }
        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        lua_pushnumber(L, uJoint->GetMotorTorque(luaL_checknumber(L, 2)));
        return 1;
    }

    static int whell_joint_set_max_torque(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        uJoint->SetMaxMotorTorque(luaL_checknumber(L, 2));
        return 0;
    }

    static int whell_joint_set_limit(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }
        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        uJoint->EnableLimit(lua_toboolean(L, 2));
        return 0;
    }
    static int whell_joint_set_limits(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }
        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        uJoint->SetLimits(luaL_checknumber(L, 2), luaL_checknumber(L, 3));
        return 0;
    }
    static int whell_joint_set_damping(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        uJoint->SetDamping(luaL_checknumber(L, 2));
        return 0;
    }

    static int whell_joint_get_damping(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }
        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        lua_pushnumber(L, uJoint->GetDamping());
        return 1;
    }
    static int whell_joint_set_stiffness(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        uJoint->SetStiffness(luaL_checknumber(L, 2));
        return 0;
    }

    static int whell_joint_get_stiffness(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        lua_pushnumber(L, uJoint->GetStiffness());
        return 1;
    }

    static int whell_joint_get_joint_translation(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        lua_pushnumber(L, uJoint->GetJointTranslation());
        return 1;
    }

    static int whell_joint_get_joint_linear_speed(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        lua_pushnumber(L, uJoint->GetJointLinearSpeed());
        return 1;
    }

    static int whell_joint_get_joint_angle(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        lua_pushnumber(L, uJoint->GetJointAngle());
        return 1;
    }

    static int whell_joint_get_joint_angular_speed(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        lua_pushnumber(L, uJoint->GetJointAngularSpeed());
        return 1;
    }

    static int whell_joint_set_max_motor_torque(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        uJoint->SetMaxMotorTorque(luaL_checknumber(L, 2));
        return 0;
    }

    static int whell_joint_get_max_motor_torque(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Joint object");
        }

        b2WheelJoint *uJoint = (b2WheelJoint *)*joint_ptr;
        lua_pushnumber(L, uJoint->GetMaxMotorTorque());
        return 1;
    }

    static int wheel_joint_gc(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "Wheel Joint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }
        b2Joint *joint = *joint_ptr;
        TraceLog(LOG_INFO, "Joint free");
        world->DestroyJoint(joint);
        return 0;
    }
    static int wheel_joint_remove(lua_State *L)
    {
        b2Joint **joint_ptr = (b2Joint **)luaL_checkudata(L, 1, "WheelJoint");
        if (joint_ptr == nullptr || *joint_ptr == nullptr)
        {
            return luaL_error(L, "Invalid Wheel Joint object");
        }
        b2Joint *joint = *joint_ptr;
        jointsScheduledForRemoval.push_back(joint);
        return 0;
    }

    void register_wheel_joint(lua_State *L)
    {

        luaL_newmetatable(L, "WheelJoint");
        lua_pushcfunction(L, wheel_joint_gc);
        lua_setfield(L, -2, "__gc");
        lua_newtable(L);

        lua_pushcfunction(L, wheel_joint_remove);
        lua_setfield(L, -2, "remove");

        lua_pushcfunction(L, whell_joint_set_limit);
        lua_setfield(L, -2, "setLimitsEnabled");

        lua_pushcfunction(L, whell_joint_set_limits);
        lua_setfield(L, -2, "setLimits");

        lua_pushcfunction(L, whell_joint_set_motor_speed);
        lua_setfield(L, -2, "setMotorSpeed");
        lua_pushcfunction(L, whell_joint_get_motor_speed);
        lua_setfield(L, -2, "getMotorSpeed");
        lua_pushcfunction(L, whell_joint_set_max_motor_torque);
        lua_setfield(L, -2, "setMaxMotorTorque");
        lua_pushcfunction(L, whell_joint_get_max_motor_torque);
        lua_setfield(L, -2, "getMaxMotorTorque");
        lua_pushcfunction(L, whell_joint_set_motor);
        lua_setfield(L, -2, "setMotorEnabled");
        lua_pushcfunction(L, whell_joint_set_damping);
        lua_setfield(L, -2, "setDamping");
        lua_pushcfunction(L, whell_joint_get_damping);
        lua_setfield(L, -2, "getDamping");
        lua_pushcfunction(L, whell_joint_set_stiffness);
        lua_setfield(L, -2, "setStiffness");
        lua_pushcfunction(L, whell_joint_get_stiffness);
        lua_setfield(L, -2, "getStiffness");
        lua_pushcfunction(L, whell_joint_get_joint_translation);
        lua_setfield(L, -2, "getJointTranslation");
        lua_pushcfunction(L, whell_joint_get_joint_linear_speed);
        lua_setfield(L, -2, "getJointLinearSpeed");
        lua_pushcfunction(L, whell_joint_get_joint_angle);
        lua_setfield(L, -2, "getJointAngle");
        lua_pushcfunction(L, whell_joint_get_joint_angular_speed);
        lua_setfield(L, -2, "getJointAngularSpeed");

        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);
    }
}

namespace nWorld
{

    static int world_new_RevoluteJoint(lua_State *L)
    {
        b2Body *bodyA = getBodyAt(L, 1);
        if (!bodyA)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body args0 object");
            return 1;
        }
        b2Body *bodyB = getBodyAt(L, 2);
        if (!bodyB)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body args1 object");
            return 1;
        }
        float x = Get_float(L, 3);
        float y = Get_float(L, 4);
        bool collideConnected = Get_bool(L, 5);

        b2RevoluteJointDef jd;
        jd.Initialize(bodyA, bodyB, pixelToWorld(x, y));
        jd.collideConnected = collideConnected;

        b2RevoluteJoint *joint = (b2RevoluteJoint *)world->CreateJoint(&jd);

        b2RevoluteJoint **userdata = (b2RevoluteJoint **)lua_newuserdata(L, sizeof(b2RevoluteJoint *));
        *userdata = joint;
        luaL_getmetatable(L, "RevoluteJoint");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int world_new_DistanceJoint(lua_State *L)
    {
        b2Body *bodyA = getBodyAt(L, 1);
        if (!bodyA)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body args0 object");
            return 1;
        }
        b2Body *bodyB = getBodyAt(L, 2);
        if (!bodyB)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body args1 object");
            return 1;
        }
        float x = Get_float(L, 3);
        float y = Get_float(L, 4);
        float ax = Get_float(L, 4);
        float ay = Get_float(L, 5);
        bool collideConnected = Get_bool(L, 6);

        b2DistanceJointDef jd;
        jd.collideConnected = collideConnected;
        jd.Initialize(bodyA, bodyB, pixelToWorld(x, y), pixelToWorld(ax, ay));

        b2DistanceJoint *joint = (b2DistanceJoint *)world->CreateJoint(&jd);

        b2DistanceJoint **userdata = (b2DistanceJoint **)lua_newuserdata(L, sizeof(b2DistanceJoint *));
        *userdata = joint;
        luaL_getmetatable(L, "DistanceJoint");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int world_new_PrismaticJoint(lua_State *L)
    {
        b2Body *bodyA = getBodyAt(L, 1);
        if (!bodyA)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body args0 object");
            return 1;
        }
        b2Body *bodyB = getBodyAt(L, 2);
        if (!bodyB)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body args1 object");
            return 1;
        }
        float x = Get_float(L, 3);
        float y = Get_float(L, 4);
        float ax = Get_float(L, 4);
        float ay = Get_float(L, 5);

        bool collideConnected = Get_bool(L, 6);

        b2PrismaticJointDef jd;
        jd.Initialize(bodyA, bodyB, pixelToWorld(x, y), pixelToWorld(ax, ay));
        jd.collideConnected = collideConnected;

        b2PrismaticJoint *joint = (b2PrismaticJoint *)world->CreateJoint(&jd);

        b2PrismaticJoint **userdata = (b2PrismaticJoint **)lua_newuserdata(L, sizeof(b2PrismaticJoint *));
        *userdata = joint;
        luaL_getmetatable(L, "PrismaticJoint");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int world_new_WheelJoint(lua_State *L)
    {
        b2Body *bodyA = getBodyAt(L, 1);
        if (!bodyA)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body args0 object");
            return 1;
        }
        b2Body *bodyB = getBodyAt(L, 2);
        if (!bodyB)
        {
            lua_pushnil(L);
            luaL_error(L, "Invalid Body args1 object");
            return 1;
        }
        float x = Get_float(L, 3);
        float y = Get_float(L, 4);
        float ax = Get_float(L, 5);
        float ay = Get_float(L, 6);
        bool collideConnected = Get_bool(L, 7);

        b2WheelJointDef jd;
        jd.collideConnected = collideConnected;
        jd.Initialize(bodyA, bodyB, pixelToWorld(x, y), b2Vec2(ax, ay));
        b2WheelJoint *joint = (b2WheelJoint *)world->CreateJoint(&jd);
        b2WheelJoint **userdata = (b2WheelJoint **)lua_newuserdata(L, sizeof(b2WheelJoint *));
        *userdata = joint;
        luaL_getmetatable(L, "WheelJoint");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int world_newBody(lua_State *L)
    {

        int type = Get_int(L, 1);

        b2BodyDef bodyDef;

        if (type == 0)
            bodyDef.type = b2_dynamicBody;
        else if (type == 1)
            bodyDef.type = b2_staticBody;
        else if (type == 2)
            bodyDef.type = b2_kinematicBody;

        float x = 0;
        float y = 0;
        float angle = 0;

        if (lua_gettop(L) == 3)
        {
            x = Get_float(L, 2);
            y = Get_float(L, 3);
            bodyDef.angle = degreesToRadians(angle);
            bodyDef.position = pixelToWorld(x, y);
        }
        else if (lua_gettop(L) == 4)
        {
            x = Get_float(L, 2);
            y = Get_float(L, 3);
            angle = Get_float(L, 4);
            bodyDef.angle = degreesToRadians(angle);
            bodyDef.position = pixelToWorld(x, y);
        }

        b2Body *body = world->CreateBody(&bodyDef);

        b2Body **userdata = (b2Body **)lua_newuserdata(L, sizeof(b2Body *));
        *userdata = body;
        luaL_getmetatable(L, "Body");
        lua_setmetatable(L, -2);

        return 1;
    }

    static int world_update(lua_State *L)
    {
        if (!world)
            return 0;

        float timeStep = Get_float(L, 1);

        world->Step(timeStep, velocityIterations, positionIterations);

        if (jointsScheduledForRemoval.size() > 0)
        {
            for (auto joint : jointsScheduledForRemoval)
            {
                auto it = std::find(jointsScheduledForRemoval.begin(), jointsScheduledForRemoval.end(), joint);
                if (it != jointsScheduledForRemoval.end())
                {
                    world->DestroyJoint(joint);
                }
            }
            jointsScheduledForRemoval.clear();
        }

        if (bodysScheduledForRemoval.size() > 0)
        {
            for (auto body : bodysScheduledForRemoval)
            {
                auto it = std::find(bodysScheduledForRemoval.begin(), bodysScheduledForRemoval.end(), body);
                if (it != bodysScheduledForRemoval.end())
                {
                    world->DestroyBody(body);
                }
            }
            bodysScheduledForRemoval.clear();
        }

        return 0;
    }

    class QueryCallback : public b2QueryCallback
    {
    public:
        QueryCallback(const b2Vec2 &point)
        {
            m_point = point;
            m_fixture = NULL;
        }

        bool ReportFixture(b2Fixture *fixture) override
        {
            b2Body *body = fixture->GetBody();
            if (body->GetType() == b2_dynamicBody)
            {
                bool inside = fixture->TestPoint(m_point);
                if (inside)
                {
                    m_fixture = fixture;

                    // We are done, terminate the query.
                    return false;
                }
            }

            // Continue the query.
            return true;
        }

        b2Vec2 m_point;
        b2Fixture *m_fixture;
    };
    b2MouseJoint *m_mouseJoint;

    static int world_draw(lua_State *L)
    {

        if (!world)
            return 0;

        // b2Vec2 p = pixelToWorld(GetMouseX() / _camera.zoom - _camera.offset.x + _camera.target.x,
        //                         GetMouseY() / _camera.zoom - _camera.offset.y + _camera.target.y);

        // b2AABB aabb;
        // b2Vec2 d;
        // d.Set(0.001f, 0.001f);
        // aabb.lowerBound = p - d;
        // aabb.upperBound = p + d;

        // if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && !m_mouseJoint)
        // {
        //     QueryCallback callback(p);
        //     world->QueryAABB(&callback, aabb);

        //     if (callback.m_fixture)
        //     {
        //         float frequencyHz = 5.0f;
        //         float dampingRatio = 0.7f;

        //         b2Body *body = callback.m_fixture->GetBody();
        //         b2MouseJointDef jd;
        //         jd.bodyA = groundBody;
        //         jd.bodyB = body;
        //         jd.target = p;
        //         jd.maxForce = 1000.0f * body->GetMass();
        //         b2LinearStiffness(jd.stiffness, jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);

        //         m_mouseJoint = (b2MouseJoint *)world->CreateJoint(&jd);
        //         body->SetAwake(true);
        //     }
        // }

        // if (m_mouseJoint)
        // {
        //     m_mouseJoint->SetTarget(p);
        // }

        // if (IsMouseButtonUp(MOUSE_BUTTON_RIGHT))
        // {
        //     if (m_mouseJoint)
        //     {
        //         world->DestroyJoint(m_mouseJoint);
        //         m_mouseJoint = NULL;
        //     }
        // }

        world->DebugDraw();
        return 0;
    }

    static int world_set_gravity(lua_State *L)
    {
        if (!world)
            return 0;
        float x = luaL_checknumber(L, 1);
        float y = luaL_checknumber(L, 2);
        world->SetGravity(b2Vec2(x, y));

        return 0;
    }

  static int world_debug_fill(lua_State *L)
    {

        bool mode = lua_toboolean(L, 1);

        drawFilled = mode;

        return 0;
    }

}

void register_box_physics(lua_State *L)
{
    using namespace nWorld;

    luaL_Reg reg[] =
        {
            {"setGravity", world_set_gravity},
            {"setDebugFill", world_debug_fill},
            {"newBody", world_newBody},
            {"newRevoluteJoint", world_new_RevoluteJoint},
            {"newDistanceJoint", world_new_DistanceJoint},
            {"newPrismaticJoint", world_new_PrismaticJoint},
            {"newWheelJoint", world_new_WheelJoint},
            {"update", world_update},
            {"draw", world_draw},
            {0, 0},
        };
    luaL_newlib(L, reg);
}

void clear_box2d()
{
    if (world != nullptr)
    {
        delete world;
        world = nullptr;
    }
    Log(LOG_INFO, "Initializing Box2D");
    world = new b2World(b2Vec2(0.0f, 9.8f));
    world->SetDebugDraw(&debugDraw);
    uint32 flags = 0;
    flags += b2Draw::e_shapeBit;
    flags += b2Draw::e_jointBit;
    //   flags += b2Draw::e_aabbBit;
    //    flags += b2Draw::e_pairBit;
    flags += b2Draw::e_centerOfMassBit;

    debugDraw.SetFlags(flags);
}

void init_box2d()
{
    Log(LOG_INFO, "Initializing Box2D");
    world = new b2World(b2Vec2(0.0f, 9.8f));
    world->SetDebugDraw(&debugDraw);
    uint32 flags = 0;
    flags += b2Draw::e_shapeBit;
    flags += b2Draw::e_jointBit;
    //   flags += b2Draw::e_aabbBit;
    //    flags += b2Draw::e_pairBit;
    flags += b2Draw::e_centerOfMassBit;

    debugDraw.SetFlags(flags);
    // nWorld::CreateBox(*world, 200, 200);
    // nWorld::CreateGround(*world, 400.f, 500.f);

    b2BodyDef bodyDef;
    groundBody = world->CreateBody(&bodyDef);
}
void free_box2d()
{
    Log(LOG_INFO, "Freeing Box2D");
    delete world;
}

int luaclass_box2d(lua_State *L)
{
    nBody::register_box2d_Body(L);
    nShape::register_box2d_Shape(L);

    nJoint::register_distance_joint(L);
    nJoint::register_revolt_joint(L);
    nJoint::register_wheel_joint(L);

    return 0;
}

int luaopen_box2d(lua_State *L)
{
    register_box_physics(L);
    return 0;
}

#endif