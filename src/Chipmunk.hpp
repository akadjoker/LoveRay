#ifndef CHIPPHYSICS_H
#define CHIPPHYSICS_H

#include <chipmunk/chipmunk_private.h>
#include <chipmunk/chipmunk.h>
#include <lua.hpp>
#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>

#define GRABBABLE_MASK_BIT (1 << 31)

int luaopen_chipmunk(lua_State *L);
int luaclass_chipmunk(lua_State *L);

void init_chipmunk();
void free_chipmunk();

#endif

#ifdef CHIP_PHYSICS_IMPLEMENTATION

// cpShapeFilter GRAB_FILTER = {CP_NO_GROUP, GRABBABLE_MASK_BIT, GRABBABLE_MASK_BIT};
// cpShapeFilter NOT_GRABBABLE_FILTER = {CP_NO_GROUP, ~GRABBABLE_MASK_BIT, ~GRABBABLE_MASK_BIT};

cpShapeFilter getShapeFilter(lua_State *L)
{

    cpShapeFilter filter;

    if (!lua_istable(L, -1))
    {
        luaL_error(L, "Invalid ShapeFilter table");
        return filter;
    }

    lua_getfield(L, -1, "group");
    filter.group = luaL_checkinteger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "categories");
    filter.categories = luaL_checkinteger(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "mask");
    filter.mask = luaL_checkinteger(L, -1);
    lua_pop(L, 1);

    return filter;
}


cpTransform getTransform(lua_State *L)
{

    cpTransform cpTransform;

    if (!lua_istable(L, -1))
    {
        luaL_error(L, "Invalid Transform table");
        return cpTransform;
    }

    // lua_getfield(L, -1, "group");
    // filter.group = luaL_checkinteger(L, -1);
    // lua_pop(L, 1);

    // lua_getfield(L, -1, "categories");
    // filter.categories = luaL_checkinteger(L, -1);
    // lua_pop(L, 1);

    // lua_getfield(L, -1, "mask");
    // filter.mask = luaL_checkinteger(L, -1);
    // lua_pop(L, 1);

    return cpTransform;
}

cpBody *getBody(lua_State *L)
{
    cpBody **body_ptr = (cpBody **)luaL_checkudata(L, 1, "Body");
    if (body_ptr == nullptr || *body_ptr == nullptr)
    {
        luaL_error(L, "Invalid Body object");
    }
    return *body_ptr;
}

cpBody *getBodyAt(lua_State *L, int index)
{
    cpBody **body_ptr = (cpBody **)luaL_checkudata(L, index, "Body");
    if (body_ptr == nullptr || *body_ptr == nullptr)
    {
        luaL_error(L, "Invalid Body object");
    }
    return *body_ptr;
}

cpShape *getShape(lua_State *L)
{

    cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
    if (shape_ptr == nullptr || *shape_ptr == nullptr)
    {
        luaL_error(L, "Invalid Shape object");
    }
    return *shape_ptr;
}

cpConstraint *getJoint(lua_State *L)
{

    cpConstraint **joint_ptr = (cpConstraint **)luaL_checkudata(L, 1, "Joint");
    if (joint_ptr == nullptr || *joint_ptr == nullptr)
    {
        luaL_error(L, "Invalid Joint object");
    }
    return *joint_ptr;
}

cpConstraint *getJointAt(lua_State *L, int index)
{

    cpConstraint **joint_ptr = (cpConstraint **)luaL_checkudata(L, index, "Joint");
    if (joint_ptr == nullptr || *joint_ptr == nullptr)
    {
        luaL_error(L, "Invalid Joint object");
    }
    return *joint_ptr;
}
//***********************************************************************************
//***********************************************************************************
//                                DEBUG
//***********************************************************************************
//***********************************************************************************

namespace ChipmunkDebug
{
    static bool drawFilled = false;
    cpSpaceDebugColor RGBAColor(float r, float g, float b, float a)
    {
        cpSpaceDebugColor color = {r, g, b, a};
        return color;
    }

    cpSpaceDebugColor LAColor(float l, float a)
    {
        cpSpaceDebugColor color = {l, l, l, a};
        return color;
    }

    Color newColor(float r, float g, float b, float a)
    {
        Color c;
        c.r = r * 255.0f;
        c.g = g * 255.0f;
        c.b = b * 255.0f;
        c.a = a * 255.0f;
        return c;
    }

    Color cColot2Color(const cpSpaceDebugColor &color)
    {
        Color c;
        c.r = color.r * 255.0f;
        c.g = color.g * 255.0f;
        c.b = color.b * 255.0f;
        c.a = color.a * 255.0f;
        return c;
    }

    static inline unsigned char uclerp(unsigned char f1, unsigned char f2, unsigned char t)
    {
        return f1 * (255 - t) + f2 * t;
    }

    static void cDrawDot(cpFloat size, cpVect pos, cpSpaceDebugColor color, cpDataPointer data)
    {
        (void)data;
        DrawCircle(pos.x, pos.y, size, cColot2Color(color));
    }

    static void cDrawSegment(cpVect a, cpVect b, cpSpaceDebugColor color, cpDataPointer data)
    {
        (void)data;

        int linex0 = (int)a.x;
        int liney0 = (int)a.y;
        int linex2 = (int)b.x;
        int liney2 = (int)b.y;

        DrawLine(linex0, liney0, linex2, liney2, cColot2Color(color));
    }

    static void cDrawFatSegment(cpVect _a, cpVect _b, cpFloat r, cpSpaceDebugColor outline, cpSpaceDebugColor fill, cpDataPointer data)
    {
        (void)data;
        Vector2 a;
        a.x = _a.x;
        a.y = _a.y;
        Vector2 b;
        b.x = _b.x;
        b.y = _b.y;

        Vector2 ab = Vector2Subtract(b, a);
        Vector2 normal;
        normal.x = -ab.y;
        normal.y = ab.x;

        normal = Vector2Normalize(normal);
        normal = Vector2Scale(normal, r);

        // Calcula os vértices do segmento grosso
        Vector2 v1 = Vector2Add(a, normal);
        Vector2 v2 = Vector2Add(b, normal);
        Vector2 v3 = Vector2Subtract(b, normal);
        Vector2 v4 = Vector2Subtract(a, normal);

        // Desenha o preenchimento do segmento
        if (drawFilled)
        {
            DrawTriangle(v1, v2, v3, cColot2Color(fill));
            DrawTriangle(v1, v3, v4, cColot2Color(fill));
            DrawTriangleLines(v1, v2, v3, cColot2Color(fill));
            DrawTriangleLines(v1, v3, v4, cColot2Color(fill));
        }

        if (!drawFilled)
        {

            DrawTriangleLines(v1, v2, v3, cColot2Color(outline));
            DrawTriangleLines(v1, v3, v4, cColot2Color(outline));
        }
    }

    static void
    cDrawCircle(cpVect center, cpFloat angle, cpFloat radius, cpSpaceDebugColor outline, cpSpaceDebugColor fill, cpDataPointer data)
    {
        (void)data;
        if (drawFilled)
        {
            DrawCircle(center.x, center.y, radius, cColot2Color(fill));
            DrawCircleLines((int)center.x, (int)center.y, cpfmax(radius, 1.0), cColot2Color(outline));
        }
        else
            DrawCircleLines((int)center.x, (int)center.y, cpfmax(radius, 1.0), cColot2Color(fill));
        cDrawSegment(center, cpvadd(center, cpvmult(cpvforangle(angle), 0.75f * radius)), outline, data);
        //   DrawLine(center.x,center.y,center.x+cos( angle )*radius,center.y+sin( angle )*radius ,cColot2Color(fill));
    }

    Vector2 vec2(float x, float y)
    {

        Vector2 v;
        v.x = x;
        v.y = y;
        return v;
    }

    static void
    cDrawPolygon(int count, const cpVect *verts, cpFloat r, cpSpaceDebugColor outline, cpSpaceDebugColor fill, cpDataPointer data)
    {
        (void)data;
        if (drawFilled)
        {
            if (count >= 3)
            {
                DrawLine(
                    verts[0].x, verts[0].y,
                    verts[count - 1].x, verts[count - 1].y, cColot2Color(outline));

                rlBegin(RL_QUADS);

                for (int i = 1; i < count - 1; i++)
                {

                    rlColor4f(fill.r, fill.g, fill.b, fill.a);

                    Vector2 v2 = vec2(verts[0].x, verts[0].y);
                    Vector2 v1 = vec2(verts[i].x, verts[i].y);
                    Vector2 v0 = vec2(verts[i + 1].x, verts[i + 1].y);
                    rlVertex2f(v0.x, v0.y);
                    rlVertex2f(v1.x, v1.y);
                    rlVertex2f(v2.x, v2.y);
                    rlVertex2f(v2.x, v2.y);
                }
                rlEnd();
            }
            for (int i = 0; i < count; i++)
            {
                int j = (i + 1) % count;

                Vector2 v1 = vec2(verts[i].x, verts[i].y);
                Vector2 v0 = vec2(verts[j].x, verts[j].y);

                DrawLine(v0.x, v0.y, v1.x, v1.y, cColot2Color(outline));
            }
        }
        else
        {
            for (int i = 0; i < count; i++)
            {
                int j = (i + 1) % count;

                Vector2 v1 = vec2(verts[i].x, verts[i].y);
                Vector2 v0 = vec2(verts[j].x, verts[j].y);

                DrawLine(v0.x, v0.y, v1.x, v1.y, cColot2Color(fill));
            }
        }
    }

    static cpSpaceDebugColor Colors[] = {
        {0xb5 / 255.0f, 0x89 / 255.0f, 0x00 / 255.0f, 1.0f},
        {0xcb / 255.0f, 0x4b / 255.0f, 0x16 / 255.0f, 1.0f},
        {0xdc / 255.0f, 0x32 / 255.0f, 0x2f / 255.0f, 1.0f},
        {0xd3 / 255.0f, 0x36 / 255.0f, 0x82 / 255.0f, 1.0f},
        {0x6c / 255.0f, 0x71 / 255.0f, 0xc4 / 255.0f, 1.0f},
        {0x26 / 255.0f, 0x8b / 255.0f, 0xd2 / 255.0f, 1.0f},
        {0x2a / 255.0f, 0xa1 / 255.0f, 0x98 / 255.0f, 1.0f},
        {0x85 / 255.0f, 0x99 / 255.0f, 0x00 / 255.0f, 1.0f},
    };

    static cpSpaceDebugColor ColorForShape(cpShape *shape, cpDataPointer data)
    {
        (void)data;
        if (cpShapeGetSensor(shape))
        {
            return LAColor(1.0f, 0.1f);
        }
        else
        {
            cpBody *body = cpShapeGetBody(shape);

            if (cpBodyIsSleeping(body))
            {
                return RGBAColor(0x58 / 255.0f, 0x6e / 255.0f, 0x75 / 255.0f, 0.4f);
            }
            else if (body->sleeping.idleTime > shape->space->sleepTimeThreshold)
            {
                return RGBAColor(0x93 / 255.0f, 0xa1 / 255.0f, 0xa1 / 255.0f, 1.0f);
            }
            else
            {
                uint32_t val = (uint32_t)shape->hashid;

                // scramble the bits up using Robert Jenkins' 32 bit integer hash function
                val = (val + 0x7ed55d16) + (val << 12);
                val = (val ^ 0xc761c23c) ^ (val >> 19);
                val = (val + 0x165667b1) + (val << 5);
                val = (val + 0xd3a2646c) ^ (val << 9);
                val = (val + 0xfd7046c5) + (val << 3);
                val = (val ^ 0xb55a4f09) ^ (val >> 16);
                return Colors[val & 0x7];
            }
        }
    }
}

//***********************************************************************************
//***********************************************************************************
//                                WORLD
//***********************************************************************************
//***********************************************************************************

using namespace ChipmunkDebug;

static void ShapeFreeWrap(cpSpace *space, cpShape *shape)
{
    cpSpaceRemoveShape(space, shape);
    cpShapeFree(shape);
}

static void PostShapeFree(cpShape *shape, cpSpace *space)
{
    cpSpaceAddPostStepCallback(space, (cpPostStepFunc)ShapeFreeWrap, shape, NULL);
}

static void ConstraintFreeWrap(cpSpace *space, cpConstraint *constraint)
{
    cpSpaceRemoveConstraint(space, constraint);
    cpConstraintFree(constraint);
}

static void
PostConstraintFree(cpConstraint *constraint, cpSpace *space)
{
    cpSpaceAddPostStepCallback(space, (cpPostStepFunc)ConstraintFreeWrap, constraint, NULL);
}

static void BodyFreeWrap(cpSpace *space, cpBody *body)
{
    cpSpaceRemoveBody(space, body);
    cpBodyFree(body);
}

static void PostBodyFree(cpBody *body, cpSpace *space)
{
    cpSpaceAddPostStepCallback(space, (cpPostStepFunc)BodyFreeWrap, body, NULL);
}

static void freeSpaceChildren(cpSpace *space)
{
    // Must remove these BEFORE freeing the body or you will access dangling pointers.
    cpSpaceEachShape(space, (cpSpaceShapeIteratorFunc)PostShapeFree, space);
    cpSpaceEachConstraint(space, (cpSpaceConstraintIteratorFunc)PostConstraintFree, space);
    cpSpaceEachBody(space, (cpSpaceBodyIteratorFunc)PostBodyFree, space);
}

cpSpace *m_space;
cpSpaceDebugDrawOptions drawOptions;

void init_chipmunk()
{
    m_space = cpSpaceNew();

    cpSpaceSetIterations(m_space, 5);

    cpSpaceSetSleepTimeThreshold(m_space, 0.5f);
    cpSpaceSetCollisionSlop(m_space, 0.5f);

    drawOptions =
        {
            cDrawCircle,
            cDrawSegment,
            cDrawFatSegment,
            cDrawPolygon,
            cDrawDot,

            (cpSpaceDebugDrawFlags)(CP_SPACE_DEBUG_DRAW_SHAPES | CP_SPACE_DEBUG_DRAW_CONSTRAINTS | CP_SPACE_DEBUG_DRAW_COLLISION_POINTS),

            {0xEE / 255.0f, 0xE8 / 255.0f, 0xD5 / 255.0f, 1.0f}, // Outline color
            ColorForShape,
            {0.0f, 0.75f, 0.0f, 1.0f}, // Constraint color
            {1.0f, 0.0f, 0.0f, 1.0f},  // Collision point color
            NULL,
        };

    cpSpaceDebugDraw(m_space, &drawOptions);
    Log(LOG_INFO, "Chipmunk initialized");
}

void free_chipmunk()
{
    if (!m_space)
        return;

    Log(LOG_INFO, "Chipmunk free");
    freeSpaceChildren(m_space);
    cpSpaceFree(m_space);
}

//***********************************************************************************
//***********************************************************************************
//                                WORLD
//***********************************************************************************
//***********************************************************************************

namespace ChipmunkWorld
{

    static int world_newDynamic(lua_State *L)
    {

        float x = 0;
        float y = 0;
        float angle = 0;
        float mass = 1;

        if (lua_gettop(L) == 2)
        {
            x = luaL_checknumber(L, 1);
            y = luaL_checknumber(L, 2);
        }
        else if (lua_gettop(L) == 3)
        {
            x = luaL_checknumber(L, 1);
            y = luaL_checknumber(L, 2);
            angle = luaL_checknumber(L, 3);
        }
        else if (lua_gettop(L) == 4)
        {
            x = luaL_checknumber(L, 1);
            y = luaL_checknumber(L, 2);
            angle = luaL_checknumber(L, 3);
            mass = luaL_checknumber(L, 4);
        }
        else
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid number of arguments");
        }

        cpBody *body = cpBodyNew(mass, INFINITY);
        cpBodySetPosition(body, cpv(x, y));
        cpBodySetAngle(body, angle);
        cpSpaceAddBody(m_space, body);

        cpBody **userdata = (cpBody **)lua_newuserdata(L, sizeof(cpBody *));
        *userdata = body;

        luaL_getmetatable(L, "Body");
        lua_setmetatable(L, -2);

        return 1;
    }

    static int world_getStatic(lua_State *L)
    {

        cpBody *body = cpSpaceGetStaticBody(m_space);
        cpBody **userdata = (cpBody **)lua_newuserdata(L, sizeof(cpBody *));
        *userdata = body;
        luaL_getmetatable(L, "Body");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int world_newStatic(lua_State *L)
    {

        float x = 0;
        float y = 0;
        float angle = 0;

        cpBody *body = cpBodyNewStatic(); // cpSpaceGetStaticBody(m_space);

        if (lua_gettop(L) == 3)
        {
            x = luaL_checknumber(L, 1);
            y = luaL_checknumber(L, 2);
        }
        else if (lua_gettop(L) == 4)
        {
            x = luaL_checknumber(L, 1);
            y = luaL_checknumber(L, 2);
            angle = luaL_checknumber(L, 3);
        }
        else
        {
            return luaL_error(L, "Invalid number of arguments %d ", lua_gettop(L));
        }

        // Log(LOG_INFO, "Static body created %f %f %f", x, y, angle);

        cpBodySetPosition(body, cpv(x, y));
        cpBodySetAngle(body, angle);
        cpSpaceAddBody(m_space, body);

        cpBody **userdata = (cpBody **)lua_newuserdata(L, sizeof(cpBody *));
        *userdata = body;

        luaL_getmetatable(L, "Body");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int world_newKinematic(lua_State *L)
    {

        float x = 0;
        float y = 0;
        float angle = 0;

        cpBody *body = cpBodyNewKinematic();

        if (lua_gettop(L) == 3)
        {
            x = luaL_checknumber(L, 2);
            y = luaL_checknumber(L, 3);
        }
        else if (lua_gettop(L) == 4)
        {
            x = luaL_checknumber(L, 2);
            y = luaL_checknumber(L, 3);
            angle = luaL_checknumber(L, 4);
        }
        else
        {
            return luaL_error(L, "Invalid number of arguments");
        }

        cpBodySetPosition(body, cpv(x, y));
        cpBodySetAngle(body, angle);
        cpSpaceAddBody(m_space, body);

        cpBody **userdata = (cpBody **)lua_newuserdata(L, sizeof(cpBody *));
        *userdata = body;

        luaL_getmetatable(L, "Body");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int world_update(lua_State *L)
    {
        if (!m_space)
            return 0;

        float dt = luaL_checknumber(L, 1);
        cpSpaceStep(m_space, dt);
        return 0;
    }

    static int world_draw(lua_State *L)
    {
        (void)L;
        if (!m_space)
            return 0;
        rlSetTexture(0);
        cpSpaceDebugDraw(m_space, &drawOptions);
        return 0;
    }

    static int world_clear(lua_State *L)
    {
        (void)L;
        if (!m_space)
            return 0;

        freeSpaceChildren(m_space);
        return 0;
    }

    static int world_set_gravity(lua_State *L)
    {
        if (!m_space)
            return 0;
        float x = luaL_checknumber(L, 1);
        float y = luaL_checknumber(L, 2);
        cpSpaceSetGravity(m_space, cpv(x, y));

        return 0;
    }

    // cpConstraint* cpGearJointNew(cpBody *a, cpBody *b, cpFloat phase, cpFloat ratio);
    static int world_new_gear_joint(lua_State *L)
    {
        cpBody *a = getBodyAt(L, 1);
        cpBody *b = getBodyAt(L, 2);

        if (!a)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at arg 1");
        }

        if (!b)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at agr 2");
        }

        float phase = luaL_checknumber(L, 3);
        float ratio = luaL_checknumber(L, 4);

        cpConstraint *joint = cpGearJointNew(a, b, phase, ratio);
        cpSpaceAddConstraint(m_space, joint);

        cpConstraint **userdata = (cpConstraint **)lua_newuserdata(L, sizeof(cpConstraint *));
        *userdata = joint;

        luaL_getmetatable(L, "Constraint");
        lua_setmetatable(L, -2);
        return 1;
    }

    // cpConstraint* cpGrooveJointNew(cpBody *a, cpBody *b, cpVect groove_a, cpVect groove_b, cpVect anchorB);
    static int world_new_groove_joint(lua_State *L)
    {
        cpBody *a = getBodyAt(L, 1);
        cpBody *b = getBodyAt(L, 2);

        if (!a)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at arg 1");
        }

        if (!b)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at agr 2");
        }
        cpVect groove_a;
        cpVect groove_b;
        cpVect anchorB;

        groove_a.x = luaL_checknumber(L, 3);
        groove_a.y = luaL_checknumber(L, 4);

        groove_b.x = luaL_checknumber(L, 5);
        groove_b.y = luaL_checknumber(L, 6);

        anchorB.x = luaL_checknumber(L, 7);
        anchorB.y = luaL_checknumber(L, 8);

        cpConstraint *joint = cpGrooveJointNew(a, b, groove_a, groove_b, anchorB);
        cpSpaceAddConstraint(m_space, joint);

        cpConstraint **userdata = (cpConstraint **)lua_newuserdata(L, sizeof(cpConstraint *));
        *userdata = joint;

        luaL_getmetatable(L, "Constraint");
        lua_setmetatable(L, -2);
        return 1;
    }

    // cpConstraint* cpPinJointNew(cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB);
    static int world_new_pin_joint(lua_State *L)
    {
        cpBody *a = getBodyAt(L, 1);
        cpBody *b = getBodyAt(L, 2);

        if (!a)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at arg 1");
        }

        if (!b)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at agr 2");
        }

        cpVect anchorA;
        cpVect anchorB;

        anchorA.x = luaL_checknumber(L, 3);
        anchorA.y = luaL_checknumber(L, 4);

        anchorB.x = luaL_checknumber(L, 5);
        anchorB.y = luaL_checknumber(L, 6);

        cpConstraint *joint = cpPinJointNew(a, b, anchorA, anchorB);
        cpSpaceAddConstraint(m_space, joint);

        cpConstraint **userdata = (cpConstraint **)lua_newuserdata(L, sizeof(cpConstraint *));
        *userdata = joint;

        luaL_getmetatable(L, "Joint");
        lua_setmetatable(L, -2);
        return 1;
    }

    // cpConstraint* cpRotaryLimitJointNew(cpBody *a, cpBody *b, cpFloat min, cpFloat max);
    static int world_new_rotary_limit_joint(lua_State *L)
    {
        cpBody *a = getBodyAt(L, 1);
        cpBody *b = getBodyAt(L, 2);

        if (!a)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at arg 1");
        }

        if (!b)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at agr 2");
        }

        float min = luaL_checknumber(L, 3);
        float max = luaL_checknumber(L, 4);

        cpConstraint *joint = cpRotaryLimitJointNew(a, b, min, max);
        cpSpaceAddConstraint(m_space, joint);

        cpConstraint **userdata = (cpConstraint **)lua_newuserdata(L, sizeof(cpConstraint *));
        *userdata = joint;

        luaL_getmetatable(L, "Joint");
        lua_setmetatable(L, -2);
        return 1;
    }

    // cpConstraint* cpSlideJointNew(cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB, cpFloat min, cpFloat max);
    static int world_new_slide_joint(lua_State *L)
    {
        cpBody *a = getBodyAt(L, 1);
        cpBody *b = getBodyAt(L, 2);

        if (!a)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at arg 1");
        }

        if (!b)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at agr 2");
        }

        cpVect anchorA;
        cpVect anchorB;

        anchorA.x = luaL_checknumber(L, 3);
        anchorA.y = luaL_checknumber(L, 4);

        anchorB.x = luaL_checknumber(L, 5);
        anchorB.y = luaL_checknumber(L, 6);

        float min = luaL_checknumber(L, 7);
        float max = luaL_checknumber(L, 8);

        cpConstraint *joint = cpSlideJointNew(a, b, anchorA, anchorB, min, max);
        cpSpaceAddConstraint(m_space, joint);

        cpConstraint **userdata = (cpConstraint **)lua_newuserdata(L, sizeof(cpConstraint *));
        *userdata = joint;

        luaL_getmetatable(L, "Joint");
        lua_setmetatable(L, -2);
        return 1;
    }

    // cpConstraint* cpRatchetJointNew(cpBody *a, cpBody *b, cpFloat phase, cpFloat ratchet);

    static int world_new_ratchet_joint(lua_State *L)
    {
        cpBody *a = getBodyAt(L, 1);
        cpBody *b = getBodyAt(L, 2);

        if (!a)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at arg 1");
        }

        if (!b)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at agr 2");
        }

        float phase = luaL_checknumber(L, 3);
        float ratchet = luaL_checknumber(L, 4);

        cpConstraint *joint = cpRatchetJointNew(a, b, phase, ratchet);
        cpSpaceAddConstraint(m_space, joint);

        cpConstraint **userdata = (cpConstraint **)lua_newuserdata(L, sizeof(cpConstraint *));
        *userdata = joint;

        luaL_getmetatable(L, "Joint");
        lua_setmetatable(L, -2);
        return 1;
    }

    // cpConstraint* cpPivotJointNew(cpBody *a, cpBody *b, cpVect pivot);

    static int world_new_pivot_joint(lua_State *L)
    {
        cpBody *a = getBodyAt(L, 1);
        cpBody *b = getBodyAt(L, 2);

        if (!a)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at arg 1");
        }

        if (!b)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at agr 2");
        }

        cpConstraint *joint = nullptr;

        if (lua_gettop(L) == 4)
        {

            cpVect pivot;
            pivot.x = luaL_checknumber(L, 3);
            pivot.y = luaL_checknumber(L, 4);

            joint = cpPivotJointNew(a, b, pivot);
        }
        else if (lua_gettop(L) == 6)
        {

            cpVect anchorA;
            cpVect anchorB;

            anchorA.x = luaL_checknumber(L, 3);
            anchorA.y = luaL_checknumber(L, 4);

            anchorB.x = luaL_checknumber(L, 5);
            anchorB.y = luaL_checknumber(L, 6);

            joint = cpPivotJointNew2(a, b, anchorA, anchorB);
        }

        cpSpaceAddConstraint(m_space, joint);
        cpConstraint **userdata = (cpConstraint **)lua_newuserdata(L, sizeof(cpConstraint *));
        *userdata = joint;

        luaL_getmetatable(L, "Joint");
        lua_setmetatable(L, -2);
        return 1;
    }

    // cpConstraint* cpSimpleMotorNew(cpBody *a, cpBody *b, cpFloat rate);

    static int world_new_motor_joint(lua_State *L)
    {
        cpBody *a = getBodyAt(L, 1);
        cpBody *b = getBodyAt(L, 2);

        if (!a)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at arg 1");
        }

        if (!b)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body at agr 2");
        }

        float rate = luaL_checknumber(L, 3);

        cpConstraint *joint = cpSimpleMotorNew(a, b, rate);
        cpSpaceAddConstraint(m_space, joint);

        cpConstraint **userdata = (cpConstraint **)lua_newuserdata(L, sizeof(cpConstraint *));
        *userdata = joint;

        luaL_getmetatable(L, "Joint");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int world_debug_fill(lua_State *L)
    {

        bool mode = lua_toboolean(L, 1);

        ChipmunkDebug::drawFilled = mode;

        return 0;
    }

    static int world_get_body_at_point(lua_State *L)
    {
        cpPointQueryInfo info;
        cpVect point;
        point.x = luaL_checknumber(L, 1);
        point.y = luaL_checknumber(L, 2);
        float distance = 0;

        // int mask = CP_SHAPE_FILTER_ALL;

        // unsigned int

        //  luaL_checknumber(L, 3);
        //  luaL_checknumber(L, 4);

        cpShape *shape = cpSpacePointQueryNearest(m_space, point, 0, CP_SHAPE_FILTER_ALL, &info);
        if (shape)
        {
            cpBody **userdata = (cpBody **)lua_newuserdata(L, sizeof(cpBody *));
            *userdata = cpShapeGetBody(shape);

            /*
            cpVect point;
                /// The distance to the point. The distance is negative if the point is inside the shape.
                cpFloat distance;
            */
            luaL_getmetatable(L, "Body");
            lua_setmetatable(L, -2);

            return 1;
        }
        lua_pushnil(L);
        return 1;
    }

    static int world_get_body_at_mouse(lua_State *L)
    {
        cpPointQueryInfo info;
        cpVect point;
        point.x = GetMouseX();
        point.y = GetMouseY();

        cpShape *shape = cpSpacePointQueryNearest(m_space, point, 0, CP_SHAPE_FILTER_ALL, &info);
        if (shape)
        {
            cpBody **userdata = (cpBody **)lua_newuserdata(L, sizeof(cpBody *));
            *userdata = cpShapeGetBody(shape);

            luaL_getmetatable(L, "Body");
            lua_setmetatable(L, -2);

            return 1;
        }
        lua_pushnil(L);
        return 1;
    }

    static int world_remove_body(lua_State *L)
    {
        cpBody *body = getBodyAt(L, 1);
        if (!body)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid body object");
        }

        BodyFreeWrap(m_space, body);
        return 0;
    }
    static int world_remove_joint(lua_State *L)
    {
        cpConstraint *constraint = getJoint(L);
        if (!constraint)
        {
            lua_pushnil(L);
            return luaL_error(L, "Invalid joint object");
        }

        ConstraintFreeWrap(m_space, constraint);
        return 0;
    }

    static int world_new_shape_filter(lua_State *L)
    {
        cpShapeFilter filter;
        filter.group = luaL_checkinteger(L, 1);
        filter.categories = luaL_checkinteger(L, 2);
        filter.mask = luaL_checkinteger(L, 3);

        lua_newtable(L);

        lua_pushstring(L, "group");
        lua_pushinteger(L, filter.group);
        lua_settable(L, -3);

        lua_pushstring(L, "categories");
        lua_pushinteger(L, filter.categories);
        lua_settable(L, -3);

        lua_pushstring(L, "mask");
        lua_pushinteger(L, filter.mask);
        lua_settable(L, -3);

        return 1;
    }

    int register_chipmunk_physics(lua_State *L)
    {

        luaL_Reg reg[] =

            {
                {"newShapeFilter", world_new_shape_filter},
                {"newDynamic", world_newDynamic},
                {"newStatic", world_newStatic},
                {"getStaticBody", world_getStatic},
                {"newKinematic", world_newKinematic},

                {"newPinJoint", world_new_pin_joint},
                {"newSlideJoint", world_new_slide_joint},
                {"newPivotJoint", world_new_pivot_joint},
                {"newRatchetJoint", world_new_ratchet_joint},
                {"newGearJoint", world_new_gear_joint},
                {"newMotor", world_new_motor_joint},
                {"newGrooveJoint", world_new_groove_joint},
                //    {"newDampedSpring", world_new_damped_spring},
                //    {"newDampedRotarySpring", world_new_damped_rotary_spring},
                {"newRotaryLimitJoint", world_new_rotary_limit_joint},

                {"update", world_update},
                {"draw", world_draw},
                {"clear", world_clear},
                {"setGravity", world_set_gravity},
                {"setDebugFill", world_debug_fill},

                {"getBodyAtPoint", world_get_body_at_point},
                {"getBodyAtMouse", world_get_body_at_mouse},

                {"removeBody", world_remove_body},

                //   {"__gc", world_gc},
                //   {"__tostring", world_tostring},

                {0, 0},
            };
        luaL_newlib(L, reg);
        return 0;
    }
}
//***********************************************************************************
//***********************************************************************************
//                                BODY
//***********************************************************************************
//***********************************************************************************

namespace ChipmunkBody
{

    static int body_add_line(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[addLine]Invalid Body object");
        }

        if (lua_gettop(L) != 5 && lua_gettop(L) != 6)
        {
            lua_pushnil(L);
            return luaL_error(L, "[addLine] Invalid number of arguments");
        }

        cpBodyType type = cpBodyGetType(body);

        float x1 = luaL_checknumber(L, 2);
        float y1 = luaL_checknumber(L, 3);
        float x2 = luaL_checknumber(L, 4);
        float y2 = luaL_checknumber(L, 5);
        float radius = 0;

        if (lua_gettop(L) == 6)
        {
            radius = luaL_checknumber(L, 6);
        }

        cpShape **userdata = (cpShape **)lua_newuserdata(L, sizeof(cpShape *));
        cpShape *m_shape = nullptr;

        m_shape = cpSegmentShapeNew(body, cpv(x1, y1), cpv(x2, y2), radius);
        if (type == CP_BODY_TYPE_DYNAMIC)
        {
            cpBodySetMoment(body, cpMomentForSegment(cpBodyGetMass(body), cpv(x1, y1), cpv(x2, y2), radius));
        }

        cpSpaceAddShape(m_space, m_shape);
        *userdata = m_shape;
        luaL_getmetatable(L, "Shape");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int body_add_box(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[addBox]Invalid Body object");
        }

        if (lua_gettop(L) != 3 && lua_gettop(L) != 5)
        {
            lua_pushnil(L);
            return luaL_error(L, "[addBox] Invalid number of arguments");
        }

        cpShape *m_shape = nullptr;
        cpBodyType type = cpBodyGetType(body);

        //     Log(LOG_INFO, "body_add_box %d  %d ", lua_gettop(L),(int)type);

        if (lua_gettop(L) == 3)
        {

            float w = luaL_checknumber(L, 2);
            float h = luaL_checknumber(L, 3);

            m_shape = cpBoxShapeNew(body, w, h, 0.0f);
            if (type == CP_BODY_TYPE_DYNAMIC)
            {
                //   Log(LOG_INFO, "body_add_box %d  %d  %f %f %f", lua_gettop(L), (int)type, w, h, cpBodyGetMass(body));
                cpBodySetMoment(body, cpMomentForBox(cpBodyGetMass(body), w, h));
            }
        }
        else if (lua_gettop(L) == 5)
        {
            float left = luaL_checknumber(L, 2);
            float bottom = luaL_checknumber(L, 3);
            float right = luaL_checknumber(L, 4);
            float top = luaL_checknumber(L, 5);

            cpBB bb = cpBBNew(left, bottom, right, top);
            m_shape = cpBoxShapeNew2(body, bb, 0.0f);
            if (type == CP_BODY_TYPE_DYNAMIC)
            {
                cpBodySetMoment(body, cpMomentForBox2(cpBodyGetMass(body), bb));
            }
        }
        else
        {
            lua_pushnil(L);
            return luaL_error(L, "[addBox] Invalid number of arguments");
        }

        cpShape **userdata = (cpShape **)lua_newuserdata(L, sizeof(cpShape *));

        cpSpaceAddShape(m_space, m_shape);
        *userdata = m_shape;
        luaL_getmetatable(L, "Shape");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int body_add_circle(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[addCircle] Invalid Body object");
        }

        if (lua_gettop(L) != 4 && lua_gettop(L) != 2)
        {
            lua_pushnil(L);
            return luaL_error(L, "[addCircle] Invalid number of arguments");
        }

        cpShape *m_shape = nullptr;
        cpBodyType type = cpBodyGetType(body);

        float radius = luaL_checknumber(L, 2);
        float x = 0;
        float y = 0;

        // Log(LOG_INFO, "body_add_box %d", lua_gettop(L));

        if (lua_gettop(L) == 4)
        {
            x = luaL_checknumber(L, 3);
            y = luaL_checknumber(L, 4);
        }

        m_shape = cpCircleShapeNew(body, radius, cpv(x, y));

        if (type == CP_BODY_TYPE_DYNAMIC)
        {
            cpBodySetMoment(body, cpMomentForCircle(cpBodyGetMass(body), 0.0f, radius, cpv(x, y)));
        }

        cpShape **userdata = (cpShape **)lua_newuserdata(L, sizeof(cpShape *));

        cpSpaceAddShape(m_space, m_shape);
        *userdata = m_shape;
        luaL_getmetatable(L, "Shape");
        lua_setmetatable(L, -2);
        return 1;
    }

    static int body_set_position(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[setPosition]Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);
        cpBodySetPosition(body, cpv(x, y));

        return 0;
    }

    static int body_get_position(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getPosition] Invalid Body object");
        }

        cpVect pos = cpBodyGetPosition(body);
        lua_pushnumber(L, pos.x);
        lua_pushnumber(L, pos.y);
        return 2;
    }

    static int body_set_angle(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[setAngle] Invalid Body object");
        }

        float angle = luaL_checknumber(L, 2);
        cpBodySetAngle(body, angle);
        return 0;
    }

    static int body_get_angle(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getAngle] Invalid Body object");
        }

        float angle = cpBodyGetAngle(body);
        lua_pushnumber(L, angle);
        return 1;
    }

    static int body_set_mass(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[setMass] Invalid Body object");
        }

        float mass = luaL_checknumber(L, 2);
        cpBodySetMass(body, mass);
        return 0;
    }

    static int body_get_mass(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getMass] Invalid Body object");
        }

        float mass = cpBodyGetMass(body);
        lua_pushnumber(L, mass);
        return 1;
    }

    static int body_set_velocity(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[setVelocity] Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);

        cpBodySetVelocity(body, cpv(x, y));
        return 0;
    }

    static int body_get_velocity(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getVelocity] Invalid Body object");
        }

        cpVect vel = cpBodyGetVelocity(body);
        lua_pushnumber(L, vel.x);
        lua_pushnumber(L, vel.y);
        return 2;
    }

    static int body_set_angular_velocity(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[setAngularVelociry] Invalid Body object");
        }

        float angular_velocity = luaL_checknumber(L, 2);

        cpBodySetAngularVelocity(body, angular_velocity);
        return 0;
    }

    static int body_get_angular_velocity(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getAngularVelocity] Invalid Body object");
        }

        float angular_velocity = cpBodyGetAngularVelocity(body);
        lua_pushnumber(L, angular_velocity);
        return 1;
    }

    static int body_set_force(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[setForce] Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);

        cpBodySetForce(body, cpv(x, y));
        return 0;
    }

    static int body_get_force(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getForce] Invalid Body object");
        }

        cpVect force = cpBodyGetForce(body);
        lua_pushnumber(L, force.x);
        lua_pushnumber(L, force.y);
        return 2;
    }

    static int body_set_torque(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[setTorque] Invalid Body object");
        }

        float torque = luaL_checknumber(L, 2);

        cpBodySetTorque(body, torque);
        return 0;
    }

    static int body_get_torque(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getTorque] Invalid Body object");
        }

        float torque = cpBodyGetTorque(body);
        lua_pushnumber(L, torque);
        return 1;
    }

    static int body_apply_force_at_world_point(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[applyForceAtWorldPoint] Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);
        float fx = luaL_checknumber(L, 4);
        float fy = luaL_checknumber(L, 5);

        cpBodyApplyForceAtWorldPoint(body, cpv(fx, fy), cpv(x, y));
        return 0;
    }

    static int body_apply_force_at_local_point(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[applyForceAtLocalPoint] Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);
        float fx = luaL_checknumber(L, 4);
        float fy = luaL_checknumber(L, 5);

        cpBodyApplyForceAtLocalPoint(body, cpv(fx, fy), cpv(x, y));
        return 0;
    }

    static int body_apply_impulse_at_world_point(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[applyImpulseAtWorldPoint] Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);
        float fx = luaL_checknumber(L, 4);
        float fy = luaL_checknumber(L, 5);

        cpBodyApplyImpulseAtWorldPoint(body, cpv(fx, fy), cpv(x, y));
        return 0;
    }

    static int body_apply_impulse_at_local_point(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[applyImpulseAtLocalPoint] Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);
        float fx = luaL_checknumber(L, 4);
        float fy = luaL_checknumber(L, 5);

        cpBodyApplyImpulseAtLocalPoint(body, cpv(fx, fy), cpv(x, y));
        return 0;
    }

    static int body_get_velocity_at_world_point(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getVelocityAtWorldPoint] Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);

        cpVect vel = cpBodyGetVelocityAtWorldPoint(body, cpv(x, y));
        lua_pushnumber(L, vel.x);
        lua_pushnumber(L, vel.y);
        return 2;
    }

    static int body_get_velocity_at_local_point(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getVelocityAtLocalPoint] Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);

        cpVect vel = cpBodyGetVelocityAtLocalPoint(body, cpv(x, y));
        lua_pushnumber(L, vel.x);
        lua_pushnumber(L, vel.y);
        return 2;
    }

    static int body_set_center_of_gravity(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[setCenterOfGravity] Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);

        cpBodySetCenterOfGravity(body, cpv(x, y));
        return 0;
    }

    static int body_get_center_of_gravity(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getCenterOfGravity] Invalid Body object");
        }

        cpVect cog = cpBodyGetCenterOfGravity(body);
        lua_pushnumber(L, cog.x);
        lua_pushnumber(L, cog.y);
        return 2;
    }
    static int body_get_local_to_world(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getLocalToWorld] Invalid Body object");
        }

        float x = luaL_checknumber(L, 2);
        float y = luaL_checknumber(L, 3);

        cpVect local = cpv(x, y);
        cpVect world = cpBodyLocalToWorld(body, local);
        lua_pushnumber(L, world.x);
        lua_pushnumber(L, world.y);
        return 2;
    }

    static int body_get_world_to_local(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "[getWorldToLocal] Invalid Body object");
        }
        cpVect world;
        world.x = luaL_checknumber(L, 2);
        world.y = luaL_checknumber(L, 3);

        cpVect local = cpBodyWorldToLocal(body, world);
        lua_pushnumber(L, local.x);
        lua_pushnumber(L, local.y);
        return 2;
    }

    static int body_remove(lua_State *L)
    {
        cpBody *body = getBody(L);
        if (body == nullptr)
        {
            return luaL_error(L, "Invalid Body object");
        }
        TraceLog(LOG_INFO, "Body free");
        if (m_space)
            BodyFreeWrap(m_space, body);
        body = nullptr;
        return 0;
    }

    // static int body_gc(lua_State *L)
    // {
    //     cpBody *body = getBody(L);
    //     if (body == nullptr)
    //     {
    //         return luaL_error(L, "Invalid Body object");
    //     }
    //     TraceLog(LOG_INFO, "Body free");
    //     if (m_space)
    //         BodyFreeWrap(m_space, body);
    //     body = nullptr;
    //     return 0;
    // }

    int register_chipmunk_Body(lua_State *L)
    {

        luaL_newmetatable(L, "Body");
        // lua_pushcfunction(L, body_gc);
        //  lua_setfield(L, -2, "__gc");

        lua_newtable(L);

        lua_pushcfunction(L, body_remove);
        lua_setfield(L, -2, "remove");

        lua_pushcfunction(L, body_add_box);
        lua_setfield(L, -2, "addBox");

        lua_pushcfunction(L, body_add_circle);
        lua_setfield(L, -2, "addCircle");

        lua_pushcfunction(L, body_add_line);
        lua_setfield(L, -2, "addLine");

        lua_pushcfunction(L, body_set_position);
        lua_setfield(L, -2, "setPosition");

        lua_pushcfunction(L, body_get_position);
        lua_setfield(L, -2, "getPosition");

        lua_pushcfunction(L, body_set_angle);
        lua_setfield(L, -2, "setAngle");

        lua_pushcfunction(L, body_get_angle);
        lua_setfield(L, -2, "getAngle");

        lua_pushcfunction(L, body_set_mass);
        lua_setfield(L, -2, "setMass");

        lua_pushcfunction(L, body_get_mass);
        lua_setfield(L, -2, "getMass");

        lua_pushcfunction(L, body_get_velocity);
        lua_setfield(L, -2, "getVelocity");

        lua_pushcfunction(L, body_set_velocity);
        lua_setfield(L, -2, "setVelocity");

        lua_pushcfunction(L, body_get_angular_velocity);
        lua_setfield(L, -2, "getAngularVelocity");

        lua_pushcfunction(L, body_set_angular_velocity);
        lua_setfield(L, -2, "setAngularVelocity");

        lua_pushcfunction(L, body_set_force);
        lua_setfield(L, -2, "setForce");

        lua_pushcfunction(L, body_get_force);
        lua_setfield(L, -2, "getForce");

        lua_pushcfunction(L, body_set_torque);
        lua_setfield(L, -2, "setTorque");

        lua_pushcfunction(L, body_get_torque);
        lua_setfield(L, -2, "getTorque");

        lua_pushcfunction(L, body_apply_force_at_world_point);
        lua_setfield(L, -2, "applyForceAtWorldPoint");

        lua_pushcfunction(L, body_apply_force_at_local_point);
        lua_setfield(L, -2, "applyForceAtLocalPoint");

        lua_pushcfunction(L, body_apply_impulse_at_world_point);
        lua_setfield(L, -2, "applyImpulseAtWorldPoint");

        lua_pushcfunction(L, body_apply_impulse_at_local_point);
        lua_setfield(L, -2, "applyImpulseAtLocalPoint");

        lua_pushcfunction(L, body_get_velocity_at_world_point);
        lua_setfield(L, -2, "getVelocityAtWorldPoint");

        lua_pushcfunction(L, body_get_velocity_at_local_point);
        lua_setfield(L, -2, "getVelocityAtLocalPoint");

        lua_pushcfunction(L, body_set_center_of_gravity);
        lua_setfield(L, -2, "setCenterOfGravity");

        lua_pushcfunction(L, body_get_center_of_gravity);
        lua_setfield(L, -2, "getCenterOfGravity");

        lua_pushcfunction(L, body_get_local_to_world);
        lua_setfield(L, -2, "localToWorld");

        lua_pushcfunction(L, body_get_world_to_local);
        lua_setfield(L, -2, "worldToLocal");

        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);
        return 0;
    }

}
//***********************************************************************************
//***********************************************************************************
//                                SHAPE
//***********************************************************************************
//***********************************************************************************
namespace ChipmunkShape
{

    static int shape_set_sensor(lua_State *L)
    {
        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[setSensor] Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;
        cpBool sensor = lua_toboolean(L, 2);
        cpShapeSetSensor(shape, sensor);
        return 0;
    }

    static int shape_get_sensor(lua_State *L)
    {
        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[getSensor]  Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;
        lua_pushboolean(L, cpShapeGetSensor(shape));
        return 1;
    }

    static int shape_set_elasticity(lua_State *L)
    {
        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[setElasticity]  Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;
        cpFloat elasticity = luaL_checknumber(L, 2);
        cpShapeSetElasticity(shape, elasticity);
        return 0;
    }

    static int shape_get_elasticity(lua_State *L)
    {
        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[getElasticity] Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;
        lua_pushnumber(L, cpShapeGetElasticity(shape));
        return 1;
    }

    static int shape_set_friction(lua_State *L)
    {
        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[setFriction]  Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;
        cpFloat friction = luaL_checknumber(L, 2);
        cpShapeSetFriction(shape, friction);
        return 0;
    }

    static int shape_get_friction(lua_State *L)
    {
        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[getFriction] Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;
        lua_pushnumber(L, cpShapeGetFriction(shape));
        return 1;
    }

    static int shape_set_surface_velocity(lua_State *L)
    {
        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[setSurfaceVelocity] Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;

        cpVect surfaceVelocity;
        surfaceVelocity.x = luaL_checknumber(L, 2);
        surfaceVelocity.y = luaL_checknumber(L, 3);

        cpShapeSetSurfaceVelocity(shape, surfaceVelocity);
        return 0;
    }

    static int shape_get_surface_velocity(lua_State *L)
    {
        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[getSurfaceVelocity]  Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;
        cpVect surfaceVelocity = cpShapeGetSurfaceVelocity(shape);
        lua_pushnumber(L, surfaceVelocity.x);
        lua_pushnumber(L, surfaceVelocity.y);
        return 2;
    }
    static int shape_set_filter(lua_State *L)
    {
        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[setFilter]  Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;
        cpShapeFilter filter = getShapeFilter(L);
        cpShapeSetFilter(shape, filter);
        return 0;
            
    }

    static int shape_get_filter(lua_State *L)
    {

        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[getFilter]  Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;

        cpShapeFilter filter=cpShapeGetFilter(shape);

        lua_newtable(L);

        lua_pushstring(L, "group");
        lua_pushinteger(L, filter.group);
        lua_settable(L, -3);

        lua_pushstring(L, "categories");
        lua_pushinteger(L, filter.categories);
        lua_settable(L, -3);

        lua_pushstring(L, "mask");
        lua_pushinteger(L, filter.mask);
        lua_settable(L, -3);  

        return 1; 
    }



    static int shape_get_moment(lua_State *L)
    {

        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[getMoment]  Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;

        lua_pushnumber(L, cpShapeGetMoment(shape));
        return 1; 
    }

    static int shape_get_area(lua_State *L)
    {

        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[getArea]  Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;

        lua_pushnumber(L, cpShapeGetArea(shape));
        return 1; 
    }

    static int shape_get_center_of_gravity(lua_State *L)
    {

        cpShape **shape_ptr = (cpShape **)luaL_checkudata(L, 1, "Shape");
        if (shape_ptr == nullptr || *shape_ptr == nullptr)
        {
            return luaL_error(L, "[getCenterOfGravity]  Invalid Shape object");
        }
        cpShape *shape = *shape_ptr;

        cpVect centerOfGravity=cpShapeGetCenterOfGravity(shape);

        lua_pushnumber(L, centerOfGravity.x);
        lua_pushnumber(L, centerOfGravity.y);
        return 2; 
    }




    int register_chipmunk_Shape(lua_State *L)
    {
        luaL_newmetatable(L, "Shape");


        lua_pushcfunction(L, shape_set_sensor);
        lua_setfield(L, -2, "setSensor");

        lua_pushcfunction(L, shape_get_sensor);
        lua_setfield(L, -2, "getSensor");

        lua_pushcfunction(L, shape_set_elasticity);
        lua_setfield(L, -2, "setElasticity");

        lua_pushcfunction(L, shape_get_elasticity);
        lua_setfield(L, -2, "getElasticity");

        lua_pushcfunction(L, shape_set_friction);
        lua_setfield(L, -2, "setFriction");

        lua_pushcfunction(L, shape_get_friction);
        lua_setfield(L, -2, "getFriction");

        lua_pushcfunction(L, shape_set_surface_velocity);
        lua_setfield(L, -2, "setSurfaceVelocity");

        lua_pushcfunction(L, shape_get_surface_velocity);
        lua_setfield(L, -2, "getSurfaceVelocity");

        lua_pushcfunction(L, shape_set_filter);
        lua_setfield(L, -2, "setFilter");

        lua_pushcfunction(L, shape_get_filter);
        lua_setfield(L, -2, "getFilter");

        lua_pushcfunction(L, shape_get_moment);
        lua_setfield(L, -2, "getMoment");

        lua_pushcfunction(L, shape_get_area);
        lua_setfield(L, -2, "getArea");

        lua_pushcfunction(L, shape_get_center_of_gravity);
        lua_setfield(L, -2, "getCenterOfGravity");

        lua_newtable(L);
        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);
        return 0;
    }
}
//***********************************************************************************
//***********************************************************************************
//                                JOINT
//***********************************************************************************
//***********************************************************************************
namespace ChipmunkJoint
{

    static int joint_remove(lua_State *L)
    {
        cpConstraint *joint = getJoint(L);
        if (joint == nullptr)
        {
            return luaL_error(L, "[remove] Invalid Joint object");
        }

        ConstraintFreeWrap(m_space, joint);
        joint = nullptr;

        return 0;
    }

    static int joint_set_anchor_b(lua_State *L)
    {
        cpConstraint *joint = getJoint(L);
        if (joint == nullptr)
        {
            return luaL_error(L, "[setAnchorB] Invalid Joint object");
        }
        cpVect anchorB;
        anchorB.x = luaL_checknumber(L, 2);
        anchorB.y = luaL_checknumber(L, 3);
        cpPivotJointSetAnchorB(joint, anchorB);
        return 0;
    }

    static int joint_set_anchor_a(lua_State *L)
    {
        cpConstraint *joint = getJoint(L);
        if (joint == nullptr)
        {
            return luaL_error(L, "[setAnchorA] Invalid Joint object");
        }
        cpVect anchorA;
        anchorA.x = luaL_checknumber(L, 2);
        anchorA.y = luaL_checknumber(L, 3);
        cpPivotJointSetAnchorA(joint, anchorA);
        return 0;
    }

    // cpConstraintSetMaxForce

    static int joint_set_max_force(lua_State *L)
    {
        cpConstraint *joint = getJoint(L);
        if (joint == nullptr)
        {
            return luaL_error(L, "[setMaxForce] Invalid Joint object");
        }
        cpFloat maxForce = luaL_checknumber(L, 2);
        cpConstraintSetMaxForce(joint, maxForce);
        return 0;
    }

    int register_chipmunk_Joint(lua_State *L)
    {

        luaL_newmetatable(L, "Joint");
        lua_newtable(L);

        lua_pushcfunction(L, joint_remove);
        lua_setfield(L, -2, "remove");

        lua_pushcfunction(L, joint_set_anchor_b);
        lua_setfield(L, -2, "setAnchorB");

        lua_pushcfunction(L, joint_set_anchor_a);
        lua_setfield(L, -2, "setAnchorA");

        lua_pushcfunction(L, joint_set_max_force);
        lua_setfield(L, -2, "setMaxForce");

        lua_setfield(L, -2, "__index");
        lua_pop(L, 1);
        return 0;
    }

}

//***********************************************************************************
//***********************************************************************************
//                                BINDING
//***********************************************************************************
//***********************************************************************************

int register_chipmunk_shape_filter(lua_State *L)
{

    luaL_newmetatable(L, "ShapeFilter");
    lua_newtable(L);

    lua_setfield(L, -2, "__index");
    lua_pop(L, 1);
    return 1;
}

int luaclass_chipmunk(lua_State *L)
{

    ChipmunkBody::register_chipmunk_Body(L);
    ChipmunkShape::register_chipmunk_Shape(L);
    ChipmunkJoint::register_chipmunk_Joint(L);

    return 0;
}

int luaopen_chipmunk(lua_State *L)
{
    ChipmunkWorld::register_chipmunk_physics(L);
    return 0;
}

#endif