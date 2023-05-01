#ifndef LOVEPHYSICS_H
#define LOVEPHYSICS_H

#include <string>
#include <vector>

enum BodyType
{
    DYNAMIC = 0,
    STATIC = 1,
    KINEMATIC = 1

};

int luaopen_physics(lua_State *L);
int luaclass_physics(lua_State *L);

void init_physics();
void free_physics();


#endif

#ifdef PHYSICS_IMPLEMENTATION


#define CHIP_PHYSICS_IMPLEMENTATION
#include "Chipmunk.hpp"

void init_physics()
{
  init_chipmunk();
}

void free_physics()
{
  free_chipmunk();
}

int luaclass_physics(lua_State *L)
{
    luaclass_chipmunk(L);
    return 0;
}

int luaopen_physics(lua_State *L)
{
  luaopen_chipmunk(L);
  return 0;
}


#endif