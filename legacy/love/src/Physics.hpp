#ifndef PHYSICS_H
#define PHYSICS_H


#define USE_BOX2D
//#define USE_CHIP

int luaopen_physics(lua_State *L);
int luaclass_physics(lua_State *L);

void init_physics();
void free_physics();
void clear_physics();


#endif

#ifdef PHYSICS_IMPLEMENTATION




#ifdef USE_BOX2D
#define BOX_PHYSICS_IMPLEMENTATION
#include "Box2d.hpp"
#endif

#ifdef USE_CHIP
#define CHIP_PHYSICS_IMPLEMENTATION
#include "Chipmunk.hpp"
#endif 

void init_physics()
{
  #ifdef USE_BOX2D
  init_box2d();
  #endif

  #ifdef USE_CHIP
  init_chipmunk();
  #endif
}

void clear_physics()
{
  #ifdef USE_BOX2D
  clear_box2d();
  #endif

  #ifdef USE_CHIP
  clear_chipmunk();
  #endif
}

void free_physics()
{
  #ifdef USE_BOX2D
  free_box2d();
  #endif

  #ifdef USE_CHIP
  free_chipmunk();
  #endif

}

int luaclass_physics(lua_State *L)
{

    #ifdef USE_BOX2D
    luaclass_box2d(L);
    #endif

    #ifdef USE_CHIP
    luaclass_chipmunk(L);
    #endif

    return 0;
}

int luaopen_physics(lua_State *L)
{
  #ifdef USE_BOX2D
  luaopen_box2d(L);
  #endif

  #ifdef USE_CHIP
  luaopen_chipmunk(L);
  #endif
  return 0;
}


#endif