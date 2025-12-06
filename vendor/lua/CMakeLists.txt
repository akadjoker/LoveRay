 
project(lua)


set(LUA_SOURCES
    # Core
    src/lapi.c
    src/lcode.c
    src/lctype.c
    src/ldebug.c
    src/ldo.c
    src/ldump.c
    src/lfunc.c
    src/lgc.c
    src/llex.c
    src/lmem.c
    src/lobject.c
    src/lopcodes.c
    src/lparser.c
    src/lstate.c
    src/lstring.c
    src/ltable.c
    src/ltm.c
    src/lundump.c
    src/lvm.c
    src/lzio.c
    # Lib
    src/lauxlib.c
    src/lbaselib.c
    src/lcorolib.c
    src/ldblib.c
    src/liolib.c
    src/lmathlib.c
    src/loadlib.c
    src/loslib.c
    src/lstrlib.c
    src/ltablib.c
    src/lutf8lib.c
    src/linit.c
)


add_library(lua STATIC ${LUA_SOURCES})


target_include_directories(lua PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# === Flags de compilação ===
target_compile_options(lua PRIVATE -Wall -Wextra -O2)
target_compile_definitions(lua PUBLIC LUA_COMPAT_5_3)

# === Platform específico ===
if(UNIX AND NOT APPLE)
    #target_compile_definitions(lua PUBLIC LUA_USE_LINUX LUA_USE_POSIX LUA_USE_DLOPEN)
    target_link_libraries(lua PUBLIC m dl)
    target_link_options(lua PUBLIC -Wl,-E)
elseif(APPLE)
    target_compile_definitions(lua PUBLIC LUA_USE_MACOSX LUA_USE_POSIX LUA_USE_DLOPEN)
    target_link_libraries(lua PUBLIC m dl)
elseif(WIN32)
    target_compile_definitions(lua PUBLIC LUA_BUILD_AS_DLL)
else()
    target_compile_definitions(lua PUBLIC LUA_USE_POSIX)
endif()
 
set_source_files_properties(
    src/llex.c src/lparser.c src/lcode.c
    PROPERTIES COMPILE_FLAGS "-Os"
)

message(STATUS "Lua: liblua.a -> ${CMAKE_CURRENT_SOURCE_DIR}/include")

