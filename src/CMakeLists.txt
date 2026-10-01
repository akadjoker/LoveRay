# LoveRay runtime: a Love2D-compatible API on top of raylib, Lua 5.4 and Box2D.

set(LOVERAY_SOURCES
    main.cpp
    resources.cpp
    love.cpp
    filesystem.cpp
    vfs.cpp
    window.cpp
    graphics.cpp
    graphics_objects.cpp
    shader.cpp
    particles.cpp
    event.cpp
    keyboard.cpp
    mouse.cpp
    joystick.cpp
    timer.cpp
    math.cpp
    audio.cpp
    system.cpp
    image.cpp
    physics.cpp
    physics_joints.cpp
)

# Lua scripts and binary assets that are compiled into the executable.
set(LOVERAY_RESOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/lua/boot.lua
    ${CMAKE_CURRENT_SOURCE_DIR}/lua/nogame.lua
    ${CMAKE_CURRENT_SOURCE_DIR}/assets/DejaVuSans.ttf
)
set(LOVERAY_RESOURCE_HEADER ${CMAKE_CURRENT_BINARY_DIR}/generated/resources.h)

add_custom_command(
    OUTPUT ${LOVERAY_RESOURCE_HEADER}
    COMMAND ${CMAKE_COMMAND}
            -DOUTPUT=${LOVERAY_RESOURCE_HEADER}
            "-DINPUTS=${LOVERAY_RESOURCES}"
            -P ${CMAKE_SOURCE_DIR}/cmake/EmbedResources.cmake
    DEPENDS ${LOVERAY_RESOURCES} ${CMAKE_SOURCE_DIR}/cmake/EmbedResources.cmake
    COMMENT "Embedding Lua boot scripts and default font"
    VERBATIM
)

add_executable(love ${LOVERAY_SOURCES} ${LOVERAY_RESOURCE_HEADER})

set_target_properties(love PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)

target_include_directories(love PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/generated
    ${CMAKE_SOURCE_DIR}/vendor/lua/include
    ${CMAKE_SOURCE_DIR}/vendor/raylib/include
    ${CMAKE_SOURCE_DIR}/vendor/box2d/include
)

target_compile_definitions(love PRIVATE
    LOVERAY_VERSION="${PROJECT_VERSION}"
)

target_link_libraries(love PRIVATE box2d lua raylib)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(love PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
endif()

if(LOVERAY_SANITIZE AND CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_options(love PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(love PRIVATE -fsanitize=address,undefined)
endif()

if(MINGW)
    # Self-contained executable: no MinGW runtime DLLs next to love.exe.
    target_link_options(love PRIVATE -static -static-libgcc -static-libstdc++)
endif()

if(EMSCRIPTEN)
    # The whole examples folder is preloaded at /game; the shell page picks
    # one with ?game=<name>. Override with -DLOVERAY_WEB_GAME=<dir>.
    set(LOVERAY_WEB_GAME "${CMAKE_SOURCE_DIR}/examples" CACHE PATH "Directory preloaded into the web build")
    set_target_properties(love PROPERTIES SUFFIX ".html")
    target_link_options(love PRIVATE
        -sUSE_GLFW=3
        -sASYNCIFY
        -sASYNCIFY_STACK_SIZE=1048576
        -sALLOW_MEMORY_GROWTH=1
        -sINITIAL_MEMORY=128MB
        -sSTACK_SIZE=4MB
        -sFORCE_FILESYSTEM=1
        -sEXIT_RUNTIME=1
        "-sEXPORTED_RUNTIME_METHODS=['ccall','FS']"
        --shell-file ${CMAKE_SOURCE_DIR}/src/web/shell.html
        --preload-file ${LOVERAY_WEB_GAME}@/game
    )
    set_property(TARGET love APPEND PROPERTY LINK_DEPENDS ${CMAKE_SOURCE_DIR}/src/web/shell.html)
endif()
