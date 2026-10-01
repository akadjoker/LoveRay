project(raylib)

# === Sources do Raylib ===
set(RAY_SOURCES
    src/raudio.c
    src/rcore.c
    src/rmodels.c
    src/rshapes.c
    src/rtext.c
    src/rtextures.c
    src/utils.c
)
if(NOT EMSCRIPTEN)
    list(APPEND RAY_SOURCES src/rglfw.c)
endif()

# === Biblioteca estática ===
add_library(raylib STATIC ${RAY_SOURCES})

target_include_directories(raylib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(raylib PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/external/glfw/include
)

# === Flags de compilação ===
target_compile_options(raylib PRIVATE 
    -W 
    -O2
    -Wno-missing-braces
    -Wno-unused-parameter
)

# === Definições gerais ===
if(EMSCRIPTEN)
    target_compile_definitions(raylib PUBLIC
        PLATFORM_WEB
        GRAPHICS_API_OPENGL_ES2
    )
else()
    target_compile_definitions(raylib PUBLIC
        PLATFORM_DESKTOP
        GRAPHICS_API_OPENGL_33
    )
endif()

# === Platform específico ===
if(EMSCRIPTEN)
    target_compile_options(raylib PRIVATE -Os)
    target_link_options(raylib PUBLIC -sUSE_GLFW=3)
elseif(UNIX AND NOT APPLE)
    # Linux
    target_compile_definitions(raylib PUBLIC
        _DEFAULT_SOURCE
        _GLFW_X11
    )
    
    # OpenGL + X11 + threads
    find_package(OpenGL REQUIRED)
    find_package(X11 REQUIRED)
    
    target_link_libraries(raylib PUBLIC
        OpenGL::GL
        ${X11_LIBRARIES}
        m
        pthread
        dl
    )
    
    # Include X11
    target_include_directories(raylib PRIVATE ${X11_INCLUDE_DIR})
    
    target_link_options(raylib PUBLIC -Wl,-E)

    
elseif(WIN32)
    # Windows/MinGW
    target_compile_definitions(raylib PUBLIC
        _CRT_SECURE_NO_WARNINGS
    )
    
    # OpenGL + Windows libs
    target_link_libraries(raylib PUBLIC
        opengl32
        gdi32
        winmm
        kernel32
    )
    
elseif(APPLE)
    target_compile_definitions(raylib PUBLIC _GLFW_COCOA)
    set_source_files_properties(src/rglfw.c PROPERTIES COMPILE_FLAGS "-x objective-c")
    target_link_libraries(raylib PUBLIC
        "-framework OpenGL"
        "-framework Cocoa"
        "-framework IOKit"
        "-framework CoreVideo"
        "-framework CoreFoundation"
        "-framework CoreAudio"
        "-framework AudioToolbox"
        m
        pthread
    )
else()
    # Generic Unix/POSIX
    target_compile_definitions(raylib PUBLIC
        _DEFAULT_SOURCE
    )
    
    find_package(OpenGL REQUIRED)
    target_link_libraries(raylib PUBLIC
        OpenGL::GL
        m
        pthread
    )
endif()

message(STATUS "Raylib: libraylib.a -> ${CMAKE_CURRENT_SOURCE_DIR}/include")
