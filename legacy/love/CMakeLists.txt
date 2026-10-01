project(love)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_executable(love
  src/main.cpp
)

find_package(OpenGL REQUIRED)
 
target_link_libraries(love PRIVATE 
box2d
lua
OpenGL::GL
raylib
)

target_compile_options(love PRIVATE 
    -Wno-missing-braces
    -Wno-unused-parameter
	-Wall 

	-pedantic 
	-O2
)

 
target_include_directories(love PRIVATE 
  ${CMAKE_SOURCE_DIR}/vendor/lua/include
  ${CMAKE_SOURCE_DIR}/vendor/raylib/include
  ${CMAKE_SOURCE_DIR}/vendor/box2d/include
  ${CMAKE_SOURCE_DIR}/vendor/box2d/include/box2d

  )


 

if(CMAKE_BUILD_TYPE MATCHES Debug)
     target_compile_options(love PRIVATE -fsanitize=address -fsanitize=undefined -fsanitize=leak)  
     target_link_options(love PRIVATE -fsanitize=address -fsanitize=undefined -fsanitize=leak ) 
endif()
