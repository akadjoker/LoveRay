cmake_minimum_required(VERSION 3.16)
project(LuaSampoles LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



add_subdirectory(vendor/box2d)        
add_subdirectory(vendor/lua)        
add_subdirectory(vendor/raylib)    


add_subdirectory(love)

