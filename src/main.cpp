// main.cpp - process entry point for the LoveRay runtime.
//
// Usage:  love [options] <game directory | main.lua>
//
// The heavy lifting happens in boot.lua (embedded); this file only loops while
// the game asks for a restart through love.event.quit("restart").
#include "love.hpp"

int main(int argc, char **argv)
{
    love::BootResult result;
    do
    {
        result = love::boot(argc, argv);
    } while (result.restart);

    love::window::shutdown();
    return result.exitCode;
}
