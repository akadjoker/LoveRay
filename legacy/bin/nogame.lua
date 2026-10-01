--
-- Copyright (c) 2023 djoker
--
-- This library is free software; you can redistribute it and/or modify it
-- under the terms of the MIT license. See LICENSE for details.
--
print("nogame.lua loaded")
function love.nogame()

    function love.load()
        love.graphics.setBackgroundColor(89/255, 157/255, 220/255)
        font = love.graphics.getFont()
    end
  
    local function drawText(str, y)
      local screenw = love.graphics.getWidth() /2
      love.graphics.print(str, (screenw - font:getWidth(str)) , y)
    end
  
    function love.draw()
      love.graphics.setColor(1, 1, 1)
      drawText("<< RayLove " .. love.getVersion() .. " >>", 90)
      drawText("NO GAME", love.graphics.getHeight() / 2 -20 )
      love.graphics.setColor(195/255, 81/255, 55/255)
      drawText("Press ESCAPE to quit", love.graphics.getHeight() /2+100)
    end
  
  end