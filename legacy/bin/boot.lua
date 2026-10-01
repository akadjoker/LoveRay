--
-- Copyright (c) 2023 djoker
--
-- This library is free software; you can redistribute it and/or modify it
-- under the terms of the MIT license. See LICENSE for details.
--


function print_table (t)
    for k,v in pairs(t) do
      print(k, v)
      if type(v) == "table" then
        print_table(v)
      end
    end
  end


local modtime = 0
local mainScript = "main.lua"

function love.boot()
    

    --print("love.boot() called")

    package.path = "?.lua;"
    --package.path = package.path .. ";assets/?.lua;assets/?/init.lua;assets/scripts/?.lua;assets/scripts/?/init.lua;assets/lua/?.lua;assets/lua/?/init.lua;../?.lua;../assets/?.lua;../assets/?/init.lua;../lua/?.lua;../lua/?/init.lua;../scripts/?.lua;../scripts/?/init.lua"

    --package.path = "?.lua;?/init.lua"
    table.insert(package.searchers, 1, function(modname)
      modname = modname:gsub("%.", "/")
      --print("modname: " .. modname)
      for x in package.path:gmatch("[^;]+") do
        local file = x:gsub("?", modname)
        if love.filesystem.exists(file) then
            mainScript = file
            modtime = love.filesystem.getInfo(file).modtime
            return assert(load(love.filesystem.read(file), "=" .. file))
        end
        --print("file: " .. file)
      end
    end)
    
    local c = 
    {
		title = "DjokerSoft RayLove2D@",
		version = love.getVersion(),
		window = 
        {
			width = 800,
			height = 600,
            fps = 60,
            title ="RayLove2D by Djoker",
            vsync = true,
            fullscreen = false,
            borderless = false,
            resizable = false,
        }
    }
            
    local args = {}
    for i = 2, #love.argv do
      args[i - 1] = love.argv[i]

      if love.filesystem.isFile(love.argv[i] .. "/conf.lua") then
        love.log(1,"conf.lua loaded")
        local confScript= love.argv[i] .. "/conf.lua"
        assert(load(love.filesystem.read(confScript), "=" .. confScript)) 
        local extension = string.sub(confScript, string.find(confScript, "%.[^%.]+$"))
        local name_without_extension = string.sub(confScript, 1, #confScript - #extension)
        require(name_without_extension)
        love.conf(c)
        
     end

      if love.filesystem.isFile(love.argv[i] .. "/main.lua") then
         mainScript= love.argv[i] .. "/main.lua"
         modtime = love.filesystem.getInfo(mainScript).modtime
         assert(load(love.filesystem.read(mainScript), "=" .. mainScript)) 
         break
      end
    end

    
    love.setup(c.window.width, c.window.height, c.window.fps, c.window.title, c.window.vsync, c.window.fullscreen, c.window.borderless, c.window.resizable)
    love.init()
  
    -- Load main.lua or init `nogame` state
    if love.filesystem.isFile(mainScript) then
        path = love.filesystem.getPath(mainScript)
        local extension = string.sub(mainScript, string.find(mainScript, "%.[^%.]+$"))
        local name_without_extension = string.sub(mainScript, 1, #mainScript - #extension)
        --love.log(0,"filename : " ..  extension .. " path: " .. name_without_extension )
        require(name_without_extension)
    else
      love.nogame()
    end
    
    love.run()
  end
  
  
  function love.run()
    -- Prepare arguments
    local args = {}
    for i = 2, #love.argv do
      args[i - 1] = love.argv[i]
    end
  

  --  print("love.run() called")
    -- Do load callback
    if love.load then love.load(args) end
    
  
    while true do
        
      if (love.close()) then
          return
      end
      -- Update
      
      local dt = love.timer.getDelta()
      if love.update then love.update(dt) end
      -- Draw

      love.graphics.clear()
      if love.draw then love.draw() end
      love.graphics.present()
    end
  end
  
  
  function love.errhand(msg)
    love.log(2,msg)
     love.graphics.setBackgroundColor(89/255, 157/255, 220/255)
    -- Do error main loop
     while true do
        
      
        if love.close() then
            return
        end

        if love.filesystem.getInfo(mainScript).modtime > modtime then
            package.loaded[mainScript] = nil
            modtime = love.filesystem.getInfo(mainScript).modtime
            assert(load(love.filesystem.read(mainScript), "=" .. mainScript))
            local filename = string.match(mainScript, "([^/]+)%.%w+$")
            require(filename)
            love.run()
            return
        end

        
      love.graphics.clear()
      love.graphics.print(msg, 6, 6)
      love.graphics.present()
    end
  end
  
  
  xpcall(love.boot, love.errhand)