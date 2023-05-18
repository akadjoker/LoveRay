function love.load()
 
    love.physics.setGravity(0, 100)
    love.physics.setDebugFill(true)

    

    demo = love.filesystem.load("scripts/demo3.lua")
    
    demo:load()
     

    -- for key, value in pairs(love) do
    --     if type(value) == "function" then
    --       local info = debug.getinfo(value, "u")
    --       local numParams = info.nparams or 0
    --       print("love." .. key .. " ()")
    --     end
        
    --     -- Se o valor for uma tabela, itera sobre ela para imprimir suas funções
    --     if type(value) == "table" then
    --       for subKey, subValue in pairs(value) do
    --         if type(subValue) == "function" then
    --           local info = debug.getinfo(subValue, "u")
    --           local numParams = info.nparams or 0
    --           print("\tlove." .. key .. "." .. subKey .. " ()")
    --         end
    --       end
    --     end
    --   end
  
      
      
    

end
    
    

function love.draw()

love.graphics.setColor(1, 0, 1,1)
love.graphics.print(" FPS: " .. tostring(love.timer.getFPS()) , 5, 5)
love.graphics.setColor(1, 1, 1,1)

demo:draw()
love.physics.draw()




end

function love.update(dt)
    
demo:update(dt)    
love.physics.update(1.0/60.0)



end 
    
    
    