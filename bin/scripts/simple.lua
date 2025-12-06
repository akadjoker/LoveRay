local demo = {}

function demo.load()
    image = love.graphics.newImage("images/zazaka.png")  
print( image:getWidth() )


end


local function drawText(str, y)
    local screenw = love.graphics.getWidth()/2
    local font = love.graphics.getFont()
   love.graphics.print(str, (screenw - font:getWidth(str)) , y)
   
  end

function demo.draw()
        drawText("RayLove " .. love.getVersion(), 90)

  love.graphics.print("Hello game!", 400, 300) 
  love.graphics.setColor(1, 1, 1,1)
  love.graphics.draw(image, 200, 300)
end

function demo.update(dt)
    
end

return demo
