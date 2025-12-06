local demo = {}

function demo.load()
          
    
      st = love.physics.newStatic(0,0,0)
      st:addBox(5,590,800-5,595)
   
      

      st:addBox(5,6,800-5,10)
      st:addBox(5,5,10,600)
      st:addBox(800-5,5,800,600)
      st:addBox(100,500,300,515)

      dt = love.physics.newDynamic(100,100,0,1)
      dt:addBox(25,25)
end

function demo.draw()
    
end

function demo.update(dt)
    
if (love.mouse.isDown(0)) then
    dt = love.physics.newDynamic(love.mouse.getX(),love.mouse.getY(),0,1)
    dt:addBox(25,25)

end


if (love.mouse.isDown(1)) then
    dt = love.physics.newDynamic(love.mouse.getX(),love.mouse.getY(),5,1)
    dt:addCircle(15)

end

end

return demo
 


  
    
