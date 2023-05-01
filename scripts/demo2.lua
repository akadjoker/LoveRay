local demo = {}

function demo.load()

        fisica={}
        fisica.staticBody = love.physics.getStaticBody()  
        fisica.selecBody = nil
        fisica.mouseJoint =nil
        fisica.draw=false

      st = love.physics.newStatic(0,0,0)
      st:addBox(5,590,800-5,595)
      st:addBox(5,6,800-5,10)
      st:addBox(5,5,10,600)
      st:addBox(800-5,5,800,600)
   
      body1 = love.physics.newDynamic(100,100)
      body1:addBox(25,25)
  
      body1 = love.physics.newDynamic(300,200)
      body1:addBox(50,50)
  

end

function demo.draw()
    love.graphics.setColor(1, 0, 0,1)
    

end

function demo.update(dt)
         

    local mx,my = love.mouse.getPosition()

    if (love.mouse.isDown(1)) then
        dt = love.physics.newDynamic(love.mouse.getX(),love.mouse.getY(),5,1)
        dt:addCircle(15)
    
    end
    


    if (love.mouse.press(0)) then
        if not fisica.selecBody then

        fisica.selecBody = love.physics.getBodyAtPoint(mx,my)

        if (fisica.selecBody) then
            local localPosX,localPosY= fisica.selecBody:worldToLocal(mx,my)

            fisica.mouseJoint = love.physics.newPivotJoint(fisica.staticBody,fisica.selecBody ,0,0, localPosX,localPosY)

            fisica.mouseJoint:setMaxForce(1000.0)

      
               

        end
    end 

    elseif (love.mouse.release(0)) then
       
       
        if (fisica.mouseJoint) then
            fisica.mouseJoint:remove()
            fisica.mouseJoint = nil
            fisica.selecBody = nil
       end
            
       
      

    end 

    if (fisica.mouseJoint) then
        local localPosX,localPosY= fisica.selecBody:localToWorld(mx,my)
        fisica.mouseJoint:setAnchorA(mx,my)
    end


            
end

return demo


