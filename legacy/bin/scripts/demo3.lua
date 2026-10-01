local demo = {}

local width = love.graphics.getWidth()
local height = love.graphics.getHeight()

camera = {
    x = 0,
    y = 0
 }

function addBox(x,y,w,h)

    body = love.physics.newBody(0,x,y)
    shape = body:addBox(w,h)

end

function addCircle(x,y,r)

   circle = love.physics.newBody(0,x,y)
   circle:addCircle(25)

end


function demo.load()
    
    bodyImage = love.graphics.newImage("images/carro.png")
    rodaImage = love.graphics.newImage("images/roda.png")
    love.physics.setGravity(0,9.8)

   

    body = love.physics.newBody(1,0,0)
    body:addBox(10,height,5,height/2,0)

 


    lines = get_track()
 
    for i, line in ipairs(lines) do
        local x1, y1, x2, y2 = line[1], line[2], line[3], line[4]
        ln = body:addLine(x1, y1, x2, y2)
        ln:setFriction(1.0)
    end



    local x = 400
    local y = 100
    car = love.physics.newBody(0,x,y)
    local chassis = car:addBox(140,40)
    chassis:setDensity(0.1)

    local py =32

     wheel1 = love.physics.newBody(0,x-50,y+py)
    local sp1 = wheel1:addCircle(10)
    sp1:setFriction(1.2)
    sp1:setDensity(0.8)

     wheel2 = love.physics.newBody(0,x+60,y+py)
    local sp2 = wheel2:addCircle(10)
    sp2:setFriction(1.0)
    sp2:setDensity(0.8)

    local axisX=0.0
    local axisY=1.0



      joint1 = love.physics.newWheelJoint(car,wheel1,x-50,y+py,axisX,axisY)
      joint2 = love.physics.newWheelJoint(car,wheel2,x+60,y+py,axisX,axisY)

     local hertz = 4.0
     local dampingRatio = 0.7
     local omega = 2.0 * math.pi * hertz
 
     local mass = wheel1:getMass()
     
     angle =0 
 
     local stiffness = mass * omega * omega
     local damping = 2.0 * mass * dampingRatio * omega
 
     joint1:setMotorEnabled(true)
     joint1:setMotorSpeed(0.0)
     joint1:setMaxMotorTorque(500.0)
     joint1:setStiffness(stiffness)
     joint1:setDamping(damping)
     --joint1:setLimitsEnabled(true)
    --'' joint1:setLimits(-0.55,0.55)
 
      joint2:setMotorEnabled(false)
      joint2:setMotorSpeed(0.0)
      joint2:setMaxMotorTorque(500.0)
     joint2:setStiffness(stiffness)
     joint2:setDamping(damping)
     --joint2:setLimitsEnabled(true)
  --   joint2:setLimits(-0.55,0.55)

  love.graphics.setCameraOrigin(width/2,height/2)
end

function demo.draw()
    



    local x, y = car:getPosition()
    local angle = car:getAngle()
    local roda1_x,roda1_y = wheel1:getPosition()
    local roda2_x,roda2_y = wheel2:getPosition()
    local roda1_angle = wheel1:getAngle()
    local roda2_angle = wheel2:getAngle()


    local w = bodyImage:getWidth()  /2
    local h = bodyImage:getHeight() /2

    local rw = rodaImage:getWidth()  /2
    local rh = rodaImage:getHeight() /2

    
  
    love.graphics.beginCamera() 
    love.physics.draw()

    love.graphics.draw(bodyImage,x,y, angle, 1.0,1.0, w,h, 0,0)
    love.graphics.draw(rodaImage,roda1_x,roda1_y, roda1_angle, 1.0,1.0, rw,rh, 0,0)
    love.graphics.draw(rodaImage,roda2_x,roda2_y, roda2_angle, 1.0,1.0, rw,rh, 0,0)

    love.graphics.endCamera()


end

function demo.update(dt)
    local car_x,car_y = car:getPosition()
    love.graphics.setCameraPosition(car_x,car_y)


   if (love.keyboard.press("space")) then
       addBox(love.mouse.getWorldX(),love.mouse.getWorldY(),20,20) 
   end

  

   if (love.keyboard.down("a")) then
       joint1:setMotorSpeed(-100.0)
     --  joint2:setMotorSpeed(-100.0)
       
   end
   if (love.keyboard.down("d")) then
       joint1:setMotorSpeed(60.0)
     --  joint2:setMotorSpeed(80.0)
   end

   if (love.keyboard.down("s")) then
       joint1:setMotorSpeed(0.0)
       joint2:setMotorSpeed(0.0)
   end
   
end




function get_track()
    return {
        {-42, 345, 303, 352},
        {303, 352, 571, 411},
        {571, 411, 997, 438},
        {997, 438, 1451, 436},
        {1451, 436, 1535, 422},
        {1535, 422, 1575, 412},
        {1575, 412, 1605, 405},
        {1605, 405, 1775, 379},
        {1775, 379, 1914, 336},
        {1914, 336, 2047, 332},
        {2047, 332, 2105, 387},
        {2105, 387, 2226, 462},
        {2226, 462, 2374, 462},
        {2374, 462, 2544, 458},
        {2544, 458, 2724, 456},
        {2724, 456, 2922, 448},
        {2922, 448, 3478, 531},
        {3478, 531, 3717, 531},
        {3717, 531, 3852, 510},
        {3852, 510, 3888, 541},
        {3888, 541, 3969, 543},
        {3969, 543, 4067, 537},
        {4067, 537, 4150, 539},
        {4150, 539, 4246, 533},
        {4246, 533, 4359, 509},
        {4359, 509, 4435, 532},
        {4435, 532, 4541, 534},
        {4541, 534, 4711, 534},
        {4711, 534, 4861, 534},
        {4861, 534, 5001, 534},
        {5001, 534, 5275, 491},
        {5275, 491, 5560, 485},
        {5560, 485, 5800, 447},
        {5800, 447, 5974, 422},
        {5974, 422, 6296, 335},
        {6296, 335, 6630, 293},
        {6630, 293, 6914, 293},
        {6914, 293, 7144, 305},
        {7144, 305, 7340, 293},
        {7340, 293, 7593, 271},
        {7593, 271, 7843, 258},
        {7843, 258, 8043, 258},
        {8043, 258, 8153, 258},
        {8153, 258, 8666, 370},
        {8666, 370, 8838, 397},
        {8838, 397, 9098, 411},
        {9098, 411, 9320, 407},
        {9320, 407, 9632, 407},
        {9632, 407, 9912, 407},
        }
end

return demo
