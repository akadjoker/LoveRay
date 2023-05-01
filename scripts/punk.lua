local gravity = 1.8

function sign(x)
  if x > 0 then
      return 1
  elseif x < 0 then
      return -1
  else
      return 0
  end
end


-- Objeto Entity
local Entity = {}
Entity.__index = Entity

function Entity:new(x, y, w, h, name)
  local e = {}
  setmetatable(e, Entity)
  e.scene = nil
  e.x = x or 0
  e.y = y or 0
  e.w = w or 0
  e.h = h or 0
  e.onGround = false
  e._x=0
  e._y=0
  e._moveX=0
  e._moveY=0
  e.name = name or "entity"
  e.vx = 0
  e.vy = 0
  e.acx=0
  e.acy=0
  e.frictionX=0
  e.frictionY=0
  e.maxVelocityX=0
  e.maxVelocityY=0
  e.active = true
  e.alive = true
  e.visible = true
  e.solid = true
  e.hitbox = {
      w = w or 0,
      h = h or 0,
      ox = 0,
      oy = 0
  }

  return e
end

function Entity:update(dt)
  if self.solid then
     return
  end
    self.vx = self.vx + self.acx
    self.vy = self.vy + self.acy 
    --self.x = self.x + self.vx * dt
    --self.y = self.y + self.vy * dt
    self.onGround = false

    self:moveBy(self.vx, self.vy, "solid")

    self.vy =self.vy + gravity 

    if (self.maxVelocityX > 0 and math.abs(self.vx) > self.maxVelocityX) then
				self.vx = self.maxVelocityX * sign(self.vx)
		end

    if (self.maxVelocityY > 0 and math.abs(self.vy) > self.maxVelocityY) then
        self.vy = self.maxVelocityY * sign(self.vy)
    end



end


function Entity:setHitBox( w, h, ox, oy)
    self.hitbox.w = w
    self.hitbox.h = h
    self.hitbox.ox = ox
    self.hitbox.oy = oy
end


function Entity:centerOrigin()
  
  self.hitbox.ox = self.hitbox.w /2.0
  self.hitbox.oy = self.hitbox.h /2.0
end

function Entity:setOrigin(x,y)
  
  self.hitbox.ox = x
  self.hitbox.oy = y
end


function Entity:collidesWith(other)
  return (self.x + self.hitbox.ox + self.hitbox.w > other.x + other.hitbox.ox and
          self.y + self.hitbox.oy + self.hitbox.h > other.y + other.hitbox.oy and
          self.x + self.hitbox.ox < other.x + other.hitbox.ox + other.hitbox.w and
          self.y + self.hitbox.oy < other.y + other.hitbox.oy + other.hitbox.h)
end


function Entity:collidesPointWith(x,y,other)
  return (x + self.hitbox.ox + self.hitbox.w > other.x + other.hitbox.ox and
          y + self.hitbox.oy + self.hitbox.h > other.y + other.hitbox.oy and
          x + self.hitbox.ox < other.x + other.hitbox.ox + other.hitbox.w and
          y + self.hitbox.oy < other.y + other.hitbox.oy + other.hitbox.h)
end

function Entity:moveCollideX(e)

--   -- Verificar se a colisão é em cima ou embaixo
   local direction = sign(self.y + self.hitbox.oy - e.y - e.hitbox.oy)
--   -- Empurrar o objeto na direção oposta à colisão
--   self.vx = -self.vx * self.frictionX
--   --self.vy = self.vy * self.frictionY
--   -- Se o objeto ainda estiver colidindo, deslocar verticalmente
   while self:collidesWith(e) do
     self.x = self.x + direction
   end

    self.vx = 0
    self.vy =self.vy * self.frictionY

	if (math.abs(self.vy) < 1) then
       self.vy = 0
    end  
  return true
end

function Entity:moveCollideY(e)


    if (self.vy * sign(gravity) > 0) then 
            self.onGround = true
     end 

    self.vy = 0
	self.vx =self.vx * self.frictionX

	if (math.abs(self.vx) < 0.5) then 
       self.vx = 0
    end 

  return true
end



function Entity:collide(x, y, with)
  if not self.scene then
    return nil
  end
  self._x = self.x
  self._y = self.y
	self.x = self.x
  self.y = self.y


  for _, entity in ipairs(self.scene.entities) do
    if entity ~= self and entity.name == with and self:collidesPointWith(x, y, entity) then

      self.x = self._x 
      self.y = self._y

        return entity
    end
  end

  self.x = self._x 
  self.y = self._y
  return nil
end

function Entity:moveBy(x,y,type)

    self._moveX = self._moveX + x
    self._moveY = self._moveY + y
    x = math.floor(self._moveX)
    y = math.floor(self._moveY)
    self._moveX = self._moveX - x
    self._moveY = self._moveY - y
    
    local sign
        
        if x ~= 0 then
            if sweep or self:collide(self.x + x, self.y, type) then
                sign = x > 0 and 1 or -1
                
                while x ~= 0 do
                    e = self:collide(self.x + sign, self.y, type )
                    
                    if e then
                        if self:moveCollideX(e) then
                            break
                        else
                            self.x = self.x + sign
                        end
                    else
                        self.x = self.x + sign
                    end
                    
                    x = x - sign
                end
            else
                self.x = self.x + x
            end
        end
        
        if y ~= 0 then
            if sweep or self:collide(self.x, self.y + y, type) then
                sign = y > 0 and 1 or -1
                
                while y ~= 0 do
                    e = self:collide(self.x, self.y + sign, type)
                    
                    if e then
                      if self:moveCollideY(e) then
                          break
                      else
                          self.y = self.y + sign
                      end
                  else
                      self.y = self.y + sign
                  end
                  
                  y = y - sign
              end
          else
              self.y = self.y + y
          end
      end


end


function Entity:draw()
  love.graphics.setColor(1, 1, 1)  
  love.graphics.rectangle('line', self.x, self.y, self.w, self.h)
  love.graphics.setColor(1, 0, 0)  
  love.graphics.rectangle('fill', self.x + self.hitbox.ox, self.y + self.hitbox.oy, self.hitbox.w, self.hitbox.h)

  love.graphics.setColor(1, 1, 1)  


  

end

-- Objeto Scene
local Scene = {}
Scene.__index = Scene
function Scene:new()
    local s = {}
    setmetatable(s, Scene)
    s.entities = {}
    return s
end

function Scene:add(entity)
    entity.scene = self
    table.insert(self.entities, entity)
end

function Scene:update(dt)
    for _, entity in ipairs(self.entities) do
        entity:update(dt)
    end
end

function Scene:draw()
    for _, entity in ipairs(self.entities) do
        entity:draw()
    end
end







local demo = {}

function demo.load()
    
image = love.graphics.newImage("images/zazaka.png")  
print( image:getWidth() )
quad = love.graphics.newQuad(0, 0, 50, 50)

-- Exemplo de uso
scene = Scene:new()

kMoveSpeed = 0.8
kJumpForce = 20

player = Entity:new(100, 100, 32, 32,"player")
player.solid=false
player.maxVelocityX = kMoveSpeed *4
player.maxVelocityY = kJumpForce
player.frictionX = 0.82 -- floor friction
player.frictionY = 0.99 -- wall friction
player.jump = 0 
player:setHitBox(16, 32, 8, 0)


e2 = Entity:new(200, 400, 100, 5,"solid")


e3 = Entity:new(5, 600-50, 800, 20, "solid")


e4 = Entity:new(5, 0, 5, 800, "solid")
e5 = Entity:new(800-5, 0,5, 800, "solid")

e6 = Entity:new(450, 400, 100, 5,"solid")

e7 = Entity:new(400, 200, 100, 5,"solid")

e8 = Entity:new(50, 500, 100, 5,"solid")


scene:add(player)
scene:add(e2)
scene:add(e3)
scene:add(e4)
scene:add(e5)
scene:add(e6)
scene:add(e7)
scene:add(e8)





acc_x=0
acc_y=0

end

function demo.draw()
 
  local fl = player.onGround and "true" or "false"
  love.graphics.print("player vx" .. sign(player.vx) .. " vy" .. sign(player.vy) .. " OnFloor :" .. fl , 10, 20)



  
  scene:draw()
end

function demo.update(obj,dt)

    player.acx =0
    player.acy = 0

    


  if love.keyboard.isDown("left") then
    player.acx =- kMoveSpeed
  end

  if love.keyboard.isDown("right") then
    player.acx = kMoveSpeed
  end

  if love.keyboard.press("space") and player.jump<2 then
    player.acy = -sign(gravity) * kJumpForce; 
    player.jump = player.jump + 1
    player.onGround = false
  end

  if (player.onGround) then
    player.jump=0
end




    scene:update(dt)
end

return demo
