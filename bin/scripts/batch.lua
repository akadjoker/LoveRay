local demo = {}
local sprites = {} -- cria a tabela "sprite" para armazenar as informações do sprite
local type =0
function demo.load()
    local sprite = {} -- cria a tabela "sprite" para armazenar as informações do sprite
    sprite.x = 100 -- posição inicial X do sprite
    sprite.y = 100 -- posição inicial Y do sprite
    sprite.speed = 100 -- velocidade do sprite em pixels por segundo
    sprite.direction = {x = 1, y = 1} -- direção inicial do sprite
    table.insert(sprites, sprite) -- adiciona o sprite à tabela de sprites
    image = love.graphics.newImage("images/zazaka.png")

end

function demo.draw()
    love.graphics.setColor(1, 1, 1,1)
    for i, sprite in ipairs(sprites) do
        
            love.graphics.draw(image,sprite.x, sprite.y)
    end
    love.graphics.setColor(0, 0, 0,1)
    love.graphics.rectangle("fill",200,10,240,22)
    love.graphics.setColor(1, 0, 1,1)
    love.graphics.print("Sprites: " .. #sprites .. " FPS: " .. tostring(love.timer.getFPS()) , 200, 10)
end

function demo.update(d,dt)
    
    for i, sprite in ipairs(sprites) do
        sprite.x = sprite.x + sprite.speed * sprite.direction.x * dt
        sprite.y = sprite.y + sprite.speed * sprite.direction.y * dt
    
        -- verifica se o sprite bateu na borda da tela
        if sprite.x < 0 or sprite.x > love.graphics.getWidth() - 20 then
          sprite.direction.x = -sprite.direction.x -- inverte a direção X do sprite
        end
        if sprite.y < 0 or sprite.y > love.graphics.getHeight() - 20 then
          sprite.direction.y = -sprite.direction.y -- inverte a direção Y do sprite
        end
      end
    if (love.keyboard.isDown("right")) then
        type = 1
    end
    if (love.keyboard.isDown("left")) then
        type = 0
    end

    if (love.mouse.isDown(0)) then
        for i=1,100 do -- adiciona 100 novos sprites
            local newSprite = {}
            newSprite.x = math.random(0, love.graphics.getWidth() - 20) -- posição X aleatória
            newSprite.y = math.random(0, love.graphics.getHeight() - 20) -- posição Y aleatória
            newSprite.speed = math.random(50, 200) -- velocidade aleatória
            newSprite.direction = {x = math.random(0, 1) == 0 and -1 or 1, y = math.random(0, 1) == 0 and -1 or 1} -- direção aleatória
            table.insert(sprites, newSprite) -- adiciona o novo sprite à tabela de sprites
          end
    end
end

return demo
