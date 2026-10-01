-- Images, quads, sprite batches and keyboard/mouse input.

local image, quad, batch
local sprites = {}
local player = { x = 400, y = 300, speed = 220, angle = 0 }

local function addSprite(x, y)
    sprites[#sprites + 1] = {
        x = x,
        y = y,
        vx = love.math.random(-120, 120),
        vy = love.math.random(-120, 120),
        scale = 0.5 + love.math.random() * 0.8,
    }
end

function love.load()
    love.graphics.setBackgroundColor(0.09, 0.1, 0.14)
    image = love.graphics.newImage("images/zazaka.png")
    quad = love.graphics.newQuad(0, 0, image:getWidth() / 2, image:getHeight(), image)
    batch = love.graphics.newSpriteBatch(image, 2000)
    for _ = 1, 50 do
        addSprite(love.math.random(0, 800), love.math.random(0, 600))
    end
end

function love.update(dt)
    local w, h = love.graphics.getDimensions()
    for _, s in ipairs(sprites) do
        s.x = s.x + s.vx * dt
        s.y = s.y + s.vy * dt
        if s.x < 0 or s.x > w then s.vx = -s.vx end
        if s.y < 0 or s.y > h then s.vy = -s.vy end
    end

    local dx, dy = 0, 0
    if love.keyboard.isDown("left", "a") then dx = dx - 1 end
    if love.keyboard.isDown("right", "d") then dx = dx + 1 end
    if love.keyboard.isDown("up", "w") then dy = dy - 1 end
    if love.keyboard.isDown("down", "s") then dy = dy + 1 end
    player.x = player.x + dx * player.speed * dt
    player.y = player.y + dy * player.speed * dt
    player.angle = player.angle + dt

    if love.mouse.isDown(1) then
        addSprite(love.mouse.getPosition())
    end

    batch:clear()
    for _, s in ipairs(sprites) do
        batch:add(s.x, s.y, 0, s.scale, s.scale, image:getWidth() / 2, image:getHeight() / 2)
    end
end

function love.draw()
    love.graphics.setColor(1, 1, 1, 0.85)
    love.graphics.draw(batch)

    love.graphics.setColor(1, 1, 1)
    love.graphics.draw(image, player.x, player.y, player.angle, 1.5, 1.5, image:getWidth() / 2, image:getHeight() / 2)

    -- Left half of the image through a quad, flipped horizontally with a negative scale.
    love.graphics.draw(image, quad, 60, 500, 0, 1, 1)
    love.graphics.draw(image, quad, 200, 500, 0, -1, 1)

    love.graphics.setColor(0, 0, 0, 0.6)
    love.graphics.rectangle("fill", 0, 0, 330, 48)
    love.graphics.setColor(1, 1, 1)
    love.graphics.print(string.format("sprites: %d   fps: %d", #sprites, love.timer.getFPS()), 10, 8)
    love.graphics.print("WASD/arrows move, left click spawns, Esc quits", 10, 26)
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    elseif key == "space" then
        for _ = 1, 100 do
            addSprite(love.math.random(0, 800), love.math.random(0, 600))
        end
    end
end
