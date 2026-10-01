local W, H = 640, 400

local player, bullets, enemies, shots, booms, stars
local score, lives, over, spawnTimer, spawned

local function overlap(a, b)
    return math.abs(a.x - b.x) * 2 < a.w + b.w and math.abs(a.y - b.y) * 2 < a.h + b.h
end

local function boom(x, y)
    booms[#booms + 1] = {x = x, y = y, life = 0.45}
end

local function newGame()
    player = {x = 60, y = 200, w = 24, h = 16, cool = 0, inv = 0}
    bullets, enemies, shots, booms = {}, {}, {}, {}
    score, lives, over, spawnTimer, spawned = 0, 3, false, 1.5, 0
end

local function drawShip(x, y, flip, body, accent)
    love.graphics.push()
    love.graphics.translate(x, y)
    love.graphics.scale(flip, 1)
    love.graphics.setColor(body)
    love.graphics.rectangle("fill", -12, -2, 8, 4)
    love.graphics.rectangle("fill", -6, -4, 14, 8)
    love.graphics.rectangle("fill", -10, 2, 8, 4)
    love.graphics.setColor(accent)
    love.graphics.rectangle("fill", 6, -1, 6, 2)
    love.graphics.rectangle("fill", -2, -6, 4, 3)
    love.graphics.rectangle("fill", -12, 6, 4, 2)
    love.graphics.pop()
end

function love.load()
    love.graphics.setBackgroundColor(0.012, 0.024, 0.047)
    love.graphics.setFont(love.graphics.newFont(16))
    stars = {}
    for i = 1, 60 do
        stars[i] = {x = love.math.random(0, W), y = love.math.random(0, H), speed = love.math.random(1, 3) * 30}
    end
    newGame()
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    elseif (key == "r" or key == "return") and over then
        newGame()
    end
end

function love.update(dt)
    for _, s in ipairs(stars) do
        s.x = s.x - s.speed * dt
        if s.x < -2 then
            s.x = W + 2
            s.y = love.math.random(0, H)
        end
    end
    for i = #booms, 1, -1 do
        booms[i].life = booms[i].life - dt
        if booms[i].life <= 0 then
            table.remove(booms, i)
        end
    end
    if over then
        return
    end

    local dy = 0
    if love.keyboard.isDown("up", "w") then dy = dy - 1 end
    if love.keyboard.isDown("down", "s") then dy = dy + 1 end
    player.y = math.max(20, math.min(H - 20, player.y + dy * 300 * dt))
    player.inv = math.max(0, player.inv - dt)

    player.cool = player.cool - dt
    if love.keyboard.isDown("z", "space") and player.cool <= 0 then
        player.cool = 0.13
        bullets[#bullets + 1] = {x = player.x + 20, y = player.y, w = 12, h = 3}
    end

    for i = #bullets, 1, -1 do
        local b = bullets[i]
        b.x = b.x + 720 * dt
        local dead = b.x > W + 20
        for j = #enemies, 1, -1 do
            local e = enemies[j]
            if overlap(b, e) then
                e.hp = e.hp - 1
                e.flash = 0.08
                dead = true
                if e.hp <= 0 then
                    score = score + 100
                    boom(e.x, e.y)
                    table.remove(enemies, j)
                end
                break
            end
        end
        if dead then
            table.remove(bullets, i)
        end
    end

    spawnTimer = spawnTimer - dt
    if spawnTimer <= 0 then
        spawned = spawned + 1
        enemies[#enemies + 1] =
        {
            x = W + 30, y = love.math.random(40, H - 40), w = 24, h = 16, hp = 2,
            vy = love.math.random(-60, 60), age = 0, shoot = love.math.random() * 1.2 + 0.6, flash = 0,
        }
        spawnTimer = spawned < 10 and 1.2 or spawned < 20 and 0.85 or 0.6
    end

    for i = #enemies, 1, -1 do
        local e = enemies[i]
        e.age = e.age + dt
        e.flash = math.max(0, e.flash - dt)
        e.x = e.x - 120 * dt
        e.y = e.y + e.vy * dt + math.sin(e.age * 3) * 90 * dt
        if e.y < 20 or e.y > H - 20 then
            e.vy = -e.vy
            e.y = math.max(20, math.min(H - 20, e.y))
        end
        e.shoot = e.shoot - dt
        if e.shoot <= 0 and e.x < W and e.x > 80 then
            shots[#shots + 1] = {x = e.x - 12, y = e.y, w = 6, h = 6}
            e.shoot = love.math.random() * 1.3 + 0.7
        end
        local remove = e.x < -30
        if player.inv <= 0 and overlap(e, player) then
            boom(e.x, e.y)
            lives = lives - 1
            player.inv = 1.5
            remove = true
        end
        if remove then
            table.remove(enemies, i)
        end
    end

    for i = #shots, 1, -1 do
        local s = shots[i]
        s.x = s.x - 300 * dt
        local dead = s.x < -10
        if player.inv <= 0 and overlap(s, player) then
            boom(player.x, player.y)
            lives = lives - 1
            player.inv = 1.5
            dead = true
        end
        if dead then
            table.remove(shots, i)
        end
    end

    if lives <= 0 then
        over = true
        boom(player.x, player.y)
    end
end

function love.draw()
    love.graphics.setColor(0.59, 0.71, 0.86)
    for _, s in ipairs(stars) do
        love.graphics.rectangle("fill", s.x, s.y, 2, 2)
    end

    if not over and (player.inv <= 0 or math.floor(player.inv * 12) % 2 == 0) then
        drawShip(player.x, player.y, 1, {0.39, 0.78, 1}, {0.8, 0.94, 1})
    end

    love.graphics.setColor(1, 1, 0.8)
    for _, b in ipairs(bullets) do
        love.graphics.rectangle("fill", b.x - 6, b.y - 1, b.w, b.h)
    end

    for _, e in ipairs(enemies) do
        local body = e.flash > 0 and {1, 1, 0.4} or {0.86, 0.24, 0.24}
        drawShip(e.x, e.y, -1, body, {1, 0.63, 0.24})
    end

    love.graphics.setColor(1, 0.47, 0.24)
    for _, s in ipairs(shots) do
        love.graphics.circle("fill", s.x, s.y, 3)
    end

    for _, e in ipairs(booms) do
        local t = e.life / 0.45
        love.graphics.setColor(1, 0.78, 0.24, t)
        love.graphics.circle("fill", e.x, e.y, 16 * (1.3 - t * 0.6))
        love.graphics.setColor(1, 0.94, 0.78, t)
        love.graphics.circle("fill", e.x, e.y, 6)
    end

    love.graphics.setColor(0.49, 0.88, 1)
    love.graphics.print("SCORE " .. score, 12, 8)
    love.graphics.setColor(1, 0.42, 0.42)
    love.graphics.printf("LIVES " .. lives, 0, 8, W - 12, "right")

    if over then
        love.graphics.setColor(1, 0.88, 0.44)
        love.graphics.printf("*** GAME OVER ***", 0, 170, W, "center")
        love.graphics.setColor(0.49, 0.88, 1)
        love.graphics.printf("Final score " .. score .. " - R or Enter to restart", 0, 200, W, "center")
    end
end
