local W, H = 640, 480
local TAU = math.pi * 2

local sizes = {
    [3] = {radius = 20, score = 20, speed = 40},
    [2] = {radius = 12, score = 50, speed = 70},
    [1] = {radius = 7, score = 100, speed = 100},
}

local ship, bullets, rocks, booms, stars
local score, lives, wave, over

local function wrap(o, margin)
    if o.x < -margin then o.x = o.x + W + margin * 2 end
    if o.x > W + margin then o.x = o.x - W - margin * 2 end
    if o.y < -margin then o.y = o.y + H + margin * 2 end
    if o.y > H + margin then o.y = o.y - H - margin * 2 end
end

local function newRock(x, y, size, angle)
    local points = {}
    local count = 9
    for i = 0, count - 1 do
        local a = i / count * TAU
        local r = sizes[size].radius * (0.75 + love.math.random() * 0.45)
        points[#points + 1] = math.cos(a) * r
        points[#points + 1] = math.sin(a) * r
    end
    local speed = sizes[size].speed * (0.6 + love.math.random() * 0.8)
    rocks[#rocks + 1] =
    {
        x = x, y = y, size = size, points = points,
        vx = math.cos(angle) * speed, vy = math.sin(angle) * speed,
        rot = love.math.random() * TAU, spin = love.math.random() * 2 - 1,
    }
end

local function spawnWave()
    wave = wave + 1
    local n = math.min(wave + 3, 9)
    for _ = 1, n do
        local x, y
        repeat
            x, y = love.math.random(0, W), love.math.random(0, H)
        until (x - ship.x) ^ 2 + (y - ship.y) ^ 2 > 150 ^ 2
        newRock(x, y, 3, love.math.random() * TAU)
    end
end

local function boom(x, y, r)
    booms[#booms + 1] = {x = x, y = y, r = r, life = 0.4}
end

local function resetShip()
    ship = {x = W / 2, y = H / 2, vx = 0, vy = 0, a = -math.pi / 2, cool = 0, inv = 2}
end

local function newGame()
    bullets, rocks, booms = {}, {}, {}
    score, lives, wave, over = 0, 3, 0, false
    resetShip()
    spawnWave()
end

function love.load()
    love.graphics.setBackgroundColor(0.012, 0.024, 0.047)
    love.graphics.setFont(love.graphics.newFont(16))
    stars = {}
    for i = 1, 80 do
        stars[i] = {love.math.random(0, W), love.math.random(0, H), love.math.random() * 0.4 + 0.15}
    end
    newGame()
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    elseif (key == "r" or key == "return") and over then
        newGame()
    elseif key == "h" and not over then
        ship.x, ship.y = love.math.random(20, W - 20), love.math.random(20, H - 20)
        ship.vx, ship.vy = 0, 0
    end
end

local function hit(a, b, r)
    return (a.x - b.x) ^ 2 + (a.y - b.y) ^ 2 < r * r
end

function love.update(dt)
    for i = #booms, 1, -1 do
        booms[i].life = booms[i].life - dt
        if booms[i].life <= 0 then
            table.remove(booms, i)
        end
    end

    for _, r in ipairs(rocks) do
        r.x = r.x + r.vx * dt
        r.y = r.y + r.vy * dt
        r.rot = r.rot + r.spin * dt
        wrap(r, 30)
    end

    if over then
        return
    end

    if love.keyboard.isDown("left") then ship.a = ship.a - 4.5 * dt end
    if love.keyboard.isDown("right") then ship.a = ship.a + 4.5 * dt end
    if love.keyboard.isDown("up") then
        ship.vx = ship.vx + math.cos(ship.a) * 260 * dt
        ship.vy = ship.vy + math.sin(ship.a) * 260 * dt
    end
    local drag = 0.98 ^ (dt * 60)
    ship.vx, ship.vy = ship.vx * drag, ship.vy * drag
    local speed = math.sqrt(ship.vx ^ 2 + ship.vy ^ 2)
    if speed > 480 then
        ship.vx, ship.vy = ship.vx / speed * 480, ship.vy / speed * 480
    end
    ship.x = ship.x + ship.vx * dt
    ship.y = ship.y + ship.vy * dt
    wrap(ship, 10)
    ship.inv = math.max(0, ship.inv - dt)

    ship.cool = ship.cool - dt
    if love.keyboard.isDown("space") and ship.cool <= 0 then
        ship.cool = 0.2
        bullets[#bullets + 1] =
        {
            x = ship.x + math.cos(ship.a) * 12, y = ship.y + math.sin(ship.a) * 12,
            vx = math.cos(ship.a) * 720 + ship.vx, vy = math.sin(ship.a) * 720 + ship.vy,
            life = 1,
        }
    end

    for i = #bullets, 1, -1 do
        local b = bullets[i]
        b.x = b.x + b.vx * dt
        b.y = b.y + b.vy * dt
        b.life = b.life - dt
        wrap(b, 4)
        local dead = b.life <= 0
        for j = #rocks, 1, -1 do
            local r = rocks[j]
            if hit(b, r, sizes[r.size].radius + 2) then
                score = score + sizes[r.size].score
                boom(r.x, r.y, sizes[r.size].radius * 1.6)
                table.remove(rocks, j)
                if r.size > 1 then
                    newRock(r.x, r.y, r.size - 1, love.math.random() * TAU)
                    newRock(r.x, r.y, r.size - 1, love.math.random() * TAU)
                end
                dead = true
                break
            end
        end
        if dead then
            table.remove(bullets, i)
        end
    end

    if ship.inv <= 0 then
        for j = #rocks, 1, -1 do
            local r = rocks[j]
            if hit(ship, r, sizes[r.size].radius + 8) then
                boom(ship.x, ship.y, 30)
                table.remove(rocks, j)
                lives = lives - 1
                if lives <= 0 then
                    over = true
                else
                    resetShip()
                end
                break
            end
        end
    end

    if #rocks == 0 and not over then
        spawnWave()
    end
end

function love.draw()
    for _, s in ipairs(stars) do
        love.graphics.setColor(0.4, 0.55, 0.8, s[3])
        love.graphics.rectangle("fill", s[1], s[2], 1, 1)
    end

    love.graphics.setLineWidth(1.5)
    love.graphics.setColor(0.78, 0.65, 0.45)
    for _, r in ipairs(rocks) do
        love.graphics.push()
        love.graphics.translate(r.x, r.y)
        love.graphics.rotate(r.rot)
        love.graphics.polygon("line", r.points)
        love.graphics.pop()
    end

    love.graphics.setColor(1, 1, 0.5)
    for _, b in ipairs(bullets) do
        love.graphics.circle("fill", b.x, b.y, 2)
    end

    for _, e in ipairs(booms) do
        local t = e.life / 0.4
        love.graphics.setColor(1, 0.7, 0.3, t)
        love.graphics.circle("line", e.x, e.y, e.r * (1.4 - t))
    end

    if not over and (ship.inv <= 0 or math.floor(ship.inv * 10) % 2 == 0) then
        love.graphics.push()
        love.graphics.translate(ship.x, ship.y)
        love.graphics.rotate(ship.a)
        love.graphics.setColor(0.47, 0.86, 1)
        love.graphics.polygon("line", 12, 0, -8, -7, -5, 0, -8, 7)
        if love.keyboard.isDown("up") then
            love.graphics.setColor(1, 0.63, 0.24)
            love.graphics.polygon("line", -6, -3, -13 - love.math.random() * 4, 0, -6, 3)
        end
        love.graphics.pop()
    end

    love.graphics.setColor(0.49, 0.88, 1)
    love.graphics.print("SCORE " .. score, 12, 8)
    love.graphics.setColor(0.48, 0.69, 0.88)
    love.graphics.printf("WAVE " .. wave, 0, 8, W, "center")
    love.graphics.setColor(1, 0.42, 0.42)
    love.graphics.printf("LIVES " .. lives, 0, 8, W - 12, "right")

    if over then
        love.graphics.setColor(1, 0.88, 0.44)
        love.graphics.printf("*** GAME OVER ***", 0, 210, W, "center")
        love.graphics.setColor(0.49, 0.88, 1)
        love.graphics.printf("Press R or Enter", 0, 238, W, "center")
    end
end
