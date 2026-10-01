local W, H = 640, 400
local WORLD_W = 3200
local GRAVITY, JUMP, MOVE = 1500, -520, 220

local solids = {}
local coins = {}
local spikes = {}
local player, camera, collected, lives, goal, done, clouds

local function solid(x, y, w, h, r, g, b)
    solids[#solids + 1] = {x = x, y = y, w = w, h = h, color = {r / 255, g / 255, b / 255}}
end

local function build()
    local ground =
    {
        {0, 400}, {400, 300}, {700, 200}, {900, 400}, {1300, 300}, {1600, 400}, {2000, 500}, {2500, 700},
    }
    for _, g in ipairs(ground) do
        solid(g[1], 384, g[2], 16, 60, 100, 50)
    end

    local platforms =
    {
        {130, 310, 64}, {230, 260, 64}, {330, 210, 80}, {480, 320, 48}, {570, 280, 48},
        {720, 330, 48}, {780, 290, 48}, {840, 250, 48}, {980, 290, 64}, {1080, 330, 64},
        {1250, 300, 48}, {1320, 250, 48}, {1390, 200, 48}, {1550, 300, 48}, {1650, 280, 48},
        {1750, 260, 48}, {1850, 280, 48}, {2050, 320, 64}, {2150, 280, 64}, {2280, 240, 80},
    }
    for _, p in ipairs(platforms) do
        solid(p[1], p[2], p[3], 12, 70, 110, 140)
    end

    for _, x in ipairs({430, 675, 875, 1200, 1500, 1950, 1980, 1300, 1320}) do
        spikes[#spikes + 1] = {x = x, y = 374, w = 14, h = 10}
    end

    local coinSpots =
    {
        {150, 280}, {250, 230}, {350, 180}, {500, 290}, {590, 250}, {740, 300}, {800, 260}, {860, 220},
        {1000, 260}, {1100, 300}, {1270, 270}, {1340, 220}, {1410, 170}, {1570, 270}, {1670, 250},
        {1770, 230}, {1870, 250}, {2070, 290}, {2170, 250}, {2300, 210},
    }
    for _, c in ipairs(coinSpots) do
        coins[#coins + 1] = {x = c[1], y = c[2], phase = love.math.random() * 6, taken = false}
    end

    goal = {x = 3100, y = 360, w = 16, h = 24}
end

local function respawn()
    player.x, player.y, player.vx, player.vy = 80, 100, 0, 0
end

local function overlaps(a, b)
    return a.x < b.x + b.w and a.x + a.w > b.x and a.y < b.y + b.h and a.y + a.h > b.y
end

local function hitsSolid()
    for _, s in ipairs(solids) do
        if overlaps(player, s) then
            return s
        end
    end
end

local function reset()
    player = {x = 80, y = 100, w = 12, h = 16, vx = 0, vy = 0, ground = false, facing = 1}
    for _, c in ipairs(coins) do
        c.taken = false
    end
    collected, lives, done = 0, 3, false
    camera = 0
end

function love.load()
    love.graphics.setBackgroundColor(0.047, 0.075, 0.125)
    love.graphics.setFont(love.graphics.newFont(16))
    build()
    clouds = {}
    for i = 1, 14 do
        clouds[i] = {x = love.math.random(0, WORLD_W), y = love.math.random(20, 200), w = love.math.random(40, 120)}
    end
    reset()
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    elseif key == "r" then
        reset()
    elseif (key == "up" or key == "z" or key == "space") and player.ground and not done then
        player.vy = JUMP
        player.ground = false
    end
end

function love.update(dt)
    if done then
        return
    end

    player.vx = 0
    if love.keyboard.isDown("left") then
        player.vx = -MOVE
        player.facing = -1
    end
    if love.keyboard.isDown("right") then
        player.vx = MOVE
        player.facing = 1
    end

    player.vy = math.min(player.vy + GRAVITY * dt, 720)

    player.x = player.x + player.vx * dt
    local s = hitsSolid()
    if s then
        if player.vx > 0 then
            player.x = s.x - player.w
        else
            player.x = s.x + s.w
        end
    end

    player.y = player.y + player.vy * dt
    player.ground = false
    s = hitsSolid()
    if s then
        if player.vy > 0 then
            player.y = s.y - player.h
            player.ground = true
        else
            player.y = s.y + s.h
        end
        player.vy = 0
    end

    player.x = math.max(0, player.x)

    for _, k in ipairs(spikes) do
        if overlaps(player, k) then
            lives = lives - 1
            respawn()
            break
        end
    end
    if player.y > 450 then
        lives = lives - 1
        respawn()
    end
    if lives <= 0 then
        reset()
    end

    for _, c in ipairs(coins) do
        if not c.taken and overlaps(player, {x = c.x - 5, y = c.y - 5, w = 10, h = 10}) then
            c.taken = true
            collected = collected + 1
        end
    end
    if overlaps(player, goal) then
        done = true
    end

    local target = math.max(0, math.min(WORLD_W - W, player.x - W / 2))
    camera = camera + (target - camera) * math.min(1, 6 * dt)
end

function love.draw()
    local t = love.timer.getTime()

    love.graphics.setColor(1, 1, 1, 0.05)
    for _, c in ipairs(clouds) do
        local x = c.x - camera * 0.4
        love.graphics.rectangle("fill", x, c.y, c.w, 14, 7)
    end

    love.graphics.push()
    love.graphics.translate(-math.floor(camera), 0)

    for _, s in ipairs(solids) do
        local c = s.color
        love.graphics.setColor(c[1], c[2], c[3])
        love.graphics.rectangle("fill", s.x, s.y, s.w, s.h)
        love.graphics.setColor(c[1] + 0.12, c[2] + 0.12, c[3] + 0.08)
        love.graphics.rectangle("fill", s.x, s.y, s.w, 4)
    end

    love.graphics.setColor(0.86, 0.39, 0.39)
    for _, k in ipairs(spikes) do
        love.graphics.polygon("fill", k.x, k.y + k.h, k.x + k.w / 2, k.y, k.x + k.w, k.y + k.h)
    end

    for _, c in ipairs(coins) do
        if not c.taken then
            local y = c.y + math.sin(t * 5 + c.phase) * 3
            love.graphics.setColor(1, 0.82, 0.16)
            love.graphics.circle("fill", c.x, y, 5)
            love.graphics.setColor(1, 0.98, 0.47)
            love.graphics.circle("fill", c.x, y, 2)
        end
    end

    love.graphics.setColor(0.47, 0.39, 0.24)
    love.graphics.rectangle("fill", goal.x, goal.y, 2, goal.h)
    love.graphics.setColor(1, 0.78, 0.24)
    love.graphics.polygon("fill", goal.x + 2, goal.y + 2, goal.x + 16, goal.y + 6, goal.x + 2, goal.y + 10)

    local px, py = player.x, player.y
    love.graphics.push()
    love.graphics.translate(px + player.w / 2, py)
    love.graphics.scale(player.facing, 1)
    love.graphics.setColor(0.39, 0.86, 0.39)
    love.graphics.rectangle("fill", -3, 1, 6, 5)
    love.graphics.setColor(0.08, 0.08, 0.08)
    love.graphics.rectangle("fill", 0, 3, 1, 1)
    love.graphics.rectangle("fill", 2, 3, 1, 1)
    love.graphics.setColor(0.31, 0.7, 0.31)
    love.graphics.rectangle("fill", -2, 6, 4, 6)
    love.graphics.setColor(0.24, 0.55, 0.24)
    love.graphics.rectangle("fill", -3, 12, 2, 4)
    love.graphics.rectangle("fill", 1, 12, 2, 4)
    love.graphics.pop()

    love.graphics.pop()

    love.graphics.setColor(1, 0.84, 0)
    love.graphics.print("Coins " .. collected .. "/" .. #coins, 10, 8)
    love.graphics.setColor(1, 0.42, 0.42)
    love.graphics.printf("Lives " .. lives, 0, 8, W - 10, "right")
    if done then
        love.graphics.setColor(0.42, 1, 0.6)
        love.graphics.printf("*** LEVEL COMPLETE ***  (R to replay)", 0, 180, W, "center")
    end
end
