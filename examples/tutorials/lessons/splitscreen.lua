local lesson =
{
    title = "Split screen",
    apis = "love.graphics.setScissor  push/pop  translate  independent cameras",
    help = "Player one: WASD. Player two: arrow keys. Each half has its own region and camera.",
}

local players, walls

function lesson.load()
    walls = {}
    for i = 1, 60 do
        walls[i] = {x = love.math.random(-600, 1400), y = love.math.random(-400, 1000), w = love.math.random(30, 120), h = love.math.random(30, 120)}
    end
end

function lesson.enter()
    players =
    {
        {x = 100, y = 100, color = {0.31, 0.8, 0.77}, keys = {"a", "d", "w", "s"}},
        {x = 400, y = 300, color = {1, 0.55, 0.35}, keys = {"left", "right", "up", "down"}},
    }
end

function lesson.update(dt)
    for _, p in ipairs(players) do
        local k = p.keys
        local dx = (love.keyboard.isDown(k[2]) and 1 or 0) - (love.keyboard.isDown(k[1]) and 1 or 0)
        local dy = (love.keyboard.isDown(k[4]) and 1 or 0) - (love.keyboard.isDown(k[3]) and 1 or 0)
        p.x = p.x + dx * 260 * dt
        p.y = p.y + dy * 260 * dt
    end
end

local function drawWorld(self)
    love.graphics.setColor(0.1, 0.16, 0.24)
    love.graphics.rectangle("fill", -2000, -2000, 4000, 4000)
    love.graphics.setColor(0.16, 0.24, 0.34)
    for gx = -1000, 2000, 100 do
        love.graphics.line(gx, -1000, gx, 2000)
    end
    for gy = -1000, 2000, 100 do
        love.graphics.line(-1000, gy, 2000, gy)
    end
    love.graphics.setColor(0.35, 0.42, 0.5)
    for _, w in ipairs(walls) do
        love.graphics.rectangle("fill", w.x, w.y, w.w, w.h)
    end
    for _, p in ipairs(players) do
        love.graphics.setColor(p.color)
        love.graphics.circle("fill", p.x, p.y, 14)
        love.graphics.setColor(1, 1, 1)
        love.graphics.circle("line", p.x, p.y, 14)
    end
end

function lesson.draw()
    local top, height = 50, 258
    for i, p in ipairs(players) do
        local y = top + (i - 1) * (height + 2)
        love.graphics.setScissor(0, y, 800, height)
        love.graphics.push()
        love.graphics.translate(400 - p.x, y + height / 2 - p.y)
        drawWorld()
        love.graphics.pop()
        love.graphics.setScissor()
        love.graphics.setColor(p.color)
        love.graphics.rectangle("line", 1, y, 798, height)
    end
end

return lesson
