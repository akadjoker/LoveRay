local lesson =
{
    title = "Collisions",
    apis = "circle overlap  distance  table.remove",
    help = "Move the ball with the arrows and eat the targets. They respawn once all are gone.",
}

local player, targets, score, flashes

local function spawn()
    targets = {}
    for i = 1, 6 do
        targets[i] = {x = love.math.random(60, 740), y = love.math.random(90, 520), radius = love.math.random(14, 26)}
    end
end

function lesson.enter()
    player = {x = 100, y = 300, radius = 20}
    score, flashes = 0, {}
    spawn()
end

function lesson.update(dt)
    local dx = (love.keyboard.isDown("right") and 1 or 0) - (love.keyboard.isDown("left") and 1 or 0)
    local dy = (love.keyboard.isDown("down") and 1 or 0) - (love.keyboard.isDown("up") and 1 or 0)
    player.x = math.max(player.radius, math.min(800 - player.radius, player.x + dx * 260 * dt))
    player.y = math.max(50 + player.radius, math.min(560 - player.radius, player.y + dy * 260 * dt))

    for i = #targets, 1, -1 do
        local t = targets[i]
        local distance = math.sqrt((t.x - player.x) ^ 2 + (t.y - player.y) ^ 2)
        if distance < t.radius + player.radius then
            score = score + 1
            flashes[#flashes + 1] = {x = t.x, y = t.y, life = 0.35, radius = t.radius}
            table.remove(targets, i)
        end
    end
    if #targets == 0 then
        spawn()
    end
    for i = #flashes, 1, -1 do
        flashes[i].life = flashes[i].life - dt
        if flashes[i].life <= 0 then
            table.remove(flashes, i)
        end
    end
end

function lesson.draw()
    for _, t in ipairs(targets) do
        local touching = (t.x - player.x) ^ 2 + (t.y - player.y) ^ 2 < (t.radius + player.radius + 40) ^ 2
        love.graphics.setColor(touching and 1 or 0.5, 0.5, 0.55)
        love.graphics.circle("fill", t.x, t.y, t.radius)
    end
    for _, f in ipairs(flashes) do
        love.graphics.setColor(1, 0.9, 0.4, f.life / 0.35)
        love.graphics.circle("line", f.x, f.y, f.radius * (2 - f.life / 0.35))
    end
    love.graphics.setColor(0.31, 0.8, 0.77)
    love.graphics.circle("fill", player.x, player.y, player.radius)
    love.graphics.setColor(1, 1, 1)
    love.graphics.print("score = " .. score .. "   (targets turn red when close)", 12, 52)
end

return lesson
