local lesson =
{
    title = "Movement",
    apis = "love.keyboard.isDown  love.update(dt)  love.graphics.circle",
    help = "Arrow keys or WASD move the ball. Speed is in pixels per second, scaled by dt.",
}

local ball, trail

function lesson.enter()
    ball = {x = 400, y = 300, radius = 18}
    trail = {}
end

function lesson.update(dt)
    local dx, dy = 0, 0
    if love.keyboard.isDown("left", "a") then dx = dx - 1 end
    if love.keyboard.isDown("right", "d") then dx = dx + 1 end
    if love.keyboard.isDown("up", "w") then dy = dy - 1 end
    if love.keyboard.isDown("down", "s") then dy = dy + 1 end

    if dx ~= 0 and dy ~= 0 then
        dx, dy = dx * 0.7071, dy * 0.7071
    end

    local speed = love.keyboard.isDown("lshift", "rshift") and 480 or 240
    ball.x = math.max(ball.radius, math.min(800 - ball.radius, ball.x + dx * speed * dt))
    ball.y = math.max(60 + ball.radius, math.min(560 - ball.radius, ball.y + dy * speed * dt))

    table.insert(trail, 1, {ball.x, ball.y})
    if #trail > 30 then
        table.remove(trail)
    end
end

function lesson.draw()
    for i = #trail, 1, -1 do
        local t = trail[i]
        love.graphics.setColor(0.31, 0.8, 0.77, (1 - i / #trail) * 0.4)
        love.graphics.circle("fill", t[1], t[2], ball.radius * (1 - i / #trail * 0.6))
    end
    love.graphics.setColor(0.31, 0.8, 0.77)
    love.graphics.circle("fill", ball.x, ball.y, ball.radius)
    love.graphics.setColor(1, 1, 1)
    love.graphics.circle("line", ball.x, ball.y, ball.radius)
    love.graphics.print(("x=%.0f y=%.0f   hold Shift to run"):format(ball.x, ball.y), 12, 52)
end

return lesson
