local lesson =
{
    title = "Angles and advance",
    apis = "math.cos  math.sin  math.atan2  love.graphics.rotate/translate",
    help = "Left/Right rotate the arrow, Up advances along its angle. The turret aims at the mouse.",
}

local arrow, turret

function lesson.enter()
    arrow = {x = 400, y = 300, angle = 0}
    turret = {x = 130, y = 460}
end

function lesson.update(dt)
    if love.keyboard.isDown("left") then arrow.angle = arrow.angle - 4 * dt end
    if love.keyboard.isDown("right") then arrow.angle = arrow.angle + 4 * dt end
    if love.keyboard.isDown("up") then
        arrow.x = arrow.x + math.cos(arrow.angle) * 220 * dt
        arrow.y = arrow.y + math.sin(arrow.angle) * 220 * dt
    end
    arrow.x = math.max(20, math.min(780, arrow.x))
    arrow.y = math.max(70, math.min(550, arrow.y))
end

local function drawArrow(x, y, angle, scale)
    love.graphics.push()
    love.graphics.translate(x, y)
    love.graphics.rotate(angle)
    love.graphics.polygon("fill", 20 * scale, 0, -14 * scale, -12 * scale, -6 * scale, 0, -14 * scale, 12 * scale)
    love.graphics.pop()
end

function lesson.draw()
    love.graphics.setColor(0.61, 0.78, 0.95)
    drawArrow(arrow.x, arrow.y, arrow.angle, 1.4)
    love.graphics.setColor(1, 1, 1, 0.4)
    love.graphics.line(arrow.x, arrow.y, arrow.x + math.cos(arrow.angle) * 70, arrow.y + math.sin(arrow.angle) * 70)
    love.graphics.print(("angle = %.0f degrees"):format(math.deg(arrow.angle) % 360), 12, 52)

    local mx, my = love.mouse.getPosition()
    local aim = math.atan2(my - turret.y, mx - turret.x)
    love.graphics.setColor(0.35, 0.4, 0.48)
    love.graphics.circle("fill", turret.x, turret.y, 26)
    love.graphics.setColor(1, 0.84, 0)
    drawArrow(turret.x, turret.y, aim, 1.8)
    love.graphics.setColor(1, 1, 1, 0.25)
    love.graphics.line(turret.x, turret.y, mx, my)
    love.graphics.setColor(1, 1, 1)
    love.graphics.print(("aim = %.0f, distance = %.0f"):format(math.deg(aim) % 360,
        math.sqrt((mx - turret.x) ^ 2 + (my - turret.y) ^ 2)), turret.x - 60, turret.y + 40)
end

return lesson
