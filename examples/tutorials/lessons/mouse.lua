local lesson =
{
    title = "Mouse and input",
    apis = "love.mouse.getPosition  isDown  mousepressed  wheelmoved  love.keyboard",
    help = "Left click drops a dot, right click removes the closest one, the wheel resizes the cursor.",
}

local dots, radius, held

function lesson.enter()
    dots, radius, held = {}, 20, {}
end

function lesson.mousepressed(x, y, button)
    if y < 50 or y > 560 then
        return
    end
    if button == 1 then
        dots[#dots + 1] = {x = x, y = y, r = radius, hue = love.math.random()}
    elseif button == 2 then
        local best, bestDistance
        for i, d in ipairs(dots) do
            local distance = (d.x - x) ^ 2 + (d.y - y) ^ 2
            if not bestDistance or distance < bestDistance then
                best, bestDistance = i, distance
            end
        end
        if best then
            table.remove(dots, best)
        end
    end
end

function lesson.wheelmoved(x, y)
    radius = math.max(4, math.min(80, radius + y * 3))
end

function lesson.keypressed(key)
    if key == "c" then
        dots = {}
    end
end

function lesson.draw()
    for _, d in ipairs(dots) do
        love.graphics.setColor(0.5 + 0.5 * math.sin(d.hue * 6.28), 0.5 + 0.5 * math.sin(d.hue * 6.28 + 2), 0.5 + 0.5 * math.sin(d.hue * 6.28 + 4), 0.85)
        love.graphics.circle("fill", d.x, d.y, d.r)
    end

    local mx, my = love.mouse.getPosition()
    love.graphics.setColor(1, 1, 1)
    love.graphics.circle("line", mx, my, radius)
    love.graphics.line(mx - 6, my, mx + 6, my)
    love.graphics.line(mx, my - 6, mx, my + 6)

    local buttons = {}
    for b = 1, 3 do
        buttons[#buttons + 1] = love.mouse.isDown(b) and ("[" .. b .. "]") or (" " .. b .. " ")
    end
    love.graphics.setColor(0.84, 0.9, 0.96)
    love.graphics.print(("mouse %d, %d   buttons %s   dots %d   radius %d   C clears"):format(mx, my, table.concat(buttons), #dots, radius), 12, 52)

    local down = {}
    for _, k in ipairs({"w", "a", "s", "d", "space", "lshift", "lctrl", "up", "down", "left", "right"}) do
        if love.keyboard.isDown(k) then
            down[#down + 1] = k
        end
    end
    love.graphics.print("keys down: " .. table.concat(down, " "), 12, 540)
end

return lesson
