local lesson =
{
    title = "Collision shapes",
    apis = "circle vs circle  box vs box  circle vs box  compound boxes",
    help = "Drag with the left mouse button. Each shape tests against the moving probe in its own way.",
}

local probe

local function circleCircle(a, b)
    return (a.x - b.x) ^ 2 + (a.y - b.y) ^ 2 < (a.r + b.r) ^ 2
end

local function boxBox(a, b)
    return a.x < b.x + b.w and a.x + a.w > b.x and a.y < b.y + b.h and a.y + a.h > b.y
end

local function circleBox(c, b)
    local nx = math.max(b.x, math.min(c.x, b.x + b.w))
    local ny = math.max(b.y, math.min(c.y, b.y + b.h))
    return (c.x - nx) ^ 2 + (c.y - ny) ^ 2 < c.r * c.r
end

local circle = {x = 200, y = 200, r = 50}
local box = {x = 450, y = 150, w = 120, h = 90}
local compound =
{
    {x = 150, y = 400, w = 160, h = 24},
    {x = 226, y = 360, w = 24, h = 110},
    {x = 380, y = 380, w = 60, h = 60},
    {x = 460, y = 420, w = 100, h = 24},
}

function lesson.enter()
    probe = {x = 400, y = 300, r = 24, dragging = false}
end

function lesson.mousepressed(x, y, button)
    if button == 1 then
        probe.dragging = true
    end
end

function lesson.mousereleased(x, y, button)
    if button == 1 then
        probe.dragging = false
    end
end

function lesson.update(dt)
    if probe.dragging then
        probe.x, probe.y = love.mouse.getPosition()
    end
end

local function tint(hit)
    if hit then
        love.graphics.setColor(1, 0.42, 0.42)
    else
        love.graphics.setColor(0.31, 0.8, 0.77)
    end
end

function lesson.draw()
    local probeBox = {x = probe.x - probe.r, y = probe.y - probe.r, w = probe.r * 2, h = probe.r * 2}

    tint(circleCircle(probe, circle))
    love.graphics.circle("fill", circle.x, circle.y, circle.r)
    love.graphics.setColor(1, 1, 1)
    love.graphics.print("circle vs circle", circle.x - 45, circle.y + 60)

    tint(circleBox(probe, box))
    love.graphics.rectangle("fill", box.x, box.y, box.w, box.h)
    love.graphics.setColor(1, 1, 1)
    love.graphics.print("circle vs box", box.x + 10, box.y + box.h + 8)

    local hit = false
    for _, b in ipairs(compound) do
        hit = hit or boxBox(probeBox, b)
    end
    tint(hit)
    for _, b in ipairs(compound) do
        love.graphics.rectangle("fill", b.x, b.y, b.w, b.h)
    end
    love.graphics.setColor(1, 1, 1)
    love.graphics.print("compound boxes vs probe box", 150, 500)

    love.graphics.setColor(1, 0.84, 0, 0.9)
    love.graphics.circle("line", probe.x, probe.y, probe.r)
    love.graphics.setColor(1, 0.84, 0, 0.35)
    love.graphics.rectangle("line", probeBox.x, probeBox.y, probeBox.w, probeBox.h)
end

return lesson
