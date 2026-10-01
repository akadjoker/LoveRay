local lesson =
{
    title = "Math and trigonometry",
    apis = "math.sin  math.cos  math.atan2  love.math.noise  lerp",
    help = "The unit circle drives sine and cosine waves. Move the mouse to change the target angle.",
}

local function lerp(a, b, t)
    return a + (b - a) * t
end

local smooth = 0
local history

function lesson.enter()
    history = {}
    smooth = 0
end

function lesson.update(dt)
    local mx, my = love.mouse.getPosition()
    local target = math.atan2(my - 300, mx - 190)
    local diff = (target - smooth + math.pi) % (math.pi * 2) - math.pi
    smooth = smooth + diff * math.min(1, 6 * dt)

    table.insert(history, 1, {math.sin(smooth), math.cos(smooth)})
    if #history > 330 then
        table.remove(history)
    end
end

function lesson.draw()
    local cx, cy, r = 190, 300, 120
    love.graphics.setColor(0.2, 0.28, 0.38)
    love.graphics.circle("line", cx, cy, r)
    love.graphics.line(cx - r - 20, cy, cx + r + 20, cy)
    love.graphics.line(cx, cy - r - 20, cx, cy + r + 20)

    local c, s = math.cos(smooth), math.sin(smooth)
    local px, py = cx + c * r, cy + s * r
    love.graphics.setColor(1, 0.84, 0)
    love.graphics.line(cx, cy, px, py)
    love.graphics.circle("fill", px, py, 6)
    love.graphics.setColor(1, 0.42, 0.42)
    love.graphics.line(cx, cy, px, cy)
    love.graphics.setColor(0.31, 0.8, 0.77)
    love.graphics.line(px, cy, px, py)

    love.graphics.setColor(0.84, 0.9, 0.96)
    love.graphics.print(("angle %.1f deg   cos %.3f   sin %.3f"):format(math.deg(smooth) % 360, c, s), 70, 450)

    local gx, gy = 400, 190
    love.graphics.setColor(0.2, 0.28, 0.38)
    love.graphics.line(gx, gy, gx + 340, gy)
    love.graphics.line(gx, gy + 120, gx + 340, gy + 120)
    local sinPoints, cosPoints = {}, {}
    for i, h in ipairs(history) do
        local x = gx + (i - 1)
        sinPoints[#sinPoints + 1] = x
        sinPoints[#sinPoints + 1] = gy - h[1] * 50 + 0
        cosPoints[#cosPoints + 1] = x
        cosPoints[#cosPoints + 1] = gy + 120 - h[2] * 50
    end
    love.graphics.setLineWidth(2)
    if #history > 1 then
        love.graphics.setColor(0.31, 0.8, 0.77)
        love.graphics.line(sinPoints)
        love.graphics.setColor(1, 0.42, 0.42)
        love.graphics.line(cosPoints)
    end
    love.graphics.setLineWidth(1)
    love.graphics.setColor(0.31, 0.8, 0.77)
    love.graphics.print("sin", gx + 346, gy - 8)
    love.graphics.setColor(1, 0.42, 0.42)
    love.graphics.print("cos", gx + 346, gy + 112)

    local t = love.timer.getTime()
    love.graphics.setColor(0.84, 0.9, 0.96)
    love.graphics.print("lerp, noise and a Lissajous curve", 400, 360)
    local x = lerp(400, 740, (math.sin(t) + 1) / 2)
    love.graphics.setColor(1, 0.84, 0)
    love.graphics.circle("fill", x, 395, 7)
    love.graphics.setColor(0.77, 0.54, 1)
    local pts = {}
    for i = 0, 100 do
        local noise = love.math.noise(i / 12, t * 0.6)
        pts[#pts + 1] = 400 + i * 3.4
        pts[#pts + 1] = 430 + (noise - 0.5) * 60
    end
    love.graphics.line(pts)
    love.graphics.setColor(0.61, 0.78, 0.95)
    local liss = {}
    for i = 0, 200 do
        local a = i / 200 * math.pi * 2
        liss[#liss + 1] = 570 + math.sin(a * 3 + t) * 80
        liss[#liss + 1] = 510 + math.sin(a * 2) * 40
    end
    love.graphics.line(liss)
end

return lesson
