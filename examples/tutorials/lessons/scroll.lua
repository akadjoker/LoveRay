local lesson =
{
    title = "Scroll and regions",
    apis = "love.graphics.setScissor  translate  parallax layers",
    help = "Left/Right scroll the world. The scene is clipped to a region with setScissor.",
}

local region = {x = 60, y = 80, w = 680, h = 400}
local camera, hills, trees

function lesson.load()
    hills, trees = {}, {}
    for i = 1, 40 do
        hills[i] = {x = i * 140, h = love.math.random(60, 160), w = love.math.random(120, 220)}
        trees[i] = {x = i * 90 + love.math.random(0, 40), h = love.math.random(30, 60)}
    end
end

function lesson.enter()
    camera = 0
end

function lesson.update(dt)
    local speed = love.keyboard.isDown("lshift", "rshift") and 600 or 240
    if love.keyboard.isDown("left") then camera = math.max(0, camera - speed * dt) end
    if love.keyboard.isDown("right") then camera = camera + speed * dt end
end

function lesson.draw()
    love.graphics.setColor(1, 1, 1, 0.3)
    love.graphics.rectangle("line", region.x - 1, region.y - 1, region.w + 2, region.h + 2)

    love.graphics.setScissor(region.x, region.y, region.w, region.h)
    love.graphics.setColor(0.2, 0.35, 0.55)
    love.graphics.rectangle("fill", region.x, region.y, region.w, region.h)

    love.graphics.setColor(0.28, 0.42, 0.62)
    for _, h in ipairs(hills) do
        local x = region.x + h.x - camera * 0.3
        love.graphics.polygon("fill", x, region.y + region.h, x + h.w / 2, region.y + region.h - h.h, x + h.w, region.y + region.h)
    end

    love.graphics.setColor(0.15, 0.4, 0.25)
    love.graphics.rectangle("fill", region.x, region.y + region.h - 50, region.w, 50)

    for _, t in ipairs(trees) do
        local x = region.x + t.x - camera * 0.7
        love.graphics.setColor(0.4, 0.28, 0.18)
        love.graphics.rectangle("fill", x - 3, region.y + region.h - 50 - t.h, 6, t.h)
        love.graphics.setColor(0.1, 0.5, 0.25)
        love.graphics.circle("fill", x, region.y + region.h - 50 - t.h, 16)
    end

    love.graphics.setColor(0.3, 0.2, 0.12)
    for i = -1, 40 do
        local x = region.x + (i * 64 - camera) % (region.w + 64) - 64
        love.graphics.rectangle("fill", x, region.y + region.h - 14, 32, 14)
    end
    love.graphics.setScissor()

    love.graphics.setColor(1, 1, 1)
    love.graphics.print(("camera = %.0f   three layers scroll at 30%%, 70%% and 100%%"):format(camera), 12, 52)
end

return lesson
