-- Offscreen rendering, blend modes, scissoring and text alignment.

local canvas, font
local time = 0

function love.load()
    love.graphics.setBackgroundColor(0.15, 0.15, 0.2)
    font = love.graphics.newFont(13)
    canvas = love.graphics.newCanvas(256, 256)
end

function love.update(dt)
    time = time + dt
end

local function renderCanvas()
    love.graphics.setCanvas(canvas)
    love.graphics.clear(0, 0, 0, 0)
    love.graphics.setColor(0.2, 0.6, 1)
    love.graphics.circle("fill", 128, 128, 100)
    love.graphics.setColor(1, 1, 1)
    love.graphics.setLineWidth(6)
    love.graphics.circle("line", 128, 128, 100)
    love.graphics.setLineWidth(1)
    love.graphics.push()
    love.graphics.translate(128, 128)
    love.graphics.rotate(time)
    love.graphics.setColor(1, 0.9, 0.2)
    love.graphics.rectangle("fill", -60, -12, 120, 24)
    love.graphics.pop()
    love.graphics.setCanvas()
end

function love.draw()
    renderCanvas()
    love.graphics.setFont(font)

    love.graphics.setColor(1, 1, 1)
    love.graphics.draw(canvas, 20, 20)
    love.graphics.draw(canvas, 300, 20, 0, 0.5, 0.5)
    love.graphics.draw(canvas, 440, 20, math.pi / 8, 0.5, 0.5)
    love.graphics.print("canvas drawn three times", 20, 285)

    local modes = { "alpha", "add", "subtract", "multiply", "lighten", "darken", "screen", "replace" }
    for i, mode in ipairs(modes) do
        local x = 20 + (i - 1) * 95
        local y = 330
        love.graphics.setBlendMode("alpha")
        love.graphics.setColor(0.9, 0.3, 0.3)
        love.graphics.rectangle("fill", x, y, 60, 60)
        if mode == "multiply" then
            love.graphics.setBlendMode(mode, "premultiplied")
        else
            love.graphics.setBlendMode(mode)
        end
        love.graphics.setColor(0.3, 0.5, 0.9, 0.8)
        love.graphics.circle("fill", x + 50, y + 50, 30)
        love.graphics.setBlendMode("alpha")
        love.graphics.setColor(1, 1, 1)
        love.graphics.print(mode, x, y + 85)
    end

    -- Scissor clips everything to a rectangle, whatever the transform.
    love.graphics.setScissor(20, 450, 360, 120)
    love.graphics.setColor(0.2, 0.8, 0.5)
    for i = 0, 12 do
        love.graphics.circle("fill", 20 + i * 40 + (time * 40) % 40, 510, 25)
    end
    love.graphics.setScissor()
    love.graphics.setColor(1, 1, 1)
    love.graphics.rectangle("line", 20, 450, 360, 120)

    love.graphics.printf("left aligned text wraps inside its limit", 420, 450, 160, "left")
    love.graphics.printf("centered text wraps inside its limit", 600, 450, 160, "center")
    love.graphics.printf("right aligned text wraps inside its limit", 420, 520, 340, "right")
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    end
end
