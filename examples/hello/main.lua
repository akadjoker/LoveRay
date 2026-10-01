-- Shapes, text and the transform stack.

local time = 0
local titleFont, bodyFont

function love.load()
    love.graphics.setBackgroundColor(0.12, 0.12, 0.16)
    titleFont = love.graphics.newFont(32)
    bodyFont = love.graphics.newFont(14)
end

function love.update(dt)
    time = time + dt
end

local function drawShapes(x, y)
    love.graphics.push()
    love.graphics.translate(x, y)

    love.graphics.setColor(0.9, 0.3, 0.3)
    love.graphics.rectangle("fill", 0, 0, 80, 60)
    love.graphics.setColor(1, 1, 1)
    love.graphics.rectangle("line", 0, 0, 80, 60)

    love.graphics.setColor(0.3, 0.8, 0.4)
    love.graphics.rectangle("fill", 100, 0, 80, 60, 12, 12)

    love.graphics.setColor(0.3, 0.5, 0.9)
    love.graphics.circle("fill", 240, 30, 30)
    love.graphics.setLineWidth(3)
    love.graphics.setColor(1, 1, 1)
    love.graphics.circle("line", 240, 30, 30)
    love.graphics.setLineWidth(1)

    love.graphics.setColor(0.9, 0.7, 0.2)
    love.graphics.ellipse("fill", 330, 30, 40, 22)

    love.graphics.setColor(0.8, 0.4, 0.9)
    love.graphics.arc("fill", 420, 30, 30, 0, math.pi * 1.5)
    love.graphics.setColor(1, 1, 1)
    love.graphics.arc("line", "open", 420, 30, 30, 0, math.pi * 1.5)

    love.graphics.setColor(0.2, 0.8, 0.8)
    love.graphics.polygon("fill", 480, 60, 510, 0, 540, 60, 510, 40)
    love.graphics.setColor(1, 1, 1)
    love.graphics.polygon("line", 480, 60, 510, 0, 540, 60, 510, 40)

    love.graphics.setColor(1, 0.5, 0.2)
    love.graphics.setLineWidth(4)
    love.graphics.line(560, 60, 580, 0, 600, 50, 640, 10)
    love.graphics.setLineWidth(1)

    love.graphics.setPointSize(4)
    love.graphics.points(660, 10, 670, 30, 680, 50)

    love.graphics.pop()
end

function love.draw()
    love.graphics.setFont(titleFont)
    love.graphics.setColor(1, 1, 1)
    love.graphics.print("Hello, LoveRay!", 40, 30)

    love.graphics.setFont(bodyFont)
    love.graphics.setColor(0.8, 0.8, 0.8)
    love.graphics.printf("This window is drawn with the Love2D API running on raylib. "
        .. "Text wraps to the limit you pass to printf and can be aligned left, center or right.",
        40, 80, 400, "left")

    drawShapes(40, 180)

    -- A rotating, scaled group with its own origin.
    love.graphics.push()
    love.graphics.translate(400, 420)
    love.graphics.rotate(time)
    love.graphics.scale(1 + 0.25 * math.sin(time * 2))
    love.graphics.setColor(0.95, 0.6, 0.1)
    love.graphics.rectangle("fill", -50, -50, 100, 100)
    love.graphics.setColor(0.1, 0.1, 0.1)
    love.graphics.printf("spin", -50, -8, 100, "center")
    love.graphics.pop()

    -- Colored text segments.
    love.graphics.print({{1, 0.4, 0.4}, "red ", {0.4, 1, 0.4}, "green ", {0.4, 0.6, 1}, "blue"}, 40, 520)

    love.graphics.setColor(0.6, 0.6, 0.6)
    love.graphics.printf(string.format("%d fps", love.timer.getFPS()), 0, 570, love.graphics.getWidth() - 20, "right")
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    end
end
