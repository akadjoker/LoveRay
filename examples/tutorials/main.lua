local order =
{
    "movement", "angles", "collisions", "shapes", "procedural", "scroll",
    "splitscreen", "pathfinding", "text", "primitives", "mouse", "trigonometry",
}

local lessons = {}
local current = 1
local titleFont, bodyFont

local function enter(index)
    current = (index - 1) % #order + 1
    local lesson = lessons[current]
    if lesson.enter then
        lesson.enter()
    end
end

function love.load(args)
    love.graphics.setBackgroundColor(0.055, 0.1, 0.16)
    titleFont = love.graphics.newFont(20)
    bodyFont = love.graphics.newFont(13)
    for i, name in ipairs(order) do
        lessons[i] = require("lessons." .. name)
        if lessons[i].load then
            lessons[i].load()
        end
    end
    local start = tonumber(args and args[1])
    enter(start or 1)
end

function love.keypressed(key, scancode, isrepeat)
    if key == "escape" then
        love.event.quit()
    elseif key == "pagedown" or key == "tab" and not love.keyboard.isDown("lshift", "rshift") then
        enter(current + 1)
    elseif key == "pageup" or key == "tab" then
        enter(current - 1)
    elseif key:match("^f%d+$") and tonumber(key:sub(2)) <= #order then
        enter(tonumber(key:sub(2)))
    else
        local lesson = lessons[current]
        if lesson.keypressed then
            lesson.keypressed(key, isrepeat)
        end
    end
end

function love.textinput(t)
    local lesson = lessons[current]
    if lesson.textinput and t ~= "\t" then
        lesson.textinput(t)
    end
end

function love.mousepressed(x, y, button)
    local lesson = lessons[current]
    if lesson.mousepressed then
        lesson.mousepressed(x, y, button)
    end
end

function love.mousereleased(x, y, button)
    local lesson = lessons[current]
    if lesson.mousereleased then
        lesson.mousereleased(x, y, button)
    end
end

function love.wheelmoved(x, y)
    local lesson = lessons[current]
    if lesson.wheelmoved then
        lesson.wheelmoved(x, y)
    end
end

function love.update(dt)
    local lesson = lessons[current]
    if lesson.update then
        lesson.update(dt)
    end
end

function love.draw()
    local lesson = lessons[current]
    lesson.draw()

    love.graphics.setColor(0.04, 0.07, 0.11, 0.9)
    love.graphics.rectangle("fill", 0, 0, 800, 44)
    love.graphics.rectangle("fill", 0, 566, 800, 34)

    love.graphics.setFont(titleFont)
    love.graphics.setColor(0.61, 0.78, 0.95)
    love.graphics.print(("%d/%d  %s"):format(current, #order, lesson.title), 12, 10)

    love.graphics.setFont(bodyFont)
    love.graphics.setColor(0.58, 0.68, 0.8)
    love.graphics.printf(lesson.apis, 0, 14, 788, "right")
    love.graphics.setColor(0.84, 0.9, 0.96)
    love.graphics.print(lesson.help, 12, 570)
    love.graphics.setColor(0.45, 0.55, 0.65)
    love.graphics.print("PageUp/PageDown, Tab or F1-F12: switch lesson   Esc: quit", 12, 584)
end
