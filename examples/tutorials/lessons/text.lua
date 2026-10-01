local lesson =
{
    title = "Text and fonts",
    apis = "love.graphics.newFont  print  printf  Font:getWidth  Font:getHeight  getWrap",
    help = "Type to edit the text. Backspace deletes. Fonts come from the embedded default or a file.",
}

local fonts, typed, sample

function lesson.load()
    fonts = {}
    for _, size in ipairs({12, 18, 28, 44}) do
        fonts[size] = love.graphics.newFont(size)
    end
    sample = "printf wraps long text inside a box and aligns it left, right, centered or justified."
end

function lesson.enter()
    typed = "type here"
end

function lesson.keypressed(key)
    if key == "backspace" then
        typed = typed:sub(1, -2)
    end
end

function lesson.textinput(t)
    typed = typed .. t
end

function lesson.draw()
    local y = 60
    for _, size in ipairs({12, 18, 28, 44}) do
        love.graphics.setFont(fonts[size])
        love.graphics.setColor(0.61, 0.78, 0.95)
        love.graphics.print(("Font size %d"):format(size), 20, y)
        love.graphics.setColor(0.5, 0.58, 0.68)
        love.graphics.print(("width %d"):format(fonts[size]:getWidth("Font size " .. size)), 330, y + (size > 20 and 8 or 0))
        y = y + fonts[size]:getHeight() + 6
    end

    local modes = {"left", "center", "right", "justify"}
    love.graphics.setFont(fonts[12])
    for i, mode in ipairs(modes) do
        local bx, by = 20 + (i - 1) * 195, 270
        love.graphics.setColor(0.12, 0.18, 0.27)
        love.graphics.rectangle("fill", bx, by, 185, 120)
        love.graphics.setColor(0.84, 0.9, 0.96)
        love.graphics.printf(sample, bx + 6, by + 8, 173, mode)
        love.graphics.setColor(1, 0.84, 0)
        love.graphics.print(mode, bx + 6, by + 128)
    end

    love.graphics.setFont(fonts[28])
    local blink = love.timer.getTime() % 1 < 0.5 and "_" or ""
    love.graphics.setColor(0.31, 0.8, 0.77)
    love.graphics.print(typed .. blink, 20, 440)

    love.graphics.setFont(fonts[12])
    local width, lines = fonts[12]:getWrap(sample, 173)
    love.graphics.setColor(0.5, 0.58, 0.68)
    love.graphics.print(("getWrap(173): width %d, %d lines"):format(width, #lines), 20, 500)

    love.graphics.setFont(fonts[18])
    local t = love.timer.getTime()
    local word = "Wave"
    local x = 20
    for i = 1, #word do
        local ch = word:sub(i, i)
        love.graphics.setColor(0.5 + 0.5 * math.sin(t * 3 + i), 0.5 + 0.5 * math.sin(t * 3 + i + 2), 0.5 + 0.5 * math.sin(t * 3 + i + 4))
        love.graphics.print(ch, x, 525 + math.sin(t * 5 + i) * 6)
        x = x + fonts[18]:getWidth(ch)
    end
end

return lesson
