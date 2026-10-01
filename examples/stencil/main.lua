-- Stencil masks. Keys 1 to 3 pick a mode, `love examples/stencil windows` starts with one.
--   spotlight: a dark scene revealed around the mouse
--   windows:   rotating shapes cut holes into a cover
--   inverse:   everything except the mask

local modes = { "spotlight", "windows", "inverse" }
local mode = 1
local time = 0
local image, font

function love.load(args)
    love.graphics.setBackgroundColor(0.05, 0.05, 0.08)
    image = love.graphics.newImage("images/zazaka.png")
    font = love.graphics.newFont(16)
    for i, name in ipairs(modes) do
        if name == args[1] then
            mode = i
        end
    end
end

function love.update(dt)
    time = time + dt
end

local function drawScene(brightness)
    local w, h = love.graphics.getDimensions()
    for i = 0, 15 do
        love.graphics.setColor(
            (0.5 + 0.5 * math.sin(i * 0.7)) * brightness,
            (0.5 + 0.5 * math.sin(i * 0.7 + 2)) * brightness,
            (0.5 + 0.5 * math.sin(i * 0.7 + 4)) * brightness,
            1)
        love.graphics.rectangle("fill", i * w / 16, 0, w / 16 + 1, h)
    end
    love.graphics.setColor(brightness, brightness, brightness, 1)
    for gx = 40, w, 80 do
        for gy = 40, h, 80 do
            love.graphics.circle("line", gx, gy, 26)
        end
    end
    love.graphics.draw(image, w / 2, h / 2, math.sin(time) * 0.3, 1.8, 1.8, image:getWidth() / 2, image:getHeight() / 2)
    love.graphics.setFont(font)
    love.graphics.print("Hidden in the dark: the stencil buffer lets you draw through any shape", 20, h - 30)
end

local function mouseDisc()
    local mx, my = love.mouse.getPosition()
    local radius = 110 + math.sin(time * 3) * 12
    love.graphics.circle("fill", mx, my, radius)
end

local function windows()
    local w, h = love.graphics.getDimensions()
    for i = 1, 4 do
        local a = time * 0.7 + i * math.pi / 2
        local cx, cy = w / 2 + math.cos(a) * 200, h / 2 + math.sin(a) * 140
        love.graphics.push()
        love.graphics.translate(cx, cy)
        love.graphics.rotate(time + i)
        love.graphics.rectangle("fill", -50, -50, 100, 100)
        love.graphics.pop()
    end
    love.graphics.circle("fill", w / 2, h / 2, 70)
end

function love.draw()
    local name = modes[mode]
    if name == "spotlight" then
        drawScene(0.12)
        love.graphics.stencil(mouseDisc)
        love.graphics.setStencilTest("greater", 0)
        drawScene(1)
        love.graphics.setStencilTest()
    elseif name == "windows" then
        drawScene(1)
        love.graphics.stencil(windows)
        love.graphics.setStencilTest("equal", 0)
        love.graphics.setColor(0.08, 0.08, 0.12, 1)
        love.graphics.rectangle("fill", 0, 0, love.graphics.getWidth(), love.graphics.getHeight())
        love.graphics.setStencilTest()
    else
        drawScene(1)
        love.graphics.stencil(mouseDisc)
        love.graphics.setStencilTest("equal", 0)
        love.graphics.setColor(0.08, 0.08, 0.12, 1)
        love.graphics.rectangle("fill", 0, 0, love.graphics.getWidth(), love.graphics.getHeight())
        love.graphics.setStencilTest()
    end

    love.graphics.setColor(0, 0, 0, 0.6)
    love.graphics.rectangle("fill", 10, 10, 200, 28 + #modes * 20)
    love.graphics.setFont(font)
    for i, m in ipairs(modes) do
        love.graphics.setColor(i == mode and 1 or 0.6, i == mode and 0.85 or 0.6, i == mode and 0.3 or 0.6, 1)
        love.graphics.print(i .. "  " .. m, 20, 18 + (i - 1) * 20)
    end
end

function love.keypressed(key)
    local n = tonumber(key)
    if n and modes[n] then
        mode = n
    elseif key == "escape" then
        love.event.quit()
    end
end
