--
-- nogame.lua - the screen shown when LoveRay is started without a game.
-- Written against the public Love2D API only, so it doubles as a smoke test.
--

return function()
    local bubbles = {}
    local time = 0
    local titleFont, bodyFont, smallFont

    local function spawnBubble(i)
        local w, h = love.graphics.getDimensions()
        bubbles[i] = {
            x = love.math.random() * w,
            y = h + love.math.random() * h,
            r = 12 + love.math.random() * 36,
            speed = 20 + love.math.random() * 50,
            drift = (love.math.random() - 0.5) * 30,
            hue = love.math.random(),
        }
    end

    function love.load()
        love.graphics.setBackgroundColor(0.11, 0.13, 0.19)
        titleFont = love.graphics.newFont(56)
        bodyFont = love.graphics.newFont(20)
        smallFont = love.graphics.newFont(13)
        local h = love.graphics.getHeight()
        for i = 1, 24 do
            spawnBubble(i)
            bubbles[i].y = love.math.random() * h * 2
        end
    end

    function love.update(dt)
        time = time + dt
        local h = love.graphics.getHeight()
        for i, b in ipairs(bubbles) do
            b.y = b.y - b.speed * dt
            b.x = b.x + math.sin(time + i) * b.drift * dt
            if b.y + b.r < 0 then
                spawnBubble(i)
                bubbles[i].y = h + bubbles[i].r
            end
        end
    end

    local function centered(font, text, y)
        love.graphics.setFont(font)
        local w = love.graphics.getWidth()
        love.graphics.printf(text, 0, y, w, "center")
    end

    function love.draw()
        local w, h = love.graphics.getDimensions()

        for _, b in ipairs(bubbles) do
            love.graphics.setColor(0.95, 0.35 + 0.4 * b.hue, 0.55, 0.18)
            love.graphics.circle("fill", b.x, b.y, b.r)
            love.graphics.setColor(1, 1, 1, 0.25)
            love.graphics.circle("line", b.x, b.y, b.r)
        end

        local bob = math.sin(time * 2) * 6
        love.graphics.setColor(1, 1, 1, 1)
        centered(titleFont, "LoveRay", h * 0.32 + bob)
        love.graphics.setColor(0.98, 0.78, 0.35, 1)
        centered(bodyFont, "no game", h * 0.32 + 72 + bob)

        love.graphics.setColor(1, 1, 1, 0.85)
        centered(bodyFont, "Run a game with:  love <game directory>", h * 0.62)

        love.graphics.setColor(1, 1, 1, 0.5)
        centered(smallFont, string.format("LoveRay %s  -  Love2D API %s  -  raylib %s  -  %s",
            love.loveray.version, love._version, love.loveray.raylib, love.loveray.lua), h - 36)
    end

    function love.keypressed(key)
        if key == "escape" then
            love.event.quit()
        end
    end
end
