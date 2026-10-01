local W, H = 860, 520
local COLORS = {{1, 0.42, 0.42}, {0.31, 0.8, 0.77}, {0.58, 0.88, 0.83}}

local timers = {}
local balls = {}
local fps, frames, fpsClock
local countdown, countdownClock
local small, tiny

local function ball(x, y, radius, delay)
    balls[#balls + 1] = {x = x, y = y, radius = radius, delay = delay, state = 1, last = 0}
end

function love.load()
    love.graphics.setBackgroundColor(0.055, 0.1, 0.16)
    small = love.graphics.newFont(14)
    tiny = love.graphics.newFont(12)
    for i = 1, 6 do
        timers[i] = 0
    end
    ball(200, 200, 30, 0.5)
    ball(350, 200, 30, 1)
    ball(500, 200, 30, 2)
    fps, frames, fpsClock = 0, 0, 0
    countdown, countdownClock = 10, 0
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    elseif key == "space" then
        for i = 1, 6 do
            timers[i] = 0
        end
    elseif key == "r" then
        countdown, countdownClock = 10, 0
    end
end

function love.update(dt)
    for i = 1, 6 do
        timers[i] = (timers[i] + dt) % i
    end

    frames = frames + 1
    fpsClock = fpsClock + dt
    if fpsClock >= 1 then
        fps = frames / fpsClock
        frames, fpsClock = 0, 0
    end

    for _, b in ipairs(balls) do
        b.last = b.last + dt
        if b.last >= b.delay then
            b.last = b.last - b.delay
            b.state = b.state % 3 + 1
        end
    end

    if countdown > 0 then
        countdownClock = countdownClock + dt
        while countdownClock >= 1 and countdown > 0 do
            countdownClock = countdownClock - 1
            countdown = countdown - 1
        end
    end
end

function love.draw()
    love.graphics.setFont(small)
    love.graphics.setColor(1, 1, 1)
    love.graphics.print("Timer System Demo", 10, 10)
    love.graphics.setColor(0.58, 0.68, 0.8)
    love.graphics.print("Timers accumulate dt. Space resets them, R restarts the countdown.", 10, 28)

    for _, b in ipairs(balls) do
        local c = COLORS[b.state]
        love.graphics.setColor(c[1], c[2], c[3])
        love.graphics.circle("fill", b.x, b.y, b.radius)
        love.graphics.setColor(1, 1, 1, 0.6)
        love.graphics.printf(("every %.1fs"):format(b.delay), b.x - 50, b.y + b.radius + 10, 100, "center")
    end

    local radius = 25 + math.sin(love.timer.getTime() * 6) * 10
    love.graphics.setColor(0.95, 0.5, 0.5)
    love.graphics.circle("fill", 660, 200, radius)
    love.graphics.setColor(1, 1, 1)
    love.graphics.printf("Pulsing!", 610, 200 - radius - 22, 100, "center")

    if countdown > 0 then
        love.graphics.setColor(0.31, 0.8, 0.77)
    elseif love.timer.getTime() % 0.5 < 0.25 then
        love.graphics.setColor(1, 0.42, 0.42)
    else
        love.graphics.setColor(1, 1, 1)
    end
    love.graphics.printf("Countdown: " .. countdown, 330, 350, 200, "center")

    love.graphics.setFont(tiny)
    love.graphics.setColor(0.58, 0.68, 0.8)
    love.graphics.print("Accumulators: timer[n] counts dt and wraps every n seconds", 20, 392)
    for i = 1, 6 do
        local x = 40 + (i - 1) * 130
        love.graphics.setColor(0.16, 0.24, 0.34)
        love.graphics.rectangle("fill", x, 428, 110, 8)
        love.graphics.setColor(0.31, 0.8, 0.77)
        love.graphics.rectangle("fill", x, 428, 110 * timers[i] / i, 8)
        love.graphics.setColor(0.58, 0.68, 0.8)
        love.graphics.print(("timer[%d] = %.2f"):format(i, timers[i]), x, 410)
    end

    love.graphics.setColor(0.84, 0.9, 0.96)
    love.graphics.print(("love.timer.getTime() = %.3f s"):format(love.timer.getTime()), 20, 450)
    love.graphics.print(("love.timer.getDelta() = %.4f s"):format(love.timer.getDelta()), 20, 466)
    love.graphics.print(("love.timer.getFPS() = %d   measured = %.1f"):format(love.timer.getFPS(), fps), 20, 482)
end
