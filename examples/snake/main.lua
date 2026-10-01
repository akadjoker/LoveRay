local CELL = 20
local COLS, ROWS = 43, 26
local STEP = 0.11

local snake, direction, queued, apple, score, best, alive, timer, interval

local function placeApple()
    while true do
        local x, y = love.math.random(0, COLS - 1), love.math.random(0, ROWS - 1)
        local free = true
        for _, s in ipairs(snake) do
            if s.x == x and s.y == y then
                free = false
                break
            end
        end
        if free then
            apple = {x = x, y = y}
            return
        end
    end
end

local function newGame()
    snake = {}
    for i = 0, 2 do
        snake[#snake + 1] = {x = 10 - i, y = 13}
    end
    direction = {x = 1, y = 0}
    queued = {x = 1, y = 0}
    score = 0
    alive = true
    timer = 0
    interval = STEP
    placeApple()
end

function love.load()
    love.graphics.setBackgroundColor(0.055, 0.1, 0.16)
    love.graphics.setFont(love.graphics.newFont(20))
    best = 0
    newGame()
end

local turns = {
    left = {x = -1, y = 0},
    right = {x = 1, y = 0},
    up = {x = 0, y = -1},
    down = {x = 0, y = 1},
}

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    elseif key == "space" and not alive then
        newGame()
    elseif turns[key] then
        local t = turns[key]
        if t.x ~= -direction.x or t.y ~= -direction.y then
            queued = t
        end
    end
end

local function step()
    direction = queued
    local head = snake[1]
    local nx, ny = head.x + direction.x, head.y + direction.y

    if nx < 0 or nx >= COLS or ny < 0 or ny >= ROWS then
        alive = false
        return
    end
    for i = 1, #snake - 1 do
        if snake[i].x == nx and snake[i].y == ny then
            alive = false
            return
        end
    end

    table.insert(snake, 1, {x = nx, y = ny})
    if nx == apple.x and ny == apple.y then
        score = score + 10
        best = math.max(best, score)
        interval = math.max(0.05, interval - 0.003)
        placeApple()
    else
        table.remove(snake)
    end
end

function love.update(dt)
    if not alive then
        return
    end
    timer = timer + dt
    while timer >= interval do
        timer = timer - interval
        step()
        if not alive then
            break
        end
    end
end

function love.draw()
    love.graphics.setColor(1, 0.42, 0.42)
    love.graphics.circle("fill", apple.x * CELL + CELL / 2, apple.y * CELL + CELL / 2, CELL / 2 - 2)

    for i = #snake, 1, -1 do
        local s = snake[i]
        if i == 1 then
            love.graphics.setColor(0.31, 0.8, 0.77)
        else
            love.graphics.setColor(0.58, 0.88, 0.83)
        end
        love.graphics.rectangle("fill", s.x * CELL + 1, s.y * CELL + 1, CELL - 2, CELL - 2)
    end

    love.graphics.setColor(1, 1, 1)
    love.graphics.print("Score " .. score .. "   Best " .. best, 10, 6)
    if not alive then
        love.graphics.setColor(1, 0.42, 0.42)
        love.graphics.printf("GAME OVER - press Space", 0, 240, COLS * CELL, "center")
    end
end
