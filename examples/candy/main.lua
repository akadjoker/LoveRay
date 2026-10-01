local COLS, ROWS = 8, 8
local CELL = 48
local BX, BY = 38, 60
local COLORS =
{
    {1, 0.35, 0.35},
    {0.35, 1, 0.63},
    {0.36, 0.78, 1},
    {1, 0.83, 0.35},
    {0.77, 0.54, 1},
}

local board, matched, score, selected, moves

local function at(x, y)
    return board[y * COLS + x + 1]
end

local function set(x, y, v)
    board[y * COLS + x + 1] = v
end

local function randomColor()
    return love.math.random(1, #COLORS)
end

local function findMatches()
    for i = 1, COLS * ROWS do
        matched[i] = false
    end
    local count = 0

    local function mark(x, y)
        local i = y * COLS + x + 1
        if not matched[i] then
            matched[i] = true
            count = count + 1
        end
    end

    for y = 0, ROWS - 1 do
        local x = 0
        while x < COLS do
            local run = 1
            while x + run < COLS and at(x + run, y) == at(x, y) do
                run = run + 1
            end
            if run >= 3 then
                for k = x, x + run - 1 do
                    mark(k, y)
                end
            end
            x = x + run
        end
    end

    for x = 0, COLS - 1 do
        local y = 0
        while y < ROWS do
            local run = 1
            while y + run < ROWS and at(x, y + run) == at(x, y) do
                run = run + 1
            end
            if run >= 3 then
                for k = y, y + run - 1 do
                    mark(x, k)
                end
            end
            y = y + run
        end
    end

    return count
end

local function collapse()
    for x = 0, COLS - 1 do
        local write = ROWS - 1
        for y = ROWS - 1, 0, -1 do
            if not matched[y * COLS + x + 1] then
                set(x, write, at(x, y))
                write = write - 1
            end
        end
        for y = write, 0, -1 do
            set(x, y, randomColor())
        end
    end
end

local function resolve()
    local total = 0
    local combo = 0
    local found = findMatches()
    while found > 0 do
        combo = combo + 1
        total = total + found * 10 * combo
        collapse()
        found = findMatches()
    end
    return total
end

local function newBoard()
    board, matched = {}, {}
    for i = 1, COLS * ROWS do
        board[i] = randomColor()
    end
    while findMatches() > 0 do
        collapse()
    end
    score, moves = 0, 0
    selected = nil
end

function love.load()
    love.graphics.setBackgroundColor(0.055, 0.1, 0.16)
    love.graphics.setFont(love.graphics.newFont(20))
    newBoard()
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    elseif key == "r" then
        newBoard()
    end
end

local function adjacent(a, b)
    return math.abs(a.x - b.x) + math.abs(a.y - b.y) == 1
end

local function swap(a, b)
    local t = at(a.x, a.y)
    set(a.x, a.y, at(b.x, b.y))
    set(b.x, b.y, t)
end

function love.mousepressed(mx, my, button)
    if button ~= 1 then
        return
    end
    local gx, gy = math.floor((mx - BX) / CELL), math.floor((my - BY) / CELL)
    if gx < 0 or gy < 0 or gx >= COLS or gy >= ROWS then
        return
    end
    local cell = {x = gx, y = gy}
    if not selected then
        selected = cell
    elseif adjacent(selected, cell) then
        swap(selected, cell)
        if findMatches() > 0 then
            moves = moves + 1
            score = score + resolve()
        else
            swap(selected, cell)
        end
        selected = nil
    else
        selected = cell
    end
end

function love.draw()
    local mx, my = love.mouse.getPosition()
    local hx, hy = math.floor((mx - BX) / CELL), math.floor((my - BY) / CELL)

    for y = 0, ROWS - 1 do
        for x = 0, COLS - 1 do
            local c = COLORS[at(x, y)]
            local px, py = BX + x * CELL, BY + y * CELL
            love.graphics.setColor(c[1], c[2], c[3])
            love.graphics.rectangle("fill", px + 3, py + 3, CELL - 6, CELL - 6, 8)
            love.graphics.setColor(1, 1, 1, 0.25)
            love.graphics.rectangle("fill", px + 8, py + 8, CELL - 22, 6, 3)
            if hx == x and hy == y then
                love.graphics.setColor(1, 1, 1, 0.18)
                love.graphics.rectangle("fill", px + 3, py + 3, CELL - 6, CELL - 6, 8)
            end
            if selected and selected.x == x and selected.y == y then
                love.graphics.setColor(1, 1, 1)
                love.graphics.setLineWidth(3)
                love.graphics.rectangle("line", px + 1, py + 1, CELL - 2, CELL - 2, 8)
            end
        end
    end

    love.graphics.setColor(0.84, 0.9, 0.96)
    love.graphics.print("score: " .. score, BX, 20)
    love.graphics.printf("moves: " .. moves, BX, 20, COLS * CELL, "right")
end
