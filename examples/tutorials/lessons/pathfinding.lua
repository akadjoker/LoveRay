local lesson =
{
    title = "Pathfinding",
    apis = "A* search  grid  love.mouse.isDown  love.graphics.line",
    help = "Left button paints walls, right button erases. Space moves the goal to the mouse, R clears.",
}

local COLS, ROWS, CELL = 32, 21, 22
local OX, OY = 48, 74

local walls, start, goal, path, searched

local function key(x, y)
    return y * COLS + x
end

local function solve()
    local open = {{x = start.x, y = start.y, g = 0, f = 0}}
    local best = {[key(start.x, start.y)] = 0}
    local from = {}
    local closed = {}
    searched = 0
    path = nil

    while #open > 0 do
        local index = 1
        for i = 2, #open do
            if open[i].f < open[index].f then
                index = i
            end
        end
        local node = table.remove(open, index)
        local nk = key(node.x, node.y)
        if not closed[nk] then
            closed[nk] = true
            searched = searched + 1

            if node.x == goal.x and node.y == goal.y then
                path = {}
                local k = nk
                while k do
                    table.insert(path, 1, {x = k % COLS, y = math.floor(k / COLS)})
                    k = from[k]
                end
                return
            end

            for _, d in ipairs({{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) do
                local x, y = node.x + d[1], node.y + d[2]
                if x >= 0 and x < COLS and y >= 0 and y < ROWS and not walls[key(x, y)] then
                    local g = node.g + 1
                    local k = key(x, y)
                    if not best[k] or g < best[k] then
                        best[k] = g
                        from[k] = nk
                        local h = math.abs(x - goal.x) + math.abs(y - goal.y)
                        open[#open + 1] = {x = x, y = y, g = g, f = g + h}
                    end
                end
            end
        end
    end
end

function lesson.enter()
    walls = {}
    for y = 3, 17 do
        walls[key(15, y)] = true
    end
    for x = 6, 22 do
        walls[key(x, 12)] = true
    end
    start, goal = {x = 2, y = 2}, {x = 29, y = 19}
    solve()
end

local function cellAt(mx, my)
    local x, y = math.floor((mx - OX) / CELL), math.floor((my - OY) / CELL)
    if x >= 0 and x < COLS and y >= 0 and y < ROWS then
        return x, y
    end
end

function lesson.keypressed(key_)
    if key_ == "r" then
        walls = {}
        solve()
    elseif key_ == "space" then
        local x, y = cellAt(love.mouse.getPosition())
        if x and not walls[key(x, y)] then
            goal = {x = x, y = y}
            solve()
        end
    end
end

function lesson.update(dt)
    local left, right = love.mouse.isDown(1), love.mouse.isDown(2)
    if left or right then
        local x, y = cellAt(love.mouse.getPosition())
        if x then
            local k = key(x, y)
            local want = left or nil
            local isStart = x == start.x and y == start.y
            local isGoal = x == goal.x and y == goal.y
            if walls[k] ~= want and not isStart and not isGoal then
                walls[k] = want
                solve()
            end
        end
    end
end

function lesson.draw()
    love.graphics.setColor(0.08, 0.13, 0.2)
    love.graphics.rectangle("fill", OX, OY, COLS * CELL, ROWS * CELL)
    love.graphics.setColor(0.16, 0.24, 0.34)
    for x = 0, COLS do
        love.graphics.line(OX + x * CELL, OY, OX + x * CELL, OY + ROWS * CELL)
    end
    for y = 0, ROWS do
        love.graphics.line(OX, OY + y * CELL, OX + COLS * CELL, OY + y * CELL)
    end

    love.graphics.setColor(0.45, 0.52, 0.6)
    for k in pairs(walls) do
        love.graphics.rectangle("fill", OX + (k % COLS) * CELL + 1, OY + math.floor(k / COLS) * CELL + 1, CELL - 2, CELL - 2)
    end

    if path then
        local points = {}
        for _, p in ipairs(path) do
            points[#points + 1] = OX + p.x * CELL + CELL / 2
            points[#points + 1] = OY + p.y * CELL + CELL / 2
        end
        love.graphics.setColor(1, 0.84, 0)
        love.graphics.setLineWidth(3)
        if #points >= 4 then
            love.graphics.line(points)
        end
        love.graphics.setLineWidth(1)
    end

    love.graphics.setColor(0.31, 0.8, 0.77)
    love.graphics.circle("fill", OX + start.x * CELL + CELL / 2, OY + start.y * CELL + CELL / 2, 9)
    love.graphics.setColor(1, 0.42, 0.42)
    love.graphics.circle("fill", OX + goal.x * CELL + CELL / 2, OY + goal.y * CELL + CELL / 2, 9)

    love.graphics.setColor(1, 1, 1)
    local status = path and ("path length " .. (#path - 1)) or "no path"
    love.graphics.print(status .. "   nodes expanded " .. searched, 12, 50)
end

return lesson
