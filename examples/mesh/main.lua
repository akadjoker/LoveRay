-- Meshes: a textured grid that waves, and a vertex-colored fan used as a radar sweep.

local COLUMNS, ROWS = 24, 18
local image, grid, radar
local time = 0

local function buildGrid()
    local w, h = image:getDimensions()
    local vertices = {}
    for row = 0, ROWS do
        for column = 0, COLUMNS do
            vertices[#vertices + 1] = { 0, 0, column / COLUMNS, row / ROWS, 1, 1, 1, 1 }
        end
    end
    local mesh = love.graphics.newMesh(vertices, "triangles", "dynamic")
    local map = {}
    for row = 0, ROWS - 1 do
        for column = 0, COLUMNS - 1 do
            local a = row * (COLUMNS + 1) + column + 1
            local b = a + 1
            local c = a + COLUMNS + 1
            local d = c + 1
            map[#map + 1], map[#map + 2], map[#map + 3] = a, b, c
            map[#map + 1], map[#map + 2], map[#map + 3] = b, d, c
        end
    end
    mesh:setVertexMap(map)
    mesh:setTexture(image)
    return mesh
end

local function updateGrid(t)
    local w, h = 360, 420
    local index = 1
    for row = 0, ROWS do
        for column = 0, COLUMNS do
            local x = column / COLUMNS * w
            local y = row / ROWS * h
            local wave = math.sin(t * 2 + row * 0.5) * 14 * (row / ROWS)
            local sway = math.cos(t * 1.5 + column * 0.4) * 6
            grid:setVertexAttribute(index, 1, x + wave, y + sway)
            local shade = 0.75 + 0.25 * math.sin(t * 2 + row * 0.5 + column * 0.2)
            grid:setVertexAttribute(index, 3, shade, shade, shade, 1)
            index = index + 1
        end
    end
end

local function buildRadar()
    local vertices = { { 0, 0, 0, 0, 0.2, 1, 0.4, 0.9 } }
    local steps = 24
    for i = 0, steps do
        local angle = -i / steps * 1.2
        local fade = 1 - i / steps
        vertices[#vertices + 1] = { math.cos(angle) * 150, math.sin(angle) * 150, 0, 0, 0.2, 1, 0.4, 0.8 * fade * fade }
    end
    return love.graphics.newMesh(vertices, "fan", "static")
end

function love.load()
    love.graphics.setBackgroundColor(0.06, 0.07, 0.1)
    image = love.graphics.newImage("images/zazaka.png")
    grid = buildGrid()
    radar = buildRadar()
end

function love.update(dt)
    time = time + dt
    updateGrid(time)
end

function love.draw()
    love.graphics.setColor(1, 1, 1, 1)
    love.graphics.draw(grid, 60, 90)

    love.graphics.push()
    love.graphics.translate(600, 300)
    love.graphics.setColor(0.2, 1, 0.4, 0.35)
    for _, radius in ipairs({ 50, 100, 150 }) do
        love.graphics.circle("line", 0, 0, radius)
    end
    love.graphics.setColor(1, 1, 1, 1)
    love.graphics.rotate(time * 2)
    love.graphics.draw(radar)
    love.graphics.pop()

    love.graphics.setColor(0.7, 0.7, 0.7, 1)
    love.graphics.print(string.format("%d vertices  %d fps", grid:getVertexCount(), love.timer.getFPS()), 20, 20)
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    end
end
