local failures = {}
local passed = 0

local function check(name, ok, detail)
    if ok then
        passed = passed + 1
    else
        table.insert(failures, name .. (detail and (": " .. tostring(detail)) or ""))
    end
end

local function near(a, b, eps)
    return math.abs(a - b) <= (eps or 0.03)
end

local canvas

local function render(fn)
    love.graphics.setCanvas(canvas)
    love.graphics.clear(0, 0, 0, 0)
    love.graphics.setColor(1, 1, 1, 1)
    fn()
    love.graphics.setShader()
    love.graphics.setCanvas()
    return canvas:newImageData()
end

local function filled(data, x, y)
    return select(4, data:getPixel(x, y)) > 0.5
end

local function run()
    canvas = love.graphics.newCanvas(64, 64)

    local square = {
        { 8, 8, 0, 0, 1, 0, 0, 1 },
        { 56, 8, 1, 0, 1, 0, 0, 1 },
        { 56, 56, 1, 1, 1, 0, 0, 1 },
        { 8, 56, 0, 1, 1, 0, 0, 1 },
    }

    local mesh = love.graphics.newMesh(square)
    check("type", mesh:type() == "Mesh" and mesh:typeOf("Drawable") == false or mesh:typeOf("Object"))
    check("defaults", mesh:getVertexCount() == 4 and mesh:getDrawMode() == "fan")

    -- Fan
    local data = render(function() love.graphics.draw(mesh) end)
    check("fan fills a quad", filled(data, 32, 32) and filled(data, 12, 12) and filled(data, 52, 52) and not filled(data, 2, 2))
    local r, g, b = data:getPixel(32, 32)
    check("vertex color", near(r, 1) and near(g, 0) and near(b, 0))

    -- Winding does not matter
    local clockwise = love.graphics.newMesh({ { 8, 8 }, { 8, 56 }, { 56, 56 } }, "triangles")
    local counter = love.graphics.newMesh({ { 8, 8 }, { 56, 56 }, { 8, 56 } }, "triangles")
    data = render(function() love.graphics.draw(clockwise) end)
    check("clockwise triangle is visible", filled(data, 16, 40))
    data = render(function() love.graphics.draw(counter) end)
    check("counter-clockwise triangle is visible", filled(data, 16, 40))

    -- Interpolated colors
    local rainbow = love.graphics.newMesh({
        { 0, 0, 0, 0, 1, 0, 0, 1 },
        { 64, 0, 0, 0, 0, 1, 0, 1 },
        { 0, 64, 0, 0, 0, 0, 1, 1 },
    }, "triangles")
    data = render(function() love.graphics.draw(rainbow) end)
    r, g, b = data:getPixel(2, 2)
    check("color near the red vertex", r > 0.85 and g < 0.1)
    r, g, b = data:getPixel(20, 20)
    check("colors blend inside", r > 0.15 and g > 0.15 and b > 0.15, r .. "," .. g .. "," .. b)

    -- Strip
    local strip = love.graphics.newMesh({ { 8, 8 }, { 8, 24 }, { 24, 8 }, { 24, 24 }, { 40, 8 }, { 40, 24 } }, "strip")
    data = render(function() love.graphics.draw(strip) end)
    check("strip covers all cells", filled(data, 14, 16) and filled(data, 32, 16) and not filled(data, 50, 16))

    -- Triangles with a vertex map
    local mapped = love.graphics.newMesh({ { 8, 8 }, { 56, 8 }, { 56, 56 }, { 8, 56 } }, "triangles")
    mapped:setVertexMap(1, 2, 3, 1, 3, 4)
    data = render(function() love.graphics.draw(mapped) end)
    check("vertex map draws a quad", filled(data, 12, 12) and filled(data, 52, 52) and filled(data, 12, 52) and filled(data, 52, 12))
    local map = mapped:getVertexMap()
    check("getVertexMap", #map == 6 and map[1] == 1 and map[6] == 4)
    mapped:setVertexMap({ 1, 2, 3 })
    data = render(function() love.graphics.draw(mapped) end)
    check("vertex map as table, one triangle", filled(data, 40, 20) and not filled(data, 12, 52))
    mapped:setVertexMap()
    check("clearing the map", mapped:getVertexMap() == nil)
    check("bad vertex map", not pcall(mapped.setVertexMap, mapped, 1, 2, 9))

    -- Draw range
    local ranged = love.graphics.newMesh({ { 8, 8 }, { 24, 8 }, { 8, 24 }, { 40, 40 }, { 56, 40 }, { 40, 56 } }, "triangles")
    ranged:setDrawRange(4, 3)
    local start, count = ranged:getDrawRange()
    check("getDrawRange", start == 4 and count == 3)
    data = render(function() love.graphics.draw(ranged) end)
    check("draw range limits what is drawn", filled(data, 44, 44) and not filled(data, 12, 12))
    ranged:setDrawRange()
    check("clearing the draw range", ranged:getDrawRange() == nil)

    -- Points
    local points = love.graphics.newMesh({ { 20, 20 }, { 40, 40 } }, "points")
    love.graphics.setPointSize(6)
    data = render(function() love.graphics.draw(points) end)
    love.graphics.setPointSize(1)
    check("points", filled(data, 20, 20) and filled(data, 40, 40) and not filled(data, 30, 30))

    -- Texture
    local pixels = love.image.newImageData(2, 1)
    pixels:setPixel(0, 0, 0, 1, 0, 1)
    pixels:setPixel(1, 0, 0, 0, 1, 1)
    local image = love.graphics.newImage(pixels)
    image:setFilter("nearest", "nearest")
    local textured = love.graphics.newMesh({ { 0, 0, 0, 0 }, { 64, 0, 1, 0 }, { 64, 64, 1, 1 }, { 0, 64, 0, 1 } })
    textured:setTexture(image)
    check("getTexture", textured:getTexture() == image)
    data = render(function() love.graphics.draw(textured) end)
    r, g, b = data:getPixel(10, 32)
    check("texture left half", near(g, 1) and near(b, 0), g .. "," .. b)
    r, g, b = data:getPixel(54, 32)
    check("texture right half", near(g, 0) and near(b, 1), g .. "," .. b)
    textured:setTexture()
    check("removing the texture", textured:getTexture() == nil)

    -- Vertex colors multiply with the current color
    data = render(function()
        love.graphics.setColor(0, 1, 0, 1)
        love.graphics.draw(mesh)
    end)
    r, g, b = data:getPixel(32, 32)
    check("current color tints vertex colors", near(r, 0) and near(g, 0))

    -- Transform arguments
    data = render(function() love.graphics.draw(mesh, 10, 0) end)
    check("draw offsets a mesh", filled(data, 60, 32) and not filled(data, 12, 32))

    -- Through a shader
    local invert = love.graphics.newShader([[
        vec4 effect(vec4 c, Image t, vec2 uv, vec2 sc) { vec4 p = Texel(t, uv) * c; return vec4(1.0 - p.rgb, p.a); }
    ]])
    data = render(function()
        love.graphics.setShader(invert)
        love.graphics.draw(mesh)
    end)
    r, g, b = data:getPixel(32, 32)
    check("meshes go through shaders", near(r, 0) and near(g, 1) and near(b, 1), r .. "," .. g .. "," .. b)

    -- Accessors
    mesh:setVertex(1, 1, 2, 3, 4, 0.1, 0.2, 0.3, 0.4)
    local x, y, u, v, cr, cg, cb, ca = mesh:getVertex(1)
    check("setVertex / getVertex", x == 1 and y == 2 and near(u, 3, 1e-4) and near(v, 4, 1e-4) and near(cr, 0.1, 1e-4) and near(ca, 0.4, 1e-4))
    mesh:setVertex(2, { 5, 6 })
    x, y, u, v, cr = mesh:getVertex(2)
    check("setVertex with a table uses defaults", x == 5 and y == 6 and u == 0 and cr == 1)
    mesh:setVertices({ { 9, 9 }, { 8, 8 } }, 3)
    check("setVertices with a start index", select(1, mesh:getVertex(3)) == 9 and select(1, mesh:getVertex(4)) == 8)
    mesh:setVertexAttribute(1, 3, 0.5, 0.6, 0.7, 0.8)
    local ar, ag, ab, aa = mesh:getVertexAttribute(1, 3)
    check("vertex attributes", near(ar, 0.5, 1e-4) and near(aa, 0.8, 1e-4))
    check("getVertices", #mesh:getVertices() == 4 and #mesh:getVertices()[1] == 8)
    local format = mesh:getVertexFormat()
    check("getVertexFormat", #format == 3 and format[1][1] == "VertexPosition" and format[3][2] == "byte")
    mesh:setDrawMode("strip")
    check("setDrawMode", mesh:getDrawMode() == "strip")

    -- Counts and formats
    local blank = love.graphics.newMesh(10, "triangles", "static")
    check("mesh from a count", blank:getVertexCount() == 10 and select(2, blank:getVertex(1)) == 0)
    local formatted = love.graphics.newMesh({ { "VertexPosition", "float", 2 }, { "VertexTexCoord", "float", 2 }, { "VertexColor", "byte", 4 } }, { { 1, 2 } })
    check("explicit default format", formatted:getVertexCount() == 1)

    -- Errors
    check("too many vertices", not pcall(mesh.setVertices, mesh, { {}, {}, {}, {}, {} }))
    check("bad index", not pcall(mesh.getVertex, mesh, 5))
    check("bad mode", not pcall(love.graphics.newMesh, square, "blob"))
    check("empty mesh", not pcall(love.graphics.newMesh, {}))
    check("custom attributes are rejected", not pcall(love.graphics.newMesh, { { "Custom", "float", 3 } }, 4))
    check("bad texture", not pcall(mesh.setTexture, mesh, 5))
end

local ok, err = xpcall(run, debug.traceback)
if not ok then
    table.insert(failures, "runtime error: " .. tostring(err))
end

function love.update()
    if #failures > 0 then
        print("FAILED checks:")
        for _, f in ipairs(failures) do
            print("  - " .. f)
        end
        love.event.quit(1)
    else
        print(string.format("All %d mesh checks passed", passed))
        love.event.quit(0)
    end
end
