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

local function run()
    canvas = love.graphics.newCanvas(64, 64)

    -- Compilation
    local invert = love.graphics.newShader([[
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            vec4 p = Texel(tex, uv) * color;
            return vec4(1.0 - p.rgb, p.a);
        }
    ]])
    check("newShader", invert:type() == "Shader" and invert:typeOf("Object"))

    local ok, err = pcall(love.graphics.newShader, "vec4 effect(vec4 c, Image t, vec2 uv, vec2 sc) { return undefined_name; }")
    check("compile error raised", not ok and tostring(err):find("Cannot compile shader", 1, true) ~= nil, err)
    check("compile error mentions the problem", tostring(err):find("undefined_name", 1, true) ~= nil, err)
    ok, err = pcall(love.graphics.newShader, "float nothing() { return 1.0; }")
    check("missing entry point", not ok and tostring(err):find("effect", 1, true) ~= nil, err)
    local valid, message = love.graphics.validateShader(false, "vec4 effect(vec4 c, Image t, vec2 uv, vec2 sc) { return c; }")
    check("validateShader ok", valid == true)
    valid, message = love.graphics.validateShader(false, "vec4 effect(vec4 c, Image t, vec2 uv, vec2 sc) { return nope; }")
    check("validateShader fails", valid == false and type(message) == "string" and #message > 0, message)

    -- Active shader state
    check("getShader default", love.graphics.getShader() == nil)
    love.graphics.setShader(invert)
    check("getShader", love.graphics.getShader() == invert)
    love.graphics.setShader()
    check("setShader nil", love.graphics.getShader() == nil)

    -- Pixel effect
    local data = render(function()
        love.graphics.setShader(invert)
        love.graphics.setColor(1, 0, 0, 1)
        love.graphics.rectangle("fill", 0, 0, 64, 64)
    end)
    local r, g, b, a = data:getPixel(10, 10)
    check("invert effect", near(r, 0) and near(g, 1) and near(b, 1) and near(a, 1), r .. "," .. g .. "," .. b .. "," .. a)

    -- Uniforms are applied per draw call, not per frame
    local tint = love.graphics.newShader([[
        extern vec3 tint;
        extern float strength;
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            return vec4(tint * strength, 1.0);
        }
    ]])
    check("hasUniform", tint:hasUniform("tint") and tint:hasUniform("strength") and not tint:hasUniform("nope"))
    data = render(function()
        love.graphics.setShader(tint)
        tint:send("strength", 1)
        tint:send("tint", { 0, 1, 0 })
        love.graphics.rectangle("fill", 0, 0, 32, 64)
        tint:send("tint", { 0, 0, 1 })
        love.graphics.rectangle("fill", 32, 0, 32, 64)
    end)
    r, g, b = data:getPixel(8, 8)
    check("uniform vec3 first draw", near(r, 0) and near(g, 1) and near(b, 0), r .. "," .. g .. "," .. b)
    r, g, b = data:getPixel(48, 8)
    check("uniform vec3 second draw", near(r, 0) and near(g, 0) and near(b, 1), r .. "," .. g .. "," .. b)
    check("send unknown uniform errors", not pcall(tint.send, tint, "missing", 1))
    check("send wrong count errors", not pcall(tint.send, tint, "tint", 1, 2))

    -- Scalars, ints, bools, arrays
    local mixed = love.graphics.newShader([[
        extern int steps;
        extern bool enabled;
        extern float weights[3];
        extern vec2 pair;
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            float v = enabled ? weights[0] + weights[1] + weights[2] : 0.0;
            return vec4(float(steps) / 10.0, v, pair.y, 1.0);
        }
    ]])
    mixed:send("steps", 5)
    mixed:send("enabled", true)
    mixed:send("weights", 0.1, 0.2, 0.3)
    mixed:send("pair", 0.25, 0.75)
    data = render(function()
        love.graphics.setShader(mixed)
        love.graphics.rectangle("fill", 0, 0, 64, 64)
    end)
    r, g, b = data:getPixel(5, 5)
    check("int bool array vec2 uniforms", near(r, 0.5) and near(g, 0.6) and near(b, 0.75), r .. "," .. g .. "," .. b)
    mixed:send("enabled", false)
    mixed:send("weights", { 0.5, 0.5, 0.5 })
    data = render(function()
        love.graphics.setShader(mixed)
        love.graphics.rectangle("fill", 0, 0, 64, 64)
    end)
    r, g, b = data:getPixel(5, 5)
    check("bool false", near(g, 0), g)

    -- screen_coords: origin at the top left of the target
    local coords = love.graphics.newShader([[
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            return vec4(sc.x / love_ScreenSize.x, sc.y / love_ScreenSize.y, 0.0, 1.0);
        }
    ]])
    data = render(function()
        love.graphics.setShader(coords)
        love.graphics.rectangle("fill", 0, 0, 64, 64)
    end)
    r, g = data:getPixel(2, 2)
    check("screen coords top left", r < 0.1 and g < 0.1, r .. "," .. g)
    r, g = data:getPixel(61, 61)
    check("screen coords bottom right", r > 0.9 and g > 0.9, r .. "," .. g)
    r, g = data:getPixel(61, 2)
    check("screen coords y down", r > 0.9 and g < 0.1, r .. "," .. g)

    -- Vertex shader
    local shift = love.graphics.newShader([[
        vec4 position(mat4 transform_projection, vec4 vertex_position)
        {
            return transform_projection * (vertex_position + vec4(20.0, 0.0, 0.0, 0.0));
        }
    ]])
    data = render(function()
        love.graphics.setShader(shift)
        love.graphics.setColor(1, 1, 1, 1)
        love.graphics.rectangle("fill", 0, 0, 20, 64)
    end)
    check("vertex shader moves geometry away", select(4, data:getPixel(10, 10)) < 0.1)
    check("vertex shader moves geometry here", select(4, data:getPixel(30, 10)) > 0.9)

    -- Pixel and vertex code in separate arguments, with a varying
    local both = love.graphics.newShader([[
        varying float mark;
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            return vec4(mark, 0.0, 0.0, 1.0);
        }
    ]], [[
        varying float mark;
        vec4 position(mat4 transform_projection, vec4 vertex_position)
        {
            mark = 0.5;
            return transform_projection * vertex_position;
        }
    ]])
    data = render(function()
        love.graphics.setShader(both)
        love.graphics.rectangle("fill", 0, 0, 64, 64)
    end)
    check("varying between stages", near(select(1, data:getPixel(5, 5)), 0.5))

    -- Extra textures
    local maskData = love.image.newImageData(2, 1)
    maskData:setPixel(0, 0, 1, 1, 1, 1)
    maskData:setPixel(1, 0, 0, 0, 0, 1)
    local mask = love.graphics.newImage(maskData)
    mask:setFilter("nearest", "nearest")
    local masked = love.graphics.newShader([[
        extern Image mask;
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            return vec4(Texel(mask, vec2(sc.x / love_ScreenSize.x, 0.5)).rgb, 1.0) * color;
        }
    ]])
    masked:send("mask", mask)
    data = render(function()
        love.graphics.setShader(masked)
        love.graphics.rectangle("fill", 0, 0, 64, 32)
        love.graphics.rectangle("fill", 0, 32, 64, 32)
    end)
    check("extra texture left half", near(select(1, data:getPixel(8, 8)), 1), select(1, data:getPixel(8, 8)))
    check("extra texture right half", near(select(1, data:getPixel(56, 8)), 0))
    check("extra texture survives batch flush", near(select(1, data:getPixel(8, 50)), 1), select(1, data:getPixel(8, 50)))

    -- push/pop "all" restores the shader
    love.graphics.setShader(invert)
    love.graphics.push("all")
    love.graphics.setShader(tint)
    check("shader inside push", love.graphics.getShader() == tint)
    love.graphics.pop()
    check("shader restored by pop", love.graphics.getShader() == invert)
    love.graphics.setShader()

    -- Images and canvases drawn through a shader
    local tile = love.graphics.newImage(love.image.newImageData(4, 4))
    local invertedTile = render(function()
        love.graphics.setColor(0, 1, 0, 1)
        love.graphics.rectangle("fill", 0, 0, 64, 64)
        love.graphics.setShader(invert)
        love.graphics.setColor(1, 1, 1, 1)
        love.graphics.draw(canvas, 0, 0)
    end)
    check("draw with shader does not crash", invertedTile:getWidth() == 64)

    -- Text goes through the shader as well
    data = render(function()
        love.graphics.setShader(tint)
        tint:send("tint", { 1, 0, 0 })
        love.graphics.print("MMMM", 0, 0)
    end)
    local seen = false
    for y = 0, 20 do
        for x = 0, 40 do
            local pr, pg = data:getPixel(x, y)
            if pr > 0.9 and pg < 0.1 then seen = true end
        end
    end
    check("text uses the shader", seen)
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
        print(string.format("All %d shader checks passed", passed))
        love.event.quit(0)
    end
end
