-- Post-processing with Love shaders: the scene is rendered to a canvas and
-- drawn back through the selected effect.

local scene, image, font
local time = 0
local current = 1
local effects = {}

local function effect(name, code)
    local shader = love.graphics.newShader(code)
    effects[#effects + 1] = { name = name, shader = shader }
end

function love.load(args)
    font = love.graphics.newFont(14)
    image = love.graphics.newImage("images/zazaka.png")
    scene = love.graphics.newCanvas(800, 600)

    effects[1] = { name = "none" }

    effect("grayscale", [[
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            vec4 p = Texel(tex, uv) * color;
            float g = dot(p.rgb, vec3(0.299, 0.587, 0.114));
            return vec4(vec3(g), p.a);
        }
    ]])

    effect("wave", [[
        extern number time;
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            vec2 offset = vec2(sin(uv.y * 30.0 + time * 3.0) * 0.01, cos(uv.x * 30.0 + time * 2.0) * 0.01);
            return Texel(tex, uv + offset) * color;
        }
    ]])

    effect("vignette", [[
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            vec4 p = Texel(tex, uv) * color;
            float d = distance(uv, vec2(0.5));
            float shade = smoothstep(0.85, 0.2, d);
            return vec4(p.rgb * shade, p.a);
        }
    ]])

    effect("pixelate", [[
        extern vec2 size;
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            vec2 cell = vec2(8.0) / size;
            return Texel(tex, (floor(uv / cell) + 0.5) * cell) * color;
        }
    ]])

    effect("chromatic", [[
        vec4 effect(vec4 color, Image tex, vec2 uv, vec2 sc)
        {
            float shift = 0.006;
            float r = Texel(tex, uv + vec2(shift, 0.0)).r;
            float g = Texel(tex, uv).g;
            float b = Texel(tex, uv - vec2(shift, 0.0)).b;
            return vec4(r, g, b, 1.0) * color;
        }
    ]])

    for i, e in ipairs(effects) do
        if e.name == args[1] then
            current = i
        end
    end
end

function love.update(dt)
    time = time + dt
end

local function drawScene()
    love.graphics.setCanvas(scene)
    love.graphics.clear(0.12, 0.14, 0.2, 1)

    for i = 0, 11 do
        local a = time * 0.6 + i * math.pi / 6
        love.graphics.setColor(0.5 + 0.5 * math.sin(i), 0.5 + 0.5 * math.cos(i * 1.7), 0.8, 1)
        love.graphics.circle("fill", 400 + math.cos(a) * 250, 300 + math.sin(a) * 180, 36)
    end

    love.graphics.setColor(1, 1, 1, 1)
    love.graphics.draw(image, 400, 300, math.sin(time) * 0.4, 1.4, 1.4, image:getWidth() / 2, image:getHeight() / 2)

    love.graphics.setFont(font)
    for i, e in ipairs(effects) do
        love.graphics.setColor(i == current and 1 or 0.6, i == current and 0.85 or 0.6, i == current and 0.3 or 0.6, 1)
        love.graphics.print(i .. "  " .. e.name, 20, 20 + (i - 1) * 20)
    end
    love.graphics.setCanvas()
end

function love.draw()
    drawScene()
    local e = effects[current]
    love.graphics.setColor(1, 1, 1, 1)
    if e.shader then
        if e.shader:hasUniform("time") then e.shader:send("time", time) end
        if e.shader:hasUniform("size") then e.shader:send("size", { 800, 600 }) end
        love.graphics.setShader(e.shader)
    end
    love.graphics.draw(scene, 0, 0)
    love.graphics.setShader()
end

function love.keypressed(key)
    local n = tonumber(key)
    if n and effects[n] then
        current = n
    elseif key == "right" then
        current = current % #effects + 1
    elseif key == "left" then
        current = (current - 2) % #effects + 1
    elseif key == "escape" then
        love.event.quit()
    end
end
