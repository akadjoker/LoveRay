-- Particle systems. The emitter follows the mouse, a click fires a burst.
-- Keys 1 to 5 pick an effect, `love examples/particles snow` starts with one.

local glow
local effects = {}
local current = 1
local font

local function makeGlow(size)
    local data = love.image.newImageData(size, size)
    local half = size / 2
    data:mapPixel(function(x, y)
        local d = math.sqrt((x + 0.5 - half) ^ 2 + (y + 0.5 - half) ^ 2) / half
        local a = math.max(0, 1 - d)
        return 1, 1, 1, a * a
    end)
    return love.graphics.newImage(data)
end

local function newEffect(name, blend, setup, follows)
    local ps = love.graphics.newParticleSystem(glow, 2000)
    setup(ps)
    effects[#effects + 1] = { name = name, ps = ps, blend = blend, follows = follows ~= false }
end

function love.load(args)
    love.graphics.setBackgroundColor(0.04, 0.04, 0.07)
    font = love.graphics.newFont(14)
    glow = makeGlow(64)

    newEffect("fire", "add", function(ps)
        ps:setEmissionRate(160)
        ps:setParticleLifetime(0.5, 1.2)
        ps:setDirection(-math.pi / 2)
        ps:setSpread(0.5)
        ps:setSpeed(40, 120)
        ps:setLinearAcceleration(-20, -120, 20, -40)
        ps:setSizes(0.8, 0.5, 0.05)
        ps:setSizeVariation(0.4)
        ps:setColors(1, 0.9, 0.3, 1, 1, 0.4, 0.1, 0.8, 0.8, 0.1, 0.05, 0)
        ps:setEmissionArea("normal", 12, 4)
    end)

    newEffect("smoke", "alpha", function(ps)
        ps:setEmissionRate(30)
        ps:setParticleLifetime(2, 4)
        ps:setDirection(-math.pi / 2)
        ps:setSpread(0.4)
        ps:setSpeed(20, 50)
        ps:setLinearAcceleration(10, -10, 30, -20)
        ps:setSizes(0.4, 1.4, 2.2)
        ps:setSpin(-1, 1)
        ps:setColors(0.6, 0.6, 0.62, 0.5, 0.3, 0.3, 0.32, 0.25, 0.15, 0.15, 0.17, 0)
        ps:setEmissionArea("uniform", 8, 2)
    end)

    newEffect("fountain", "add", function(ps)
        ps:setEmissionRate(250)
        ps:setParticleLifetime(1.4, 2)
        ps:setDirection(-math.pi / 2)
        ps:setSpread(0.35)
        ps:setSpeed(280, 380)
        ps:setLinearAcceleration(0, 520)
        ps:setSizes(0.35, 0.25)
        ps:setColors(0.5, 0.8, 1, 1, 0.2, 0.5, 1, 0.8, 0.1, 0.3, 0.9, 0)
    end)

    newEffect("burst", "add", function(ps)
        ps:setEmissionRate(0)
        ps:setParticleLifetime(0.6, 1.4)
        ps:setSpread(math.pi * 2)
        ps:setSpeed(100, 420)
        ps:setLinearDamping(2.5, 4)
        ps:setSizes(0.5, 0.3, 0)
        ps:setSizeVariation(0.6)
        ps:setColors(1, 1, 0.6, 1, 1, 0.5, 0.2, 1, 0.9, 0.2, 0.6, 0)
    end)

    newEffect("snow", "alpha", function(ps)
        ps:setPosition(400, -10)
        ps:setEmissionRate(80)
        ps:setParticleLifetime(6, 9)
        ps:setDirection(math.pi / 2)
        ps:setSpread(0.3)
        ps:setSpeed(30, 70)
        ps:setLinearAcceleration(-15, 0, 15, 10)
        ps:setSizes(0.12, 0.2, 0.15)
        ps:setSizeVariation(0.7)
        ps:setColors(1, 1, 1, 0.9, 1, 1, 1, 0.9, 1, 1, 1, 0)
        ps:setEmissionArea("uniform", 420, 0)
    end, false)

    for i, e in ipairs(effects) do
        if e.name == args[1] then
            current = i
        end
    end
end

function love.update(dt)
    local e = effects[current]
    if e.follows then
        e.ps:moveTo(love.mouse.getPosition())
    end
    e.ps:update(dt)
end

function love.mousepressed(x, y, button)
    local e = effects[current]
    if e.name == "burst" then
        e.ps:setPosition(x, y)
        e.ps:emit(180)
    end
end

function love.keypressed(key)
    local n = tonumber(key)
    if n and effects[n] then
        current = n
    elseif key == "escape" then
        love.event.quit()
    end
end

function love.draw()
    local e = effects[current]
    love.graphics.setBlendMode(e.blend)
    love.graphics.setColor(1, 1, 1, 1)
    love.graphics.draw(e.ps)
    love.graphics.setBlendMode("alpha")

    love.graphics.setFont(font)
    for i, effect in ipairs(effects) do
        love.graphics.setColor(i == current and 1 or 0.55, i == current and 0.85 or 0.55, i == current and 0.3 or 0.55, 1)
        love.graphics.print(i .. "  " .. effect.name, 20, 20 + (i - 1) * 20)
    end
    love.graphics.setColor(0.6, 0.6, 0.6, 1)
    love.graphics.print(string.format("%d particles  %d fps", e.ps:getCount(), love.timer.getFPS()), 20, 140)
    if e.name == "burst" then
        love.graphics.print("click anywhere", 20, 160)
    end
end
