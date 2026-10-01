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
    return math.abs(a - b) <= (eps or 1e-4)
end

local canvas

local function render(ps)
    love.graphics.setCanvas(canvas)
    love.graphics.clear(0, 0, 0, 0)
    love.graphics.setColor(1, 1, 1, 1)
    love.graphics.draw(ps)
    love.graphics.setCanvas()
    return canvas:newImageData()
end

local function alphaAt(data, x, y)
    return select(4, data:getPixel(x, y))
end

local function anyAlpha(data, x0, x1, y)
    for x = x0, x1 do
        if alphaAt(data, x, y) > 0.5 then return true end
    end
    return false
end

local function run()
    canvas = love.graphics.newCanvas(100, 100)
    local pixels = love.image.newImageData(4, 4)
    pixels:mapPixel(function() return 1, 1, 1, 1 end)
    local image = love.graphics.newImage(pixels)

    local ps = love.graphics.newParticleSystem(image, 50)
    check("type", ps:type() == "ParticleSystem" and ps:typeOf("Object"))
    check("defaults", ps:getCount() == 0 and ps:getBufferSize() == 50 and ps:isActive() and not ps:isPaused() and not ps:isStopped())
    check("default emission rate", ps:getEmissionRate() == 0)
    check("default offset is texture center", select(1, ps:getOffset()) == 2 and select(2, ps:getOffset()) == 2)
    check("getTexture", ps:getTexture() == image)

    -- Emission and lifetime
    ps:setParticleLifetime(1)
    local lo, hi = ps:getParticleLifetime()
    check("particle lifetime", lo == 1 and hi == 1)
    ps:emit(10)
    check("emit", ps:getCount() == 10)
    ps:update(0.5)
    check("particles alive before lifetime", ps:getCount() == 10)
    ps:update(0.6)
    check("particles die after lifetime", ps:getCount() == 0)
    ps:emit(500)
    check("buffer limits emission", ps:getCount() == 50)
    ps:reset()
    check("reset", ps:getCount() == 0)

    -- Continuous emission
    ps:setParticleLifetime(5)
    ps:setEmissionRate(10)
    ps:update(1)
    check("emission rate", ps:getCount() >= 9 and ps:getCount() <= 11, ps:getCount())
    ps:stop()
    check("stop", ps:isStopped() and not ps:isActive())
    local before = ps:getCount()
    ps:update(1)
    check("stopped system emits nothing", ps:getCount() == before)
    ps:start()
    check("start", ps:isActive())
    ps:pause()
    check("pause", ps:isPaused() and not ps:isActive())
    before = ps:getCount()
    ps:update(1)
    check("paused system does not update", ps:getCount() == before)
    ps:start()

    -- Emitter lifetime stops emission
    local timed = love.graphics.newParticleSystem(image, 100)
    timed:setEmissionRate(20)
    timed:setParticleLifetime(10)
    timed:setEmitterLifetime(0.5)
    timed:update(0.3)
    check("emitter still active", timed:isActive())
    timed:update(0.3)
    check("emitter lifetime expires", not timed:isActive())
    check("emitter lifetime getter", near(timed:getEmitterLifetime(), 0.5))

    -- Motion, checked through rendering
    local mover = love.graphics.newParticleSystem(image, 10)
    mover:setPosition(20, 50)
    mover:setParticleLifetime(10)
    mover:setDirection(0)
    mover:setSpeed(100)
    mover:emit(1)
    mover:update(0.2)
    local data = render(mover)
    check("particle moved along its direction", alphaAt(data, 40, 50) > 0.9 and alphaAt(data, 20, 50) < 0.1)

    local fall = love.graphics.newParticleSystem(image, 10)
    fall:setPosition(50, 10)
    fall:setParticleLifetime(10)
    fall:setLinearAcceleration(0, 100)
    fall:emit(1)
    for _ = 1, 10 do fall:update(0.1) end
    data = render(fall)
    local fell = false
    for y = 40, 70 do
        if alphaAt(data, 50, y) > 0.5 then fell = true end
    end
    check("linear acceleration", fell and alphaAt(data, 50, 10) < 0.1)

    local damped = love.graphics.newParticleSystem(image, 10)
    damped:setPosition(10, 50)
    damped:setParticleLifetime(10)
    damped:setSpeed(100)
    damped:setLinearDamping(10)
    damped:emit(1)
    for _ = 1, 20 do damped:update(0.1) end
    data = render(damped)
    check("linear damping slows particles", anyAlpha(data, 12, 30, 50) and not anyAlpha(data, 40, 99, 50))

    local radial = love.graphics.newParticleSystem(image, 10)
    radial:setPosition(50, 50)
    radial:setParticleLifetime(10)
    radial:setSpeed(10)
    radial:setDirection(0)
    radial:setRadialAcceleration(50)
    radial:emit(1)
    for _ = 1, 10 do radial:update(0.1) end
    data = render(radial)
    check("radial acceleration pushes outward", anyAlpha(data, 75, 99, 50) and not anyAlpha(data, 40, 60, 50))

    -- Colors and sizes over life
    local colored = love.graphics.newParticleSystem(image, 10)
    colored:setPosition(50, 50)
    colored:setParticleLifetime(1)
    colored:setColors(1, 0, 0, 1, 0, 0, 1, 1)
    colored:emit(1)
    data = render(colored)
    local r, g, b = data:getPixel(50, 50)
    check("start color", near(r, 1, 0.02) and near(b, 0, 0.02), r .. "," .. b)
    colored:update(0.5)
    data = render(colored)
    r, g, b = data:getPixel(50, 50)
    check("interpolated color", near(r, 0.5, 0.05) and near(b, 0.5, 0.05), r .. "," .. b)

    local grow = love.graphics.newParticleSystem(image, 10)
    grow:setPosition(50, 50)
    grow:setParticleLifetime(1)
    grow:setSizes(1, 5)
    grow:emit(1)
    data = render(grow)
    check("start size", alphaAt(data, 50, 50) > 0.9 and alphaAt(data, 55, 50) < 0.1)
    grow:update(0.99)
    data = render(grow)
    check("end size", alphaAt(data, 58, 50) > 0.9 and alphaAt(data, 62, 50) < 0.1)

    -- Tint from the current color
    love.graphics.setCanvas(canvas)
    love.graphics.clear(0, 0, 0, 0)
    love.graphics.setColor(0, 1, 0, 1)
    local plain = love.graphics.newParticleSystem(image, 5)
    plain:setPosition(50, 50)
    plain:setParticleLifetime(1)
    plain:emit(1)
    love.graphics.draw(plain)
    love.graphics.setCanvas()
    love.graphics.setColor(1, 1, 1, 1)
    data = canvas:newImageData()
    r, g, b = data:getPixel(50, 50)
    check("current color tints particles", near(r, 0, 0.02) and near(g, 1, 0.02), r .. "," .. g)

    -- Transform arguments of draw
    love.graphics.setCanvas(canvas)
    love.graphics.clear(0, 0, 0, 0)
    love.graphics.draw(plain, 30, 0)
    love.graphics.setCanvas()
    data = canvas:newImageData()
    check("draw offsets the whole system", alphaAt(data, 80, 50) > 0.9 and alphaAt(data, 50, 50) < 0.1)

    -- Emission area
    local area = love.graphics.newParticleSystem(image, 200)
    area:setPosition(50, 50)
    area:setParticleLifetime(10)
    area:setEmissionArea("uniform", 30, 0)
    area:emit(150)
    data = render(area)
    local leftSeen, rightSeen, outside = false, false, false
    for x = 0, 99 do
        if alphaAt(data, x, 50) > 0.5 then
            if x < 45 then leftSeen = true end
            if x > 55 then rightSeen = true end
            if x < 17 or x > 83 then outside = true end
        end
        if alphaAt(data, x, 20) > 0.5 or alphaAt(data, x, 80) > 0.5 then outside = true end
    end
    check("uniform emission area spreads horizontally", leftSeen and rightSeen and not outside)
    local dist, dx, dy, angle, relative = area:getEmissionArea()
    check("getEmissionArea", dist == "uniform" and dx == 30 and dy == 0 and angle == 0 and relative == false)

    -- Quads
    local quadPixels = love.image.newImageData(4, 2)
    quadPixels:setPixel(0, 0, 1, 0, 0, 1)
    quadPixels:setPixel(1, 0, 1, 0, 0, 1)
    quadPixels:setPixel(0, 1, 1, 0, 0, 1)
    quadPixels:setPixel(1, 1, 1, 0, 0, 1)
    quadPixels:setPixel(2, 0, 0, 0, 1, 1)
    quadPixels:setPixel(3, 0, 0, 0, 1, 1)
    quadPixels:setPixel(2, 1, 0, 0, 1, 1)
    quadPixels:setPixel(3, 1, 0, 0, 1, 1)
    local sheet = love.graphics.newImage(quadPixels)
    sheet:setFilter("nearest", "nearest")
    local quadSystem = love.graphics.newParticleSystem(sheet, 5)
    quadSystem:setQuads(love.graphics.newQuad(0, 0, 2, 2, sheet), love.graphics.newQuad(2, 0, 2, 2, sheet))
    quadSystem:setOffset(1, 1)
    quadSystem:setPosition(50, 50)
    quadSystem:setParticleLifetime(1)
    quadSystem:emit(1)
    data = render(quadSystem)
    r, g, b = data:getPixel(50, 50)
    check("first quad at birth", near(r, 1, 0.05) and near(b, 0, 0.05), r .. "," .. b)
    quadSystem:update(0.75)
    data = render(quadSystem)
    r, g, b = data:getPixel(50, 50)
    check("second quad late in life", near(r, 0, 0.05) and near(b, 1, 0.05), r .. "," .. b)
    check("getQuads", #quadSystem:getQuads() == 2)

    -- Getters and setters
    ps:setSizes(1, 2, 3)
    local s1, s2, s3 = ps:getSizes()
    check("sizes roundtrip", s1 == 1 and s2 == 2 and s3 == 3)
    check("too many sizes", not pcall(ps.setSizes, ps, 1, 2, 3, 4, 5, 6, 7, 8, 9))
    ps:setColors({ 1, 0, 0 }, { 0, 1, 0, 0.5 })
    local c = { ps:getColors() }
    check("colors from tables", #c == 8 and c[4] == 1 and c[8] == 0.5, #c)
    check("bad colors", not pcall(ps.setColors, ps, 1, 0, 0))
    ps:setSpread(1.5)
    check("spread", near(ps:getSpread(), 1.5))
    ps:setSpeed(10, 20)
    local smin, smax = ps:getSpeed()
    check("speed", smin == 10 and smax == 20)
    ps:setLinearAcceleration(1, 2, 3, 4)
    local ax1, ay1, ax2, ay2 = ps:getLinearAcceleration()
    check("linear acceleration", ax1 == 1 and ay1 == 2 and ax2 == 3 and ay2 == 4)
    ps:setSpin(1, 2)
    ps:setSpinVariation(0.5)
    local sp1, sp2, spv = ps:getSpin()
    check("spin", sp1 == 1 and sp2 == 2 and spv == 0.5)
    ps:setInsertMode("bottom")
    check("insert mode", ps:getInsertMode() == "bottom")
    check("bad insert mode", not pcall(ps.setInsertMode, ps, "sideways"))
    ps:setRelativeRotation(true)
    check("relative rotation", ps:hasRelativeRotation())
    check("bad buffer size", not pcall(ps.setBufferSize, ps, 0))
    check("bad size variation", not pcall(ps.setSizeVariation, ps, 2))
    ps:setBufferSize(5)
    check("shrinking the buffer drops particles", ps:getCount() <= 5)

    -- Clone
    local original = love.graphics.newParticleSystem(image, 77)
    original:setSpeed(5, 6)
    original:setEmissionRate(9)
    original:setSizes(2, 3)
    original:emit(3)
    local clone = original:clone()
    check("clone settings", clone:getBufferSize() == 77 and clone:getEmissionRate() == 9 and select(2, clone:getSpeed()) == 6)
    check("clone sizes", select(2, clone:getSizes()) == 3)
    check("clone starts stopped and empty", clone:getCount() == 0 and clone:isStopped())
    check("clone is independent", original:getCount() == 3)

    -- moveTo interpolates the emitter between frames
    local sweeper = love.graphics.newParticleSystem(image, 100)
    sweeper:setPosition(0, 50)
    sweeper:setParticleLifetime(10)
    sweeper:setEmissionRate(1000)
    sweeper:moveTo(100, 50)
    sweeper:update(0.05)
    data = render(sweeper)
    local covered = 0
    for x = 0, 99, 10 do
        if alphaAt(data, x, 50) > 0.5 then covered = covered + 1 end
    end
    check("moveTo spreads spawns along the path", covered >= 5, covered)
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
        print(string.format("All %d particle checks passed", passed))
        love.event.quit(0)
    end
end
