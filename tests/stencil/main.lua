local failures = {}
local passed = 0

local function check(name, ok, detail)
    if ok then
        passed = passed + 1
    else
        table.insert(failures, name .. (detail and (": " .. tostring(detail)) or ""))
    end
end

local canvas

local function render(fn)
    love.graphics.setCanvas(canvas)
    love.graphics.clear(0, 0, 0, 0)
    love.graphics.setColor(1, 1, 1, 1)
    fn()
    love.graphics.setStencilTest()
    love.graphics.setCanvas()
    return canvas:newImageData()
end

local function drawn(data, x, y)
    return select(4, data:getPixel(x, y)) > 0.5
end

local function disc(x, y, r)
    return function() love.graphics.circle("fill", x, y, r) end
end

local function fill()
    love.graphics.setColor(1, 0, 0, 1)
    love.graphics.rectangle("fill", 0, 0, 64, 64)
end

local screenResult

local function run()
    canvas = love.graphics.newCanvas(64, 64)

    check("default test", select(1, love.graphics.getStencilTest()) == "always" and select(2, love.graphics.getStencilTest()) == 0)

    -- A circle selects its own pixels
    local data = render(function()
        love.graphics.stencil(disc(32, 32, 16))
        love.graphics.setStencilTest("greater", 0)
        fill()
    end)
    check("greater: inside is drawn", drawn(data, 32, 32))
    check("greater: outside is hidden", not drawn(data, 4, 4) and not drawn(data, 60, 60))

    data = render(function()
        love.graphics.stencil(disc(32, 32, 16))
        love.graphics.setStencilTest("equal", 0)
        fill()
    end)
    check("equal 0: outside is drawn", drawn(data, 4, 4) and drawn(data, 60, 32))
    check("equal 0: inside is hidden", not drawn(data, 32, 32))

    -- Disabling the test draws everywhere again
    data = render(function()
        love.graphics.stencil(disc(32, 32, 16))
        love.graphics.setStencilTest("greater", 0)
        love.graphics.setStencilTest()
        fill()
    end)
    check("setStencilTest() disables", drawn(data, 4, 4) and drawn(data, 32, 32))

    -- Actions
    local function squares(action, keep)
        love.graphics.stencil(function() love.graphics.rectangle("fill", 8, 8, 32, 32) end, action)
        love.graphics.stencil(function() love.graphics.rectangle("fill", 24, 24, 32, 32) end, action, 1, keep)
    end

    data = render(function()
        squares("increment", true)
        love.graphics.setStencilTest("equal", 2)
        fill()
    end)
    check("increment: only the overlap reaches 2", drawn(data, 32, 32) and not drawn(data, 12, 12) and not drawn(data, 50, 50))

    data = render(function()
        squares("increment", true)
        love.graphics.setStencilTest("greater", 0)
        fill()
    end)
    check("increment: union is above zero", drawn(data, 12, 12) and drawn(data, 32, 32) and drawn(data, 50, 50) and not drawn(data, 60, 4))

    data = render(function()
        squares("replace", false)
        love.graphics.setStencilTest("greater", 0)
        fill()
    end)
    check("keepvalues false discards earlier stencils", not drawn(data, 12, 12) and drawn(data, 32, 32) and drawn(data, 50, 50))

    data = render(function()
        squares("replace", true)
        love.graphics.setStencilTest("greater", 0)
        fill()
    end)
    check("keepvalues true keeps earlier stencils", drawn(data, 12, 12) and drawn(data, 50, 50))

    data = render(function()
        love.graphics.stencil(disc(32, 32, 16), "replace", 5)
        love.graphics.setStencilTest("equal", 5)
        fill()
    end)
    check("replace with a value", drawn(data, 32, 32) and not drawn(data, 4, 4))

    data = render(function()
        love.graphics.stencil(disc(32, 32, 16), "replace", 3)
        love.graphics.stencil(disc(32, 32, 16), "decrement", 1, true)
        love.graphics.setStencilTest("equal", 2)
        fill()
    end)
    check("decrement", drawn(data, 32, 32))

    data = render(function()
        love.graphics.stencil(disc(32, 32, 16), "invert")
        love.graphics.stencil(disc(32, 32, 16), "invert", 1, true)
        love.graphics.setStencilTest("greater", 0)
        fill()
    end)
    check("invert twice restores zero", not drawn(data, 32, 32))

    data = render(function()
        love.graphics.stencil(disc(32, 32, 16), "incrementwrap")
        love.graphics.setStencilTest("equal", 1)
        fill()
    end)
    check("incrementwrap", drawn(data, 32, 32))

    -- Comparison modes
    local function compare(mode, value)
        return render(function()
            love.graphics.stencil(disc(32, 32, 16), "replace", 4)
            love.graphics.setStencilTest(mode, value)
            fill()
        end)
    end
    data = compare("greater", 3)
    check("greater 3 passes 4", drawn(data, 32, 32) and not drawn(data, 4, 4))
    data = compare("greater", 4)
    check("greater 4 fails 4", not drawn(data, 32, 32))
    data = compare("gequal", 4)
    check("gequal 4 passes 4", drawn(data, 32, 32))
    data = compare("less", 5)
    check("less 5 passes 4 and 0", drawn(data, 32, 32) and drawn(data, 4, 4))
    data = compare("less", 4)
    check("less 4 fails 4 but passes 0", not drawn(data, 32, 32) and drawn(data, 4, 4))
    data = compare("lequal", 4)
    check("lequal 4 passes 4", drawn(data, 32, 32) and drawn(data, 4, 4))
    data = compare("notequal", 4)
    check("notequal 4", not drawn(data, 32, 32) and drawn(data, 4, 4))

    -- Getters, errors and state handling
    love.graphics.setStencilTest("lequal", 9)
    local mode, value = love.graphics.getStencilTest()
    check("getStencilTest", mode == "lequal" and value == 9)
    love.graphics.setStencilTest()
    check("bad compare mode", not pcall(love.graphics.setStencilTest, "bigger", 1))
    check("bad action", not pcall(love.graphics.stencil, function() end, "explode"))
    check("stencil needs a function", not pcall(love.graphics.stencil, 5))

    love.graphics.setStencilTest("equal", 3)
    love.graphics.push("all")
    love.graphics.setStencilTest("greater", 1)
    love.graphics.pop()
    mode, value = love.graphics.getStencilTest()
    check("push all / pop restores the test", mode == "equal" and value == 3)
    love.graphics.reset()
    check("reset disables the test", love.graphics.getStencilTest() == "always")

    -- An error inside the stencil function restores the drawing state
    local ok = pcall(love.graphics.stencil, function() error("boom") end)
    check("error in stencil function propagates", not ok)
    data = render(function()
        love.graphics.setColor(0, 1, 0, 1)
        love.graphics.rectangle("fill", 0, 0, 64, 64)
    end)
    check("drawing works after a failed stencil", drawn(data, 32, 32))

    -- clear() resets the stencil buffer
    data = render(function()
        love.graphics.stencil(disc(32, 32, 16))
        love.graphics.clear(0, 0, 0, 0)
        love.graphics.setStencilTest("greater", 0)
        fill()
    end)
    check("clear resets the stencil", not drawn(data, 32, 32))

    data = render(function()
        love.graphics.stencil(disc(32, 32, 16))
        love.graphics.clear(0, 0, 0, 0, false)
        love.graphics.setStencilTest("greater", 0)
        fill()
    end)
    check("clear(..., false) keeps the stencil", drawn(data, 32, 32))

    -- Stencil content belongs to each canvas
    local other = love.graphics.newCanvas(64, 64)
    love.graphics.setCanvas(canvas)
    love.graphics.clear(0, 0, 0, 0)
    love.graphics.stencil(disc(32, 32, 16))
    love.graphics.setCanvas(other)
    love.graphics.clear(0, 0, 0, 0)
    love.graphics.setStencilTest("greater", 0)
    fill()
    love.graphics.setStencilTest()
    love.graphics.setCanvas()
    data = other:newImageData()
    check("a canvas does not see another canvas's stencil", not drawn(data, 32, 32))
end

local ok, err = xpcall(run, debug.traceback)
if not ok then
    table.insert(failures, "runtime error: " .. tostring(err))
end

local requested = false

function love.draw()
    love.graphics.reset()
    love.graphics.clear(0, 0, 0, 1)
    love.graphics.stencil(disc(64, 64, 30))
    love.graphics.setStencilTest("greater", 0)
    love.graphics.setColor(1, 0, 0, 1)
    love.graphics.rectangle("fill", 0, 0, 128, 128)
    love.graphics.setStencilTest()
    if not requested then
        requested = true
        love.graphics.captureScreenshot(function(image)
            local r, g, b = image:getPixel(64, 64)
            local cr, cg, cb = image:getPixel(5, 5)
            screenResult = { inside = r, outside = cr + cg + cb }
        end)
    end
end

function love.update()
    if screenResult == nil then
        return
    end
    check("window framebuffer has a stencil buffer: inside", screenResult.inside > 0.9, screenResult.inside)
    check("window framebuffer has a stencil buffer: outside", screenResult.outside < 0.1, screenResult.outside)
    if #failures > 0 then
        print("FAILED checks:")
        for _, f in ipairs(failures) do
            print("  - " .. f)
        end
        love.event.quit(1)
    else
        print(string.format("All %d stencil checks passed", passed))
        love.event.quit(0)
    end
end
