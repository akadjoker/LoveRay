local name, root, frames

local keys =
{
    "left", "right", "up", "down", "space", "z", "w", "a", "s", "d", "return", "r", "h", "c", "lshift",
    "pageup", "pagedown", "tab",
}

local held = {}
local isDown = function(...)
    for _, k in ipairs({...}) do
        if held[k] then
            return true
        end
    end
    return false
end

local game = {}
local function capture()
    for _, callback in ipairs({"load", "update", "draw", "keypressed", "keyreleased", "mousepressed", "mousereleased", "wheelmoved", "textinput"}) do
        game[callback] = love[callback]
        love[callback] = nil
    end
end

love.load = function(args)
    name = args[1]
    frames = tonumber(args[2]) or 1500
    root = love.filesystem.getSource() .. "/../../examples/" .. name
    package.path = root .. "/?.lua;" .. package.path

    love.keyboard.isDown = isDown
    love.mouse.isDown = function(button)
        return held["mouse" .. button] == true
    end

    local chunk, err = loadfile(root .. "/main.lua")
    if not chunk then
        io.stderr:write(err, "\n")
        os.exit(1)
    end
    chunk()
    capture()

    local started = 0
    local function step()
        started = started + 1
        if started % 7 == 0 then
            for k in pairs(held) do
                held[k] = nil
            end
            for _ = 1, love.math.random(1, 4) do
                held[keys[love.math.random(1, #keys)]] = true
            end
            held.mouse1 = love.math.random() < 0.5 or nil
        end
        if name == "tutorials" and started % 120 == 0 then
            game.keypressed("f" .. ((started / 120 - 1) % 12 + 1), "f", false)
        end
        if started % 23 == 0 and game.keypressed then
            local k = keys[love.math.random(1, #keys)]
            if not (name == "tutorials" and (k == "pageup" or k == "pagedown" or k == "tab")) then
                game.keypressed(k, k, false)
            end
        end
        if started % 31 == 0 and game.mousepressed then
            game.mousepressed(love.math.random(0, 800), love.math.random(0, 600), love.math.random(1, 2), false, 1)
        end
        if started % 37 == 0 and game.wheelmoved then
            game.wheelmoved(0, love.math.random(-2, 2))
        end
        if started % 41 == 0 and game.textinput then
            game.textinput("x")
        end
        if game.update then
            game.update(1 / 60)
        end
        if started >= frames then
            love.event.quit()
        end
    end

    local function guard(fn)
        return function(...)
            local ok, err = xpcall(fn, debug.traceback, ...)
            if not ok then
                io.stderr:write(name, ": ", tostring(err), "\n")
                os.exit(1)
            end
        end
    end

    love.update = guard(step)
    love.draw = guard(game.draw)

    guard(game.load)({})
end
