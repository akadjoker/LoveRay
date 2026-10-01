--
-- boot.lua - LoveRay startup sequence.
--
-- Mirrors the structure of Love2D's own boot.lua so games that rely on
-- love.run / love.errorhandler / love.handlers behave the same way:
--
--   1. parse the command line (love.arg)
--   2. mount the game directory (love.filesystem)
--   3. run conf.lua and apply love.conf
--   4. open the window and load main.lua (or the no-game screen)
--   5. run the main loop until it returns an exit code
--
-- Copyright (c) 2023-2025 djoker. MIT license.
--

local love = require("love")

love.path = {}
love.arg = {}

-------------------------------------------------------------------------------
-- love.path helpers
-------------------------------------------------------------------------------

function love.path.normalslashes(p)
    return (string.gsub(p, "\\", "/"))
end

function love.path.endslash(p)
    if string.sub(p, -1) ~= "/" then
        return p .. "/"
    end
    return p
end

function love.path.abs(p)
    local tmp = love.path.normalslashes(p)
    if string.find(tmp, "/") == 1 then
        return true
    end
    if string.find(tmp, "%a:") == 1 then
        return true
    end
    return false
end

function love.path.getFull(p)
    if love.path.abs(p) then
        return love.path.normalslashes(p)
    end
    local cwd = love.filesystem.getWorkingDirectory()
    cwd = love.path.normalslashes(cwd)
    cwd = love.path.endslash(cwd)
    local full = cwd .. love.path.normalslashes(p)
    return full:match("(.-)/?$")
end

function love.path.leaf(p)
    p = love.path.normalslashes(p)
    local a = 1
    local last = p
    while a do
        a = string.find(p, "/", a + 1)
        if a then
            last = string.sub(p, a + 1)
        end
    end
    return last
end

-------------------------------------------------------------------------------
-- love.arg: command line handling
-------------------------------------------------------------------------------

love.arg.options = {
    console = { a = 0 },
    fused = { a = 0 },
    version = { a = 0 },
    help = { a = 0 },
    -- LoveRay additions used by headless testing / CI.
    frames = { a = 1 },
    screenshot = { a = 1 },
    game = { a = 1 },
}

love.arg.optionIndices = {}

function love.arg.parseOption(m, i)
    m.set = true
    if m.a > 0 then
        m.arg = {}
        for j = i, i + m.a - 1 do
            love.arg.optionIndices[j] = true
            table.insert(m.arg, arg[j])
        end
    end
    return m.a
end

function love.arg.parseOptions(arg)
    local game
    local argc = #arg
    local i = 1
    while i <= argc do
        local m = arg[i]:match("^%-%-(.*)")
        if m and m ~= "" and love.arg.options[m] and not love.arg.options[m].set then
            love.arg.optionIndices[i] = true
            i = i + love.arg.parseOption(love.arg.options[m], i + 1)
        elseif m == "" then
            -- "--" terminates option parsing.
            love.arg.optionIndices[i] = true
            if not game then
                game = i + 1
            end
            break
        elseif not game then
            game = i
        end
        i = i + 1
    end
    if not love.arg.options.game.set and game then
        love.arg.parseOption(love.arg.options.game, game)
    end
end

-- Returns the arguments meant for the game: everything that is neither a
-- runtime option nor the game path itself.
function love.arg.parseGameArguments(a)
    local out = {}
    for i = 1, #a do
        if not love.arg.optionIndices[i] then
            out[#out + 1] = a[i]
        end
    end
    return out
end

function love.arg.getLow(a)
    local m = math.huge
    for k, v in pairs(a) do
        if k < m then
            m = k
        end
    end
    return a[m], m
end

-------------------------------------------------------------------------------
-- Event handlers
-------------------------------------------------------------------------------

function love.createhandlers()
    love.handlers = setmetatable({
        keypressed = function(b, s, r)
            if love.keypressed then return love.keypressed(b, s, r) end
        end,
        keyreleased = function(b, s)
            if love.keyreleased then return love.keyreleased(b, s) end
        end,
        textinput = function(t)
            if love.textinput then return love.textinput(t) end
        end,
        textedited = function(t, s, l)
            if love.textedited then return love.textedited(t, s, l) end
        end,
        mousemoved = function(x, y, dx, dy, t)
            if love.mousemoved then return love.mousemoved(x, y, dx, dy, t) end
        end,
        mousepressed = function(x, y, b, t, c)
            if love.mousepressed then return love.mousepressed(x, y, b, t, c) end
        end,
        mousereleased = function(x, y, b, t, c)
            if love.mousereleased then return love.mousereleased(x, y, b, t, c) end
        end,
        wheelmoved = function(x, y)
            if love.wheelmoved then return love.wheelmoved(x, y) end
        end,
        joystickpressed = function(j, b)
            if love.joystickpressed then return love.joystickpressed(j, b) end
        end,
        joystickreleased = function(j, b)
            if love.joystickreleased then return love.joystickreleased(j, b) end
        end,
        joystickaxis = function(j, a, v)
            if love.joystickaxis then return love.joystickaxis(j, a, v) end
        end,
        joystickhat = function(j, h, v)
            if love.joystickhat then return love.joystickhat(j, h, v) end
        end,
        gamepadpressed = function(j, b)
            if love.gamepadpressed then return love.gamepadpressed(j, b) end
        end,
        gamepadreleased = function(j, b)
            if love.gamepadreleased then return love.gamepadreleased(j, b) end
        end,
        gamepadaxis = function(j, a, v)
            if love.gamepadaxis then return love.gamepadaxis(j, a, v) end
        end,
        joystickadded = function(j)
            if love.joystickadded then return love.joystickadded(j) end
        end,
        joystickremoved = function(j)
            if love.joystickremoved then return love.joystickremoved(j) end
        end,
        focus = function(f)
            if love.focus then return love.focus(f) end
        end,
        mousefocus = function(f)
            if love.mousefocus then return love.mousefocus(f) end
        end,
        visible = function(v)
            if love.visible then return love.visible(v) end
        end,
        quit = function()
            return
        end,
        threaderror = function(t, err)
            if love.threaderror then return love.threaderror(t, err) end
        end,
        resize = function(w, h)
            if love.resize then return love.resize(w, h) end
        end,
        filedropped = function(f)
            if love.filedropped then return love.filedropped(f) end
        end,
        directorydropped = function(dir)
            if love.directorydropped then return love.directorydropped(dir) end
        end,
        lowmemory = function()
            if love.lowmemory then love.lowmemory() end
            collectgarbage()
            collectgarbage()
        end,
        displayrotated = function(display, orient)
            if love.displayrotated then return love.displayrotated(display, orient) end
        end,
    }, {
        __index = function(self, name)
            error("Unknown event: " .. tostring(name))
        end,
    })
end

-------------------------------------------------------------------------------
-- Boot
-------------------------------------------------------------------------------

local function usage()
    print("LoveRay " .. love.loveray.version .. " (Love2D API " .. love._version .. ")")
    print("")
    print("Usage: love [options] <game directory>")
    print("")
    print("Options:")
    print("  --version           print version information and exit")
    print("  --help              print this message and exit")
    print("  --frames N          quit after N frames (headless testing)")
    print("  --screenshot FILE   save a screenshot of the last frame (with --frames)")
    print("  --console           accepted for Love2D compatibility, no effect")
    print("  --fused             accepted for Love2D compatibility, no effect")
end

function love.boot()
    -- Load the no-game screen first so love.nogame exists even for errors.
    love.nogame = love._loadResource("nogame.lua")()

    love.filesystem.init(arg[0] or "love")
    local fused = love.filesystem.isFused()

    if not fused then
        love.arg.parseOptions(arg)
    end
    local o = love.arg.options

    if o.version.set then
        print(string.format("LoveRay %s  (Love2D API %s, raylib %s, %s, Box2D %s)",
            love.loveray.version, love._version, love.loveray.raylib, love.loveray.lua, love.loveray.box2d))
        return 0
    end
    if o.help.set then
        usage()
        return 0
    end

    local gamePath = o.game.arg and o.game.arg[1]
    local hasGame = false
    local identity = ""

    if fused then
        hasGame = love.filesystem.getInfo("main.lua", "file") ~= nil
        identity = love.path.leaf(love.filesystem.getExecutablePath()):gsub("%.exe$", "")
    elseif gamePath and gamePath ~= "" then
        local full = love.path.getFull(gamePath)
        local info = love.filesystem.getRealInfo(full)
        local isArchive = info and info.type == "file" and full:lower():match("%.love$") or full:lower():match("%.zip$")
        if info and info.type == "file" and not isArchive then
            -- `love path/to/main.lua` is accepted as a convenience.
            full = full:match("^(.*)/[^/]*$") or "."
            info = love.filesystem.getRealInfo(full)
        end
        if not info or (info.type ~= "directory" and not isArchive) then
            error("Cannot open game '" .. gamePath .. "'")
        end
        love.filesystem.setSource(full)
        hasGame = love.filesystem.getInfo("main.lua", "file") ~= nil
        if not hasGame and not love.filesystem.getInfo("conf.lua", "file") then
            print("No main.lua found in '" .. full .. "'")
        end
        identity = love.path.leaf(full):gsub("%.love$", ""):gsub("%.zip$", "")
    end

    -- Default configuration, identical to Love2D 11.
    local c = {
        title = "Untitled",
        version = love._version,
        window = {
            width = 800,
            height = 600,
            x = nil,
            y = nil,
            minwidth = 1,
            minheight = 1,
            fullscreen = false,
            fullscreentype = "desktop",
            display = 1,
            vsync = 1,
            msaa = 0,
            borderless = false,
            resizable = false,
            centered = true,
            highdpi = false,
            usedpiscale = true,
            depth = nil,
            stencil = nil,
        },
        modules = {
            data = true, event = true, keyboard = true, mouse = true, timer = true,
            joystick = true, touch = true, image = true, graphics = true, audio = true,
            math = true, physics = true, sound = true, system = true, font = true,
            thread = true, window = true, video = true,
        },
        audio = { mixwithsystem = true, mic = false },
        console = false,
        identity = false,
        appendidentity = false,
        accelerometerjoystick = true,
        externalstorage = false,
        gammacorrect = false,
        -- LoveRay specific settings.
        loveray = {
            maxfps = 0,       -- frame cap when vsync is unavailable (0 = uncapped)
            hotreload = false -- restart the game automatically when main.lua changes
        },
    }

    -- Require the game's conf.lua when it exists.
    if hasGame or love.filesystem.getInfo("conf.lua", "file") then
        local confok, conferr = pcall(require, "conf")
        if not confok and conferr and not conferr:find("module 'conf' not found", 1, true) then
            error(conferr, 0)
        end
        if love.conf then
            local ok, err = pcall(love.conf, c)
            if not ok then
                error(err, 0)
            end
        end
    end

    if c.identity then
        identity = c.identity
    end
    if identity == "" then
        identity = "loveray-nogame"
    end
    love.filesystem.setIdentity(identity, c.appendidentity)

    -- Keyboard, mouse and other modules are always available: they are part of
    -- the executable. Unused `c.modules` entries only disable the Lua table.
    for name, enabled in pairs(c.modules) do
        if not enabled and love[name] and name ~= "window" and name ~= "graphics"
            and name ~= "event" and name ~= "timer" and name ~= "filesystem" then
            love[name] = nil
        end
    end

    if love.event then
        love.createhandlers()
    end

    -- Open the window.
    if c.window then
        love.window.setTitle(c.window.title or c.title)
        local ok, err = love.window.setMode(c.window.width, c.window.height, {
            fullscreen = c.window.fullscreen,
            fullscreentype = c.window.fullscreentype,
            vsync = c.window.vsync,
            msaa = c.window.msaa,
            resizable = c.window.resizable,
            borderless = c.window.borderless,
            centered = c.window.centered,
            display = c.window.display,
            minwidth = c.window.minwidth,
            minheight = c.window.minheight,
            highdpi = c.window.highdpi,
            usedpiscale = c.window.usedpiscale,
            x = c.window.x,
            y = c.window.y,
            maxfps = c.loveray and c.loveray.maxfps or 0,
        })
        if not ok then
            error("Could not open window: " .. tostring(err))
        end
        if c.window.icon then
            local iconok, icondata = pcall(love.image.newImageData, c.window.icon)
            if iconok then
                love.window.setIcon(icondata)
            end
        end
    else
        love.window._openHidden()
    end

    love._hotreload = c.loveray and c.loveray.hotreload or false

    if hasGame then
        require("main")
    else
        love.nogame()
    end
end

-------------------------------------------------------------------------------
-- Main loop (Love2D 11 semantics: love.run returns the per-frame function)
-------------------------------------------------------------------------------

local function frameLimitReached()
    local o = love.arg.options
    if not o.frames.set then
        return false
    end
    love._frameCount = (love._frameCount or 0) + 1
    local limit = tonumber(o.frames.arg[1]) or 0
    if love._frameCount >= limit then
        if o.screenshot.set and o.screenshot.arg[1] then
            love.graphics.captureScreenshot(o.screenshot.arg[1])
        end
        return true
    end
    return false
end

function love.run()
    if love.load then
        love.load(love.arg.parseGameArguments(arg), arg)
    end

    -- Do not count the time spent loading.
    if love.timer then
        love.timer.step()
    end

    local dt = 0

    return function()
        if love.event then
            love.event.pump()
            for name, a, b, c, d, e, f in love.event.poll() do
                if name == "quit" then
                    if not love.quit or not love.quit() then
                        return a or 0
                    end
                end
                love.handlers[name](a, b, c, d, e, f)
            end
        end

        if love.timer then
            dt = love.timer.step()
        end

        if love.update then
            love.update(dt)
        end

        if love.graphics and love.graphics.isActive() then
            love.graphics.origin()
            love.graphics.clear(love.graphics.getBackgroundColor())
            if love.draw then
                love.draw()
            end
            if frameLimitReached() then
                love.graphics.present()
                return 0
            end
            love.graphics.present()
        end

        if love._hotreload and love.filesystem._sourceChanged() then
            return "restart"
        end

        if love.timer then
            love.timer.sleep(0.001)
        end
    end
end

-------------------------------------------------------------------------------
-- Error screen
-------------------------------------------------------------------------------

local function error_printer(msg, layer)
    print((debug.traceback("Error: " .. tostring(msg), 1 + (layer or 1)):gsub("\n[^\n]+$", "")))
end

function love.errorhandler(msg)
    msg = tostring(msg)
    error_printer(msg, 2)

    if not love.window or not love.graphics or not love.event then
        return
    end

    if not love.graphics.isCreated() or not love.window.isOpen() then
        local success, status = pcall(love.window.setMode, 800, 600)
        if not success or not status then
            return
        end
    end

    -- Reset state.
    if love.mouse then
        love.mouse.setVisible(true)
        love.mouse.setGrabbed(false)
        love.mouse.setRelativeMode(false)
        if love.mouse.isCursorSupported() then
            love.mouse.setCursor()
        end
    end
    if love.joystick then
        for i, v in ipairs(love.joystick.getJoysticks()) do
            v:setVibration()
        end
    end
    if love.audio then
        love.audio.stop()
    end

    love.graphics.reset()
    local font = love.graphics.setNewFont(14)
    love.graphics.setColor(1, 1, 1)

    local trace = debug.traceback()

    love.graphics.origin()

    local sanitizedmsg = {}
    for char in msg:gmatch(utf8.charpattern) do
        table.insert(sanitizedmsg, char)
    end
    sanitizedmsg = table.concat(sanitizedmsg)

    local err = {}
    table.insert(err, "Error\n")
    table.insert(err, sanitizedmsg)
    if #sanitizedmsg ~= #msg then
        table.insert(err, "Invalid UTF-8 string in error message.")
    end
    table.insert(err, "\n")

    for l in trace:gmatch("(.-)\n") do
        if not l:match("boot.lua") then
            l = l:gsub("stack traceback:", "Traceback\n")
            table.insert(err, l)
        end
    end

    local p = table.concat(err, "\n")
    p = p:gsub("\t", "")
    p = p:gsub("%[string \"(.-)\"%]", "%1")
    p = p .. "\n\nPress R to restart, Escape to quit, Ctrl+C to copy the error."

    local function draw()
        if not love.graphics.isActive() then
            return
        end
        local pos = 70
        love.graphics.clear(89 / 255, 157 / 255, 220 / 255)
        love.graphics.printf(p, pos, pos, love.graphics.getWidth() - pos)
        love.graphics.present()
    end

    local fullErrorText = p
    local function copyToClipboard()
        if not love.system then
            return
        end
        love.system.setClipboardText(fullErrorText)
        p = p .. "\nCopied to clipboard!"
    end

    return function()
        love.event.pump()
        if frameLimitReached() then
            return 1
        end
        for e, a, b, c in love.event.poll() do
            if e == "quit" then
                return 1
            elseif e == "keypressed" and a == "escape" then
                return 1
            elseif e == "keypressed" and a == "r" then
                return "restart"
            elseif e == "keypressed" and a == "c" and love.keyboard.isDown("lctrl", "rctrl") then
                copyToClipboard()
            elseif e == "touchpressed" then
                local name = love.window.getTitle()
                if #name == 0 or name == "Untitled" then
                    name = "Game"
                end
                local buttons = { "OK", "Cancel", "Restart" }
                local pressed = love.window.showMessageBox("Quit " .. name .. "?", "", buttons)
                if pressed == 1 then
                    return 1
                elseif pressed == 3 then
                    return "restart"
                end
            end
        end

        -- Edit-and-reload workflow: restart as soon as the game source changes.
        if love.filesystem._sourceChanged() then
            return "restart"
        end

        draw()

        if love.timer then
            love.timer.sleep(0.1)
        end
    end
end

love.errhand = love.errorhandler

-------------------------------------------------------------------------------
-- Entry point
-------------------------------------------------------------------------------

local function deferErrhand(...)
    local errhand = love.errorhandler or love.errhand
    local handler = (errhand ~= love.errorhandler or errhand) and errhand or love.errorhandler
    return handler(...)
end

local function earlyinit()
    local result = xpcall(love.boot, error_printer)
    if not result then
        return 1
    end
    -- love.boot may have returned an exit code (--version, --help).
    return nil
end

return (function()
    local bootresult = earlyinit()
    if bootresult then
        return bootresult
    end

    -- --version / --help return early without opening a window.
    if love.arg.options.version.set or love.arg.options.help.set then
        return 0
    end

    local result, main = xpcall(love.run, deferErrhand)
    if not result then
        -- love.run itself raised; `main` is now the error-screen loop.
        if type(main) ~= "function" then
            return 1
        end
    end

    local looping = true
    local ret
    while looping do
        local ok, value = xpcall(main, deferErrhand)
        if not ok then
            -- The error handler returned a new loop (the error screen).
            if type(value) == "function" then
                main = value
            else
                return 1
            end
        elseif value ~= nil then
            ret = value
            looping = false
        end
    end

    return ret
end)()
