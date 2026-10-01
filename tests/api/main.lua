-- API conformance checks. Runs headless with:  love tests/api --frames 5
-- Exits with code 1 when any check fails.

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

local function expectError(name, fn, ...)
    local ok = pcall(fn, ...)
    check(name, not ok, "expected an error")
end

local function run()
    -- love
    local major, minor = love.getVersion()
    check("getVersion", major == 11 and minor == 5)
    check("isVersionCompatible", love.isVersionCompatible(11, 4) and not love.isVersionCompatible(12, 0))
    check("love.loveray info", type(love.loveray.version) == "string" and type(love.loveray.raylib) == "string")

    -- filesystem
    check("getSource", love.filesystem.getSource():match("tests/api$") ~= nil, love.filesystem.getSource())
    check("getIdentity", love.filesystem.getIdentity() == "loveray-api-tests", love.filesystem.getIdentity())
    local info = love.filesystem.getInfo("main.lua")
    check("getInfo file", info and info.type == "file" and info.size > 0)
    check("getInfo filter", love.filesystem.getInfo("main.lua", "directory") == nil)
    check("getInfo missing", love.filesystem.getInfo("nope.lua") == nil)
    check("getInfo dir", love.filesystem.getInfo("data", "directory") ~= nil)
    local contents, size = love.filesystem.read("data/hello.txt")
    check("read", contents == "hello\nworld\n" and size == 12, contents)
    check("read limit", love.filesystem.read("data/hello.txt", 5) == "hello")
    local items = love.filesystem.getDirectoryItems("data")
    check("getDirectoryItems", #items == 3 and items[1] == "hello.txt" and items[2] == "sub" and items[3] == "tile.png", table.concat(items, ","))
    local lines = {}
    for line in love.filesystem.lines("data/hello.txt") do
        lines[#lines + 1] = line
    end
    check("lines", #lines == 2 and lines[2] == "world")
    check("write", love.filesystem.write("out/test.txt", "abc") == true)
    check("append", love.filesystem.append("out/test.txt", "def") == true)
    check("read written", love.filesystem.read("out/test.txt") == "abcdef")
    check("getRealDirectory save", love.filesystem.getRealDirectory("out/test.txt") == love.filesystem.getSaveDirectory())
    check("remove", love.filesystem.remove("out/test.txt") == true)
    check("remove gone", love.filesystem.getInfo("out/test.txt") == nil)
    check("escape rejected", love.filesystem.getInfo("../main.lua") == nil)
    local chunk = love.filesystem.load("data/sub/module.lua")
    check("load", type(chunk) == "function" and chunk().value == 42)
    local mod = require("data.sub.module")
    check("require from game dir", mod.value == 42)
    local f = love.filesystem.newFile("out/file.txt")
    check("File open w", f:open("w"))
    check("File write", f:write("line1\nline2\n"))
    f:close()
    check("File open r", f:open("r") and f:getSize() == 12)
    check("File read", f:read(5) == "line1")
    check("File tell", f:tell() == 5)
    f:close()
    local fd = love.filesystem.newFileData("payload", "x.bin")
    check("FileData", fd:getSize() == 7 and fd:getExtension() == "bin" and fd:getString() == "payload")
    check("isFused", love.filesystem.isFused() == false)

    -- window
    local w, h, flags = love.window.getMode()
    check("window mode", w == 640 and h == 480, w .. "x" .. h)
    check("window flags", flags.vsync == 0 and flags.resizable == true)
    check("window title", love.window.getTitle() == "LoveRay API tests")
    check("window open", love.window.isOpen())
    check("desktop dimensions", love.window.getDesktopDimensions() > 0)
    check("dpi scale", love.window.getDPIScale() > 0)

    -- graphics state
    love.graphics.setColor(0.5, 0.25, 1)
    local r, g, b, a = love.graphics.getColor()
    check("setColor default alpha", near(r, 0.5, 0.01) and near(g, 0.25, 0.01) and near(b, 1) and near(a, 1))
    love.graphics.setColor({0.1, 0.2, 0.3, 0.4})
    r, g, b, a = love.graphics.getColor()
    check("setColor table", near(r, 0.1, 0.01) and near(a, 0.4, 0.01))
    love.graphics.setBackgroundColor(0, 0, 0)
    check("blend default", love.graphics.getBlendMode() == "alpha")
    love.graphics.setBlendMode("add")
    check("blend add", love.graphics.getBlendMode() == "add")
    expectError("multiply needs premultiplied", love.graphics.setBlendMode, "multiply")
    love.graphics.setBlendMode("alpha")
    love.graphics.setLineWidth(3)
    check("line width", love.graphics.getLineWidth() == 3)
    love.graphics.setScissor(10, 20, 30, 40)
    local sx, sy, sw, sh = love.graphics.getScissor()
    check("scissor", sx == 10 and sy == 20 and sw == 30 and sh == 40)
    love.graphics.intersectScissor(20, 20, 100, 10)
    sx, sy, sw, sh = love.graphics.getScissor()
    check("intersectScissor", sx == 20 and sw == 20 and sh == 10, sw)
    love.graphics.setScissor()
    check("scissor cleared", love.graphics.getScissor() == nil)
    check("dimensions", love.graphics.getWidth() == 640 and love.graphics.getHeight() == 480)
    local name = love.graphics.getRendererInfo()
    check("renderer", name == "OpenGL")
    check("supported table", type(love.graphics.getSupported().canvas) == "boolean")

    -- fonts
    local font = love.graphics.getFont()
    check("default font height", font:getHeight() == 12, font:getHeight())
    local big = love.graphics.newFont(24)
    check("font size", big:getHeight() == 24)
    check("font width grows", big:getWidth("Hello") > font:getWidth("Hello"))
    check("font width empty", font:getWidth("") == 0)
    local wrapW, wrapped = font:getWrap("one two three four five six seven eight nine ten", 60)
    check("getWrap", #wrapped > 1 and wrapW <= 60, #wrapped .. " lines, " .. wrapW)
    check("hasGlyphs", font:hasGlyphs("abc", "ção") and not font:hasGlyphs("\u{4E2D}"))
    love.graphics.setFont(big)
    check("setFont", love.graphics.getFont() == big)
    love.graphics.setFont(font)
    local newf = love.graphics.setNewFont(16)
    check("setNewFont", love.graphics.getFont() == newf and newf:getHeight() == 16)
    love.graphics.setFont(font)
    check("font type", font:type() == "Font" and font:typeOf("Font") and font:typeOf("Object"))

    -- images / quads / canvas
    local image = love.graphics.newImage("data/tile.png")
    check("image size", image:getWidth() == 16 and image:getHeight() == 8, image:getWidth() .. "x" .. image:getHeight())
    image:setFilter("nearest", "nearest")
    local fmin, fmag = image:getFilter()
    check("image filter", fmin == "nearest" and fmag == "nearest")
    local quad = love.graphics.newQuad(8, 0, 8, 8, image)
    local qx, qy, qw, qh = quad:getViewport()
    check("quad viewport", qx == 8 and qw == 8 and qh == 8)
    local tw, th = quad:getTextureDimensions()
    check("quad texture dims", tw == 16 and th == 8)
    local canvas = love.graphics.newCanvas(32, 16)
    check("canvas size", canvas:getWidth() == 32 and canvas:getHeight() == 16)
    love.graphics.setCanvas(canvas)
    check("getCanvas", love.graphics.getCanvas() == canvas)
    love.graphics.clear(1, 0, 0, 1)
    love.graphics.setColor(0, 0, 1, 1)
    love.graphics.rectangle("fill", 16, 0, 16, 16)
    love.graphics.setCanvas()
    check("getCanvas nil", love.graphics.getCanvas() == nil)
    local pixels = canvas:newImageData()
    local pr, pg, pb = pixels:getPixel(2, 2)
    local qr, qg, qb = pixels:getPixel(20, 2)
    check("canvas pixels left red", near(pr, 1, 0.02) and near(pg, 0, 0.02) and near(pb, 0, 0.02), pr .. "," .. pg .. "," .. pb)
    check("canvas pixels right blue", near(qr, 0, 0.02) and near(qb, 1, 0.02), qr .. "," .. qg .. "," .. qb)
    local batch = love.graphics.newSpriteBatch(image, 10)
    local id = batch:add(quad, 0, 0)
    batch:add(5, 5)
    check("spritebatch", id == 1 and batch:getCount() == 2)
    batch:set(1, 1, 1)
    batch:clear()
    check("spritebatch clear", batch:getCount() == 0)
    local text = love.graphics.newText(font, "hello")
    check("text width", text:getWidth() == font:getWidth("hello") and text:getHeight() == font:getHeight())
    text:setf("a b c d e f g h i j k l m n o p", 30, "left")
    check("text wrapped height", text:getHeight() > font:getHeight())
    love.graphics.setColor(1, 1, 1)

    -- image data
    local data = love.image.newImageData(4, 4)
    data:setPixel(1, 1, 0.2, 0.4, 0.6, 1)
    local dr, dg, db, da = data:getPixel(1, 1)
    check("imagedata pixel", near(dr, 0.2, 0.01) and near(dg, 0.4, 0.01) and near(db, 0.6, 0.01) and da == 1)
    data:mapPixel(function(x, y, r, g, b, a) return 1, 1, 1, 1 end)
    check("mapPixel", select(1, data:getPixel(3, 3)) == 1)
    local png = data:encode("png")
    check("encode", png:getSize() > 0)
    local loaded = love.image.newImageData("data/tile.png")
    check("imagedata load", loaded:getWidth() == 16)
    local fromData = love.graphics.newImage(loaded)
    check("newImage from ImageData", fromData:getWidth() == 16)

    -- transforms
    local t = love.math.newTransform(10, 20, 0, 2, 2)
    local px, py = t:transformPoint(1, 1)
    check("transform point", px == 12 and py == 22, px .. "," .. py)
    local ix, iy = t:inverseTransformPoint(12, 22)
    check("inverse point", near(ix, 1) and near(iy, 1))
    local t2 = love.math.newTransform():translate(5, 0):rotate(math.pi / 2)
    local rx, ry = t2:transformPoint(1, 0)
    check("translate then rotate", near(rx, 5) and near(ry, 1), rx .. "," .. ry)
    local m = {t:getMatrix()}
    check("getMatrix", #m == 16 and m[4] == 10 and m[8] == 20 and m[1] == 2)
    local combined = t * t2
    check("transform mul", combined:type() == "Transform")
    love.graphics.push()
    love.graphics.translate(100, 50)
    love.graphics.scale(2)
    local gx, gy = love.graphics.transformPoint(1, 1)
    check("graphics transformPoint", near(gx, 102) and near(gy, 52), gx .. "," .. gy)
    local ux, uy = love.graphics.inverseTransformPoint(102, 52)
    check("graphics inverseTransformPoint", near(ux, 1) and near(uy, 1))
    love.graphics.pop()
    expectError("pop without push", love.graphics.pop)

    -- math
    love.math.setRandomSeed(1234)
    local a1 = love.math.random(1, 100)
    love.math.setRandomSeed(1234)
    check("seeded random", love.math.random(1, 100) == a1)
    local ok = true
    for _ = 1, 1000 do
        local v = love.math.random(3, 7)
        if v < 3 or v > 7 or v ~= math.floor(v) then ok = false end
        local f = love.math.random()
        if f < 0 or f >= 1 then ok = false end
    end
    check("random ranges", ok)
    local rng = love.math.newRandomGenerator(99)
    local s1 = rng:random()
    local state = rng:getState()
    local s2 = rng:random()
    rng:setState(state)
    check("rng state", rng:random() == s2 and s1 ~= s2)
    local n = love.math.noise(1.5, 2.5)
    check("noise range", n >= 0 and n <= 1 and love.math.noise(1.5, 2.5) == n)
    check("noise 1-4 args", love.math.noise(0.3) and love.math.noise(0.1, 0.2, 0.3) and love.math.noise(0.1, 0.2, 0.3, 0.4))
    local cr, cg, cb, ca = love.math.colorFromBytes(255, 128, 0, 255)
    check("colorFromBytes", cr == 1 and near(cg, 128 / 255) and cb == 0 and ca == 1)
    check("colorToBytes", select(2, love.math.colorToBytes(1, 0.5, 0)) == 128)
    check("isConvex", love.math.isConvex(0, 0, 10, 0, 10, 10, 0, 10) and not love.math.isConvex(0, 0, 10, 0, 5, 5, 10, 10, 0, 10))
    local tris = love.math.triangulate(0, 0, 10, 0, 5, 5, 10, 10, 0, 10)
    check("triangulate", #tris == 3 and #tris[1] == 6, #tris)
    check("gamma", near(love.math.gammaToLinear(1), 1) and love.math.linearToGamma(0.5) > 0.5)
    local curve = love.math.newBezierCurve(0, 0, 50, 100, 100, 0)
    local bx, by = curve:evaluate(0.5)
    check("bezier evaluate", near(bx, 50) and near(by, 50), bx .. "," .. by)
    check("bezier render", #curve:render(3) > 6)
    check("bezier degree", curve:getDegree() == 2)

    -- keyboard / mouse / joystick
    check("keyboard isDown", love.keyboard.isDown("a", "space") == false)
    check("keyboard unknown key", love.keyboard.isDown("notakey") == false)
    love.keyboard.setKeyRepeat(true)
    check("key repeat", love.keyboard.hasKeyRepeat())
    check("mouse isDown", love.mouse.isDown(1, 2, 3) == false)
    check("mouse cursor", love.mouse.getSystemCursor("hand"):getType() == "hand")
    check("mouse visible", love.mouse.isVisible())
    check("joysticks", #love.joystick.getJoysticks() == love.joystick.getJoystickCount())

    -- timer / system / event
    check("timer getTime", love.timer.getTime() > 0)
    check("timer step", love.timer.step() >= 0)
    check("system os", love.system.getOS() == "Linux" or love.system.getOS() == "Windows" or love.system.getOS() == "OS X")
    check("processor count", love.system.getProcessorCount() >= 1)
    love.event.push("custom", 1, "two", true)
    local got = false
    for name, p1, p2, p3 in love.event.poll() do
        if name == "custom" then
            got = p1 == 1 and p2 == "two" and p3 == true
        end
    end
    check("event push/poll", got)

    -- audio (no device in CI, so only API shape)
    check("audio volume", love.audio.getVolume() == 1)
    check("active sources", love.audio.getActiveSourceCount() == 0)
end

local ok, err = xpcall(run, debug.traceback)
if not ok then
    table.insert(failures, "runtime error: " .. tostring(err))
end

function love.draw()
    love.graphics.print("API tests: " .. passed .. " passed, " .. #failures .. " failed", 10, 10)
end

function love.update()
    if #failures > 0 then
        print("FAILED checks:")
        for _, f in ipairs(failures) do
            print("  - " .. f)
        end
        love.event.quit(1)
    else
        print(string.format("All %d API checks passed", passed))
        love.event.quit(0)
    end
end
