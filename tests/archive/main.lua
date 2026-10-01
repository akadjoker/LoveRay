local failures = {}
local passed = 0

local function check(name, ok, detail)
    if ok then
        passed = passed + 1
    else
        table.insert(failures, name .. (detail and (": " .. tostring(detail)) or ""))
    end
end

local function run(args)
    local zip, mode = args[1], args[2]
    check("arguments", zip ~= nil and mode ~= nil, "needs <data.zip> <mode>")
    local dir = zip:match("^(.*)/[^/]*$") or "."

    check("fused flag", love.filesystem.isFused() == (mode == "fused"), tostring(love.filesystem.isFused()))
    local origin = love.filesystem.getRealDirectory("main.lua")
    if mode == "dir" then
        check("source is a directory", origin and origin:match("tests/archive$"), origin)
    elseif mode == "archive" then
        check("source is the .love file", origin and origin:match("archive%.love$"), origin)
    else
        check("source is the executable", origin ~= nil and love.filesystem.getSource() == origin, origin)
    end
    check("own files readable", love.filesystem.getInfo("conf.lua", "file") ~= nil)

    -- Mounting at the root and at a mount point
    check("mount", love.filesystem.mount(zip, "data") == true)
    check("missing before unmount of other", love.filesystem.getInfo("plain.txt") == nil)
    check("stored entry", love.filesystem.read("data/plain.txt") == "stored content")
    local big = love.filesystem.read("data/big.txt")
    check("deflated entry", big and #big > 100000 and big:sub(1, 7) == "line 0\n" and big:sub(-12) == "line 19999\n\n" or big:sub(-11) == "line 19999\n", big and #big)
    check("getInfo file", love.filesystem.getInfo("data/plain.txt").size == 14)
    check("getInfo type filter", love.filesystem.getInfo("data/plain.txt", "directory") == nil)
    check("getInfo directory", love.filesystem.getInfo("data/sub/deep", "directory") ~= nil)
    check("explicit directory entry", love.filesystem.getInfo("data/empty", "directory") ~= nil)
    check("mount point is a directory", love.filesystem.getInfo("data", "directory") ~= nil)
    check("modtime", (love.filesystem.getInfo("data/plain.txt").modtime or 0) > 0)
    local items = love.filesystem.getDirectoryItems("data")
    table.sort(items)
    check("directory listing", table.concat(items, ",") == "big.txt,crlf.txt,empty,inner.zip,plain.txt,sub,tile.png", table.concat(items, ","))
    check("nested listing", love.filesystem.getDirectoryItems("data/sub")[1] == "deep")
    check("mount point listed at the root", (function()
        for _, name in ipairs(love.filesystem.getDirectoryItems("")) do
            if name == "data" then return true end
        end
    end)())
    check("getRealDirectory points at the archive", love.filesystem.getRealDirectory("data/plain.txt") == zip, love.filesystem.getRealDirectory("data/plain.txt"))

    -- Code and assets from inside the archive
    local chunk = love.filesystem.load("data/sub/deep/file.lua")
    check("load", type(chunk) == "function" and chunk() == 7)
    check("require through a mount point", require("data.sub.deep.file") == 7)
    local image = love.graphics.newImage("data/tile.png")
    check("image from archive", image:getWidth() == 8 and image:getHeight() == 4)
    local imageData = love.image.newImageData("data/tile.png")
    local r = imageData:getPixel(1, 1)
    check("imagedata from archive", r == 1)

    -- File objects
    local file = love.filesystem.newFile("data/plain.txt")
    check("File open", file:open("r") and file:getSize() == 14)
    check("File read part", file:read(6) == "stored")
    check("File tell", file:tell() == 6)
    check("File seek", file:seek(0) and file:read(3) == "sto")
    check("File eof", not file:isEOF() and file:read() == "red content" and file:isEOF())
    file:close()
    local lines = {}
    for line in love.filesystem.lines("data/crlf.txt") do lines[#lines + 1] = line end
    check("lines strip CRLF", table.concat(lines, "|") == "one|two|three", table.concat(lines, "|"))

    -- Nested archive, taken from inside another archive
    check("mount nested archive", love.filesystem.mount("data/inner.zip", "nested") == true)
    check("nested archive content", love.filesystem.read("nested/inner.txt") == "from the inner archive")
    check("unmount nested", love.filesystem.unmount("data/inner.zip") == true and love.filesystem.getInfo("nested/inner.txt") == nil)

    -- Mount order
    check("mount second archive on top", love.filesystem.mount(dir .. "/data2.zip", "data") == true)
    check("later mount wins", love.filesystem.read("data/plain.txt") == "second")
    check("files only in the second", love.filesystem.read("data/only2.txt") == "only in the second archive")
    check("unmount second", love.filesystem.unmount(dir .. "/data2.zip") == true)
    check("first wins again", love.filesystem.read("data/plain.txt") == "stored content")
    check("mount appended", love.filesystem.mount(dir .. "/data2.zip", "data", true) == true)
    check("appended mount loses", love.filesystem.read("data/plain.txt") == "stored content")
    check("appended mount still adds files", love.filesystem.read("data/only2.txt") ~= nil)
    love.filesystem.unmount(dir .. "/data2.zip")

    -- Mounting from memory
    local handle = io.open(zip, "rb")
    local content = handle:read("a")
    handle:close()
    local memory = love.filesystem.newFileData(content, "memory.zip")
    check("mount FileData", love.filesystem.mount(memory, "memory.zip", "mem") == true)
    check("memory archive content", love.filesystem.read("mem/plain.txt") == "stored content")
    check("unmount by name", love.filesystem.unmount("memory.zip") == true)

    -- Errors
    local ok, err = love.filesystem.mount(dir .. "/notazip.zip", "bad")
    check("mounting a non-zip fails", ok == false and type(err) == "string", err)
    check("mounting a missing file fails", love.filesystem.mount("nope.zip", "x") == false)
    check("unmounting something never mounted", love.filesystem.unmount("never") == false)
    check("damaged archive mounts", love.filesystem.mount(dir .. "/bad.zip", "bad") == true)
    local value, message = love.filesystem.read("bad/bad.txt")
    check("checksum mismatch is reported", value == nil and tostring(message):find("Checksum", 1, true) ~= nil, message)
    love.filesystem.unmount(dir .. "/bad.zip")
    check("escaping paths stays rejected", love.filesystem.getInfo("data/../main.lua") == nil)
    love.filesystem.unmount(zip)
    check("unmounted files disappear", love.filesystem.read("data/plain.txt") == nil)
end

function love.load(args)
    local ok, err = xpcall(run, debug.traceback, args)
    if not ok then
        table.insert(failures, "runtime error: " .. tostring(err))
    end
    if #failures > 0 then
        print("FAILED checks:")
        for _, f in ipairs(failures) do
            print("  - " .. f)
        end
        love.event.quit(1)
    else
        print(string.format("All %d archive checks passed (%s)", passed, args[2] or "?"))
        love.event.quit(0)
    end
end
