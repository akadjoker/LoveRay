-- Every Love2D callback prints to a scrolling log.

local log = {}
local text = ""

local function add(line)
    table.insert(log, 1, line)
    if #log > 24 then
        table.remove(log)
    end
end

function love.load()
    love.graphics.setBackgroundColor(0.1, 0.1, 0.1)
    love.keyboard.setKeyRepeat(true)
    add("ready: press keys, move the mouse, type text, resize the window")
end

function love.keypressed(key, scancode, isrepeat)
    add(("keypressed %s (%s)%s"):format(key, scancode, isrepeat and " repeat" or ""))
    if key == "escape" then
        love.event.quit()
    elseif key == "backspace" then
        text = text:sub(1, -2)
    end
end

function love.keyreleased(key)
    add("keyreleased " .. key)
end

function love.textinput(t)
    text = text .. t
end

function love.mousepressed(x, y, button, istouch, presses)
    add(("mousepressed %d,%d button %d presses %d"):format(x, y, button, presses))
end

function love.mousereleased(x, y, button)
    add(("mousereleased %d,%d button %d"):format(x, y, button))
end

function love.wheelmoved(x, y)
    add(("wheelmoved %d,%d"):format(x, y))
end

function love.resize(w, h)
    add(("resize %dx%d"):format(w, h))
end

function love.focus(f)
    add("focus " .. tostring(f))
end

function love.gamepadpressed(joystick, button)
    add("gamepadpressed " .. joystick:getName() .. " " .. button)
end

function love.joystickadded(joystick)
    add("joystickadded " .. joystick:getName())
end

function love.draw()
    love.graphics.setColor(1, 1, 1)
    love.graphics.print("typed: " .. text .. "_", 10, 10)
    local mx, my = love.mouse.getPosition()
    love.graphics.print(("mouse %d,%d  down: %s %s %s"):format(mx, my,
        tostring(love.mouse.isDown(1)), tostring(love.mouse.isDown(2)), tostring(love.mouse.isDown(3))), 10, 30)
    love.graphics.print("shift held: " .. tostring(love.keyboard.isDown("lshift", "rshift")), 10, 50)

    for i, line in ipairs(log) do
        love.graphics.setColor(1, 1, 1, 1 - (i - 1) / #log * 0.8)
        love.graphics.print(line, 10, 80 + (i - 1) * 18)
    end

    love.graphics.setColor(1, 0.8, 0.2)
    love.graphics.circle("line", mx, my, 12)
end
