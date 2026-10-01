-- Box2D through the Love2D physics API: bodies, shapes, joints and contacts.

local world
local objects = {}
local ground, car, wheels = nil, nil, {}
local mouseJoint
local contacts = 0
local debugDraw = false

local function addBox(x, y, w, h, kind)
    local body = love.physics.newBody(world, x, y, kind or "dynamic")
    local shape = love.physics.newRectangleShape(w, h)
    local fixture = love.physics.newFixture(body, shape, 1)
    fixture:setFriction(0.6)
    fixture:setRestitution(0.1)
    objects[#objects + 1] = { body = body, shape = shape, fixture = fixture, color = { 0.9, 0.5, 0.2 } }
    return body
end

local function addBall(x, y, r)
    local body = love.physics.newBody(world, x, y, "dynamic")
    local shape = love.physics.newCircleShape(r)
    local fixture = love.physics.newFixture(body, shape, 1)
    fixture:setRestitution(0.6)
    objects[#objects + 1] = { body = body, shape = shape, fixture = fixture, color = { 0.3, 0.7, 1 } }
    return body
end

function love.load()
    love.graphics.setBackgroundColor(0.1, 0.12, 0.15)
    love.physics.setMeter(32)
    world = love.physics.newWorld(0, 9.81 * 32, true)
    world:setCallbacks(function(a, b, contact)
        contacts = contacts + 1
    end)

    ground = love.physics.newBody(world, 0, 0, "static")
    love.physics.newFixture(ground, love.physics.newEdgeShape(0, 560, 800, 560))
    love.physics.newFixture(ground, love.physics.newEdgeShape(0, 0, 0, 600))
    love.physics.newFixture(ground, love.physics.newEdgeShape(800, 0, 800, 600))
    love.physics.newFixture(ground, love.physics.newChainShape(false, 420, 560, 520, 500, 620, 480, 720, 560))

    for i = 1, 6 do
        addBox(100 + (i % 2) * 10, 500 - i * 42, 60, 40)
    end
    for i = 1, 5 do
        addBall(300 + i * 30, 100 + i * 20, 12 + i * 2)
    end

    -- Pendulum: a ball on a rope hanging from a static anchor.
    local anchor = love.physics.newBody(world, 650, 120, "static")
    local bob = addBall(760, 120, 16)
    love.physics.newDistanceJoint(anchor, bob, 650, 120, 760, 120)

    -- A small car with two wheel joints.
    car = addBox(220, 300, 90, 24)
    for i, offset in ipairs({ -32, 32 }) do
        local wheel = addBall(220 + offset, 320, 12)
        local joint = love.physics.newWheelJoint(car, wheel, 220 + offset, 320, 0, 1)
        joint:setMotorEnabled(true)
        joint:setMaxMotorTorque(400)
        joint:setSpringFrequency(4)
        wheels[i] = joint
    end
end

function love.update(dt)
    local speed = 0
    if love.keyboard.isDown("left") then speed = 12 end
    if love.keyboard.isDown("right") then speed = -12 end
    for _, joint in ipairs(wheels) do
        joint:setMotorSpeed(speed)
    end

    if mouseJoint then
        mouseJoint:setTarget(love.mouse.getPosition())
    end

    world:update(math.min(dt, 1 / 30))
end

function love.mousepressed(x, y, button)
    if button == 1 then
        world:queryBoundingBox(x - 1, y - 1, x + 1, y + 1, function(fixture)
            local body = fixture:getBody()
            if body:getType() == "dynamic" and fixture:testPoint(x, y) then
                mouseJoint = love.physics.newMouseJoint(body, x, y)
                return false
            end
            return true
        end)
    elseif button == 2 then
        addBall(x, y, 10 + love.math.random() * 10)
    end
end

function love.mousereleased(x, y, button)
    if button == 1 and mouseJoint then
        mouseJoint:destroy()
        mouseJoint = nil
    end
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    elseif key == "d" then
        debugDraw = not debugDraw
    elseif key == "space" then
        addBox(love.math.random(100, 700), 50, 30 + love.math.random(40), 20 + love.math.random(30))
    end
end

function love.draw()
    for _, o in ipairs(objects) do
        love.graphics.setColor(o.color)
        if o.shape:getType() == "circle" then
            local x, y = o.body:getWorldPoint(o.shape:getPoint())
            love.graphics.circle("fill", x, y, o.shape:getRadius())
            love.graphics.setColor(1, 1, 1, 0.6)
            local a = o.body:getAngle()
            love.graphics.line(x, y, x + math.cos(a) * o.shape:getRadius(), y + math.sin(a) * o.shape:getRadius())
        else
            love.graphics.polygon("fill", o.body:getWorldPoints(o.shape:getPoints()))
        end
    end

    love.graphics.setColor(0.8, 0.8, 0.8)
    love.graphics.setLineWidth(2)
    for _, fixture in ipairs(ground:getFixtures()) do
        local shape = fixture:getShape()
        love.graphics.line(ground:getWorldPoints(shape:getPoints()))
    end
    love.graphics.setLineWidth(1)

    for _, joint in ipairs(world:getJoints()) do
        local x1, y1, x2, y2 = joint:getAnchors()
        love.graphics.setColor(1, 1, 0.3)
        love.graphics.line(x1, y1, x2, y2)
    end

    if debugDraw then
        love.graphics.setColor(1, 1, 1)
        world:draw()
    end

    love.graphics.setColor(1, 1, 1)
    love.graphics.print(string.format("bodies: %d  contacts so far: %d  fps: %d", world:getBodyCount(), contacts, love.timer.getFPS()), 10, 10)
    love.graphics.print("left/right drive the car, drag with the mouse, right click adds balls, space adds boxes, D toggles debug draw", 10, 28)
end
