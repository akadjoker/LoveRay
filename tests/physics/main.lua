-- love.physics checks. Runs headless with:  love tests/physics --frames 5

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
    return math.abs(a - b) <= (eps or 1e-3)
end

local function run()
    love.physics.setMeter(30)
    check("getMeter", love.physics.getMeter() == 30)

    local world = love.physics.newWorld(0, 300, true)
    local gx, gy = world:getGravity()
    check("gravity", gx == 0 and near(gy, 300))
    check("world type", world:type() == "World")
    check("world empty", world:getBodyCount() == 0)

    local ground = love.physics.newBody(world, 0, 0, "static")
    local groundShape = love.physics.newEdgeShape(0, 400, 800, 400)
    local groundFixture = love.physics.newFixture(ground, groundShape)
    check("ground type", ground:getType() == "static")
    check("edge points", select(2, groundShape:getPoints()) == 400)

    local box = love.physics.newBody(world, 100, 100, "dynamic")
    local boxShape = love.physics.newRectangleShape(40, 20)
    local boxFixture = love.physics.newFixture(box, boxShape, 2)
    boxFixture:setFriction(0.5)
    boxFixture:setRestitution(0.25)
    check("body position", select(1, box:getPosition()) == 100 and box:getY() == 100)
    check("fixture density", boxFixture:getDensity() == 2)
    check("fixture friction", near(boxFixture:getFriction(), 0.5))
    check("fixture restitution", near(boxFixture:getRestitution(), 0.25))
    check("mass", near(box:getMass(), 40 / 30 * 20 / 30 * 2), box:getMass())
    local points = { boxShape:getPoints() }
    check("rect points", #points == 8 and near(math.abs(points[1]), 20) and near(math.abs(points[2]), 10))
    local wx, wy = box:getWorldPoint(20, 10)
    check("world point", near(wx, 120) and near(wy, 110))
    local lx, ly = box:getLocalPoint(120, 110)
    check("local point", near(lx, 20) and near(ly, 10))
    check("fixture getShape", boxFixture:getShape():getType() == "polygon")
    check("fixture getBody", boxFixture:getBody() == box)
    check("body identity", world:getBodies()[1] == box)
    check("fixture identity", box:getFixtures()[1] == boxFixture)
    check("shape identity", boxFixture:getShape() == boxFixture:getShape())

    box:setUserData({ name = "box" })
    check("body userdata", box:getUserData().name == "box")
    boxFixture:setUserData("f")
    check("fixture userdata", boxFixture:getUserData() == "f")

    local ball = love.physics.newBody(world, 300, 50, "dynamic")
    local ballShape = love.physics.newCircleShape(15)
    love.physics.newFixture(ball, ballShape, 1)
    check("circle radius", near(ballShape:getRadius(), 15))
    local cx, cy = ballShape:getPoint()
    check("circle point", cx == 0 and cy == 0)

    local poly = love.physics.newPolygonShape(0, 0, 30, 0, 30, 30, 0, 30)
    check("polygon validate", poly:validate() and #{ poly:getPoints() } == 8)
    local chain = love.physics.newChainShape(false, 0, 0, 10, 0, 20, 10)
    check("chain vertex count", chain:getVertexCount() == 3 and chain:getChildCount() == 2)

    local begins, ends = 0, 0
    world:setCallbacks(function(a, b, c)
        begins = begins + 1
        check("contact fixtures", a:typeOf("Fixture") and b:typeOf("Fixture"))
        check("contact touching", c:isTouching())
        local nx, ny = c:getNormal()
        check("contact normal", near(nx * nx + ny * ny, 1))
    end, function(a, b, c)
        ends = ends + 1
    end)

    local y0 = box:getY()
    for _ = 1, 400 do
        world:update(1 / 60)
    end
    check("gravity moved box down", box:getY() > y0 + 100, box:getY())
    check("box rests on ground", box:getY() < 400 and box:getY() > 380, box:getY())
    check("beginContact called", begins >= 2, begins)
    check("isTouching", box:isTouching(ground))
    check("contacts list", #world:getContacts() >= 1)
    check("body touching ground", #box:getContacts() >= 1)

    box:setLinearVelocity(50, 0)
    local vx = box:getLinearVelocity()
    check("linear velocity", near(vx, 50))
    box:applyLinearImpulse(0, -200)
    world:update(1 / 60)
    check("impulse moves", select(2, box:getLinearVelocity()) < 0)
    for _ = 1, 300 do
        world:update(1 / 60)
    end

    local a = love.physics.newBody(world, 500, 100, "dynamic")
    love.physics.newFixture(a, love.physics.newCircleShape(10))
    local b = love.physics.newBody(world, 560, 100, "dynamic")
    love.physics.newFixture(b, love.physics.newCircleShape(10))
    local dj = love.physics.newDistanceJoint(a, b, 500, 100, 560, 100)
    check("distance joint type", dj:getType() == "distance" and dj:type() == "DistanceJoint" and dj:typeOf("Joint"))
    check("distance length", near(dj:getLength(), 60))
    local ba, bb = dj:getBodies()
    check("joint bodies", ba == a and bb == b)
    local ax1, ay1, ax2, ay2 = dj:getAnchors()
    check("joint anchors", near(ax1, 500) and near(ay1, 100) and near(ax2, 560) and near(ay2, 100))
    check("joint in world", #world:getJoints() == 1 and world:getJoints()[1] == dj)
    check("body joints", a:getJoints()[1] == dj)

    local rj = love.physics.newRevoluteJoint(ground, a, 500, 100)
    rj:setMotorEnabled(true)
    rj:setMotorSpeed(2)
    rj:setMaxMotorTorque(100)
    rj:setLimits(-1, 1)
    rj:setLimitsEnabled(true)
    check("revolute motor", rj:isMotorEnabled() and rj:getMotorSpeed() == 2)
    local lo, hi = rj:getLimits()
    check("revolute limits", near(lo, -1) and near(hi, 1) and rj:areLimitsEnabled())

    local wj = love.physics.newWheelJoint(a, b, 560, 100, 0, 1)
    wj:setSpringFrequency(3)
    check("wheel axis", select(2, wj:getAxis()) == 1)
    check("wheel frequency", near(wj:getSpringFrequency(), 3, 0.05), wj:getSpringFrequency())

    local mj = love.physics.newMouseJoint(b, 560, 100)
    mj:setTarget(600, 120)
    local tx, ty = mj:getTarget()
    check("mouse target", near(tx, 600) and near(ty, 120))
    check("mouse joint type", mj:getType() == "mouse")

    local rope = love.physics.newRopeJoint(a, b, 500, 100, 560, 100, 80)
    check("rope joint", rope:getType() == "rope" and near(rope:getMaxLength(), 80))

    local weld = love.physics.newWeldJoint(a, b, 530, 100)
    check("weld joint", weld:getType() == "weld")
    local motor = love.physics.newMotorJoint(a, b)
    check("motor joint", motor:getType() == "motor" and near(motor:getCorrectionFactor(), 0.3))
    local friction = love.physics.newFrictionJoint(a, b, 530, 100)
    friction:setMaxForce(10)
    check("friction joint", friction:getType() == "friction" and near(friction:getMaxForce(), 10))
    local prismatic = love.physics.newPrismaticJoint(a, b, 530, 100, 1, 0)
    check("prismatic joint", prismatic:getType() == "prismatic" and select(1, prismatic:getAxis()) == 1)
    local gear = love.physics.newGearJoint(rj, prismatic, 2)
    check("gear joint", gear:getType() == "gear" and gear:getRatio() == 2)
    local pulley = love.physics.newPulleyJoint(a, b, 500, 0, 560, 0, 500, 100, 560, 100, 1)
    check("pulley joint", pulley:getType() == "pulley" and near(pulley:getRatio(), 1))
    check("joint count", world:getJointCount() == 11, world:getJointCount())

    for _ = 1, 30 do
        world:update(1 / 60)
    end

    local found = {}
    world:queryBoundingBox(0, 0, 800, 600, function(fixture)
        found[#found + 1] = fixture
        return true
    end)
    check("query finds fixtures", #found >= 5, #found)
    local hits = 0
    world:rayCast(0, 395, 800, 395, function(fixture, x, y, xn, yn, fraction)
        hits = hits + 1
        return 1
    end)
    check("raycast hits", hits >= 1, hits)
    check("testPoint", boxFixture:testPoint(box:getPosition()))
    local d = love.physics.getDistance(boxFixture, groundFixture)
    check("getDistance", d >= 0 and d < 5, d)

    gear:destroy()
    check("joint destroyed", gear:isDestroyed())
    check("destroyed joint errors", not pcall(function() return gear:getRatio() end))
    local count = world:getBodyCount()
    ball:destroy()
    check("body destroyed", ball:isDestroyed() and world:getBodyCount() == count - 1)
    check("destroyed body errors", not pcall(function() return ball:getX() end))

    local victim = love.physics.newBody(world, 100, 300, "dynamic")
    love.physics.newFixture(victim, love.physics.newCircleShape(10))
    world:setCallbacks(function(fa, fb)
        if fa:getBody() == victim or fb:getBody() == victim then
            victim:destroy()
        end
    end)
    for _ = 1, 120 do
        world:update(1 / 60)
    end
    check("deferred destroy", victim:isDestroyed())

    world:setCallbacks(function() error("boom") end)
    local w2 = love.physics.newBody(world, 100, 300, "dynamic")
    love.physics.newFixture(w2, love.physics.newCircleShape(10))
    local okStep, err = pcall(function()
        for _ = 1, 120 do
            world:update(1 / 60)
        end
    end)
    check("callback error propagates", not okStep and tostring(err):find("boom") ~= nil, err)
    world:setCallbacks()

    world:destroy()
    check("world destroyed", world:isDestroyed())
    check("destroyed world errors", not pcall(function() return world:getBodyCount() end))
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
        print(string.format("All %d physics checks passed", passed))
        love.event.quit(0)
    end
end
