local W, H = 860, 520
local PADDLE_W, PADDLE_H, BALL = 14, 90, 12
local WIN_SCORE = 7

local left, right, ball, score, winner, serving

local function resetBall(direction)
    ball = {x = W / 2, y = H / 2, vx = 340 * direction, vy = love.math.random(-180, 180)}
    serving = 0.8
end

local function newGame()
    left = {x = 30, y = H / 2 - PADDLE_H / 2}
    right = {x = W - 30 - PADDLE_W, y = H / 2 - PADDLE_H / 2}
    score = {0, 0}
    winner = nil
    resetBall(love.math.random() < 0.5 and -1 or 1)
end

local function clampPaddle(p)
    p.y = math.max(0, math.min(H - PADDLE_H, p.y))
end

local function bounce(p, direction)
    local hit = (ball.y - (p.y + PADDLE_H / 2)) / (PADDLE_H / 2)
    local speed = math.min(math.sqrt(ball.vx ^ 2 + ball.vy ^ 2) * 1.06, 760)
    local angle = hit * math.rad(55)
    ball.vx = math.cos(angle) * speed * direction
    ball.vy = math.sin(angle) * speed
end

function love.load()
    love.graphics.setBackgroundColor(0.055, 0.1, 0.16)
    love.graphics.setFont(love.graphics.newFont(28))
    newGame()
end

function love.keypressed(key)
    if key == "escape" then
        love.event.quit()
    elseif key == "return" and winner then
        newGame()
    end
end

function love.update(dt)
    if winner then
        return
    end

    local lv, rv = 0, 0
    if love.keyboard.isDown("w") then lv = lv - 1 end
    if love.keyboard.isDown("s") then lv = lv + 1 end
    if love.keyboard.isDown("up") then rv = rv - 1 end
    if love.keyboard.isDown("down") then rv = rv + 1 end
    left.y = left.y + lv * 460 * dt
    right.y = right.y + rv * 460 * dt
    clampPaddle(left)
    clampPaddle(right)

    if serving > 0 then
        serving = serving - dt
        return
    end

    ball.x = ball.x + ball.vx * dt
    ball.y = ball.y + ball.vy * dt

    if ball.y < BALL / 2 then
        ball.y = BALL / 2
        ball.vy = -ball.vy
    elseif ball.y > H - BALL / 2 then
        ball.y = H - BALL / 2
        ball.vy = -ball.vy
    end

    if ball.vx < 0 and ball.x - BALL / 2 < left.x + PADDLE_W and ball.x > left.x
        and ball.y > left.y and ball.y < left.y + PADDLE_H then
        ball.x = left.x + PADDLE_W + BALL / 2
        bounce(left, 1)
    elseif ball.vx > 0 and ball.x + BALL / 2 > right.x and ball.x < right.x + PADDLE_W
        and ball.y > right.y and ball.y < right.y + PADDLE_H then
        ball.x = right.x - BALL / 2
        bounce(right, -1)
    end

    if ball.x < -BALL then
        score[2] = score[2] + 1
        resetBall(1)
    elseif ball.x > W + BALL then
        score[1] = score[1] + 1
        resetBall(-1)
    end

    if score[1] >= WIN_SCORE then
        winner = "Left player"
    elseif score[2] >= WIN_SCORE then
        winner = "Right player"
    end
end

function love.draw()
    love.graphics.setColor(1, 1, 1, 0.18)
    for y = 0, H, 28 do
        love.graphics.rectangle("fill", W / 2 - 2, y, 4, 14)
    end

    love.graphics.setColor(0.31, 0.8, 0.77)
    love.graphics.rectangle("fill", left.x, left.y, PADDLE_W, PADDLE_H)
    love.graphics.rectangle("fill", right.x, right.y, PADDLE_W, PADDLE_H)
    love.graphics.setColor(1, 0.84, 0)
    love.graphics.rectangle("fill", ball.x - BALL / 2, ball.y - BALL / 2, BALL, BALL)

    love.graphics.setColor(1, 1, 1)
    love.graphics.printf(score[1], 0, 24, W / 2 - 40, "right")
    love.graphics.printf(score[2], W / 2 + 40, 24, W / 2 - 40, "left")

    if winner then
        love.graphics.printf(winner .. " wins - press Enter", 0, H / 2 - 20, W, "center")
    elseif serving > 0 then
        love.graphics.setColor(1, 1, 1, 0.5)
        love.graphics.printf("W/S  and  Up/Down", 0, H - 60, W, "center")
    end
end
