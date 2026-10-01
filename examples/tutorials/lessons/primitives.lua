local lesson =
{
    title = "Drawing primitives",
    apis = "rectangle  circle  ellipse  arc  line  polygon  points  setLineWidth  setLineStyle",
    help = "Every shape comes in a fill and a line mode. The last row animates with transforms.",
}

local function cell(col, row)
    return 30 + col * 190, 70 + row * 150
end

function lesson.draw()
    love.graphics.setLineWidth(2)
    local t = love.timer.getTime()

    local x, y = cell(0, 0)
    love.graphics.setColor(0.31, 0.8, 0.77)
    love.graphics.rectangle("fill", x, y, 70, 50)
    love.graphics.rectangle("line", x + 80, y, 70, 50)
    love.graphics.rectangle("fill", x, y + 60, 150, 30, 15, 15)

    x, y = cell(1, 0)
    love.graphics.setColor(1, 0.55, 0.35)
    love.graphics.circle("fill", x + 30, y + 30, 28)
    love.graphics.circle("line", x + 100, y + 30, 28, 6)
    love.graphics.ellipse("fill", x + 60, y + 80, 60, 16)

    x, y = cell(2, 0)
    love.graphics.setColor(0.77, 0.54, 1)
    love.graphics.arc("fill", x + 40, y + 40, 38, 0, math.pi * 1.5)
    love.graphics.arc("line", "open", x + 115, y + 40, 38, math.pi, math.pi * 2)

    x, y = cell(3, 0)
    love.graphics.setColor(1, 0.84, 0.3)
    love.graphics.polygon("fill", x + 10, y + 80, x + 50, y, x + 90, y + 80)
    love.graphics.polygon("line", x + 100, y + 80, x + 100, y + 10, x + 150, y + 40, x + 140, y + 90)

    x, y = cell(0, 1)
    love.graphics.setColor(0.61, 0.78, 0.95)
    love.graphics.setLineStyle("smooth")
    for i = 0, 5 do
        love.graphics.setLineWidth(1 + i)
        love.graphics.line(x, y + i * 16, x + 150, y + i * 16 + 10)
    end
    love.graphics.setLineWidth(2)

    x, y = cell(1, 1)
    love.graphics.setColor(1, 1, 1)
    love.graphics.setPointSize(4)
    local points = {}
    for i = 0, 40 do
        points[#points + 1] = x + i * 4
        points[#points + 1] = y + 50 + math.sin(i / 4 + t * 3) * 30
    end
    love.graphics.points(points)
    love.graphics.setLineStyle("rough")
    love.graphics.line(points)
    love.graphics.setLineStyle("smooth")

    x, y = cell(2, 1)
    local curve = love.math.newBezierCurve(x, y + 80, x + 40, y - 20, x + 110, y + 130, x + 160, y + 20)
    love.graphics.setColor(1, 0.42, 0.42)
    love.graphics.line(curve:render())
    love.graphics.setColor(1, 1, 1, 0.4)
    for i = 1, curve:getControlPointCount() do
        local px, py = curve:getControlPoint(i)
        love.graphics.circle("line", px, py, 4)
    end

    x, y = cell(3, 1)
    love.graphics.setColor(0.31, 0.8, 0.77)
    love.graphics.setBlendMode("add")
    love.graphics.setColor(1, 0.2, 0.2, 0.7)
    love.graphics.circle("fill", x + 40, y + 40, 38)
    love.graphics.setColor(0.2, 1, 0.2, 0.7)
    love.graphics.circle("fill", x + 80, y + 40, 38)
    love.graphics.setColor(0.2, 0.2, 1, 0.7)
    love.graphics.circle("fill", x + 60, y + 75, 38)
    love.graphics.setBlendMode("alpha")

    for i = 0, 3 do
        love.graphics.push()
        love.graphics.translate(120 + i * 190, 470)
        love.graphics.rotate(t * (i + 1) * 0.6)
        love.graphics.scale(1 + math.sin(t * 2 + i) * 0.3)
        love.graphics.setColor(0.31 + i * 0.2, 0.8 - i * 0.15, 0.77)
        love.graphics.rectangle(i % 2 == 0 and "fill" or "line", -30, -30, 60, 60)
        love.graphics.pop()
    end
    love.graphics.setLineWidth(1)
end

return lesson
