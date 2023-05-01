# LoveRay
LoveRay2d is not 100% compatible with the original Love2d, but it includes the main events that are compatible with Love2d’s characteristics of input, draw, and transform. 

Love2d using only Lua, Raylib, and Chipmunk2d for physics.

Features:

- Fast and efficient 2D rendering using the Raylib library
- Accurate and realistic 2D physics simulation using the Chipmunk2D library
- Lua script support for game logic and events
- Easy to use 
- Support for audio, keyboard and mouse input, and other common 2D game features

<img src="https://raw.githubusercontent.com/akadjoker/LoveRay/main/001.gif">
<img src="https://raw.githubusercontent.com/akadjoker/LoveRay/main/002.gif">
<img src="https://raw.githubusercontent.com/akadjoker/LoveRay/main/003.gif">
<img src="https://raw.githubusercontent.com/akadjoker/LoveRay/main/004.gif">

Api List
chipmunk

newShapeFilter 
newDynamic
newStatic
newKinematic


newPinJoint
newSlideJoint
newPivotJoint
newRatchetJoint
newGearJoint
newMotor
newGrooveJoint  
newRotaryLimitJoint

getStaticBody

update
draw
clear
setGravity
setDebugFill

getBodyAtPoint
getBodyAtMouse


;Body
remove
addBox
addCircle
addLine
setPosition
getPosition
setAngle
getAngle
setMass
getMass
getVelocity
setVelocity
getAngularVelocity
setAngularVelocity
setForce
getForce
setTorque
getTorque
applyForceAtWorldPoint
applyForceAtLocalPoint
applyImpulseAtWorldPoint
applyImpulseAtLocalPoint
getVelocityAtWorldPoint
getVelocityAtLocalPoint
setCenterOfGravity
getCenterOfGravity
localToWorld
worldToLocal

;Shape
setSensor
getSensor
setElasticity
getElasticity
setFriction
getFriction
setSurfaceVelocity
getSurfaceVelocity
setFilter
getFilter
getMoment
getArea
getCenterOfGravity


;Joint
remove
setAnchoB
setAnchorA
setMaxForce

;Keyboard

isDown
check
down
press
release
up
keyPressed
charPressed
setExitKey

;Mouse
getPosition
getDelta
getX
getY
isDown
check
press
release

;Timer
getTime
getDelta
getFPS


;Os
getOS
getMemUsage

;Filesystem
exists
isFile
getInfo
isDirectory
getWorkingDirectory
getPath
read
load


;Audio
setMasterVolume
newSound
newMusic

;Music
isPlaying
play
pause
stop
resume
seek
setVolume
setPitch
setPan
getTimeLength
getTimePlayed

;Sound
play
stop
pause
resume
setVolume
setPitch
setPan
isPlaying

;Graphics
setBackgroundColor
setColor
getFont
getWidth
getHeight
pop
push
rotate
scale
translate
shear
clear
begin
present
draw
point
line
rectangle
circle
print
newImage
newQuad
newSound
newFont


