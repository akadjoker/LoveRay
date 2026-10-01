# API status

Follows Love2D 11.5. `love.getVersion()` returns `11, 5, 0, "LoveRay"`.

Legend: **full** works as documented, **partial** works with the listed limits, **stub** accepted but has no effect, **missing** raises an error or does not exist.

## Modules

| Module | Status | Notes |
| --- | --- | --- |
| love | full | `getVersion`, `isVersionCompatible`, `love.run`, `love.errorhandler`, `love.handlers`, all input and window callbacks |
| love.audio | partial | `Source` objects (static and stream), master volume. Positional audio is stored but not spatialized. No effects, filters, recording or queueable sources |
| love.event | full | `pump`, `poll`, `push`, `quit`, `clear`, `wait` |
| love.filesystem | full | Game folder, `.love` and zip archives (deflate and stored), a zip appended to the executable, `mount` and `unmount` for files, folders and `FileData`, mount points and mount order, save directory, `File` and `FileData` objects, `require` through mounted paths. No ZIP64, encrypted archives or other archive formats, `remove` only deletes inside the save directory |
| love.graphics | partial | See below |
| love.image | partial | `ImageData` with `getPixel`, `setPixel`, `mapPixel`, `paste`, `encode`. Only the `rgba8` format |
| love.joystick | partial | Gamepads through raylib, gamepad buttons and axes, `joystick*` and `gamepad*` events. No vibration, hats or custom mappings |
| love.keyboard | full | All key constants, key repeat, text input events. Scancodes equal key names |
| love.math | partial | Random generators, simplex noise, `Transform`, `BezierCurve`, color conversion, polygon helpers. No `compress` or `decompress` |
| love.mouse | partial | Position, buttons 1 to 7, visibility, relative mode, system cursors. No custom image cursors, grabbing is only recorded |
| love.physics | full | Box2D 2.4: `World`, `Body`, `Fixture`, all four shapes, all eleven joint types, contact callbacks, queries, ray casts. `setMeter` defaults to 30 |
| love.system | partial | OS, clipboard, URLs, processor count. No power information |
| love.timer | full | `getTime`, `getDelta`, `getFPS`, `getAverageDelta`, `step`, `sleep` |
| love.window | partial | `setMode`, `updateMode`, fullscreen, vsync, title, icon, position, minimize, maximize. Message boxes only print to the console |
| love.data, love.sound, love.font, love.thread, love.touch, love.video | missing | |

## love.graphics

| Area | Status | Notes |
| --- | --- | --- |
| State | full | Colors, blend modes (`alpha`, `add`, `subtract`, `multiply`, `lighten`, `darken`, `screen`, `replace`), line width, point size, scissor, color mask, wireframe, default filter, `reset` |
| Shapes | full | `rectangle` (rounded), `circle`, `ellipse`, `arc`, `polygon` (concave polygons are triangulated), `line`, `points` |
| Transform stack | full | `push` (`"transform"` or `"all"`), `pop`, `translate`, `rotate`, `scale`, `shear`, `origin`, `applyTransform`, `replaceTransform`, `transformPoint` |
| Text | full | `print`, `printf` with alignment and wrapping, colored text tables, `Font` and `Text` objects. The atlas covers Latin scripts |
| Images | full | `Image`, `Quad`, filters, wrap modes, mipmaps, `ImageData` input |
| Canvas | partial | `Canvas` objects, `setCanvas`, `renderTo`, `newImageData`. A single render target, no MSAA |
| SpriteBatch | partial | `add`, `set`, `clear`, colors, draw range. No attached vertex attributes |
| Shaders | partial | `newShader`, `setShader`, `getShader`, `validateShader`, `Shader:send`, `sendColor`, `hasUniform`. See below |
| ParticleSystem | full | `newParticleSystem` with every setter and getter of Love2D 11.5: emission, lifetime, speed, direction, spread, linear/radial/tangential acceleration, damping, spin, rotation, sizes, colors, quads, offset, emission areas, insert modes, `moveTo`, `clone` |
| Stencil | full | `stencil` with all six actions and `keepvalues`, `setStencilTest`/`getStencilTest` with every compare mode, `clear` resets the stencil buffer, `push("all")` saves the test. Every Canvas and the window have an 8-bit stencil buffer |
| Mesh, Video, array/cube/volume images | missing | `newMesh` and friends raise "not supported by LoveRay yet" |

### Shaders

Write shaders exactly as for Love2D: `vec4 effect(vec4 color, Image tex, vec2 texture_coords, vec2 screen_coords)` and `vec4 position(mat4 transform_projection, vec4 vertex_position)`, in one string or as separate pixel and vertex code. Code is translated to GLSL 3.30 on desktop and GLSL ES 1.00 in the browser.

- Available keywords and names: `extern`, `number`, `Image`, `Texel`, `varying`, `VaryingTexCoord`, `VaryingColor`, `love_ScreenSize`.
- `screen_coords` has its origin at the top left, also when drawing to a canvas.
- Uniform types: `float`, `int`, `bool`, `vec2` to `vec4`, `ivec2` to `ivec4`, `mat4`, `Image` and arrays of the numeric types. `mat2` and `mat3` raise an error.
- Compile errors are raised from `newShader` with the driver message and the line number of your code.
- A Canvas sent as an extra texture is vertically flipped compared to Love2D. Drawing a Canvas through a shader is not affected.
- Not available: multi-canvas output, vertex attributes beyond position, texcoord and color, `love_PixelCoord`, `Shader:getExternVariable`.

## Implemented functions

Generated from the binary with `love.<module>` tables, see `tests/api/main.lua` for the behavior that is checked.

- **audio**: getActiveEffects getActiveSourceCount getDistanceModel getDopplerScale getEffect getMaxSceneEffects getMaxSourceEffects getOrientation getPosition getRecordingDevices getSourceCount getVelocity getVolume isEffectsSupported newSource pause play setDistanceModel setDopplerScale setEffect setMixWithSystem setOrientation setPosition setVelocity setVolume stop
- **event**: clear poll pump push quit wait
- **filesystem**: append areSymlinksEnabled createDirectory exists getAppdataDirectory getCRequirePath getDirectoryItems getExecutablePath getIdentity getInfo getRealDirectory getRequirePath getSaveDirectory getSource getSourceBaseDirectory getUserDirectory getWorkingDirectory isDirectory isFile isFused lines load mount newFile newFileData read remove setCRequirePath setIdentity setRequirePath setSource setSymlinksEnabled unmount write
- **graphics**: applyTransform arc captureScreenshot circle clear discard draw ellipse getBackgroundColor getBlendMode getCanvas getCanvasFormats getColor getColorMask getDPIScale getDefaultFilter getDimensions getFont getHeight getImageFormats getLineJoin getLineStyle getLineWidth getPixelDimensions getPixelHeight getPixelWidth getPointSize getRendererInfo getScissor getShader getStats getSupported getSystemLimits getWidth intersectScissor inverseTransformPoint isActive isCreated isGammaCorrect isWireframe line newCanvas newFont newImage newParticleSystem newQuad newShader newSpriteBatch newText origin points polygon pop present print printf push rectangle replaceTransform reset rotate scale setBackgroundColor setBlendMode setCanvas setColor setColorMask setDefaultFilter setFont setLineJoin setLineStyle setLineWidth setNewFont setPointSize setScissor setShader setWireframe shear stencil transformPoint translate validateShader
- **image**: newImageData
- **joystick**: getJoystickCount getJoysticks loadGamepadMappings
- **keyboard**: hasKeyRepeat hasTextInput isDown isModifierActive isScancodeDown setKeyRepeat setTextInput
- **math**: colorFromBytes colorToBytes gammaToLinear getRandomSeed getRandomState isConvex linearToGamma newBezierCurve newRandomGenerator newTransform noise random randomNormal setRandomSeed setRandomState triangulate
- **mouse**: getCursor getPosition getRelativeMode getSystemCursor getX getY isCursorSupported isDown isGrabbed isVisible setCursor setGrabbed setPosition setRelativeMode setVisible setX setY
- **physics**: getDistance getMeter newBody newChainShape newCircleShape newDistanceJoint newEdgeShape newFixture newFrictionJoint newGearJoint newMotorJoint newMouseJoint newPolygonShape newPrismaticJoint newPulleyJoint newRectangleShape newRevoluteJoint newRopeJoint newWeldJoint newWheelJoint newWorld setMeter
- **system**: getClipboardText getOS getPowerInfo getPreferredLocales getProcessorCount openURL setClipboardText
- **timer**: getAverageDelta getDelta getFPS getTime sleep step
- **window**: close fromPixels getDPIScale getDesktopDimensions getDisplayCount getDisplayName getFullscreen getFullscreenModes getIcon getMode getPosition getSafeArea getTitle getVSync hasFocus hasMouseFocus isDisplaySleepEnabled isMaximized isMinimized isOpen isVisible maximize minimize requestAttention restore setDisplaySleepEnabled setFullscreen setIcon setMode setPosition setTitle setVSync showMessageBox toPixels updateMode
