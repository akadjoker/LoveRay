# Roadmap

Ordered by how many existing Love2D games each item unblocks.

1. **Shaders**: `love.graphics.newShader` with GLSL 1.20 style Love shaders translated to raylib shaders, `Shader:send`, `setShader`.
2. **Stencil buffer**: real `stencil` and `setStencilTest`.
3. **Particle systems**: `newParticleSystem` with the full `ParticleSystem` API.
4. **Meshes**: `newMesh` and vertex formats on top of rlgl.
5. **love.data and love.sound**: encoding, hashing, compression, `SoundData` and queueable sources.
6. **Archives**: mount `.love` files and zip archives so games ship as a single file.
7. **love.thread and love.font**: channels and threads, rasterizers and glyph data.
8. **Mobile and fused executables**: Android build and appending a `.love` to the executable.
9. **Multi-canvas and MSAA canvases**.

Contributions welcome. Every change should keep `tests/smoke.sh` green and add checks to `tests/api` or `tests/physics`.
