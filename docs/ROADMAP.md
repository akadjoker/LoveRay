# Roadmap

Ordered by how many existing Love2D games each item unblocks.

1. **Stencil buffer**: real `stencil` and `setStencilTest`.
2. **Meshes**: `newMesh` and vertex formats on top of rlgl, which also unlocks custom vertex attributes in shaders.
3. **love.data and love.sound**: encoding, hashing, compression, `SoundData` and queueable sources.
4. **love.thread and love.font**: channels and threads, rasterizers and glyph data.
5. **Mobile**: Android build.
6. **Multi-canvas and MSAA canvases**.

Contributions welcome. Every change should keep `tests/smoke.sh` green and add checks to `tests/api` or `tests/physics`.
