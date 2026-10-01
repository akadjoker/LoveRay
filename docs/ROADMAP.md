# Roadmap

Ordered by how many existing Love2D games each item unblocks.

1. **Archives**: mount `.love` files and zip archives so games ship as a single file, plus fused executables.
2. **Stencil buffer**: real `stencil` and `setStencilTest`.
3. **Meshes**: `newMesh` and vertex formats on top of rlgl, which also unlocks custom vertex attributes in shaders.
4. **love.data and love.sound**: encoding, hashing, compression, `SoundData` and queueable sources.
5. **love.thread and love.font**: channels and threads, rasterizers and glyph data.
6. **Mobile**: Android build.
7. **Multi-canvas and MSAA canvases**.

Contributions welcome. Every change should keep `tests/smoke.sh` green and add checks to `tests/api` or `tests/physics`.
