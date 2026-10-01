# Roadmap

Ordered by how many existing Love2D games each item unblocks.

1. **Meshes**: `newMesh` and vertex formats on top of rlgl, which also unlocks custom vertex attributes in shaders.
2. **love.data and love.sound**: encoding, hashing, compression, `SoundData` and queueable sources.
3. **love.thread and love.font**: channels and threads, rasterizers and glyph data.
4. **Depth buffer**: `setDepthMode` for 3D-style drawing with meshes.
5. **Mobile**: Android build.
6. **Multi-canvas and MSAA canvases**.

Contributions welcome. Every change should keep `tests/smoke.sh` green and add checks to `tests/api` or `tests/physics`.
