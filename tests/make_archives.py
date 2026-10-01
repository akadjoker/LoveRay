#!/usr/bin/env python3
"""Builds the zip files used by tests/archive: python3 tests/make_archives.py <output dir>"""
import os
import struct
import sys
import zipfile
import zlib


def png(width, height, rgba):
    raw = b"".join(b"\x00" + bytes(rgba) * width for _ in range(height))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b"")


def build(out):
    os.makedirs(out, exist_ok=True)
    root = os.path.dirname(os.path.abspath(__file__))

    inner = os.path.join(out, "inner.zip")
    with zipfile.ZipFile(inner, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("inner.txt", "from the inner archive")
    with open(inner, "rb") as f:
        inner_bytes = f.read()

    with zipfile.ZipFile(os.path.join(out, "data.zip"), "w") as z:
        z.writestr("plain.txt", "stored content", zipfile.ZIP_STORED)
        z.writestr("big.txt", "".join("line %d\n" % i for i in range(20000)), zipfile.ZIP_DEFLATED)
        z.writestr("sub/deep/file.lua", "return 7", zipfile.ZIP_DEFLATED)
        z.writestr("empty/", "")
        z.writestr("tile.png", png(8, 4, (255, 0, 0, 255)), zipfile.ZIP_DEFLATED)
        z.writestr("inner.zip", inner_bytes, zipfile.ZIP_STORED)
        z.writestr("crlf.txt", "one\r\ntwo\r\nthree", zipfile.ZIP_STORED)

    with zipfile.ZipFile(os.path.join(out, "data2.zip"), "w") as z:
        z.writestr("plain.txt", "second", zipfile.ZIP_STORED)
        z.writestr("only2.txt", "only in the second archive", zipfile.ZIP_STORED)

    with zipfile.ZipFile(os.path.join(out, "bad.zip"), "w") as z:
        z.writestr("bad.txt", "this content will be damaged", zipfile.ZIP_STORED)
    path = os.path.join(out, "bad.zip")
    with open(path, "rb") as f:
        data = bytearray(f.read())
    marker = data.index(b"this content")
    data[marker] ^= 0xFF
    with open(path, "wb") as f:
        f.write(data)

    with open(os.path.join(out, "notazip.zip"), "wb") as f:
        f.write(b"this is not a zip file" * 10)

    game = os.path.join(root, "archive")
    with zipfile.ZipFile(os.path.join(out, "archive.love"), "w", zipfile.ZIP_DEFLATED) as z:
        for name in sorted(os.listdir(game)):
            z.write(os.path.join(game, name), name)


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else "tests/out")
