# sophgo-sg200x-wifi-camera

Small camera utils for LicheeRV Nano based on the on-board CVITEK middleware. Based on @scpcom's video pipeline and mostly written by Codex.

## CLIs

1. `camera-frame`

Capture a JPEG frame: `sudo camera-frame`.

2. `camera-stream`

Record an MP4 with H.264 video and AAC audio: `sudo camera-stream -s 10`, `-s` specifies duration in seconds.

3. `camera-server`

Simple HTTP server for recording and management: `sudo camera-server -p 8001`, `-p` specifies port. `sudo systemctl enable camera-server.service` to make it auto-start.

## Data

Runtime data: `/var/lib/wifi-camera/`

Recordings: `/var/lib/wifi-camera/recordings/`

Logs: `/var/log/wifi-camera/`

## Building

Preparation:

- Sophgo/CVITEK middleware tree.
- Kernel headers for the board.
- RISC-V glibc cross compiler.
- `pkg-config`, `make`, `tar`.

Paths:

```
# Example
MIDDLEWARE_DIR=/build/middleware
KERNEL_DIR=/build/kernel/build/licheervnano-e
CROSS_COMPILE=/host-tools/gcc/riscv64-linux-x86_64/bin/riscv64-unknown-linux-gnu-
SYSROOT=/host-tools/gcc/riscv64-linux-x86_64/sysroot
```

Build & staging install:
```sh
rm -rf dist/
make install \
  DESTDIR="$PWD/dist/root" \
  PREFIX=/usr \
  MIDDLEWARE_DIR=/build/middleware \
  KERNEL_DIR=/build/kernel/build/licheervnano-e \
  CROSS_COMPILE=/host-tools/gcc/riscv64-linux-x86_64/bin/riscv64-unknown-linux-gnu- \
  SYSROOT=/host-tools/gcc/riscv64-linux-x86_64/sysroot
```
