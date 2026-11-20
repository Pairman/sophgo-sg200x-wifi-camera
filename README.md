# sophgo-sg200x-wifi-camera

Small camera utils for LicheeRV Nano based on the on-board CVITEK middleware. Based on @scpcom's video pipeline and mostly written by Codex.

## CLIs

1. `camera-frame`

Capture a JPEG frame: `sudo camera-frame -o out.jpg`.

2. `camera-stream`

Record an MP4 with H.264 video: `sudo camera-stream -s 10 -o out.mp4`, `-s` specifies duration in seconds. Resolution defaults to 720p; select `-r 1440p` for 2560×1440. Add `-a` to record AAC audio from the board microphone; audio is off by default.

3. `camera-server`

HTTP camera UI for preview, recording, photos, and file management: `sudo camera-server -p 8001`, `-p` specifies port. The UI defaults to 720p and allows selecting 1440p. Preview is started on demand while a browser requests it and is published at 1 fps; it stops after the browser stops polling or when a photo/recording takes the camera. The preview worker runs at lower CPU priority. Browser recordings are silent unless “Record sound” is enabled. The GC4653 driver provides both sensor modes; 1440p VENC acceptance still needs board validation. `sudo systemctl enable camera-server.service` to make it auto-start. User LED is used to display recording status.

## Data

Runtime data: `/var/lib/wifi-camera/`

Recordings: `/var/lib/wifi-camera/recordings/`

Logs: `/var/log/wifi-camera/`

## Camera notes

- 720p recording uses GC4653 VTS 1800 (25 fps from the 720p mode's 750-line, 60 fps timing); video encoding and MP4 timestamps are 25 fps. The 1440p sensor mode uses VTS 1800 for 25 fps from its 1500-line, 30 fps timing.
- The preview path still writes GC4653 VTS 16364 after ISP startup to reduce sensor activity. ISP/AE coordination for that direct sensor-register write remains unresolved and is intentionally unchanged.
- 1440p currently uses the same 4 Mbps H.264 bitrate as 720p. This is recorded for later tuning and is intentionally unchanged.
- Audio capture runs in its own thread and feeds a bounded PCM queue; AAC encoding and MP4 writes remain in the recording thread so the muxer is accessed by one thread. On the single-core board, this removes blocking microphone reads from the video loop, while AAC encoding still shares that loop.

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
