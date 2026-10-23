# Fysetc 5" DSI touchscreen on BTT Pi2 / CB2 (RK3566) with mainline Armbian

Get the **Fysetc 5" MIPI-DSI touchscreen** (display **and** touch) working on the
**BigTreeTech Pi2 / CB2** (Rockchip RK3566) running **official/upstream Armbian**
instead of BigTreeTech's legacy 4.19 vendor image. Everything ships as one
Debian package.

Display tested on Armbian (Debian 13 / Trixie minimal), kernels `6.18.35` and
`6.18.44-current-rockchip64`. The reworked touch poller (1.2.0) has unit tests,
but has not been verified on hardware yet.

---

## Install

1. Download `fysetc-5inch-dsi_<version>_arm64.deb` from the
   [releases](https://github.com/niklas389/armbian-fystec-5inch-display/releases)
   (or the `build-deb` workflow artifacts).
2. Install it together with the kernel headers:

   ```sh
   sudo apt install linux-headers-current-rockchip64 ./fysetc-5inch-dsi_*_arm64.deb
   sudo reboot
   ```

The package:

| Part | What it does |
|------|--------------|
| Display overlay | Installs `rk3566-btt-pi2-dsi-rpi7.dtbo` to `/boot/overlay-user/` and adds it to `user_overlays` in `/boot/armbianEnv.txt`. |
| Panel driver | DKMS module `panel-raspberrypi-touchscreen-fysetc`: overrides the in-tree driver with burst video mode and the Fysetc timing. It is rebuilt automatically when Armbian installs a new kernel. |
| Touch | `ft5406-touch.service`: polls the FT5406 over `/dev/i2c-2` and provides a multitouch device via `uinput`. Options are in `/etc/default/ft5406-touch`. |

**The kernel headers metapackage must stay installed.** DKMS can only rebuild the
panel driver for a new kernel if its headers are installed. Without them, a kernel
update silently falls back to the in-tree driver (wrong timing, shifted image and
wrong colours).

The package also removes an earlier manual install: the DKMS module
`panel-raspberrypi-touchscreen/1.x-fysetc5`, and `/etc/systemd/system/ft5406-touch.service`
together with `/usr/local/bin/ft5406-touch` (the unit is kept as `*.pre-deb`).

Uninstalling (`sudo apt remove fysetc-5inch-dsi`) removes the overlay entry, the
DKMS module and the service.

---

## Verify

```sh
# Display
cat /sys/class/drm/*DSI*/status                            # connected
dkms status panel-raspberrypi-touchscreen-fysetc           # installed for the running kernel
sudo modetest -c 2>/dev/null | sed -n '/DSI/,/props/p'     # 800x480, htotal 861, nhsync nvsync

# Touch
systemctl status ft5406-touch
sudo libinput debug-events                                 # taps produce one DOWN ... UP each
```

---

## Touch troubleshooting

Stop the service and dump the controller registers:

```sh
sudo systemctl stop ft5406-touch
sudo ft5406-touch -p
sudo systemctl start ft5406-touch
```

`-p` prints the chip ID, firmware, power-mode registers and 10 raw frames
(`0x00`–`0x20`). It marks each frame `valid` or `INVALID`. **With no finger on the
panel, byte 2 (TD_STATUS) must be `00`.**

The poller discards every frame that is not a plausible FT5x06 frame:

- device mode is not "working",
- the reserved TD_STATUS bits are set or it reports more than 10 points,
- duplicate touch IDs,
- coordinates outside the controller range (`-W`/`-H`, default `-x`/`-y`).

A new contact must also be present in 2 consecutive valid frames (`-d`) before it
is reported. After 5 consecutive unreadable or invalid frames, all contacts are
released. The journal (`journalctl -u ft5406-touch`) shows how many frames were
discarded; add `-v` in `/etc/default/ft5406-touch` to dump each one.

| Symptom | Fix |
|---------|-----|
| `i2c read errors` in the journal (EREMOTEIO) | Lower the poll rate: `-r 30`. |
| `address 0x38 ... claimed by a kernel driver` | A DT node binds `edt-ft5x06` to `2-0038`; remove it. Both drivers would read the chip concurrently. |
| Axes mirrored or rotated | `-s` swaps X/Y, `-X`/`-Y` invert (applied after the swap). |
| Touch works only in part of the screen | The controller range differs from 800x480: set `-W`/`-H` to its range. |
| Many `invalid frame(s)` while idle | The controller or the bus returns garbage: post the `-p` output in an issue. |

---

## Background

### Why it breaks (hardware)

The Fysetc 5" is electrically a **Raspberry Pi 7" DSI touchscreen clone**:

- MIPI-DSI **TC358762**-class DSI→DPI bridge.
- Panel power/backlight **MCU at I2C `0x45`** (RPi "Attiny" protocol).
- **FocalTech FT5406** capacitive touch at I2C `0x38`.

The 15-pin DSI FFC carries only 1 clock lane, 2 data lanes, **I2C2 (SCL/SDA)**,
3V3 and GND. There is **no interrupt or reset pin**, so the touch controller can
only be **polled**.

Upstream Armbian ships no DSI panel description for this board and leaves `&i2c2`
disabled, which leaves a white screen and no touch.

### Final display mode

Captured from the working vendor image and verified against a test pattern:

```
clock 26101 kHz
H: 800  hss 855  hse 857  htotal 861
V: 480  vss 487  vse 489  vtotal 510
flags: NHSYNC, NVSYNC ; MIPI video BURST mode
```

- **Burst mode is required.** The in-tree driver uses `MODE_VIDEO_SYNC_PULSE`,
  which on this bridge produces a horizontal shift *and* wrong colours. The BTT
  vendor DTB carries `snps,mixed-burst`, which is the hint.
- **`hsync_start` controls horizontal position and RGB byte phase together.**
  Keep `hsync_start = 801 + 3n` to stay colour-correct; each unit shifts the image
  ~1 px. Tune position in **steps of 3**. `hse = hss + 2`, `htotal = hss + 6`.
- If your panel revision differs, capture its modeline with `modetest -c` on a
  known-good image and substitute it in `rpi_touchscreen_modes[]`.

---

## Repository layout

```
overlay/       rk3566-btt-pi2-dsi-rpi7.dts      device-tree overlay (display + i2c2)
panel-driver/  panel-raspberrypi-touchscreen.c  DKMS driver (burst + Fysetc timing)
               Makefile  dkms.conf
touch/         ft5406_touch.c                   userspace FT5406 -> uinput poller
               test_ft5406.c                    parser/tracker unit tests (make test)
               ft5406-touch.service  ft5406-touch.default  modules-load.conf
debian/        packaging (dh + dh-dkms)
```

### Building the package

On the board (native):

```sh
sudo apt install build-essential debhelper dh-dkms device-tree-compiler
dpkg-buildpackage -b -us -uc
```

Cross-building on Debian trixie (amd64), as CI does:

```sh
sudo dpkg --add-architecture arm64 && sudo apt update
sudo apt install build-essential crossbuild-essential-arm64 debhelper dh-dkms device-tree-compiler libc6:arm64
dpkg-buildpackage -aarm64 -b -us -uc
```

Pushing a `v*` tag attaches the `.deb` to a GitHub release.

---

## Known limitations

- Timing is verified only for the **Fysetc 5" 800x480** revision the author owns.
- The panel driver is a shadowing DKMS override of the in-tree module (same module
  name). If a future kernel changes that driver's API, the DKMS build fails and
  the driver needs a refresh from the new upstream source.
- The overlay targets the BTT CB2 device tree as shipped by Armbian.

## Licensing

Mixed license:

- **`panel-driver/panel-raspberrypi-touchscreen.c`: GPL-2.0-only.** A modified copy
  of the mainline Linux driver (© 2016-2017 Broadcom, author Eric Anholt), with
  portions derived from `panel-simple.c` (© 2013 NVIDIA, MIT terms). All original
  notices are retained. See `panel-driver/LICENSE`.
- **Everything else: MIT** (see `LICENSE`). The touch poller only uses the kernel
  uAPI (`uinput`, `i2c-dev`).

## Credits

Hardware bring-up, DSI timing characterisation and the userspace touch driver by
**niklas389**. Panel driver derived from the Linux kernel
(`drivers/gpu/drm/panel/panel-raspberrypi-touchscreen.c`).
