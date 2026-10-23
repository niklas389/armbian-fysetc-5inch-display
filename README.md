# Fysetc 5" DSI touchscreen on BTT Pi2 / CB2 (RK3566) with mainline Armbian

Get the **Fysetc 5" MIPI-DSI touchscreen** (display **and** touch) working on the
**BigTreeTech Pi2 / CB2** (Rockchip RK3566) running **official/upstream Armbian**
instead of BigTreeTech's legacy 4.19 vendor image. Everything ships as one
Debian package.

Tested on Armbian 26.8.3 (Debian 13 / Trixie minimal), kernel
`6.18.44-current-rockchip64`, with KlipperScreen: display, single and
multi-touch (package 1.3.0).

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
| Touch | `ft5406-touch.service`: polls the touch MCU over `/dev/i2c-2` and provides a multitouch device via `uinput`. Options are in `/etc/default/ft5406-touch`. |

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

Stop the service and look at what the panel reports:

```sh
sudo systemctl stop ft5406-touch
sudo ft5406-touch -p          # touch the panel while it runs
sudo systemctl start ft5406-touch
```

`-p` prints 20 frames: TD_STATUS (number of touches), the raw point records
and the decoded points. With no finger on the panel, TD_STATUS must be `00`.
The printed coordinates are the panel's raw values, before `-s`/`-X`/`-Y`.

| Symptom | Fix |
|---------|-----|
| Touch mirrored | The default is `-X -Y` (both axes inverted, as the panel reports them). Remove one or both in `/etc/default/ft5406-touch`. |
| Touch rotated | `-s` swaps X/Y; `-X`/`-Y` are applied after the swap. |
| `i2c read errors` in the journal (EREMOTEIO) | Lower the poll rate: `-r 30`. |
| `address 0x38 ... claimed by a kernel driver` | A DT node binds `edt-ft5x06` to `2-0038`; remove it. A second reader shifts the panel's ring pointer. |
| Many `invalid frame(s)` | Post the `-p` output (with and without a finger) in an issue. |

### Why the stock driver cannot work

The device at `0x38` is not a FocalTech chip: an MCU on the Fysetc board
emulates the FT5x06 register map, but multi-byte reads do not follow it.

- The register selects a record: `0x02` TD_STATUS, `0x03 + 6*i` point *i*.
- Only the first byte of a read comes from that record (TD_STATUS or XH).
- Every further byte comes from a 3-entry ring (XL, YH, YL) whose read pointer
  advances with every byte and is never reset, not by STOP and not by a new
  register address.

`edt-ft5x06` reads long blocks, gets bytes from the wrong ring position and
reports ghost touches. The poller reads TD_STATUS with 1-byte reads and each
point with exactly 4 bytes (3 ring bytes: the phase stays constant), finds the
phase from the data and re-checks it on every frame. Every frame is also
validated (reserved TD_STATUS bits, IDs, coordinate range), and a new contact
must persist for 2 frames (`-d`) before it is reported.

This read behaviour was first documented by
[kkobold/fysetc-5inch-dsi-touch](https://github.com/kkobold/fysetc-5inch-dsi-touch)
(a kernel driver for Raspberry Pi OS) and confirmed on the BTT CB2.

---

## Background

### Why it breaks (hardware)

The Fysetc 5" is electrically a **Raspberry Pi 7" DSI touchscreen clone**:

- MIPI-DSI **TC358762**-class DSI→DPI bridge.
- Panel power/backlight **MCU at I2C `0x45`** (RPi "Attiny" protocol).
- Touch at I2C `0x38`: an MCU emulating a FocalTech FT5406, with a
  non-standard read protocol (see [Touch troubleshooting](#touch-troubleshooting)).

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
touch/         ft5406_touch.c                   userspace touch poller (0x38 -> uinput)
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
