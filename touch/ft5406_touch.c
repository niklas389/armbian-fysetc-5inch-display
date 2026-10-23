/*
 * ft5406_touch.c — userspace polled FocalTech FT5x06/FT5406 -> uinput
 *
 * For BigTreeTech Pi2 / CB2 with an RPi-DSI-style 5"/7" panel.
 * The touch controller (FT5406 @ I2C2 addr 0x38) has NO interrupt line broken
 * out on the 15-pin DSI FFC, so the mainline interrupt-driven edt-ft5x06 driver
 * cannot be used. This polls the controller over /dev/i2c-2 and injects a
 * standard multitouch-B device via /dev/uinput. No kernel module, survives
 * kernel upgrades.
 *
 * Build:  gcc -O2 -Wall -o ft5406-touch ft5406_touch.c
 * Run:    ./ft5406-touch -b /dev/i2c-2 -x 800 -y 480 -r 60
 * Flags:  -s swap X/Y   -X invert X   -Y invert Y   -r poll Hz   -a addr
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uinput.h>

#define MAX_TOUCH 5

static int ft_addr = 0x38;
static int swap_xy = 0, invert_x = 0, invert_y = 0;
static int scr_w = 800, scr_h = 480;

static int i2c_read_block(int fd, uint8_t reg, uint8_t *buf, int len) {
	struct i2c_msg msgs[2] = {
		{ .addr = ft_addr, .flags = 0,        .len = 1,   .buf = &reg },
		{ .addr = ft_addr, .flags = I2C_M_RD, .len = len, .buf = buf  },
	};
	struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
	return ioctl(fd, I2C_RDWR, &xfer);
}

static void setabs(int fd, int code, int max) {
	struct uinput_abs_setup a; memset(&a, 0, sizeof(a));
	a.code = code; a.absinfo.minimum = 0; a.absinfo.maximum = max;
	ioctl(fd, UI_ABS_SETUP, &a);
}

static int uinput_open(void) {
	int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	if (fd < 0) { perror("open /dev/uinput"); return -1; }

	ioctl(fd, UI_SET_EVBIT, EV_SYN);
	ioctl(fd, UI_SET_EVBIT, EV_KEY);
	ioctl(fd, UI_SET_KEYBIT, BTN_TOUCH);
	ioctl(fd, UI_SET_EVBIT, EV_ABS);
	ioctl(fd, UI_SET_PROPBIT, INPUT_PROP_DIRECT);
	ioctl(fd, UI_SET_ABSBIT, ABS_X);
	ioctl(fd, UI_SET_ABSBIT, ABS_Y);
	ioctl(fd, UI_SET_ABSBIT, ABS_MT_SLOT);
	ioctl(fd, UI_SET_ABSBIT, ABS_MT_TRACKING_ID);
	ioctl(fd, UI_SET_ABSBIT, ABS_MT_POSITION_X);
	ioctl(fd, UI_SET_ABSBIT, ABS_MT_POSITION_Y);

	int xmax = (swap_xy ? scr_h : scr_w) - 1;
	int ymax = (swap_xy ? scr_w : scr_h) - 1;
	setabs(fd, ABS_X, xmax);
	setabs(fd, ABS_Y, ymax);
	setabs(fd, ABS_MT_POSITION_X, xmax);
	setabs(fd, ABS_MT_POSITION_Y, ymax);
	setabs(fd, ABS_MT_SLOT, MAX_TOUCH - 1);
	setabs(fd, ABS_MT_TRACKING_ID, 65535);

	struct uinput_setup us; memset(&us, 0, sizeof(us));
	us.id.bustype = BUS_I2C; us.id.version = 1;
	strncpy(us.name, "ft5406-userspace-touch", UINPUT_MAX_NAME_SIZE - 1);
	if (ioctl(fd, UI_DEV_SETUP, &us) < 0) { perror("UI_DEV_SETUP"); close(fd); return -1; }
	if (ioctl(fd, UI_DEV_CREATE) < 0)     { perror("UI_DEV_CREATE"); close(fd); return -1; }
	return fd;
}

static void emit(int fd, int type, int code, int val) {
	struct input_event ev; memset(&ev, 0, sizeof(ev));
	ev.type = type; ev.code = code; ev.value = val;
	if (write(fd, &ev, sizeof(ev)) != sizeof(ev)) { /* ignore */ }
}

int main(int argc, char **argv) {
	const char *bus = "/dev/i2c-2";
	int hz = 60, opt;
	while ((opt = getopt(argc, argv, "b:a:x:y:r:sXY")) != -1) {
		switch (opt) {
		case 'b': bus = optarg; break;
		case 'a': ft_addr = (int)strtol(optarg, NULL, 0); break;
		case 'x': scr_w = atoi(optarg); break;
		case 'y': scr_h = atoi(optarg); break;
		case 'r': hz = atoi(optarg); break;
		case 's': swap_xy = 1; break;
		case 'X': invert_x = 1; break;
		case 'Y': invert_y = 1; break;
		default:
			fprintf(stderr, "usage: %s [-b bus] [-a addr] [-x W] [-y H] [-r hz] [-s] [-X] [-Y]\n", argv[0]);
			return 1;
		}
	}

	int i2c = open(bus, O_RDWR);
	if (i2c < 0) { perror("open i2c bus"); return 1; }
	int ui = uinput_open();
	if (ui < 0) { close(i2c); return 1; }

	struct timespec ts = { .tv_sec = 0, .tv_nsec = 1000000000L / (hz > 0 ? hz : 60) };
	int slot_active[MAX_TOUCH] = {0}, slot_tid[MAX_TOUCH] = {0}, next_tid = 1;
	uint8_t buf[40];

	for (;;) {
		if (i2c_read_block(i2c, 0x00, buf, 33) < 0) {
			/* FT5406 dislikes very frequent polling -> EREMOTEIO(-6); just retry */
			nanosleep(&ts, NULL);
			continue;
		}
		int n = buf[0x02] & 0x0F;
		if (n > MAX_TOUCH) n = MAX_TOUCH;

		int seen[MAX_TOUCH] = {0}, any = 0, fx = -1, fy = -1;
		int xmax = (swap_xy ? scr_h : scr_w) - 1;
		int ymax = (swap_xy ? scr_w : scr_h) - 1;

		for (int i = 0; i < n; i++) {
			uint8_t *p = &buf[0x03 + i * 6];
			int evt = (p[0] >> 6) & 0x03;           /* 0 down,1 up,2 contact,3 none */
			int x = ((p[0] & 0x0F) << 8) | p[1];
			int y = ((p[2] & 0x0F) << 8) | p[3];
			int id = (p[2] >> 4) & 0x0F;
			if (id >= MAX_TOUCH || evt == 1) continue;

			int tx = x, ty = y;
			if (swap_xy) { int t = tx; tx = ty; ty = t; }
			if (invert_x) tx = xmax - tx;
			if (invert_y) ty = ymax - ty;

			seen[id] = 1; any = 1;
			if (!slot_active[id]) { slot_active[id] = 1; slot_tid[id] = next_tid++; if (next_tid > 65000) next_tid = 1; }
			emit(ui, EV_ABS, ABS_MT_SLOT, id);
			emit(ui, EV_ABS, ABS_MT_TRACKING_ID, slot_tid[id]);
			emit(ui, EV_ABS, ABS_MT_POSITION_X, tx);
			emit(ui, EV_ABS, ABS_MT_POSITION_Y, ty);
			if (fx < 0) { fx = tx; fy = ty; }
		}
		for (int s = 0; s < MAX_TOUCH; s++) {
			if (slot_active[s] && !seen[s]) {
				slot_active[s] = 0;
				emit(ui, EV_ABS, ABS_MT_SLOT, s);
				emit(ui, EV_ABS, ABS_MT_TRACKING_ID, -1);
			}
		}
		emit(ui, EV_KEY, BTN_TOUCH, any ? 1 : 0);
		if (any) { emit(ui, EV_ABS, ABS_X, fx); emit(ui, EV_ABS, ABS_Y, fy); }
		emit(ui, EV_SYN, SYN_REPORT, 0);
		nanosleep(&ts, NULL);
	}
	return 0;
}
