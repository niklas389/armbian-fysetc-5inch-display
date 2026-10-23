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
 * Every frame is validated before use: a frame with a non-working device mode,
 * reserved TD_STATUS bits set, more points than the chip supports, duplicate
 * touch IDs or out-of-range coordinates is discarded as a whole. A new contact
 * must be seen in <debounce> consecutive valid frames before it is reported.
 *
 * Build:  gcc -O2 -Wall -o ft5406-touch ft5406_touch.c
 * Run:    ./ft5406-touch -b /dev/i2c-2 -x 800 -y 480 -r 60
 * Flags:  -s swap X/Y   -X invert X   -Y invert Y   -r poll Hz   -a addr
 *         -W/-H controller raw range (default = -x/-y, scaled to -x/-y)
 *         -R resolution in units/mm   -d debounce frames   -v verbose
 *         -p probe: print chip ID registers and raw frames, then exit
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <time.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uinput.h>

#define MAX_TOUCH    5                    /* slots exposed via uinput */
#define FT_MAX_POINTS 10                  /* FT5406 tracks up to 10 */
#define FT_REG_MODE  0x00
#define FT_REG_TD    0x02
#define FT_REG_P1    0x03
#define FT_FRAME_LEN (FT_REG_P1 + MAX_TOUCH * 6)   /* 0x00..0x20 */
#define MAX_BAD      5                    /* bad frames before releasing all */

enum { EVT_DOWN = 0, EVT_UP = 1, EVT_CONTACT = 2, EVT_NONE = 3 };

struct ft_point { int id, x, y; };

/*
 * Parse one register frame (0x00..0x20). Returns the number of points written
 * to pts, or -1 if the frame is not a plausible FT5x06 frame.
 */
static int ft_parse(const uint8_t *buf, int raw_w, int raw_h, struct ft_point *pts)
{
	if (buf[FT_REG_MODE] & 0x70) return -1;         /* not in working mode */
	if (buf[FT_REG_TD] & 0xF0) return -1;           /* reserved bits set */
	int n = buf[FT_REG_TD] & 0x0F;
	if (n > FT_MAX_POINTS) return -1;
	if (n > MAX_TOUCH) n = MAX_TOUCH;

	int cnt = 0;
	unsigned seen = 0;
	for (int i = 0; i < n; i++) {
		const uint8_t *p = &buf[FT_REG_P1 + i * 6];
		int evt = (p[0] >> 6) & 0x03;
		int x = ((p[0] & 0x0F) << 8) | p[1];
		int y = ((p[2] & 0x0F) << 8) | p[3];
		int id = (p[2] >> 4) & 0x0F;
		if (evt == EVT_NONE) continue;          /* empty record, ID may be 0xF */
		if (id >= FT_MAX_POINTS) return -1;
		if (seen & (1u << id)) return -1;       /* duplicate ID */
		seen |= 1u << id;
		if (evt == EVT_UP) continue;
		if (x >= raw_w || y >= raw_h) return -1;
		if (id >= MAX_TOUCH) continue;          /* beyond exposed slots */
		pts[cnt].id = id; pts[cnt].x = x; pts[cnt].y = y;
		cnt++;
	}
	return cnt;
}

struct cfg {
	int raw_w, raw_h;       /* controller coordinate range */
	int out_w, out_h;       /* reported range, in controller axis order */
	int swap_xy, invert_x, invert_y;
	int debounce;           /* valid frames before a contact goes down */
};

struct slot { int active, pending, tid, x, y; };

struct tracker {
	struct slot s[MAX_TOUCH];
	int next_tid, btn;
};

typedef void (*emit_fn)(void *ctx, int type, int code, int val);

static void map_point(const struct cfg *c, const struct ft_point *p, int *ox, int *oy)
{
	int x = (int)((long)p->x * c->out_w / c->raw_w);
	int y = (int)((long)p->y * c->out_h / c->raw_h);
	if (c->swap_xy) { int t = x; x = y; y = t; }
	/* inversion applies to the output axes, after the swap */
	if (c->invert_x) x = (c->swap_xy ? c->out_h : c->out_w) - 1 - x;
	if (c->invert_y) y = (c->swap_xy ? c->out_w : c->out_h) - 1 - y;
	*ox = x; *oy = y;
}

/*
 * Feed one parsed frame into the slot tracker (n = 0 releases every contact).
 * Emits only what changed; returns 1 if a SYN_REPORT was sent.
 */
static int track(struct tracker *t, const struct cfg *c, const struct ft_point *pts,
		 int n, emit_fn emit, void *ctx)
{
	int seen[MAX_TOUCH] = {0}, dirty = 0;

	for (int i = 0; i < n; i++) {
		struct slot *s = &t->s[pts[i].id];
		int x, y;
		map_point(c, &pts[i], &x, &y);
		seen[pts[i].id] = 1;
		if (!s->active && ++s->pending < c->debounce) continue;
		if (!s->active) {
			s->active = 1;
			s->tid = t->next_tid++;
			if (t->next_tid > 65000) t->next_tid = 1;
			emit(ctx, EV_ABS, ABS_MT_SLOT, pts[i].id);
			emit(ctx, EV_ABS, ABS_MT_TRACKING_ID, s->tid);
			emit(ctx, EV_ABS, ABS_MT_POSITION_X, x);
			emit(ctx, EV_ABS, ABS_MT_POSITION_Y, y);
			dirty = 1;
		} else if (x != s->x || y != s->y) {
			emit(ctx, EV_ABS, ABS_MT_SLOT, pts[i].id);
			if (x != s->x) emit(ctx, EV_ABS, ABS_MT_POSITION_X, x);
			if (y != s->y) emit(ctx, EV_ABS, ABS_MT_POSITION_Y, y);
			dirty = 1;
		}
		s->x = x; s->y = y;
	}
	for (int i = 0; i < MAX_TOUCH; i++) {
		struct slot *s = &t->s[i];
		if (seen[i]) continue;
		s->pending = 0;
		if (!s->active) continue;
		s->active = 0;
		emit(ctx, EV_ABS, ABS_MT_SLOT, i);
		emit(ctx, EV_ABS, ABS_MT_TRACKING_ID, -1);
		dirty = 1;
	}
	if (!dirty) return 0;

	/* single-touch emulation follows the lowest active slot */
	int first = -1;
	for (int i = 0; i < MAX_TOUCH; i++)
		if (t->s[i].active) { first = i; break; }
	if ((first >= 0) != t->btn) {
		t->btn = first >= 0;
		emit(ctx, EV_KEY, BTN_TOUCH, t->btn);
	}
	if (first >= 0) {
		emit(ctx, EV_ABS, ABS_X, t->s[first].x);
		emit(ctx, EV_ABS, ABS_Y, t->s[first].y);
	}
	emit(ctx, EV_SYN, SYN_REPORT, 0);
	return 1;
}

#ifndef FT5406_NO_MAIN

static int ft_addr = 0x38;
static volatile sig_atomic_t stop;

static void on_signal(int sig) { (void)sig; stop = 1; }

static int i2c_read_block(int fd, uint8_t reg, uint8_t *buf, int len) {
	struct i2c_msg msgs[2] = {
		{ .addr = ft_addr, .flags = 0,        .len = 1,   .buf = &reg },
		{ .addr = ft_addr, .flags = I2C_M_RD, .len = len, .buf = buf  },
	};
	struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
	return ioctl(fd, I2C_RDWR, &xfer);
}

static void dump(const char *tag, const uint8_t *buf, int len) {
	fprintf(stderr, "%s", tag);
	for (int i = 0; i < len; i++) fprintf(stderr, " %02x", buf[i]);
	fputc('\n', stderr);
}

static int probe(int fd, const struct cfg *c) {
	static const struct { uint8_t reg; const char *name; } regs[] = {
		{ 0x80, "ID_G_THGROUP (touch threshold)" },
		{ 0x86, "ID_G_CTRL (0=keep active, 1=auto monitor)" },
		{ 0x88, "ID_G_PERIODACTIVE (report rate)" },
		{ 0xA3, "ID_G_CIPHER (chip ID)" },
		{ 0xA5, "ID_G_PMODE (power mode)" },
		{ 0xA6, "ID_G_FIRMID (firmware)" },
		{ 0xA8, "ID_G_FT5201ID (vendor, FocalTech=0x11)" },
	};
	uint8_t buf[FT_FRAME_LEN];
	int rc = 0;
	for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
		if (i2c_read_block(fd, regs[i].reg, buf, 1) < 0) {
			fprintf(stderr, "0x%02x %-44s read error: %s\n", regs[i].reg, regs[i].name, strerror(errno));
			rc = 1;
		} else {
			fprintf(stderr, "0x%02x %-44s 0x%02x\n", regs[i].reg, regs[i].name, buf[0]);
		}
	}
	fprintf(stderr, "\n10 raw frames (0x00..0x%02x), 100 ms apart:\n", FT_FRAME_LEN - 1);
	for (int i = 0; i < 10; i++) {
		if (i2c_read_block(fd, 0x00, buf, FT_FRAME_LEN) < 0) {
			fprintf(stderr, "read error: %s\n", strerror(errno));
			rc = 1;
		} else {
			struct ft_point pts[MAX_TOUCH];
			int n = ft_parse(buf, c->raw_w, c->raw_h, pts);
			dump(n < 0 ? "INVALID" : "valid  ", buf, FT_FRAME_LEN);
		}
		usleep(100000);
	}
	return rc;
}

static void setabs(int fd, int code, int max, int res) {
	struct uinput_abs_setup a; memset(&a, 0, sizeof(a));
	a.code = code; a.absinfo.minimum = 0; a.absinfo.maximum = max;
	a.absinfo.resolution = res;
	ioctl(fd, UI_ABS_SETUP, &a);
}

static int uinput_open(int xmax, int ymax, int res) {
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

	setabs(fd, ABS_X, xmax, res);
	setabs(fd, ABS_Y, ymax, res);
	setabs(fd, ABS_MT_POSITION_X, xmax, res);
	setabs(fd, ABS_MT_POSITION_Y, ymax, res);
	setabs(fd, ABS_MT_SLOT, MAX_TOUCH - 1, 0);
	setabs(fd, ABS_MT_TRACKING_ID, 65535, 0);

	struct uinput_setup us; memset(&us, 0, sizeof(us));
	us.id.bustype = BUS_I2C; us.id.version = 2;
	strncpy(us.name, "ft5406-userspace-touch", UINPUT_MAX_NAME_SIZE - 1);
	if (ioctl(fd, UI_DEV_SETUP, &us) < 0) { perror("UI_DEV_SETUP"); close(fd); return -1; }
	if (ioctl(fd, UI_DEV_CREATE) < 0)     { perror("UI_DEV_CREATE"); close(fd); return -1; }
	return fd;
}

static void emit_uinput(void *ctx, int type, int code, int val) {
	struct input_event ev; memset(&ev, 0, sizeof(ev));
	ev.type = type; ev.code = code; ev.value = val;
	if (write(*(int *)ctx, &ev, sizeof(ev)) != sizeof(ev)) { /* ignore */ }
}

static void timespec_add_ns(struct timespec *t, long ns) {
	t->tv_nsec += ns;
	while (t->tv_nsec >= 1000000000L) { t->tv_nsec -= 1000000000L; t->tv_sec++; }
}

int main(int argc, char **argv) {
	const char *bus = "/dev/i2c-2";
	struct cfg c = { .out_w = 800, .out_h = 480, .debounce = 2 };
	int hz = 60, res = 7, verbose = 0, do_probe = 0, opt;
	while ((opt = getopt(argc, argv, "b:a:x:y:W:H:r:R:d:sXYvp")) != -1) {
		switch (opt) {
		case 'b': bus = optarg; break;
		case 'a': ft_addr = (int)strtol(optarg, NULL, 0); break;
		case 'x': c.out_w = atoi(optarg); break;
		case 'y': c.out_h = atoi(optarg); break;
		case 'W': c.raw_w = atoi(optarg); break;
		case 'H': c.raw_h = atoi(optarg); break;
		case 'r': hz = atoi(optarg); break;
		case 'R': res = atoi(optarg); break;
		case 'd': c.debounce = atoi(optarg); break;
		case 's': c.swap_xy = 1; break;
		case 'X': c.invert_x = 1; break;
		case 'Y': c.invert_y = 1; break;
		case 'v': verbose = 1; break;
		case 'p': do_probe = 1; break;
		default:
			fprintf(stderr, "usage: %s [-b bus] [-a addr] [-x W] [-y H] [-W rawW] [-H rawH] [-r hz]\n"
					"          [-R units/mm] [-d debounce] [-s] [-X] [-Y] [-v] [-p]\n", argv[0]);
			return 1;
		}
	}
	if (!c.raw_w) c.raw_w = c.out_w;
	if (!c.raw_h) c.raw_h = c.out_h;
	if (c.out_w < 2 || c.out_h < 2 || c.raw_w < 2 || c.raw_h < 2 || c.raw_w > 4096 || c.raw_h > 4096) {
		fprintf(stderr, "invalid resolution\n"); return 1;
	}
	if (hz < 1 || hz > 200) { fprintf(stderr, "poll rate must be 1..200 Hz\n"); return 1; }
	if (c.debounce < 1) c.debounce = 1;
	if (res < 0) res = 0;

	int i2c = open(bus, O_RDWR);
	if (i2c < 0) { perror("open i2c bus"); return 1; }
	/* I2C_RDWR bypasses the address-busy check, so test it explicitly: a kernel
	 * driver bound to this address would read the chip concurrently. */
	if (ioctl(i2c, I2C_SLAVE, ft_addr) < 0) {
		if (errno == EBUSY)
			fprintf(stderr, "address 0x%02x on %s is claimed by a kernel driver; "
					"unbind it (or remove its DT node) first\n", ft_addr, bus);
		else
			perror("I2C_SLAVE");
		close(i2c); return 1;
	}
	if (do_probe) { int rc = probe(i2c, &c); close(i2c); return rc; }

	int xmax = (c.swap_xy ? c.out_h : c.out_w) - 1;
	int ymax = (c.swap_xy ? c.out_w : c.out_h) - 1;
	int ui = uinput_open(xmax, ymax, res);
	if (ui < 0) { close(i2c); return 1; }

	struct sigaction sa; memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_signal;
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);

	struct tracker t; memset(&t, 0, sizeof(t)); t.next_tid = 1;
	struct ft_point pts[MAX_TOUCH];
	uint8_t buf[FT_FRAME_LEN];
	long period = 1000000000L / hz;
	int bad = 0;
	unsigned long n_ioerr = 0, n_invalid = 0;
	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	while (!stop) {
		int n;
		if (i2c_read_block(i2c, 0x00, buf, FT_FRAME_LEN) < 0) {
			/* FT5406 dislikes very frequent polling -> EREMOTEIO(-6); retry */
			n_ioerr++;
			if ((n_ioerr & (n_ioerr - 1)) == 0)             /* log 1, 2, 4, 8, ... */
				fprintf(stderr, "i2c read errors so far: %lu (last: %s)\n", n_ioerr, strerror(errno));
			n = -1;
		} else {
			n = ft_parse(buf, c.raw_w, c.raw_h, pts);
			if (n < 0) {
				n_invalid++;
				if (verbose) dump("invalid frame:", buf, FT_FRAME_LEN);
				else if ((n_invalid & (n_invalid - 1)) == 0)
					fprintf(stderr, "discarded %lu invalid frame(s) so far (-v to dump)\n", n_invalid);
			}
		}
		if (n >= 0) {
			bad = 0;
			track(&t, &c, pts, n, emit_uinput, &ui);
		} else if (++bad == MAX_BAD) {
			/* controller unreadable or returning garbage: don't leave a stuck touch */
			track(&t, &c, pts, 0, emit_uinput, &ui);
		}

		timespec_add_ns(&next, period);
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		if (now.tv_sec > next.tv_sec + 1) next = now;   /* resync after a stall */
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
	}

	track(&t, &c, pts, 0, emit_uinput, &ui);
	ioctl(ui, UI_DEV_DESTROY);
	close(ui);
	close(i2c);
	return 0;
}

#endif /* FT5406_NO_MAIN */
