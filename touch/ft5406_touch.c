/*
 * ft5406_touch.c — userspace polled touch driver for the Fysetc 5" DSI panel
 *
 * For BigTreeTech Pi2 / CB2 with the Fysetc 5" RPi-DSI-clone panel.
 * The touch device at I2C2 addr 0x38 has NO interrupt line broken out on the
 * 15-pin DSI FFC, and mainline edt-ft5x06 has no polling mode, so this polls
 * it over /dev/i2c-2 and injects a standard multitouch-B device via
 * /dev/uinput. No kernel module, survives kernel upgrades.
 *
 * The device is not a real FT5406: an MCU on the panel emulates the FT5x06
 * register map, but multi-byte reads do not follow it (behaviour documented
 * by https://github.com/kkobold/fysetc-5inch-dsi-touch, confirmed on this
 * board):
 *   - the register selects a record: 0x02 TD_STATUS, 0x03 + 6*i point i;
 *   - only the first byte read comes from that record (TD_STATUS or XH);
 *   - every further byte comes from a 3-entry ring (XL, YH, YL) whose read
 *     pointer advances per byte and is never reset (not by STOP, not by a
 *     new register address).
 * So TD_STATUS is read with 1-byte reads and each point with exactly 4 bytes:
 * 3 ring bytes per read keep the ring phase constant. The phase is found from
 * the data (ID and coordinates must be plausible) and re-checked every frame.
 *
 * Every frame is validated before use: reserved TD_STATUS bits, more points
 * than supported, duplicate IDs or out-of-range coordinates at every ring
 * phase discard the frame. A new contact must be seen in <debounce>
 * consecutive valid frames before it is reported.
 *
 * Build:  gcc -O2 -Wall -o ft5406-touch ft5406_touch.c
 * Run:    ./ft5406-touch -b /dev/i2c-2 -x 800 -y 480 -r 60 -X -Y
 * Flags:  -s swap X/Y   -X invert X   -Y invert Y   -r poll Hz   -a addr
 *         -W/-H controller raw range (default = -x/-y, scaled to -x/-y)
 *         -R resolution in units/mm   -d debounce frames   -v verbose
 *         -p probe: print raw records and decoded points, then exit
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

#define MAX_TOUCH    5                    /* points the panel MCU reports */
#define FT_MAX_POINTS 10                  /* TD_STATUS upper bound (FT5x06) */
#define FT_REG_TD    0x02
#define FT_REG_P1    0x03
#define FT_STRIDE    6                    /* register distance between points */
#define FT_REC_LEN   4                    /* XH + 3 ring bytes */
#define FT_RING      3
#define MAX_BAD      5                    /* bad frames before releasing all */

enum { EVT_DOWN = 0, EVT_UP = 1, EVT_CONTACT = 2, EVT_NONE = 3 };

struct ft_point { int id, x, y; };

/* Number of point records announced by TD_STATUS, or -1 if implausible. */
static int ft_npoints(uint8_t td)
{
	if (td & 0xF0) return -1;               /* reserved bits set */
	int n = td & 0x0F;
	if (n > FT_MAX_POINTS) return -1;
	return n > MAX_TOUCH ? MAX_TOUCH : n;
}

/* rec = XH followed by 3 ring bytes; phase = position of XL among them */
static void ft_decode(const uint8_t *rec, int phase, int *evt, int *id, int *x, int *y)
{
	uint8_t xh = rec[0];
	uint8_t xl = rec[1 + phase % FT_RING];
	uint8_t yh = rec[1 + (phase + 1) % FT_RING];
	uint8_t yl = rec[1 + (phase + 2) % FT_RING];
	*evt = (xh >> 6) & 0x03;
	*id = (yh >> 4) & 0x0F;
	*x = ((xh & 0x0F) << 8) | xl;
	*y = ((yh & 0x0F) << 8) | yl;
}

/* Decode n records at one ring phase. Returns points written, or -1. */
static int ft_decode_frame(const uint8_t (*rec)[FT_REC_LEN], int n, int phase,
			   int raw_w, int raw_h, struct ft_point *pts)
{
	int cnt = 0;
	unsigned seen = 0;
	for (int i = 0; i < n; i++) {
		int evt, id, x, y;
		ft_decode(rec[i], phase, &evt, &id, &x, &y);
		if (evt == EVT_NONE) continue;          /* empty record */
		if (id >= MAX_TOUCH || x >= raw_w || y >= raw_h) return -1;
		if (seen & (1u << id)) return -1;       /* duplicate ID */
		seen |= 1u << id;
		if (evt == EVT_UP) continue;
		pts[cnt].id = id; pts[cnt].x = x; pts[cnt].y = y;
		cnt++;
	}
	return cnt;
}

/*
 * Parse one frame: TD_STATUS and the records it announced. The current ring
 * phase is tried first; otherwise *phase moves to the one other phase that
 * fits. Returns the number of points written to pts, or -1 if the frame is
 * implausible or ambiguous.
 */
static int ft_parse(uint8_t td, const uint8_t (*rec)[FT_REC_LEN], int *phase,
		    int raw_w, int raw_h, struct ft_point *pts)
{
	int n = ft_npoints(td);
	if (n <= 0) return n;
	int cnt = ft_decode_frame(rec, n, *phase, raw_w, raw_h, pts);
	if (cnt >= 0) return cnt;

	int found = -1;
	for (int p = 0; p < FT_RING; p++) {
		if (p == *phase || ft_decode_frame(rec, n, p, raw_w, raw_h, pts) < 0) continue;
		if (found >= 0) return -1;              /* ambiguous */
		found = p;
	}
	if (found < 0) return -1;
	*phase = found;
	return ft_decode_frame(rec, n, found, raw_w, raw_h, pts);
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

/*
 * Read TD_STATUS (1 byte, keeps the ring still) and the point records it
 * announces (4 bytes each, ring phase unchanged). Returns the number of
 * records read, -1 on an I2C error, -2 if TD_STATUS is implausible.
 */
static int read_frame(int fd, uint8_t *td, uint8_t (*rec)[FT_REC_LEN]) {
	if (i2c_read_block(fd, FT_REG_TD, td, 1) < 0) return -1;
	int n = ft_npoints(*td);
	if (n < 0) return -2;
	for (int i = 0; i < n; i++)
		if (i2c_read_block(fd, FT_REG_P1 + i * FT_STRIDE, rec[i], FT_REC_LEN) < 0) return -1;
	return n;
}

static void dump_frame(const char *tag, uint8_t td, const uint8_t (*rec)[FT_REC_LEN], int n) {
	fprintf(stderr, "%s TD_STATUS %02x", tag, td);
	for (int i = 0; i < n; i++)
		fprintf(stderr, " | %02x %02x %02x %02x", rec[i][0], rec[i][1], rec[i][2], rec[i][3]);
	fputc('\n', stderr);
}

static int probe(int fd, const struct cfg *c) {
	uint8_t td, rec[MAX_TOUCH][FT_REC_LEN];
	struct ft_point pts[MAX_TOUCH];
	int phase = 0, rc = 0;
	fprintf(stderr, "20 frames, 200 ms apart (touch the panel meanwhile):\n");
	for (int f = 0; f < 20; f++) {
		int n = read_frame(fd, &td, rec);
		if (n == -1) {
			fprintf(stderr, "read error: %s\n", strerror(errno));
			rc = 1;
		} else if (n == -2) {
			fprintf(stderr, "INVALID TD_STATUS %02x\n", td);
		} else {
			int cnt = ft_parse(td, (const uint8_t (*)[FT_REC_LEN])rec, &phase, c->raw_w, c->raw_h, pts);
			dump_frame(cnt < 0 ? "INVALID" : "valid  ", td, (const uint8_t (*)[FT_REC_LEN])rec, n);
			for (int i = 0; i < cnt; i++)
				fprintf(stderr, "        ring phase %d: id %d x %d y %d\n", phase, pts[i].id, pts[i].x, pts[i].y);
		}
		usleep(200000);
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
	uint8_t td, rec[MAX_TOUCH][FT_REC_LEN];
	long period = 1000000000L / hz;
	int bad = 0, phase = 0;
	unsigned long n_ioerr = 0, n_invalid = 0;
	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	while (!stop) {
		int n = read_frame(i2c, &td, rec);
		if (n == -1) {
			/* the panel MCU dislikes very frequent polling -> EREMOTEIO(-6); retry */
			n_ioerr++;
			if ((n_ioerr & (n_ioerr - 1)) == 0)             /* log 1, 2, 4, 8, ... */
				fprintf(stderr, "i2c read errors so far: %lu (last: %s)\n", n_ioerr, strerror(errno));
		} else {
			int old_phase = phase;
			if (n >= 0) n = ft_parse(td, (const uint8_t (*)[FT_REC_LEN])rec, &phase, c.raw_w, c.raw_h, pts);
			if (phase != old_phase)
				fprintf(stderr, "ring phase %d -> %d\n", old_phase, phase);
			if (n < 0) {
				n_invalid++;
				if (verbose) dump_frame("invalid frame:", td, (const uint8_t (*)[FT_REC_LEN])rec,
							ft_npoints(td) < 0 ? 0 : ft_npoints(td));
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
