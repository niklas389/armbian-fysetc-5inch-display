/*
 * Host-side tests for the FT5406 frame parser and slot tracker.
 * Build/run: make test
 */
#define FT5406_NO_MAIN
#include "ft5406_touch.c"

#include <assert.h>

#define MAX_EV 64
struct evlog { int n; struct { int type, code, val; } e[MAX_EV]; };

static void rec(void *ctx, int type, int code, int val)
{
	struct evlog *l = ctx;
	assert(l->n < MAX_EV);
	l->e[l->n].type = type; l->e[l->n].code = code; l->e[l->n].val = val;
	l->n++;
}

static int count(const struct evlog *l, int type, int code, int val)
{
	int c = 0;
	for (int i = 0; i < l->n; i++)
		if (l->e[i].type == type && l->e[i].code == code && l->e[i].val == val) c++;
	return c;
}

static int has_code(const struct evlog *l, int type, int code)
{
	for (int i = 0; i < l->n; i++)
		if (l->e[i].type == type && l->e[i].code == code) return 1;
	return 0;
}

/* frame with n points: {evt, id, x, y} */
static void mkframe(uint8_t *buf, int n, const int (*p)[4])
{
	memset(buf, 0xFF, FT_FRAME_LEN);
	buf[0] = 0; buf[1] = 0; buf[2] = n;
	for (int i = 0; i < n; i++) {
		uint8_t *r = &buf[FT_REG_P1 + i * 6];
		r[0] = (p[i][0] << 6) | ((p[i][2] >> 8) & 0x0F);
		r[1] = p[i][2] & 0xFF;
		r[2] = (p[i][1] << 4) | ((p[i][3] >> 8) & 0x0F);
		r[3] = p[i][3] & 0xFF;
	}
}

static const struct cfg C = { .raw_w = 800, .raw_h = 480, .out_w = 800, .out_h = 480, .debounce = 2 };

static void test_parse(void)
{
	uint8_t b[FT_FRAME_LEN];
	struct ft_point pts[MAX_TOUCH];

	mkframe(b, 0, NULL);
	assert(ft_parse(b, 800, 480, pts) == 0);

	const int one[][4] = { { EVT_CONTACT, 0, 100, 200 } };
	mkframe(b, 1, one);
	assert(ft_parse(b, 800, 480, pts) == 1);
	assert(pts[0].id == 0 && pts[0].x == 100 && pts[0].y == 200);

	/* lift-off and empty records produce no point */
	const int up[][4] = { { EVT_UP, 0, 100, 200 }, { EVT_NONE, 0xF, 0xFFF, 0xFFF } };
	mkframe(b, 2, up);
	assert(ft_parse(b, 800, 480, pts) == 0);

	/* out of range, duplicate ID, bad mode, reserved TD_STATUS bits, > 10 points */
	const int oor[][4] = { { EVT_CONTACT, 4, 2049, 1800 } };
	mkframe(b, 1, oor);
	assert(ft_parse(b, 800, 480, pts) == -1);
	const int dup[][4] = { { EVT_CONTACT, 1, 10, 10 }, { EVT_CONTACT, 1, 20, 20 } };
	mkframe(b, 2, dup);
	assert(ft_parse(b, 800, 480, pts) == -1);
	mkframe(b, 1, one); b[0] = 0x40;
	assert(ft_parse(b, 800, 480, pts) == -1);
	mkframe(b, 1, one); b[2] = 0x41;
	assert(ft_parse(b, 800, 480, pts) == -1);
	mkframe(b, 0, NULL); b[2] = 11;
	assert(ft_parse(b, 800, 480, pts) == -1);

	/* the phantom stream from the field log: repeating 08 01 47, 1-byte phase shift */
	static const uint8_t pat[3] = { 0x08, 0x01, 0x47 };
	for (int phase = 0; phase < 3; phase++) {
		for (int i = 0; i < FT_FRAME_LEN; i++) b[i] = pat[(i + phase) % 3];
		assert(ft_parse(b, 800, 480, pts) == -1);
	}
}

static void test_tap_and_hold(void)
{
	struct tracker t; memset(&t, 0, sizeof(t)); t.next_tid = 1;
	struct evlog l;
	struct ft_point p = { 0, 100, 200 };

	/* frame 1: pending (debounce), nothing emitted */
	l.n = 0; assert(track(&t, &C, &p, 1, rec, &l) == 0 && l.n == 0);
	/* frame 2: down */
	l.n = 0; assert(track(&t, &C, &p, 1, rec, &l) == 1);
	assert(count(&l, EV_ABS, ABS_MT_TRACKING_ID, 1) == 1);
	assert(count(&l, EV_KEY, BTN_TOUCH, 1) == 1);
	/* frames 3..10: same position, no events at all, no TRACKING_ID change */
	for (int i = 0; i < 8; i++) { l.n = 0; assert(track(&t, &C, &p, 1, rec, &l) == 0); }
	/* move: position only */
	p.x = 110; l.n = 0; assert(track(&t, &C, &p, 1, rec, &l) == 1);
	assert(!has_code(&l, EV_ABS, ABS_MT_TRACKING_ID));
	assert(count(&l, EV_ABS, ABS_MT_POSITION_X, 110) == 1);
	assert(!has_code(&l, EV_ABS, ABS_MT_POSITION_Y));
	/* lift */
	l.n = 0; assert(track(&t, &C, NULL, 0, rec, &l) == 1);
	assert(count(&l, EV_ABS, ABS_MT_TRACKING_ID, -1) == 1);
	assert(count(&l, EV_KEY, BTN_TOUCH, 0) == 1);
}

static void test_alternating_ids_filtered(void)
{
	/* two IDs that alternate each frame never pass the debounce */
	struct tracker t; memset(&t, 0, sizeof(t)); t.next_tid = 1;
	struct evlog l;
	for (int i = 0; i < 20; i++) {
		struct ft_point p = { (i & 1) ? 0 : 4, 300, 100 };
		l.n = 0; assert(track(&t, &C, &p, 1, rec, &l) == 0);
	}
}

static void test_transform(void)
{
	struct cfg c = C;
	struct ft_point p = { 0, 0, 0 };
	int x, y;
	c.swap_xy = 1; c.invert_x = 1;
	map_point(&c, &p, &x, &y);
	assert(x == 479 && y == 0);
	c = C; c.raw_w = 1600; c.raw_h = 960;
	p.x = 1599; p.y = 959;
	map_point(&c, &p, &x, &y);
	assert(x == 799 && y == 479);
}

int main(void)
{
	test_parse();
	test_tap_and_hold();
	test_alternating_ids_filtered();
	test_transform();
	puts("all tests passed");
	return 0;
}
