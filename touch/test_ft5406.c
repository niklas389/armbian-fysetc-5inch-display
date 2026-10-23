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

/*
 * Emulation of the Fysetc panel MCU at 0x38: the register selects a record,
 * the first byte comes from it, further bytes from a 3-entry ring
 * (XL, YH, YL) with a global pointer that is never reset.
 */
struct mcu { uint8_t td; uint8_t rec[MAX_TOUCH][4]; int ptr; };

static void mcu_read(struct mcu *m, uint8_t reg, uint8_t *buf, int len)
{
	int r = 0;
	if (reg == FT_REG_TD) buf[0] = m->td;
	else { r = (reg - FT_REG_P1) / FT_STRIDE; buf[0] = m->rec[r][0]; }
	for (int i = 1; i < len; i++) {
		buf[i] = m->rec[r][1 + m->ptr];
		m->ptr = (m->ptr + 1) % 3;
	}
}

/* same sequence as read_frame() */
static int mcu_frame(struct mcu *m, uint8_t *td, uint8_t (*rec)[FT_REC_LEN])
{
	mcu_read(m, FT_REG_TD, td, 1);
	int n = ft_npoints(*td);
	for (int i = 0; i < n; i++) mcu_read(m, FT_REG_P1 + i * FT_STRIDE, rec[i], FT_REC_LEN);
	return n;
}

static void mcu_point(struct mcu *m, int i, int evt, int id, int x, int y)
{
	m->rec[i][0] = (evt << 6) | ((x >> 8) & 0x0F);
	m->rec[i][1] = x & 0xFF;
	m->rec[i][2] = (id << 4) | ((y >> 8) & 0x0F);
	m->rec[i][3] = y & 0xFF;
}

static const struct cfg C = { .raw_w = 800, .raw_h = 480, .out_w = 800, .out_h = 480, .debounce = 2 };

static void test_parse(void)
{
	struct ft_point pts[MAX_TOUCH];
	int phase;

	/* bytes captured on the board, finger at the screen centre: the 3 ring
	 * bytes in two rotations decode to the same point (393, 240) */
	const uint8_t cap0[1][4] = { { 0x81, 0x89, 0x00, 0xf0 } };
	const uint8_t cap2[1][4] = { { 0x81, 0x00, 0xf0, 0x89 } };
	phase = 0;
	assert(ft_parse(0x01, cap0, &phase, 800, 480, pts) == 1);
	assert(phase == 0 && pts[0].id == 0 && pts[0].x == 393 && pts[0].y == 240);
	assert(ft_parse(0x01, cap2, &phase, 800, 480, pts) == 1);
	assert(phase == 2 && pts[0].x == 393 && pts[0].y == 240);

	/* idle: no records */
	assert(ft_parse(0x00, cap0, &phase, 800, 480, pts) == 0);

	/* TD_STATUS: reserved bits, more than 10 points */
	assert(ft_npoints(0x43) == -1 && ft_npoints(0x14) == -1 && ft_npoints(11) == -1);
	assert(ft_npoints(7) == MAX_TOUCH);

	/* no phase fits: every rotation has an ID >= 5 or y >= 480 */
	const uint8_t bad[1][4] = { { 0x81, 0x77, 0x88, 0x99 } };
	phase = 1;
	assert(ft_parse(0x01, bad, &phase, 800, 480, pts) == -1 && phase == 1);

	/* lift-off and empty records produce no point */
	const uint8_t up[2][4] = { { 0x41, 0x89, 0x00, 0xf0 }, { 0xff, 0xff, 0xff, 0xff } };
	phase = 0;
	assert(ft_parse(0x02, up, &phase, 800, 480, pts) == 0);
}

static void test_mcu_ring(void)
{
	/* every starting ring position, two fingers: the phase is found once and
	 * stays put, coordinates are exact */
	for (int start = 0; start < 3; start++) {
		struct mcu m; memset(&m, 0, sizeof(m));
		m.ptr = start;
		m.td = 2;
		mcu_point(&m, 0, EVT_CONTACT, 0, 393, 240);
		mcu_point(&m, 1, EVT_CONTACT, 1, 758, 37);
		uint8_t td, rec[MAX_TOUCH][FT_REC_LEN];
		struct ft_point pts[MAX_TOUCH];
		int phase = 0, first = -1;
		for (int f = 0; f < 10; f++) {
			int n = mcu_frame(&m, &td, rec);
			assert(n == 2);
			assert(ft_parse(td, (const uint8_t (*)[FT_REC_LEN])rec, &phase, 800, 480, pts) == 2);
			if (first < 0) first = phase;
			assert(phase == first);
			assert(pts[0].x == 393 && pts[0].y == 240);
			assert(pts[1].x == 758 && pts[1].y == 37);
			/* idle frames in between don't move the ring */
			m.td = 0; assert(mcu_frame(&m, &td, rec) == 0); m.td = 2;
		}
	}

	/* a foreign multi-byte read shifts the ring: the phase is re-found */
	struct mcu m; memset(&m, 0, sizeof(m));
	m.td = 1;
	mcu_point(&m, 0, EVT_CONTACT, 0, 393, 240);
	uint8_t td, rec[MAX_TOUCH][FT_REC_LEN], junk[33];
	struct ft_point pts[MAX_TOUCH];
	int phase = 0;
	mcu_frame(&m, &td, rec);
	assert(ft_parse(td, (const uint8_t (*)[FT_REC_LEN])rec, &phase, 800, 480, pts) == 1);
	int before = phase;
	mcu_read(&m, 0x00, junk, 33);           /* 32 ring bytes: shift by 2 */
	mcu_frame(&m, &td, rec);
	assert(ft_parse(td, (const uint8_t (*)[FT_REC_LEN])rec, &phase, 800, 480, pts) == 1);
	assert(phase != before && pts[0].x == 393 && pts[0].y == 240);
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
	test_mcu_ring();
	test_tap_and_hold();
	test_alternating_ids_filtered();
	test_transform();
	puts("all tests passed");
	return 0;
}
