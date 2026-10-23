CC ?= gcc
CFLAGS ?= -O2 -Wall
CC_FOR_BUILD ?= cc

ft5406-touch: ft5406_touch.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) -o $@ $<

test: test_ft5406.c ft5406_touch.c
	$(CC_FOR_BUILD) -O2 -Wall -Wextra -o test_ft5406 test_ft5406.c
	./test_ft5406

clean:
	rm -f ft5406-touch test_ft5406

.PHONY: test clean
