CC      = gcc
CFLAGS  = -std=c11 -Wall -Wextra -I include/ -g -D_DEFAULT_SOURCE
LDFLAGS = -lrt -lpthread -lm

COMMON_SRCS = src/lidar_driver.c src/occupancy_grid.c src/pose_stub.c

test_lidar: src/lidar_driver.c test/test_lidar.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_grid: $(COMMON_SRCS) test/test_grid.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

map_main: $(COMMON_SRCS) src/map_main.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f test_lidar test_grid map_main

.PHONY: clean
