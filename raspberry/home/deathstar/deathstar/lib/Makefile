CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c11 -pthread -D_GNU_SOURCE
LIBS = -lgpiod -lm

# GTK4 flags and libraries
GTK_FLAGS = $(shell pkg-config --cflags gtk4)
GTK_LIBS  = $(shell pkg-config --libs gtk4)

LIB = librobot.a
OBJS = esc.o hall.o gpio_outputs.o

all: $(LIB)

$(LIB): $(OBJS)
	ar rcs $@ $^

esc.o: esc.c esc.h
hall.o: hall.c hall.h
gpio_outputs.o: gpio_outputs.c gpio_outputs.h

gui: gui_example.c $(LIB)
	$(CC) $(CFLAGS) $(GTK_FLAGS) -o gui gui_example.c $(LIB) $(LIBS) $(GTK_LIBS)

example: example.c $(LIB)
	$(CC) $(CFLAGS) -o example example.c $(LIB) $(LIBS)

cli: cli.c $(LIB)
	$(CC) $(CFLAGS) -o cli cli.c $(LIB) $(LIBS)

clean: rm -f $(OBJS) $(LIB) example cli gui

.PHONY: all clean example cli gui