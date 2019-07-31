SRC_DIR := ..
include $(SRC_DIR)/Make.defaults

OBJS := main.o \
	event.o \
	disk.o \
	fifo.o \
	worker.o \
	tcp4.o \
	fileio.o \
	gop.o \
	image.o \
	pe.o \
	host_time.o \
	terminal_curses_conin.o \
	terminal_curses_conout.o \
	terminal_curses.o

LDFLAGS := -lX11 -lpthread -lncurses

efiwrapper_host-$(TARGET_BUILD_VARIANT): $(OBJS) $(EW_LIB)
	$(CC) $(CFLAGS) $(GNU_EFI_INCS) $(LDFLAGS) $^ -o $@

.PHONY: clean
clean:
	@rm -f $(OBJS) *~

mrproper: clean
	@rm -f efiwrapper_host-*
