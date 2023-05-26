SRC_DIR := ..
include $(SRC_DIR)/Make.defaults

CFLAGS += -DPRODUCT_MANUFACTURER=\"$(PRODUCT_MANUFACTURER)\" \
	  -DPRODUCT_NAME=\"$(PRODUCT_NAME)\"

OBJS := ewvar.o \
	ewdrv.o \
	protocol.o \
	core.o \
	lib.o \
	bs.o \
	rs.o \
	conin.o \
	conout.o \
	serialio.o \
	storage.o \
	blockio.o \
	diskio.o \
	interface.o \
	media.o \
	conf_table.o \
	smbios.o \
	ewacpi.o \
	ewarg.o \
	sdio.o \
	ewlib.o \
	eraseblk.o \
	image.o \
	pe.o

$(EW_LIB): $(OBJS)
	$(AR) rcs $@ $^

.PHONY: clean
clean:
	@rm -f $(OBJS) *~

mrproper: clean
	@rm -f libefiwrapper_host-*.a
