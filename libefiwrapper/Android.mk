LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libefiwrapper-$(TARGET_BUILD_VARIANT)
LOCAL_STATIC_LIBRARIES := \
	libgnuefi \
	libefi
LOCAL_SRC_FILES := \
	ewvar.c \
	ewdrv.c \
	protocol.c \
	core.c \
	lib.c \
	bs.c \
	rs.c \
	conin.c \
	conout.c \
	serialio.c \
	storage.c \
	blockio.c \
	diskio.c \
	interface.c \
	media.c \
	conf_table.c \
	smbios.c \
	ewacpi.c
LOCAL_CFLAGS := $(EFIWRAPPER_CFLAGS)
LOCAL_C_INCLUDES := $(LOCAL_PATH)/../include/libefiwrapper
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/../include/libefiwrapper
include $(BUILD_IAFW_STATIC_LIBRARY)
