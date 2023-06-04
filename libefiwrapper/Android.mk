LOCAL_PATH := $(call my-dir)

LIBEFIWRAPPER_SRC_FILES := \
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
	ewacpi.c \
	ewarg.c \
	sdio.c \
	ewlib.c \
	eraseblk.c

include $(CLEAR_VARS)
LOCAL_MODULE := libefiwrapper-$(TARGET_BUILD_VARIANT)
LOCAL_STATIC_LIBRARIES := \
	libgnuefi \
	libefi
LOCAL_SRC_FILES := $(LIBEFIWRAPPER_SRC_FILES)
LOCAL_CFLAGS := $(EFIWRAPPER_CFLAGS)
LOCAL_C_INCLUDES := $(LOCAL_PATH)/../include/libefiwrapper \
	$(LOCAL_PATH)/../../external/gnu-efi/gnu-efi-3.0/inc \
	$(LOCAL_PATH)/../../external/gnu-efi/gnu-efi-3.0/inc/$(TARGET_EFI_ARCH_NAME) \
	$(LOCAL_PATH)/../../external/gnu-efi/gnu-efi-3.0/inc/protocol
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/../include/libefiwrapper
include $(BUILD_IAFW_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libefiwrapper_host-$(TARGET_BUILD_VARIANT)
LOCAL_SRC_FILES := $(LIBEFIWRAPPER_SRC_FILES)
LOCAL_CFLAGS := $(EFIWRAPPER_HOST_CFLAGS)
LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/../include/libefiwrapper \
	$(EFIWRAPPER_HOST_C_INCLUDES)
LOCAL_MODULE_HOST_ARCH := $(EFIWRAPPER_HOST_ARCH)
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/../include/libefiwrapper
include $(BUILD_HOST_STATIC_LIBRARY)
