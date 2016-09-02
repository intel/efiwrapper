LOCAL_PATH := $(call my-dir)

# LIBEFIWRAPPER_DRIVERS must be defined for the current product with
# the list of drivers (see directories of this very directory).

include $(CLEAR_VARS)
LOCAL_MODULE := libefiwrapper_drivers-$(TARGET_BUILD_VARIANT)
LOCAL_MODULE_CLASS := STATIC_LIBRARIES
LOCAL_STATIC_LIBRARIES := \
	libpayload \
	libefiwrapper-$(TARGET_BUILD_VARIANT) \
	libgnuefi \
	libefi
LOCAL_SRC_FILES := \
	$(foreach drv, $(LIBEFIWRAPPER_DRIVERS), \
		$(subst $(LOCAL_PATH)/,,$(wildcard $(LOCAL_PATH)/$(drv)/*.c)))
intermediates := $(local-generated-sources-dir)
GEN := $(intermediates)/drivers.c
ifeq ($(LIBEFIWRAPPER_DRIVERS),)
$(GEN):
	$(hide)	echo "/* Do not modify this auto-generated file. */" > $@
	$(hide) echo "#error \"no driver selected.  Cf. LIBEFIWRAPPER_DRIVERS Makefile variable.\"" >> $@
else
$(GEN):
	$(hide) echo "/* Do not modify this auto-generated file. */" > $@
	$(hide) echo "#include \"ewdrv.h\"" >> $@
	$(hide) $(foreach drv, $(LIBEFIWRAPPER_DRIVERS), echo "\#include" \"$(drv)/$(drv).h\" >> $@;)
	$(hide) echo "" >> $@
	$(hide) echo "static ewdrv_t *drivers[] = {" >> $@
	$(hide) $(foreach drv, $(LIBEFIWRAPPER_DRIVERS), echo "&"$(drv)_drv, >> $@;)
	$(hide) echo  "NULL" >> $@
	$(hide) echo "};" >> $@
	$(hide) echo "" >> $@
	$(hide) echo "ewdrv_t **ew_drivers = drivers;" >> $@
endif
LOCAL_GENERATED_SOURCES := $(GEN)
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)
LOCAL_CFLAGS := $(EFIWRAPPER_CFLAGS)
include $(BUILD_IAFW_STATIC_LIBRARY)
