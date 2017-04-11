SRC_DIR := .
include $(SRC_DIR)/Make.defaults

SUB_DIRS := libefiwrapper host
define submake
	$(foreach d,$(SUB_DIRS),$(MAKE) -C $(d) $(1);)
endef

efiwrapper_host-eng:       export EXTRA_CFLAGS := -DHOST
efiwrapper_host-userdebug: export EXTRA_CFLAGS := -DHOST -DUSERDEBUG
efiwrapper_host-user:      export EXTRA_CFLAGS := -DHOST -DUSERDEBUG -DUSER

efiwrapper_host-eng \
efiwrapper_host-userdebug \
efiwrapper_host-user:
	$(eval export TARGET_BUILD_VARIANT := $(subst efiwrapper_host-,,$@))
	@$(MAKE) efiwrapper_host

efiwrapper_host: $(EW_LIB)
	@$(MAKE) -C host

$(EW_LIB):
	@$(MAKE) -C libefiwrapper

.PHONY: clean
clean:
	@$(call submake,clean)

mrproper: clean
	@$(call submake,mrproper)
