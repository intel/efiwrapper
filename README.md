EfiWrapper
==========

Overview
--------

EfiWrapper is a library which simulate a UEFI firmware
implementation. Its first purpose is to run a subset of the
[Kernelflinger](https://github.com/01org/kernelflinger/) OS loader to
run in a non-UEFI environment.

Basic architecture
------------------
* libefiwrapper: library that provides a basic implementation of the
  Boot services and Runtime services.  It includes basic EFI variable
  management (no storage) and serial IO protocol support. It also
  provides an abstraction for storage class implementation.

* libefiwrapper\_drivers: library including all the protocols
  specified by the `LIBEFIWRAPPER_DRIVERS` Makefile variable.  Drivers
  are in the drivers/DRIVER\_NAME directories and might rely on
  external libraries like the `libpayload` from the
  [Coreboot](https://www.coreboot.org/) project.

* efiwrapper: library that provides the `main()` function which
  initialize the libefiwrapper library and all the drivers before
  calling the `efi_main()` function.
