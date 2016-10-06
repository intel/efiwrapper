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
* `libefiwrapper`: library that provides a basic implementation of the
  Boot services and Runtime services.  It includes basic EFI variable
  management (no storage) and serial IO protocol support. It also
  provides an abstraction for storage class implementation.  This
  library is system independent and MUST be kept that way.

* `libefiwrapper_drivers`: library including all the protocols
  specified by the `LIBEFIWRAPPER_DRIVERS` Makefile variable.  Drivers
  are in the drivers/DRIVER\_NAME directories and might rely on
  external libraries like the `libpayload` from the
  [Coreboot](https://www.coreboot.org/) project.

* efiwrapper: library that provides the `main()` entry point which
  initialize the libefiwrapper library and all the drivers before
  calling the `efi_main(EFI_HANDLE image, EFI_SYSTEM_TABLE *table)`
  function.

Dependencies
------------
* gnu-efi: libefiwrapper and efiwrapper libraries depends on the
  gnu-efi library for EFI types definitions.

* drivers/*: most of these drivers depends on the `libpayload` library
  from the [Coreboot](https://www.coreboot.org/) project.

Copyright and Licence
---------------------
EfiWrapper is licensed under the terms of the BSD 2-Clause.
