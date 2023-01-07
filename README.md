DISCONTINUATION OF PROJECT

This project will no longer be maintained by Intel.

Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project.  

Intel no longer accepts patches to this project.

If you have an ongoing need to use this project, are interested in independently developing it, or would like to maintain patches for the open source software community, please create your own fork of this project.  

Contact: webadmin@linux.intel.com
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

* `host`: produce an `efiwrapper_host` host executable that can run an
  EFI binary. See `Run an EFI binary on host` section.

* efiwrapper: library that provides the `main()` entry point which
  initialize the libefiwrapper library and all the drivers before
  calling the `efi_main(EFI_HANDLE image, EFI_SYSTEM_TABLE *table)`
  function.

Run an EFI binary on host
-------------------------

To build `efiwrapper_host`, run the following command in your Android
build environment:

``` bash
$ make efiwrapper_host-<BUILD_VARIANT>
```
Where `<BUILD_VARIANT>` is either `user`, `userdebug` or `eng`.

``` bash
$ efiwrapper_host --help
Usage: efiwrapper_host [OPTIONS] <EFI binary> [ARGS]
 OPTIONS:
 -h,--help                      Print this help
 --list-drivers                 List available drivers
 --disable-drivers=DRV1,DRV2    Disable drivers DRV1 and DRV2
```

The `efiwrapper_host` has built-in drivers:
``` bash
$ efiwrapper_host --list-drivers
Drivers list:
- disk: Emulate eMMC storage
- event: Event management for host
- tcp4: TCP/IP protocol
- fileio: File System Protocol support
- gop: Graphics Output Protocol support based on Xlib
- image: PE/COFF image
```

Drivers can be independently deactivated.  For instance, if you want to
run Kernelflinger EFI binary witout the Graphic Output Protocol support:

``` bash
$ efiwrapper_host --disable-drivers=gop kernelflinger.efi -f
```

Dependencies
------------
* gnu-efi: libefiwrapper and efiwrapper libraries depends on the
  gnu-efi library for EFI types definitions.

* drivers/*: most of these drivers depends on the `libpayload` library
  from the [Coreboot](https://www.coreboot.org/) project.

Copyright and Licence
---------------------
EfiWrapper is licensed under the terms of the BSD 2-Clause.
