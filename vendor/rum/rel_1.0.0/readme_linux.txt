Building RUM under Linux

RUM was developed both under Windows XP using AVR Studio, and under
Linux using various open-source tools.

To build RUM in Linux, you need an AVR toolchain composed of the
following packages, all built for AVR:

* binutils
* gcc
* avr-libc
* gdb
* avrdude
* avarice

Avrdude can program the microcontrollers of choice using
various programming tools.  Avarice can both program and debug the
Raven boards.

To build the toolchain, follow the directions on the avr-libc website:

http://www.nongnu.org/avr-libc/user-manual/install_tools.html

In addition to the instructions from the avr-libc website, you should
also patch the packages with patches from the WinAVR source repository
at

http://winavr.cvs.sourceforge.net/winavr/patches/

For each package, there are some patches for a specific version of
that package.  It is advisable to use the version of each package that
corresponds to the patches.  For Gnu/Linux, don't use patches starting
with 0x- or 1x-, as these are Windows-specific patches.

Alternatively, you can create a toolchain using various build scripts
available on the internet.  AvrFreaks.net has published a few scripts
for completely building the GCC toolchain for AVR.

It is possible to debug this code under Linux using avr-gdb and a
graphical debugger.  Possible choices are DDD, Inisight, and Kdevelop,
among others.
