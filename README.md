# GDIP C Source Code
To build, please first navigate to `libopencm3/` and run `git submodule init`. Then run `make` in the same directory. Next, navigate to `bootloader/` and run make. Finally, navigate to `app/` and run `make`. 

This code was written on a POSIX compliant system (Linux). No modifications should be required for it to work on MacOS or other linux distros. You may need modification to run on a Windows machine without using a tool such as Cygwin.

This code relies on `libopencm3` as a HAL. Please ensure you have the correct build tools (`arm-gcc` toolchain, make, etc) before building. This code targets an STM32F411RE chip.
