Zedboard First-Stage Bootloader (FSBL)
===========

Simple FSBL for the Zedboard. It is currently only capable of QSPI boot. It searches for a
bitstream and ELF file inside the boot binary, flashes them and jumps to the ELF file.

It can be easily adapted to other boards by changing the static DDR/DDRIOB configuration
and the used QSPI memory driver.
