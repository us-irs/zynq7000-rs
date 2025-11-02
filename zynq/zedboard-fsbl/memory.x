MEMORY
{
  /* The Zynq7000 has 192 kB of OCM memory which can be used for the FSBL */
  CODE(rx) : ORIGIN = 0x00000000, LENGTH = 192K
  OCM_UPPER(rx): ORIGIN = 0xFFFF0000, LENGTH = 64K
  /* Leave 1 MB of memory which will be configured as uncached device memory by the MMU. This can
  be used for something like DMA descriptors, but the DDR needs to be set up first in addition
  to configuring the page at address 0x400_0000 accordingly */
  UNCACHED(rx): ORIGIN = 0x4000000, LENGTH = 1M
}

REGION_ALIAS("VECTORS", CODE);
REGION_ALIAS("DATA", CODE);

SECTIONS
{
  /* Uncached memory */
  .uncached (NOLOAD) : ALIGN(4) {
    . = ALIGN(4);
    _sbss_uncached = .;
    *(.uncached .uncached.*);
    . = ALIGN(4);
    _ebss_uncached = .;
  } > UNCACHED
}
