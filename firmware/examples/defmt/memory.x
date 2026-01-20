MEMORY
{
  /* Zedboard: 512 MB DDR3. Only use 63 MB for now, should be plenty for a bare-metal app.
  Leave 1 MB of memory which will be configured as uncached device memory by the MMU. This is
  recommended for something like DMA descriptors. */
  /*CODE(rx) : ORIGIN = 0x00100000, LENGTH = 63M*/
  CODE(rx) : ORIGIN = 0x00000000, LENGTH = 192K
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
