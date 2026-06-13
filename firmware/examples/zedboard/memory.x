MEMORY
{
  /* Zedboard: 512 MB DDR3. Only use 64 MB for now, should be plenty for a bare-metal app.
  1 MB stack memory and 1 MB of memory which will be configured as uncached device memory by the
  MMU. This is recommended for something like DMA descriptors. */
  OCM : ORIGIN = 0x00000000, LENGTH = 192K
  DDR : ORIGIN = 0x00100000, LENGTH = 63M
  OCM_UPPER(rx): ORIGIN = 0xFFFF0000, LENGTH = 64K
  UNCACHED : ORIGIN = ORIGIN(DDR) + LENGTH(DDR), LENGTH = 1M
}

REGION_ALIAS("VECTORS", DDR);
REGION_ALIAS("CODE", DDR);
REGION_ALIAS("DATA", DDR);
REGION_ALIAS("STACKS", DDR);

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

  /* Add OCM memory region to allow placing items in fast OCM memory. */
  .ocm (NOLOAD) : ALIGN(4) {
      . = ALIGN(4);
      _socm = .;
      *(.ocm .ocm.*);
      . = ALIGN(4);
      _eocm = .;
  } > OCM
}

PROVIDE(_sys_stack_size = 1M);
