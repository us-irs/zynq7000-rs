
MEMORY
{
  /* Zedboard: 512 MB DDR3. Only use 256 MB for now, should be plenty for a bare-metal app. */
  CODE(rx) : ORIGIN = 0x00100000, LENGTH = 256M
}

REGION_ALIAS("DATA", CODE);
