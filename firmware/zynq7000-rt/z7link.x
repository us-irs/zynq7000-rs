/*
Complete zynq7000-rt linker script, applied to every project via the `-Clink-arg=-Tz7link.x`
rustflag (see firmware/.cargo/config.toml). This supersedes aarch32-rt's own `link.x` entirely
(that crate's `link.x` is not used here, see `firmware/.cargo/config.toml`, which only passes
`-Tz7link.x`). Its content below is a copy of aarch32-rt/link.x with `.ocm.data`/`.ocm.bss`
sections added alongside `.data`/`.bss`, since those need to be interleaved with the rest of the
memory layout rather than bolted on separately.

You must supply a file called `memory.x` in your linker search path. It must define Region
Aliases 'VECTORS', 'CODE', 'DATA', 'STACKS', and a region named 'OCM' if you want to place
anything in `.ocm.data`/`.ocm.bss` (see the zedboard example's `memory.x` for a template). If you
don't need OCM placement, `OCM` can simply be aliased to any existing region (e.g.
`REGION_ALIAS("OCM", DATA);`) since nothing will actually be placed there.

Here is an example `memory.x` file:

-------------
MEMORY {
    FLASH : ORIGIN = 0x08000000, LENGTH = 2M
    SRAM  : ORIGIN = 0x10000000, LENGTH = 512K
}

REGION_ALIAS("VECTORS", FLASH);
REGION_ALIAS("CODE", FLASH);
REGION_ALIAS("DATA", SRAM);
REGION_ALIAS("STACKS", SRAM);
REGION_ALIAS("OCM", SRAM);
-------------

The AArch32 platform uses seven separate stacks. The default sizes for each are
given at the bottom of this file. However, your `memory.x` can provide an
alternative size for any (or all) of them, provided that size is a multiple of
eight bytes. For example, your `memory.x` might include:

-------------
PROVIDE(_und_stack_size = 3456);
PROVIDE(_svc_stack_size = 3456);
PROVIDE(_abt_stack_size = 3456);
PROVIDE(_hyp_stack_size = 3456);
PROVIDE(_irq_stack_size = 3456);
PROVIDE(_fiq_stack_size = 3456);
PROVIDE(_sys_stack_size = 3456);
-------------

The stacks will be located at the top of the STACKS region by default. Use
`PROVIDE(_pack_stacks = 0)` to remove the padding and locate the stacks at the
bottom of that region instead.

Based upon the linker script from https://github.com/rust-embedded/cortex-m, via aarch32-rt.
*/

INCLUDE memory.x

ENTRY(_start);
EXTERN(_vector_table);
EXTERN(_start);
EXTERN(_default_handler);

SECTIONS {
    /* # Vector Table
     *
     * Our ARM interrupt vector table, consisting of branch instructions to
     * each exception handler.
     *
     * May include FIQ handler code at the end.
     */
    .vector_table ORIGIN(VECTORS) : {
        *(.vector_table)
    } > VECTORS

    /* # Text
     *
     * Our executable code.
     */
    .text : {
        __stext = .;

        *(.text .text*)

        __etext = .;
    } > CODE

    /* # Text
     *
     * Our constants.
     */
    .rodata : {
        __srodata = .;

        *(.rodata .rodata*)

        __erodata = .;
    } > CODE

    /* # Data
     *
     * Our global variables that are not initialised to zero.
     */
    .data : ALIGN(4) {
        . = ALIGN(4);
        __sdata = .;

        *(.data .data.*);

        . = ALIGN(4);
        /* NB: __edata defined lower down */
    } > DATA AT>CODE

    /*
     * Allow sections from user `memory.x` injected using `INSERT AFTER .data` to
     * use the .data loading mechanism by pushing __edata. Note: do not change
     * output region or load region in those user sections!
     */
    . = ALIGN(4);
    __edata = .;

    /* LMA of .data */
    __sidata = LOADADDR(.data);

    /* # Block Starting Symbol (BSS)
     *
     * Our global variables that *are* initialised to zero.
     */
    .bss (NOLOAD) : ALIGN(4) {
        . = ALIGN(4);
        __sbss = .;

        *(.bss .bss* COMMON)

        . = ALIGN(4);
        /* NB: __ebss defined lower down */
    } > DATA

    /*
     * Allow sections from user `memory.x` injected using `INSERT AFTER .bss` to
     * use the .bss zeroing mechanism by pushing __ebss. Note: do not change
     * output region or load region in those user sections!
     */
    __ebss = .;

    /* # OCM Data
     *
     * Global variables placed in OCM via `#[unsafe(link_section = ".ocm.data")]` that are not
     * initialised to zero.
     */
    .ocm.data : ALIGN(4) {
        . = ALIGN(4);
        _socmdata = .;

        *(.ocm.data .ocm.data.*);

        . = ALIGN(4);
        /* NB: _eocmdata defined lower down */
    } > OCM AT>CODE

    . = ALIGN(4);
    _eocmdata = .;

    /* LMA of .ocm.data */
    _siocmdata = LOADADDR(.ocm.data);

    /* # OCM BSS
     *
     * Global variables placed in OCM via `#[unsafe(link_section = ".ocm.bss")]` that *are*
     * initialised to zero.
     */
    .ocm.bss (NOLOAD) : ALIGN(4) {
        . = ALIGN(4);
        _socmbss = .;

        *(.ocm.bss .ocm.bss.*);

        . = ALIGN(4);
        /* NB: _eocmbss defined lower down */
    } > OCM

    _eocmbss = .;

    /* # Uninitialised Data
     *
     * Our global variables that have no defined initial value.
     */
    .uninit (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __suninit = .;

        *(.uninit .uninit.*);

        . = ALIGN(4);
        __euninit = .;
    } > DATA


    /* # Stack Padding
     *
     * A padding region to push the stacks to the top of the STACKS region.
     * If `_pack_stacks == 0`, this is forced to be zero size, putting the
     * stacks at the bottom of the STACK region.
     */
    .filler (NOLOAD) : {
        /* Move the .stacks section to the end of the STACKS memory region */
        _next_region = ORIGIN(STACKS) + LENGTH(STACKS);
        _start_moved_stacks = _next_region - SIZEOF(.stacks);
        _start_stacks = _pack_stacks ? . : _start_moved_stacks;
        FILL(0x00)
        . = _start_stacks;
    } > STACKS

    /* # Stacks
     *
     * Space for all seven stacks.
     */
    .stacks (NOLOAD) : ALIGN(8)
    {
        . = ALIGN(8);

        /* Lowest address of allocated stack */
        _stacks_low_end = .;

        /* Stack for UND mode */
        _und_stack_low_end = .;
        . += (_und_stack_size * _num_cores);
        _und_stack_high_end = .;

        /* Stack for SVC mode */
        _svc_stack_low_end = .;
        . += (_svc_stack_size * _num_cores);
        _svc_stack_high_end = .;

        /* Stack for ABT mode */
        _abt_stack_low_end = .;
        . += (_abt_stack_size * _num_cores);
        _abt_stack_high_end = .;

        /* Stack for HYP mode */
        _hyp_stack_low_end = .;
        . += (_hyp_stack_size * _num_cores);
        _hyp_stack_high_end = .;

        /* Stack for IRQ mode */
        _irq_stack_low_end = .;
        . += (_irq_stack_size * _num_cores);
        _irq_stack_high_end = .;

        /* Stack for FIQ mode */
        _fiq_stack_low_end = .;
        . += (_fiq_stack_size * _num_cores);
        _fiq_stack_high_end = .;

        /* Stack for SYS mode */
        _sys_stack_low_end = .;
        . += (_sys_stack_size * _num_cores);
        _sys_stack_high_end = .;

        /* Highest address of allocated stack */
        _stacks_high_end = .;
    } > STACKS

    /DISCARD/ : {
        /* Discard any notes */
        *(.note .note*)

        /* Discard these unwinding/exception related symbols, they are not used */
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)

        /* Discard these exception related symbols, they are not used */
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    }
}

/* We provide default sizes for the stacks for any not specified in memory.x (which was loaded first) */
PROVIDE(_und_stack_size = 2K);
PROVIDE(_svc_stack_size = 2K);
PROVIDE(_abt_stack_size = 2K);
PROVIDE(_hyp_stack_size = 1K);
PROVIDE(_irq_stack_size = 64);
PROVIDE(_fiq_stack_size = 64);
PROVIDE(_sys_stack_size = 16K);
/* Default to one CPU core (i.e. one copy of each stack) */
PROVIDE(_num_cores      = 1);

/* Set this to 1 in memory.x to remove the filler section pushing the stacks to the end of STACKS. */
PROVIDE(_pack_stacks = 0);

/* Weak aliases for ASM default handlers */
PROVIDE(_start                      = _default_start);
PROVIDE(_asm_undefined_handler      = _asm_default_undefined_handler);
PROVIDE(_asm_svc_handler            = _asm_default_svc_handler);
PROVIDE(_asm_hvc_handler            = _asm_default_hvc_handler);
PROVIDE(_asm_prefetch_abort_handler = _asm_default_prefetch_abort_handler);
PROVIDE(_asm_data_abort_handler     = _asm_default_data_abort_handler);
/* TODO: Hyp handler goes here */
PROVIDE(_asm_irq_handler            = _asm_default_irq_handler);
PROVIDE(_asm_fiq_handler            = _asm_default_fiq_handler);

/* Weak aliases for C default handlers */
PROVIDE(_undefined_handler      = _default_handler);
PROVIDE(_svc_handler            = _default_handler);
PROVIDE(_hvc_handler            = _default_handler);
PROVIDE(_prefetch_abort_handler = _default_handler);
PROVIDE(_data_abort_handler     = _default_handler);
/* TODO: Hyp handler goes here */
PROVIDE(_irq_handler            = _default_handler);
/* NB: There is no default C-language FIQ handler */

/* Check the stack sizes are all a multiple of eight bytes */
ASSERT(_und_stack_size % 8 == 0, "
ERROR(z7link.x): UND stack size (_und_stack_size) is not a multiple of 8 bytes");
ASSERT(_svc_stack_size % 8 == 0, "
ERROR(z7link.x): SVC stack size (_svc_stack_size) is not a multiple of 8 bytes");
ASSERT(_abt_stack_size % 8 == 0, "
ERROR(z7link.x): ABT stack size (_abt_stack_size) is not a multiple of 8 bytes");
ASSERT(_hyp_stack_size % 8 == 0, "
ERROR(z7link.x): HYP stack size (_hyp_stack_size) is not a multiple of 8 bytes");
ASSERT(_irq_stack_size % 8 == 0, "
ERROR(z7link.x): IRQ stack size (_irq_stack_size) is not a multiple of 8 bytes");
ASSERT(_fiq_stack_size % 8 == 0, "
ERROR(z7link.x): FIQ stack size (_fiq_stack_size) is not a multiple of 8 bytes");
ASSERT(_sys_stack_size % 8 == 0, "
ERROR(z7link.x): SYS stack size (_sys_stack_size) is not a multiple of 8 bytes");
ASSERT(_num_cores != 0, "
ERROR(z7link.x): Number of cores cannot be zero");

/* End of file */
