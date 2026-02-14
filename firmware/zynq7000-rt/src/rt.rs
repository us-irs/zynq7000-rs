//! Start-up code for Zynq 7000
//!
//! The bootup routine was is based on the one
//! [provided by Xilinx](https://github.com/Xilinx/embeddedsw/blob/master/lib/bsp/standalone/src/arm/cortexa9/gcc/boot.S)
//! but does NOT provide the L2 cache initialization.
//!
//! The boot routine includes stack, MMU and .bss/.data section initialization.
use aarch32_rt as _;

// Start-up code for Armv7-A
//
// We set up our stacks and `kmain` in system mode.
core::arch::global_asm!(
    r#"
.set PSS_L2CC_BASE_ADDR,    0xF8F02000
.set PSS_SLCR_BASE_ADDR,    0xF8000000

.set SLCRlockReg,           (PSS_SLCR_BASE_ADDR + 0x04) /*(PSS_SLCR_BASE_ADDR + XPSS_SLCR_LOCK_OFFSET)*/
.set SLCRUnlockReg,         (PSS_SLCR_BASE_ADDR + 0x08) /*(PSS_SLCR_BASE_ADDR + XPSS_SLCR_UNLOCK_OFFSET)*/
.set SLCRL2cRamReg,         (PSS_SLCR_BASE_ADDR + 0xA1C) /*(PSS_SLCR_BASE_ADDR + XPSS_SLCR_L2C_RAM_OFFSET)*/
.set SLCRCPURSTReg,         (0xF8000000 + 0x244)           /*(XPS_SYS_CTRL_BASEADDR + A9_CPU_RST_CTRL_OFFSET)*/
.set EFUSEStatus,           (0xF800D000 + 0x10)            /*(XPS_EFUSE_BASEADDR + EFUSE_STATUS_OFFSET)*/

.set CRValMmuCac,           0b01000000000101  /* Enable IDC, and MMU */
.set CRValHiVectorAddr,     0b10000000000000  /* Set the Vector address to high, 0xFFFF0000 */

.set SLCRlockKey,           0x767B      /* SLCR lock key */
.set SLCRUnlockKey,         0xDF0D      /* SLCR unlock key */
.set SLCRL2cRamConfig,      0x00020202      /* SLCR L2C ram configuration */

.set FPEXC_EN,              0x40000000    /* FPU enable bit, (1 << 30) */

.section .text.startup
.align 0

.global _start
.type _start, %function
_start:
    // only allow cpu0 through
    // Read MPIDR
    mrc     p15,0,r1,c0,c0,5
    // Extract CPU ID bits. For single-core systems, this should always be 0
    and     r1, r1, #0x3
    cmp     r1, #0
    beq     check_efuse
    b       initialize

// Zynq specific code. It is recommended to reset CPU1 according to page 160 of the datasheet
check_efuse:
    ldr     r0, =EFUSEStatus
    // Read eFuse setting
    ldr     r1, [r0]
    // Check whether device is having single core
    ands    r1,r1,#0x80
    beq     initialize

    /* single core device, reset cpu1 */
    ldr     r0,=SLCRUnlockReg               /* Load SLCR base address base + unlock register */
    ldr     r1,=SLCRUnlockKey               /* set unlock key */
    str     r1, [r0]                        /* Unlock SLCR */

    ldr     r0,=SLCRCPURSTReg
    ldr     r1,[r0]                             /* Read CPU Software Reset Control register */
    orr     r1,r1,#0x22
    str     r1,[r0]                             /* Reset CPU1 */

    ldr     r0,=SLCRlockReg           /* Load SLCR base address base + lock register */
    ldr     r1,=SLCRlockKey           /* set lock key */
    str     r1, [r0]            /* lock SLCR */
initialize:
    mrc     p15, 0, r0, c0, c0, 0   /* Get the revision */
    and     r5, r0, #0x00f00000
    and     r6, r0, #0x0000000f
    orr     r6, r6, r5, lsr #20-4

    /* set VBAR to the _vector_table address in linker script */
    ldr     r0, =_vector_table
    mcr     p15, 0, r0, c12, c0, 0

    /* Invalidate scu */
    ldr     r7, =0xf8f0000c
    ldr     r6, =0xffff
    str     r6, [r7]

    /* Invalidate caches and TLBs */
    mov     r0,#0       /* r0 = 0  */
    mcr     p15, 0, r0, c8, c7, 0   /* invalidate TLBs */
    mcr     p15, 0, r0, c7, c5, 0   /* invalidate icache */
    mcr     p15, 0, r0, c7, c5, 6   /* Invalidate branch predictor array */
    bl      invalidate_dcache   /* invalidate dcache */

    /* Disable MMU, if enabled */
    mrc     p15, 0, r0, c1, c0, 0   /* read CP15 register 1 */
    bic     r0, r0, #0x1      /* clear bit 0 */
    mcr     p15, 0, r0, c1, c0, 0   /* write value back */

    bl      _stack_setup_preallocated

    // set scu enable bit in scu
    ldr     r7, =0xf8f00000
    ldr     r0, [r7]
    orr     r0, r0, #0x1
    str     r0, [r7]

    /* Write to ACTLR */
    mrc     p15, 0, r0, c1, c0, 1   /* Read ACTLR*/
    orr     r0, r0, #(0x01 << 6)    /* set SMP bit */
    orr     r0, r0, #(0x01 )    /* Cache/TLB maintenance broadcast */
    mcr     p15, 0, r0, c1, c0, 1   /* Write ACTLR*/

    mov     r0, r0
    mrc     p15, 0, r1, c1, c0, 2   /* read cp access control register (CACR) into r1 */
    orr     r1, r1, #(0xf << 20)    /* enable full access for p10 & p11 */
    mcr     p15, 0, r1, c1, c0, 2   /* write back into CACR */

    /* enable vfp */
    fmrx    r1, FPEXC     /* read the exception register */
    orr     r1,r1, #FPEXC_EN    /* set VFP enable bit, leave the others in orig state */
    fmxr    FPEXC, r1     /* write back the exception register */

    mrc     p15,0,r0,c1,c0,0    /* flow prediction enable */
    orr     r0, r0, #(0x01 << 11)   /* #0x8000 */
    mcr     p15,0,r0,c1,c0,0

    mrc     p15,0,r0,c1,c0,1    /* read Auxiliary Control Register */
    orr     r0, r0, #(0x1 << 2)   /* enable Dside prefetch */
    orr     r0, r0, #(0x1 << 1)   /* enable L2 Prefetch hint */
    mcr     p15,0,r0,c1,c0,1    /* write Auxiliary Control Register */

    mrs     r0, cpsr      /* get the current PSR */
    bic     r0, r0, #0x100      /* enable asynchronous abort exception */
    msr     cpsr_xsf, r0

    /* Zero BSS and initialize data before calling any function which might require them. */

    // Initialise .bss
    ldr     r0, =__sbss
    ldr     r1, =__ebss
    mov     r2, 0
0:
    cmp     r1, r0
    beq     1f
    stm     r0!, {{r2}}
    b       0b
1:
    // Initialise .data
    ldr     r0, =__sdata
    ldr     r1, =__edata
    ldr     r2, =__sidata
    cmp     r0, r2          /* Shortcut if code is run from RAM and .data is there already */
    beq     data_init_done
0:
    cmp     r1, r0
    beq     data_init_done
    ldm     r2!, {{r3}}
    stm     r0!, {{r3}}
    b       0b
data_init_done:
    /* enable MMU and cache */
    /* MMU Table is in .data, so this needs to be performed after .data is relocated */
    /* (Even if in most cases, .data is already in RAM and relocation is a no-op) */
    bl      load_mmu_table

    mvn     r0,#0       /* Load MMU domains -- all ones=manager */
    mcr     p15,0,r0,c3,c0,0

    /* Enable mmu, icache and dcache */
    ldr     r0,=CRValMmuCac
    mcr     p15,0,r0,c1,c0,0    /* Enable cache and MMU */
    dsb                         /* dsb  allow the MMU to start up */
    isb                         /* isb  flush prefetch buffer */

    // Jump to application
    // Load CPU ID 0, which will be used as a function argument to the boot_core function.
    mov     r0, #0x0
    bl      kmain
    // In case the application returns, loop forever
    b       .
.size _start, . - _start

.type _invalidate_dcache, %function
invalidate_dcache:
    mrc     p15, 1, r0, c0, c0, 1   /* read CLIDR */
    ands    r3, r0, #0x7000000
    mov     r3, r3, lsr #23     /* cache level value (naturally aligned) */
    beq     finished
    mov     r10, #0       /* start with level 0 */
loop1:
    add     r2, r10, r10, lsr #1    /* work out 3xcachelevel */
    mov     r1, r0, lsr r2      /* bottom 3 bits are the Cache type for this level */
    and     r1, r1, #7      /* get those 3 bits alone */
    cmp     r1, #2
    blt     skip        /* no cache or only instruction cache at this level */
    mcr     p15, 2, r10, c0, c0, 0    /* write the Cache Size selection register */
    isb         /* isb to sync the change to the CacheSizeID reg */
    mrc     p15, 1, r1, c0, c0, 0   /* reads current Cache Size ID register */
    and     r2, r1, #7      /* extract the line length field */
    add     r2, r2, #4      /* add 4 for the line length offset (log2 16 bytes) */
    ldr     r4, =0x3ff
    ands    r4, r4, r1, lsr #3    /* r4 is the max number on the way size (right aligned) */
    clz     r5, r4        /* r5 is the bit position of the way size increment */
    ldr     r7, =0x7fff
    ands    r7, r7, r1, lsr #13   /* r7 is the max number of the index size (right aligned) */
loop2:
    mov     r9, r4        /* r9 working copy of the max way size (right aligned) */
loop3:
    orr     r11, r10, r9, lsl r5    /* factor in the way number and cache number into r11 */
    orr     r11, r11, r7, lsl r2    /* factor in the index number */
    mcr     p15, 0, r11, c7, c6, 2    /* invalidate by set/way */
    subs    r9, r9, #1      /* decrement the way number */
    bge     loop3
    subs    r7, r7, #1      /* decrement the index */
    bge     loop2
skip:
    add     r10, r10, #2      /* increment the cache number */
    cmp     r3, r10
    bgt     loop1

finished:
    mov     r10, #0       /* switch back to cache level 0 */
    mcr     p15, 2, r10, c0, c0, 0    /* select current cache level in cssr */
    dsb
    isb
    bx      lr
.size invalidate_dcache, . - invalidate_dcache
    "#,
);
