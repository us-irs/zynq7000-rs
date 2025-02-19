//! Start-up code for Zynq 7000
//!
//! The bootup routine was kepts as similar to the one
//! [provided by Xilinx](https://github.com/Xilinx/embeddedsw/blob/master/lib/bsp/standalone/src/arm/cortexa9/gcc/boot.S)
//! as possible. The boot routine includes stack, MMU, cache and .bss/.data section initialization.
use cortex_a_rt as _;
use cortex_ar::register::{Cpsr, cpsr::ProcessorMode};

// Start-up code for Armv7-A
//
// We set up our stacks and `kmain` in system mode.
core::arch::global_asm!(
    r#"
.set PSS_L2CC_BASE_ADDR, 0xF8F02000
.set PSS_SLCR_BASE_ADDR, 0xF8000000

.set RESERVED,		0x0fffff00
.set LRemap,		0xFE00000F		/* set the base address of the peripheral block as not shared */
.set L2CCWay,		(PSS_L2CC_BASE_ADDR + 0x077C)	/*(PSS_L2CC_BASE_ADDR + PSS_L2CC_CACHE_INVLD_WAY_OFFSET)*/
.set L2CCSync,		(PSS_L2CC_BASE_ADDR + 0x0730)	/*(PSS_L2CC_BASE_ADDR + PSS_L2CC_CACHE_SYNC_OFFSET)*/
.set L2CCCrtl,		(PSS_L2CC_BASE_ADDR + 0x0100)	/*(PSS_L2CC_BASE_ADDR + PSS_L2CC_CNTRL_OFFSET)*/
.set L2CCAuxCrtl,	(PSS_L2CC_BASE_ADDR + 0x0104)	/*(PSS_L2CC_BASE_ADDR + XPSS_L2CC_AUX_CNTRL_OFFSET)*/
.set L2CCTAGLatReg,	(PSS_L2CC_BASE_ADDR + 0x0108)	/*(PSS_L2CC_BASE_ADDR + XPSS_L2CC_TAG_RAM_CNTRL_OFFSET)*/
.set L2CCDataLatReg,	(PSS_L2CC_BASE_ADDR + 0x010C)	/*(PSS_L2CC_BASE_ADDR + XPSS_L2CC_DATA_RAM_CNTRL_OFFSET)*/
.set L2CCIntClear,	(PSS_L2CC_BASE_ADDR + 0x0220)	/*(PSS_L2CC_BASE_ADDR + XPSS_L2CC_IAR_OFFSET)*/
.set L2CCIntRaw,	(PSS_L2CC_BASE_ADDR + 0x021C)	/*(PSS_L2CC_BASE_ADDR + XPSS_L2CC_ISR_OFFSET)*/

.set SLCRlockReg,	    (PSS_SLCR_BASE_ADDR + 0x04)	/*(PSS_SLCR_BASE_ADDR + XPSS_SLCR_LOCK_OFFSET)*/
.set SLCRUnlockReg,     (PSS_SLCR_BASE_ADDR + 0x08)	/*(PSS_SLCR_BASE_ADDR + XPSS_SLCR_UNLOCK_OFFSET)*/
.set SLCRL2cRamReg,     (PSS_SLCR_BASE_ADDR + 0xA1C) /*(PSS_SLCR_BASE_ADDR + XPSS_SLCR_L2C_RAM_OFFSET)*/
.set SLCRCPURSTReg,     (0xF8000000 + 0x244)           /*(XPS_SYS_CTRL_BASEADDR + A9_CPU_RST_CTRL_OFFSET)*/
.set EFUSEStatus,        (0xF800D000 + 0x10)            /*(XPS_EFUSE_BASEADDR + EFUSE_STATUS_OFFSET)*/

.set CRValMmuCac,	0b01000000000101	/* Enable IDC, and MMU */
.set CRValHiVectorAddr,	0b10000000000000	/* Set the Vector address to high, 0xFFFF0000 */

.set L2CCAuxControl,	0x72360000		/* Enable all prefetching, Cache replacement policy, Parity enable,
                                        Event monitor bus enable and Way Size (64 KB) */
.set L2CCControl,	0x01			/* Enable L2CC */
.set L2CCTAGLatency,	0x0111			/* latency for TAG RAM */
.set L2CCDataLatency,	0x0121			/* latency for DATA RAM */

.set SLCRlockKey,	        0x767B			/* SLCR lock key */
.set SLCRUnlockKey,	        0xDF0D			/* SLCR unlock key */
.set SLCRL2cRamConfig,      0x00020202      /* SLCR L2C ram configuration */

.set FPEXC_EN,		0x40000000		/* FPU enable bit, (1 << 30) */

.section .text.startup
.align 0

.global _start
.type _start, %function
_start:
    // only allow cpu0 through
    // Read MPIDR
    mrc	p15,0,r1,c0,c0,5
    // Extract CPU ID bits. For single-core systems, this should always be 0
    and	r1, r1, #0x3
    cmp	r1, #0
    beq	check_efuse
    b	initialize

// Zynq specific code. It is recommended to reset CPU1 according to page 160 of the datasheet
check_efuse:
    ldr r0,=EFUSEStatus
    ldr r1,[r0]                             /* Read eFuse setting */
    ands r1,r1,#0x80                        /* Check whether device is having single core */
    beq initialize

    /* single core device, reset cpu1 */
    ldr     r0,=SLCRUnlockReg               /* Load SLCR base address base + unlock register */
    ldr     r1,=SLCRUnlockKey               /* set unlock key */
    str     r1, [r0]                        /* Unlock SLCR */

    ldr r0,=SLCRCPURSTReg
    ldr r1,[r0]                             /* Read CPU Software Reset Control register */
    orr r1,r1,#0x22
    str r1,[r0]                             /* Reset CPU1 */

    ldr	r0,=SLCRlockReg         	/* Load SLCR base address base + lock register */
    ldr	r1,=SLCRlockKey	        	/* set lock key */
    str	r1, [r0]	        	/* lock SLCR */
initialize:
    mrc     p15, 0, r0, c0, c0, 0		/* Get the revision */
    and     r5, r0, #0x00f00000
    and     r6, r0, #0x0000000f
    orr     r6, r6, r5, lsr #20-4

    /* set VBAR to the _vector_table address in linker script */
    ldr	r0, =_vector_table
    mcr	p15, 0, r0, c12, c0, 0

    /* Invalidate scu */
    ldr	r7, =0xf8f0000c
    ldr	r6, =0xffff
    str	r6, [r7]

    /* Invalidate caches and TLBs */
    mov	r0,#0				/* r0 = 0  */
    mcr	p15, 0, r0, c8, c7, 0		/* invalidate TLBs */
    mcr	p15, 0, r0, c7, c5, 0		/* invalidate icache */
    mcr	p15, 0, r0, c7, c5, 6		/* Invalidate branch predictor array */
    bl	invalidate_dcache		/* invalidate dcache */

    /* Disable MMU, if enabled */
    mrc	p15, 0, r0, c1, c0, 0		/* read CP15 register 1 */
    bic	r0, r0, #0x1			/* clear bit 0 */
    mcr	p15, 0, r0, c1, c0, 0		/* write value back */

    // Set up stacks first.
    ldr     r3, =_stack_top

    // get the current PSR
    mrs	r0, cpsr
    // mask for mode bits
    mvn	r1, #0x1f
    and	r2, r1, r0
    // IRQ mode
    orr	r2, r2, {irq_mode}
    msr	cpsr, r2
    // IRQ stack pointer
    mov	sp, r3
    ldr r1, =_irq_stack_size
    sub r3, r3, r1

    mrs	r0, cpsr
    and	r2, r1, r0
    // Supervisor mode
    orr	r2, r2, {svc_mode}
    msr	cpsr, r2
    // Supervisor stack pointer
    mov	sp, r3
    ldr r1, =_svc_stack_size
    sub r3, r3, r1

    mrs	r0, cpsr
    and	r2, r1, r0
    // Abort mode
    orr	r2, r2, {abt_mode}
    msr	cpsr, r2
    // Abort stack pointer
    mov	sp, r3
    ldr r1, =_abt_stack_size
    sub r3, r3, r1

    mrs	r0, cpsr
    and	r2, r1, r0
    // FIQ mode
    orr	r2, r2, {fiq_mode}
    msr	cpsr, r2
    // FIQ stack pointer
    mov	sp, r3
    ldr r1, =_fiq_stack_size
    sub r3, r3, r1

    mrs	r0, cpsr
    and	r2, r1, r0
    // Undefined mode
    orr	r2, r2, {und_mode}
    msr	cpsr, r2
    // Undefined stack pointer
    mov	sp, r3
    ldr r1, =_und_stack_size
    sub r3, r3, r1

    mrs	r0, cpsr
    and	r2, r1, r0
    // System mode
    orr	r2, r2, {sys_mode}
    msr	cpsr, r2
    // System stack pointer (main stack)
    mov	sp, r3

    // set scu enable bit in scu
    ldr	r7, =0xf8f00000
    ldr	r0, [r7]
    orr	r0, r0, #0x1
    str	r0, [r7]

    /* enable MMU and cache */
    bl load_mmu_table

    mvn	r0,#0				/* Load MMU domains -- all ones=manager */
    mcr	p15,0,r0,c3,c0,0

    /* Enable mmu, icahce and dcache */
    ldr	r0,=CRValMmuCac
    mcr	p15,0,r0,c1,c0,0		/* Enable cache and MMU */
    dsb					/* dsb	allow the MMU to start up */
    isb					/* isb	flush prefetch buffer */

    /* Write to ACTLR */
    mrc	p15, 0, r0, c1, c0, 1		/* Read ACTLR*/
    orr	r0, r0, #(0x01 << 6)		/* set SMP bit */
    orr	r0, r0, #(0x01 )		/* Cache/TLB maintenance broadcast */
    mcr	p15, 0, r0, c1, c0, 1		/* Write ACTLR*/

    /* Invalidate L2 Cache and enable L2 Cache*/
    /* For AMP, assume running on CPU1. Don't initialize L2 Cache (up to Linux) */
    ldr	r0,=L2CCCrtl			/* Load L2CC base address base + control register */
    mov	r1, #0				/* force the disable bit */
    str	r1, [r0]			/* disable the L2 Caches */

    ldr	r0,=L2CCAuxCrtl			/* Load L2CC base address base + Aux control register */
    ldr	r1,[r0]				/* read the register */
    ldr	r2,=L2CCAuxControl		/* set the default bits */
    orr	r1,r1,r2
    str	r1, [r0]			/* store the Aux Control Register */

    ldr	r0,=L2CCTAGLatReg		/* Load L2CC base address base + TAG Latency address */
    ldr	r1,=L2CCTAGLatency		/* set the latencies for the TAG*/
    str	r1, [r0]			/* store the TAG Latency register Register */

    ldr	r0,=L2CCDataLatReg		/* Load L2CC base address base + Data Latency address */
    ldr	r1,=L2CCDataLatency		/* set the latencies for the Data*/
    str	r1, [r0]			/* store the Data Latency register Register */

    ldr	r0,=L2CCWay			/* Load L2CC base address base + way register*/
    ldr	r2, =0xFFFF
    str	r2, [r0]			/* force invalidate */

    ldr	r0,=L2CCSync			/* need to poll 0x730, PSS_L2CC_CACHE_SYNC_OFFSET */
    /* Load L2CC base address base + sync register*/
    /* poll for completion */
Sync:
    ldr	r1, [r0]
    cmp	r1, #0
    bne	Sync

    ldr	r0,=L2CCIntRaw			/* clear pending interrupts */
    ldr	r1,[r0]
    ldr	r0,=L2CCIntClear
    str	r1,[r0]

    ldr	r0,=SLCRUnlockReg		/* Load SLCR base address base + unlock register */
    ldr	r1,=SLCRUnlockKey	    	/* set unlock key */
    str	r1, [r0]		    	/* Unlock SLCR */

    ldr	r0,=SLCRL2cRamReg		/* Load SLCR base address base + l2c Ram Control register */
    ldr	r1,=SLCRL2cRamConfig        	/* set the configuration value */
    str	r1, [r0]	        	/* store the L2c Ram Control Register */

    ldr	r0,=SLCRlockReg         	/* Load SLCR base address base + lock register */
    ldr	r1,=SLCRlockKey	        	/* set lock key */
    str	r1, [r0]	        	/* lock SLCR */

    ldr	r0,=L2CCCrtl			/* Load L2CC base address base + control register */
    ldr	r1,[r0]				/* read the register */
    mov	r2, #L2CCControl		/* set the enable bit */
    orr	r1,r1,r2
    str	r1, [r0]			/* enable the L2 Caches */

    mov	r0, r0
    mrc	p15, 0, r1, c1, c0, 2		/* read cp access control register (CACR) into r1 */
    orr	r1, r1, #(0xf << 20)		/* enable full access for p10 & p11 */
    mcr	p15, 0, r1, c1, c0, 2		/* write back into CACR */

    /* enable vfp */
    fmrx	r1, FPEXC			/* read the exception register */
    orr	r1,r1, #FPEXC_EN		/* set VFP enable bit, leave the others in orig state */
    fmxr	FPEXC, r1			/* write back the exception register */

    mrc	p15,0,r0,c1,c0,0		/* flow prediction enable */
    orr	r0, r0, #(0x01 << 11)		/* #0x8000 */
    mcr	p15,0,r0,c1,c0,0

    mrc	p15,0,r0,c1,c0,1		/* read Auxiliary Control Register */
    orr	r0, r0, #(0x1 << 2)		/* enable Dside prefetch */
    orr	r0, r0, #(0x1 << 1)		/* enable L2 Prefetch hint */
    mcr	p15,0,r0,c1,c0,1		/* write Auxiliary Control Register */

    mrs	r0, cpsr			/* get the current PSR */
    bic	r0, r0, #0x100			/* enable asynchronous abort exception */
    msr	cpsr_xsf, r0

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
0:
    cmp     r1, r0
    beq     1f
    ldm     r2!, {{r3}}
    stm     r0!, {{r3}}
    b       0b
1:
    // Jump to application
    // Load CPU ID 0, which will be used as a function argument to the boot_core function.
    mov      r0, #0x0
    bl      boot_core
    // In case the application returns, loop forever
    b       .
.size _start, . - _start

.type _invalidate_dcache, %function
invalidate_dcache:
    mrc	p15, 1, r0, c0, c0, 1		/* read CLIDR */
    ands	r3, r0, #0x7000000
    mov	r3, r3, lsr #23			/* cache level value (naturally aligned) */
    beq	finished
    mov	r10, #0				/* start with level 0 */
loop1:
    add	r2, r10, r10, lsr #1		/* work out 3xcachelevel */
    mov	r1, r0, lsr r2			/* bottom 3 bits are the Cache type for this level */
    and	r1, r1, #7			/* get those 3 bits alone */
    cmp	r1, #2
    blt	skip				/* no cache or only instruction cache at this level */
    mcr	p15, 2, r10, c0, c0, 0		/* write the Cache Size selection register */
    isb					/* isb to sync the change to the CacheSizeID reg */
    mrc	p15, 1, r1, c0, c0, 0		/* reads current Cache Size ID register */
    and	r2, r1, #7			/* extract the line length field */
    add	r2, r2, #4			/* add 4 for the line length offset (log2 16 bytes) */
    ldr	r4, =0x3ff
    ands	r4, r4, r1, lsr #3		/* r4 is the max number on the way size (right aligned) */
    clz	r5, r4				/* r5 is the bit position of the way size increment */
    ldr	r7, =0x7fff
    ands	r7, r7, r1, lsr #13		/* r7 is the max number of the index size (right aligned) */
loop2:
    mov	r9, r4				/* r9 working copy of the max way size (right aligned) */
loop3:
    orr	r11, r10, r9, lsl r5		/* factor in the way number and cache number into r11 */
    orr	r11, r11, r7, lsl r2		/* factor in the index number */
    mcr	p15, 0, r11, c7, c6, 2		/* invalidate by set/way */
    subs	r9, r9, #1			/* decrement the way number */
    bge	loop3
    subs	r7, r7, #1			/* decrement the index */
    bge	loop2
skip:
    add	r10, r10, #2			/* increment the cache number */
    cmp	r3, r10
    bgt	loop1

finished:
    mov	r10, #0				/* switch back to cache level 0 */
    mcr	p15, 2, r10, c0, c0, 0		/* select current cache level in cssr */
    dsb
    isb
    bx	lr
.size invalidate_dcache, . - invalidate_dcache
    "#,
    fiq_mode = const {
        Cpsr::new_with_raw_value(0)
            .with_mode(ProcessorMode::Fiq)
            .with_i(true)
            .with_f(true)
            .raw_value()
    },
    irq_mode = const {
        Cpsr::new_with_raw_value(0)
            .with_mode(ProcessorMode::Irq)
            .with_i(true)
            .with_f(true)
            .raw_value()
    },
    svc_mode = const {
        Cpsr::new_with_raw_value(0)
            .with_mode(ProcessorMode::Svc)
            .with_i(true)
            .with_f(true)
            .raw_value()
    },
    und_mode = const {
        Cpsr::new_with_raw_value(0)
            .with_mode(ProcessorMode::Und)
            .with_i(true)
            .with_f(true)
            .raw_value()
    },
    abt_mode = const {
        Cpsr::new_with_raw_value(0)
            .with_mode(ProcessorMode::Abt)
            .with_i(true)
            .with_f(true)
            .raw_value()
    },
    sys_mode = const {
        Cpsr::new_with_raw_value(0)
            .with_mode(ProcessorMode::Sys)
            .with_i(true)
            .with_f(true)
            .raw_value()
    },
);
