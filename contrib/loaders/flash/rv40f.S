/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * This code is embedded within: src/flash/nor/renesas_rv40f.c as a "C" array.
 *
 * To rebuild:
 *   arm-none-eabi-gcc -c rv40f.S
 *   arm-none-eabi-objcopy -O binary rv40f.o rv40f.bin
 *   xxd -c 8 -i rv40f.bin > rv40f.txt
 *
 * Then read and edit this result into the "C" source.
 */

#define FACI_BASE 0x407fe000 /* flash mode control register */
#define FACI_CMD  0x407e0000 /* FACI command issuing area address */

/* offsets of other registers relative to FPMCR */
#define FASTAT   0x10   /* Flash access status register */
#define FAEINT   0x14   /* Flash access error interrupt enable register */
#define FRDYIE   0x18   /* Flash ready interrupt enable register */
#define FSADDR   0x30   /* FACI command start address register */
#define FEADDR   0x34   /* FACI command end address register */
#define FSTATR   0x80   /* Flash status register */
#define FENTRYR  0x84   /* Flash P/E mode entry register */
#define FSUINITR 0x8c   /* Flash sequencer set-up initialization register */
#define FCMDR    0xa0   /* Flash command register */
#define FBCCNT   0xd0   /* Data flash bank check control register */
#define FBCSTAT	 0xd4   /* Data flash bank check status register */
#define FPSADDR  0xd8   /* Data flash programming start address register */
#define FAWMON   0xdc   /* Flash access window monitor register */
#define FCPSR    0xe0   /* Flash sequencer processing switching register */
#define FPCKAR   0xe4   /* Flash sequencer processing clock notification register */
#define FSUACR   0xe8   /* Flash start-up area control register */

/* values used in the FSTATR register */
#define FSTATR_DBFULL 0x400

/* rv40f_flash_write:
 *   This function writes a block of data to the code flash. If the size of the
 *   source buffer (given by 'pulWorkareaStart' and 'pulWorkareaEnd') then the
 *   source data is repeated to fill up the destination until 'ulCount' bytes
 *   have been written.
 *
 *   This function must be executed from RAM since code cannot be executed from
 *   flash while it is in programming mode.
 *
 *   The arguments to this function match the arguments passed by OpenOCD to its
 *   flash loader functions.
 *
 * Inputs:
 *   r0 - ulong ulCount - number of halfwords to write
 *   r1 - ulong * pulWorkareaStart - start of the work area in RAM
 *   r2 - ulong * pulWorkareaEnd - end address of the work area
 *   r3 - ulong * pulTarget - address to write the data To
 * Returns:
 *   r0 - result
 * Registers:
 *   r0, r1 - working registers
 *   r5 - R_FACI - base address of flash controller peripheral
 *   r4 - source address (iterated)
 *   r6 - decremented count of halfwords
 *   r7 - FACI command issuing area address
 */

    .syntax unified
    .cpu cortex-m0plus
    .thumb
    .thumb_func

.global rv40f_flash_write

rv40f_flash_write:

    mov     r9, r1
    mov     r4, r9          /* r4 => iterated source address */
    ldr     r5, =FACI_BASE  /* r5 => base register address */
    ldr     r7, =FACI_CMD   /* r7 => load FACI command issuing area address */
    mov     r9, r0
    mov     r6, r9          /* r6 => count of halfwords */

    /* set the destination address of the block */
    str     r3, [r5, #FSADDR]

    /* issue the write command */
    movs    r0, #0xE8
    strb    r0, [r7]
    /* write count of halfwords */
    mov     r0, r9
    strb    r0, [r7]

    /* write first halfword */
    ldrh    r0, [r4]
    strh    r0, [r7]
    /* increment src address */
    adds    r4, #2

loop_start:
    /* decrement counter */
    subs     r6, #1

    /* if zero, jump to completion */
    beq     complete

    /* write next halfword */
    ldrh    r0, [r4]
    strh    r0, [r7]
    /* increment src address */
    adds    r4, #2

/* wait while dbfull bit == 1 */
wait_dbfull:
    movs    r0, #FSTATR
    adds    r0, r5
    ldr     r1, [r0]
    /* check DBFULL bit */
    ldr     r0, =FSTATR_DBFULL
    tst     r0, r1
    bne     wait_dbfull
    b       loop_start

/* complete transaction by writing D0h to FACI cmd issuing area */
complete:
    movs    r0, #0xD0
    strb    r0, [r7]

exit:
    movs    r1, #FSTATR
    adds    r1, r5
    ldr     r0, [r1]
    bkpt    #0
