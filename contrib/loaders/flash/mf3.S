/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * This code is embedded within: src/flash/nor/sst_mf3.c as a "C" array.
 * To rebuild:
 *      arm-none-eabi-gcc -c mf3.S
 *      arm-none-eabi-objcopy -O binary mf3.o mf3.bin
 *      xxd -c 8 -i mf3.bin > mf3.txt
 * Then read and edit this result into the "C" source.
 */
/***************************************************************************
 *   Copyright (C) 2022 by Kevlar Harness                                  *
 *   software@klystron.com                                                 *
 ***************************************************************************/

#define FACI_FPMCR                  0x407ec100            /* flash mode control register */

/* offsets of other registers relative to FPMCR */
#define FSARL                  0x08            /* start address (low 16 bits) */
#define FSARH                  0x10            /* start address (high 16 bits) */
#define FCR                    0x14            /* flash control register */
#define FEARL                  0x18            /* end address (low 16 bits) */
#define FEARH                  0x20            /* end address (high 16 bits) */
#define FRESETR                0x24			/* flash reset register */
#define FSTATR00               0x28
#define FSTATR1                0x2c
#define FWBL0                  0x30
#define FWBH0                  0x38
#define FSTATR01               0x3c
#define FPR                    0x80            /* flash protection register */
#define FPSR                   0x84            /* flash protection status register */
#define FSTATR2                0xf0

/* values used in the FCR register */
#define FACI_FCR_CMD_PROGRAM 	    0x01			/* program data */
#define FACI_FCR_OPST				0x80			/* start processing the command */

/* values used in the FRESETR register */
#define FACI_FRESETR_FRESET			0x01		/* reset flash programming registers */

/* values used in the FSTATR1 register */
#define FACI_FSTATR1_FRDY			0x40		/* command complete */

/* values used in the FSTATR2 register */
#define FACI_FSTATR2_ILGLERR 		0x10		/* illegal command error flag */
#define FACI_FSTATR2_ERERR			0x01		/* erase error flag */

/* mf3_flash_write
 * This function writes a block of data to the code flash. If the size of the source
 * buffer (given by 'pulWorkareaStart' and 'pulWorkareaEnd') then the source data
 * is repeated to fill up the destination until 'ulCount' bytes have been written.
 * This function must be executed from RAM since code cannot be executed from flash
 * while it is in programming mode.
 * Inputs:
 *   r0 ulong ulCount - number of bytes to write
 *   r1 - ulong * pulWorkareaStart - start of the work area in RAM
 *   r2 - ulong * pulWorkareaEnd - end address of the work area
 *   r3 - ulong * pulTarget - address to write the data to
 * Returns:
 *   r0 - result
 * Registers:
 *   r4 - count - number of bytes to write
 *   r5 - R_FACI - base address of flash controller peripheral
 *   r6 - source address (rotates in the area between r1 and r2)
 *   r7 - working register
 *   r10 - start address of the work area in RAM
 *   r11 - end address of the work area in RAM
 */

	.syntax unified
	.thumb
	.thumb_func

.global mf3_flash_write

mf3_flash_write:
	movs    r0, #0          /* r0 => result code (0 == no error) */
	ldr     r5, =FACI_FPMCR /* r5 => base register address */
	mov     r6, r1          /* r6 => source address */
	mov     r10, r1         /* r10 => source start address*/
	mov     r11, r2         /* r11 => source end address*/
	movs    r4, r0          /* r4 => ulCount */
	b.n     loop_end

loop_start:
	/* set the destination address */
	mov     r7, r3
	strh    r7, [r5, #FSARL]
	lsrs    r7, r7, #16
	strh    r7, [r5, #FSARH]

	/* set the source address */
	mov     r7, r6
	strh    r7, [r5, #FWBL0]
	lsrs    r7, r7, #16
	strh    r7, [r5, #FWBH0]

	/* trigger the write command */
	movs    r7, #(FACI_FCR_OPST | FACI_FCR_CMD_PROGRAM)
	strb    r7, [r5, #FCR]
	nop

wait_complete:
	/* wait for the operation to complete */
	ldr     r7, [r5, #FSTATR1]
	movs    r3, #FACI_FSTATR1_FRDY
	ands    r7, r3
	beq.n   wait_complete

	/* reset the command register */
	movs    r7, #0
	strb    r7, [r5, #FCR]
	nop

wait_clear:
	/* wait for the command status register to clear */
	ldr     r7, [r5, #FSTATR1]
	movs    r3, #FACI_FSTATR1_FRDY
	ands    r7, r3
	bne.n   wait_complete

	/* check for errors */
	movs    r0, #FSTATR2
	ldr     r0, [r5, r0]
	mov     r7, r0
	movs    r3, #(FACI_FSTATR2_ILGLERR | FACI_FSTATR2_ERERR)
	ands    r7, r3
	beq.n   no_error

	/* an error has occurred - reset the error flags and exit */
	movs    r7, #FACI_FRESETR_FRESET
	str     r7, [r5, #FRESETR]
	movs    r7, #0
	str     r7, [r5, #FRESETR]
	b.n     exit

no_error:
	/* increment the source pointer */
	adds    r6, #4
	cmp     r6, r11
	bcc.n   no_wrap

	/* wrap the source pointer round to the beginning of the source buffer */
	mov     r6, r10

no_wrap:
	/* increment the destination pointer */
	adds    r3, #4

	/* decrement the loop counter */
	subs    r4, #4

loop_end:
	bne.n   loop_start

exit:
	movs    r0, #0x39
	bkpt    #0
