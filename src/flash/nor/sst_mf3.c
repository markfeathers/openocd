// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Kevlar Harness                                  *
 *   software@klystron.com                                                 *
 ***************************************************************************/

/***************************************************************************
 *
 * New flash setup command:
 *
 * flash bank <id> sst_mf3 <base> <size> <buswidth> <chipwidth> <target#> [driver_options ...]
 *
 * The driver options specific to this driver are:
 *   <is_data>
 *     '0' indicates that the bank is a 'code flash' bank and
 *     '1' indicates that the bank is a 'data flash' bank.
 *       If the <is_data> parameter is not specified, then the
 *       'mf3_probe' function assumes that the bank is a 'data flash'
 *       bank if the address is 0x4010.xxxx and in the 'code flash'
 *       otherwise.
 *
 ****************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>

#define FACI_SECTOR_SIZE_MF3        0x400       /* MF3 type flash is divided into 1 kib sectors */

/* addresses of the FACI registers in the S124 (R7FS3A7) */
#define FACI_FENTRYR                0x407effb2
#define FACI_DFLCTL                 0x407ec090  /* data flash control register */
#define FACI_FPMCR                  0x407ec100  /* flash mode control register */
#define FACI_FSARL                  0x407ec108  /* start address (low 16 bits) */
#define FACI_FSARH                  0x407ec110  /* start address (high 16 bits) */
#define FACI_FCR                    0x407ec114  /* flash control register */
#define FACI_FEARL                  0x407ec118  /* end address (low 16 bits) */
#define FACI_FEARH                  0x407ec120  /* end address (high 16 bits) */
#define FACI_FRESETR                0x407ec124  /* flash reset register */
#define FACI_FSTATR00               0x407ec128
#define FACI_FSTATR1                0x407ec12c
#define FACI_FSTATR01               0x407ec13c
#define FACI_FPR                    0x407ec180  /* flash protection register */
#define FACI_FPSR                   0x407ec184  /* flash protection status register */
#define FACI_FISR                   0x407ec1d8  /* flash speed select register */
#define FACI_FSTATR2                0x407ec1f0


/* values used in the FENTRY register */
#define FACI_FENTRY_FEKEY           0xaa00      /* key value for the FENTRYR register */
#define FACI_FENTRY_FENTRYD         0x80        /* set data flash to program/erase mode */
#define FACI_FENTRY_FENTRY0         0x01        /* set the code flash to program/erase mode */
#define FACI_FENTRY_READ_MODE       0x00        /* flash controller is in read (normal) mode */

/* values used in the DFLCTL register */
#define FACI_DFLCTL_DFLEN_ENABLE    0x01        /* enable access to the data flash */

/* values used in the FPMCR register */
#define FACI_FPMCR_FMS_10           0x10
#define FACI_FPMCR_FMS_DISCHARGE1   0x12
#define FACI_FPMCR_FMS_DISCHARGE2   0x92
#define FACI_FPMCR_FMS_PROGRAM_CODE 0x82
#define FACI_FPMCR_FMS_READ_MODE    0x00

/* values used in the FCR register */
#define FACI_FCR_CMD_PROGRAM        0x01        /* program data */
#define FACI_FCR_CMD_BLANK_CHECK    0x03
#define FACI_FCR_CMD_BLOCK_ERASE    0x04        /* block erase */
#define FACI_FCR_CMD_READ           0x05
#define FACI_FCR_CMD_CHIP_ERASE     0x06        /* chip erase */
#define FACI_FCR_OPST               0x80        /* start processing the command */

/* values used in the FRESETR register */
#define FACI_FRESETR_FRESET         0x01        /* reset flash programming registers */

/* values used in the FSTATR1 register */
#define FACI_FSTATR1_DRRDY          0x02        /* read data is ready */
#define FACI_FSTATR1_FRDY           0x40        /* command complete */
#define FACI_FSTATR1_EXRDY          0x80        /* extra area command complete */

/* values used in the FPR register */
#define FACI_FPR_UNLOCK             0xa5        /* unlock value for the protection unlock register */

/* values used in the FISR register */
#define FISR_PCKA_32MHZ             0x1f        /* the flash IF clock is 32MHz */

/* values used in the FSTATR2 register */
#define FACI_FSTATR2_ILGLERR        0x10        /* illegal command error flag */
#define FACI_FSTATR2_ERERR          0x01        /* erase error flag */

/* addresses of the SYSTEM registers in the S124 (R7FS3A7) */
#define SYSTEM_OPCCR                0x4001e0a0
#define SYSTEM_SCKSCR               0x4001e026
#define SYSTEM_SCKSCR_CKSEL_HOCO    0x00        /* select HOCO as the source for ICLK */
#define SYSTEM_PRCR                 0x4001e3fe

/* the base address of the main flash area (executable code / data) */
#define S124_PROGRAM_FLASH_BASE     0x00000000

/* the base address of the data flash area */
#define S124_DATA_FLASH_BASE        0x40100000

/* additional data for each flash bank which is not part of the standard
 * 'flash_bank' structure. A pointer to this structure is stored in
 * 'flash_bank->driver_priv */
struct renesas_bank {
	bool is_data; /* 0 for 'code flash' or 1 for 'data flash' */
	bool probed;  /* set when the bank has been probed */
};

const struct flash_driver sst_mf3_flash;

FLASH_BANK_COMMAND_HANDLER(flash_bank_command)
{
	struct renesas_bank *info = NULL;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int bank_index = strtoul(CMD_ARGV[0], NULL, 0);

	LOG_INFO("Processing 'flash bank' command for index %d at %08lx", bank_index, bank->base);

	/* look for an existing flash structure matching target */
	for (struct flash_bank *bank_iter = flash_bank_list(); bank_iter; bank_iter = bank_iter->next) {
		if (bank_iter->driver == &sst_mf3_flash
			&& bank_iter->target == bank->target
			&& bank->driver_priv) {
			info = bank->driver_priv;
			break;
		}
	}

	if (!info) {
		/* target not matched, make a new one */
		info = calloc(1, sizeof(struct renesas_bank));
	}

	bank->driver_priv = info;

	return ERROR_OK;
}

/* This function sets the flash mode control register which can
 * only be changed by:
 *    - writing a key value to the FPR register
 *    - writing the FPMCR value
 *    - writing the inverse of the FPMCR value
 *    - writing the FPMCR value again */
static int select_flash_mode(struct target *target, uint8_t mode)
{
	int retval;

	retval = target_write_u8(target, FACI_FPR, FACI_FPR_UNLOCK);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, FACI_FPMCR, mode);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, FACI_FPMCR, mode ^ 0xff);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u8(target, FACI_FPMCR, mode);

	return retval;
}

/* This function sets the flash controller into programming or
 * normal operation modes depending on the value of the 'enable'
 * parameter. When placed into 'programming' mode the selects
 * between 'program flash' or 'code flash' mode according to the
 * 'is_data' parameter.*/
static int set_programming_mode(struct target *target, bool enable, bool is_data)
{
	int retval;
	uint16_t status16;	/* status read from the FENTRYR */
	uint8_t status8;	/* status read from the FPSR */

	if (enable) {
		if (is_data)
			/* set data flash into programming mode */
			retval = target_write_u16(target, FACI_FENTRYR, FACI_FENTRY_FEKEY | FACI_FENTRY_FENTRYD);
		else
			/* set code flash into programming mode */
			retval = target_write_u16(target, FACI_FENTRYR, FACI_FENTRY_FEKEY | FACI_FENTRY_FENTRY0);

		if (retval != ERROR_OK)
			return retval;

		retval = select_flash_mode(target, FACI_FPMCR_FMS_DISCHARGE1);
		if (retval != ERROR_OK)
			return retval;

		/* delay for (at least) the P/E mode transition time - tDIS (2uS) */
		retval = target_read_u8 (target, FACI_FPSR, &status8);
		if (retval != ERROR_OK)
			return retval;
		if (status8 != 0) {
			LOG_ERROR("Failed to put flash in DISCHARGE1 mode");
			return ERROR_FAIL;
		}

		retval = select_flash_mode(target, FACI_FPMCR_FMS_DISCHARGE2);
		if (retval != ERROR_OK)
			return retval;

		retval = select_flash_mode(target, FACI_FPMCR_FMS_PROGRAM_CODE);
		if (retval != ERROR_OK)
			return retval;

		/*  delay for (at least) the mode setup time - tMS (5uS) */
		retval = target_read_u8 (target, FACI_FPSR, &status8);
		if (retval != ERROR_OK)
			return retval;
		if (status8 != 0) {
			LOG_ERROR("Failed to put flash in PROGRAM mode");
			return ERROR_FAIL;
		}

		retval = target_write_u8(target, FACI_FISR, FISR_PCKA_32MHZ);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = select_flash_mode(target, FACI_FPMCR_FMS_DISCHARGE2);
		if (retval != ERROR_OK)
			return retval;

		/* delay for (at least) the P/E mode transition time - tDIS (2uS) */
		retval = target_read_u8 (target, FACI_FPSR, &status8);
		if (retval != ERROR_OK)
			return retval;

		if (status8 != 0) {
			LOG_ERROR("Failed to put flash in DISCHARGE2 mode");
			return ERROR_FAIL;
		}

		retval = select_flash_mode(target, FACI_FPMCR_FMS_DISCHARGE1);
		if (retval != ERROR_OK)
			return retval;

		retval = select_flash_mode(target, FACI_FPMCR_FMS_READ_MODE);
		if (retval != ERROR_OK)
			return retval;

		/* read the status register twice in order to generate a delay of (at
		 * least) the mode setup time - tMS (5uS) */
		retval = target_read_u8 (target, FACI_FPSR, &status8);
		if (retval != ERROR_OK)
			return retval;
		retval = target_read_u8 (target, FACI_FPSR, &status8);
		if (retval != ERROR_OK)
			return retval;
		if (status8 != 0)
			return ERROR_FAIL;

		/* set the code flash back into read mode */
		retval = target_write_u16(target, FACI_FENTRYR, FACI_FENTRY_FEKEY);
		if (retval != ERROR_OK)
			return retval;

		for (int i = 0; i < 10; i++) {
			/* wait for the flash to return to read mode */
			retval = target_read_u16 (target, FACI_FENTRYR, &status16);
			if (retval != ERROR_OK)
				return retval;

			if (status16 == FACI_FENTRY_READ_MODE)
				break;
		}
	}

	return ERROR_OK;
}

/* This function does not place the flash into programming mode as
* this is done in 'mf3_erase' before potentially calling this
* function multiple times to erase successive pages. */
static int mf3_erase_page(struct flash_bank *bank, uint32_t addr)
{
	int retval = ERROR_OK;
	uint16_t status16 = 0;
	uint8_t status8 = 0;
	struct target *target = bank->target;
	uint32_t timeout;

	LOG_DEBUG("erasing flash page at 0x%08" PRIx32, addr);

	/* write the start address of the sector to be erased */
	retval = target_write_u16(target, FACI_FSARL, (addr >> 0));
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u16(target, FACI_FSARH, (addr >> 16));
	if (retval != ERROR_OK)
		return retval;

	/* write the end address of the sector to be erased */
	addr = addr + bank->size;
	retval = target_write_u16(target, FACI_FEARL, ((addr + FACI_SECTOR_SIZE_MF3 - 1) >> 0));
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u16(target, FACI_FEARH, ((addr + FACI_SECTOR_SIZE_MF3 - 1) >> 16));
	if (retval != ERROR_OK)
		return retval;

	/* trigger the erase command */
	retval = target_write_u8(target, FACI_FCR, FACI_FCR_OPST | FACI_FCR_CMD_BLOCK_ERASE);
	if (retval != ERROR_OK)
		return retval;

	for (timeout = 0; timeout < 20; timeout++) {
		/* wait for the erase operation to complete */
		alive_sleep(10);

		retval = target_read_u8(target, FACI_FSTATR1, &status8);
		if (retval != ERROR_OK)
			return retval;

		if ((status8 & FACI_FSTATR1_FRDY) != 0)
			break;
	}

	/* reset the command register */
	retval = target_write_u8(target, FACI_FCR, 0);
	if (retval != ERROR_OK)
		return retval;

	for (timeout = 0; timeout < 20; timeout++) {
		/* wait for the result flag to clear */
		retval = target_read_u8(target, FACI_FSTATR1, &status8);
		if (retval != ERROR_OK)
			return retval;

		if ((status8 & FACI_FSTATR1_FRDY) == 0)
			break;
	}

	/* check if any error flags have been set */
	retval = target_read_u16(target, FACI_FSTATR2, &status16);
	if (retval != ERROR_OK)
		return retval;

	if ((status16 & (FACI_FSTATR2_ILGLERR | FACI_FSTATR2_ERERR)) != 0) {
		/* an error occurred - reset the error flags */
		retval = target_write_u8(target, FACI_FRESETR, FACI_FRESETR_FRESET);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u8(target, FACI_FRESETR, 0);
		if (retval != ERROR_OK)
			return retval;

		retval = ERROR_FAIL;
	}

	return retval;
}

static int mf3_erase(struct flash_bank *bank, unsigned int first,
	unsigned int last)
{
	struct target *target = bank->target;
	struct renesas_bank *info = bank->driver_priv;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!info) {
		LOG_ERROR("Bank info not configured");
		return ERROR_FAIL;
	}

	/* place the flash controller into programming mode */
	set_programming_mode(target, true, info->is_data);

	for (unsigned int i = first; i <= last; i++) {
		retval = mf3_erase_page(bank, bank->base + bank->sectors[i].offset);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to erase sector %d", i);
			break;
		}
	}

	/* place the flash controller into normal operation mode */
	set_programming_mode(target, false, info->is_data);

	return retval;
}

static int mf3_protect(struct flash_bank *bank, int set, unsigned int first,
	unsigned int last)
{
	struct target *target = bank->target;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_ERROR("sector protection is not supported yet");

	return retval;
}



/* uint32_t count - number of bytes to write (in R0)
 * uint32_t workarea_start - address of the start of the work area (in R1)
 * uint32_t workarea_end - address of the end of the work area (in R2)
 * uint32_t dest - destination address in flash (in R3)
 * Returns:     result (in R0) */
static const uint8_t mf3_flash_write_code[] = {
	0x1b, 0x4d, 0x04, 0x46, 0x8a, 0x46, 0x93, 0x46,
	0x00, 0x20, 0x1f, 0x46, 0x2f, 0x81, 0x3f, 0x0c,
	0x2f, 0x82, 0x29, 0xe0, 0x1f, 0x0e, 0x05, 0xd0,
	0x0f, 0x78, 0x2f, 0x86, 0x01, 0x31, 0x01, 0x3c,
	0x06, 0xe0, 0xc0, 0x46, 0x0f, 0x68, 0x2f, 0x86,
	0x3f, 0x0c, 0x2f, 0x87, 0x04, 0x31, 0x04, 0x3c,
	0x81, 0x27, 0x2f, 0x75, 0xc0, 0x46, 0xef, 0x6a,
	0x40, 0x26, 0x37, 0x40, 0xfb, 0xd0, 0x00, 0x27,
	0x2f, 0x75, 0xc0, 0x46, 0xef, 0x6a, 0x40, 0x26,
	0x37, 0x40, 0xfb, 0xd1, 0xf0, 0x20, 0x28, 0x58,
	0x07, 0x46, 0x11, 0x26, 0x37, 0x40, 0x04, 0xd0,
	0x01, 0x27, 0x6f, 0x62, 0x00, 0x27, 0x6f, 0x62,
	0x04, 0xe0, 0x59, 0x45, 0x00, 0xd3, 0x51, 0x46,
	0x24, 0x40, 0xd3, 0xd1, 0x00, 0xbe, 0x00, 0x00,
	0x00, 0xc1, 0x7e, 0x40
};

static int mf3_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct renesas_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 2048;        /* Default minimum value */
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;

	int retval = ERROR_OK;
	unsigned int thisrun_count;

	/* Todo: check if bank is locked/protected? before trying to write */

	/* Increase buffer_size if needed */
	if (buffer_size < (target->working_area_size / 2))
		buffer_size = (target->working_area_size / 2);


	if (!info->is_data) {
		/* check alignment - code flash */
		if ((offset & 0x03) != 0) {
			LOG_WARNING("offset 0x%" PRIx32 " is not word aligned", offset);
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		}
	}

	/* allocate a buffer to hold the flash write function */
	/* this is allocated from the "work area" declared in the target configuration file */
	if (target_alloc_working_area(target, sizeof(mf3_flash_write_code) + 8,
		&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* allocate a buffer to hold the blocks of data to be written */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;       /* halve the size of the requested buffer */
		buffer_size &= ~3;      /* make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* give up if a buffer larger than 256 bytes cannot be allocated */
			target_free_working_area(target, write_algorithm);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	/* copy the 'mf3_flash_write_code' function into RAM */
	retval = target_write_buffer(target, write_algorithm->address,
	sizeof(mf3_flash_write_code), mf3_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* place the flash controller into programming mode */
	retval = set_programming_mode(target, true, info->is_data);
	if (retval != ERROR_OK)
		return retval;

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* declare the parameters to the write function */
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* number of bytes to program */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /* source start address */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); /* source end address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT); /* target start address */

	/* so obscure: dataflash has a different address for write from P/E mode,
	we must add a difference (0xfe00 0000 - 0x4010 0000) (M.S.) */
	if (info->is_data)
		address += 0xbdf00000;

	while (count > 0) {
		thisrun_count = buffer_size;
		if (thisrun_count > count)
			thisrun_count = count;

		/* copy the next chunk of data into RAM on the target */
		retval = target_write_buffer(target, source->address, thisrun_count, buffer);
		if (retval != ERROR_OK)
			break;

		/* set the parameters for the write function */
		buf_set_u32(reg_params[0].value, 0, 32, thisrun_count);
		buf_set_u32(reg_params[1].value, 0, 32, (intptr_t)source->address);
		buf_set_u32(reg_params[2].value, 0, 32, (intptr_t)(source->address + thisrun_count));
		buf_set_u32(reg_params[3].value, 0, 32, address);

		retval = target_run_algorithm(target, 0, NULL, 4, reg_params,
		write_algorithm->address, 0, 1000, &armv7m_info);

		if (retval != ERROR_OK) {
			LOG_ERROR("flash write function returned %04x", retval);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		if (count >= thisrun_count) {
			/* increment the source and destination pointers */
			buffer  += thisrun_count;
			address += thisrun_count;
			/* decrement the byte counter */
			count   -= thisrun_count;
		} else {
			break;
		}
	}

	/* place the flash controller into normal operation mode */
	/* (retain the current status in 'retval') */
	(void)set_programming_mode(target, false, info->is_data);

	/* free the allocated buffers */
	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	return retval;
}

/* Renesas seem to have defined a structure called the "Factory MCU
 * Information Flash Root Table (FMIFRT)" which may contain
 * information to allow firmware to automatically determine the
 * configuration of the flash. However, it also appears that the
 * R6AM2 is the *only* device to actually contain this table!
 *
 * Therefore, as automatically detecting the flash configuration
 * of most Renesas devices seems impossible, the addresses and
 * sizes of each flash bank need to be specified in the target
 * configuration file. */
static int mf3_probe(struct flash_bank *bank)
{
	struct renesas_bank *info = bank->driver_priv;
	uint8_t data_flash_enable;

	LOG_DEBUG("probing bank %d at %08lx", bank->bank_number, bank->base);

	/* fill in the details in the common 'flash_bank' structure */
	bank->num_sectors = bank->size / FACI_SECTOR_SIZE_MF3;

	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

	for (uint32_t i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * FACI_SECTOR_SIZE_MF3;
		bank->sectors[i].size = FACI_SECTOR_SIZE_MF3;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	/* fill in the details in the additional 'renesas_bank' structure */
	info->probed = true;

	if ((bank->base >> 24) == 0x00)
		info->is_data = false;      /* the address is in the 'code flash' */
	else
		info->is_data = true;       /* the address is in the 'data flash' */

	if (info->is_data) {
		/* check that the data flash is enabled */
		target_read_u8 (bank->target, FACI_DFLCTL, &data_flash_enable);

		if (data_flash_enable != FACI_DFLCTL_DFLEN_ENABLE) {
			LOG_INFO("Enabling data flash access");
			target_write_u8(bank->target, FACI_DFLCTL, FACI_DFLCTL_DFLEN_ENABLE);
			target_read_u8 (bank->target, FACI_DFLCTL, &data_flash_enable);
			if (data_flash_enable != FACI_DFLCTL_DFLEN_ENABLE) {
				LOG_ERROR("Failed to enable data flash access");
				return ERROR_FAIL;
			}
		}
	}

	return ERROR_OK;
}

static int mf3_auto_probe(struct flash_bank *bank)
{
	struct renesas_bank *info = bank->driver_priv;
	uint8_t operating_mode;
	uint8_t system_clock_select;

	/* check that the HOCO clock is selected */
	target_read_u8 (bank->target, SYSTEM_SCKSCR, &system_clock_select);
	if (system_clock_select != SYSTEM_SCKSCR_CKSEL_HOCO) {
		LOG_INFO("Selecting HOCO clock");
		target_write_u16(bank->target, SYSTEM_PRCR, 0xa503);
		target_write_u8(bank->target, SYSTEM_SCKSCR, SYSTEM_SCKSCR_CKSEL_HOCO);
		target_read_u8 (bank->target, SYSTEM_SCKSCR, &system_clock_select);
		if (operating_mode != SYSTEM_SCKSCR_CKSEL_HOCO) {
			LOG_ERROR("Failed to select HOCO clock");
			return ERROR_FAIL;
		}
	}

	/* check the operating mode of the processor */
	target_read_u8 (bank->target, SYSTEM_OPCCR, &operating_mode);
	if (operating_mode != SYSTEM_SCKSCR_CKSEL_HOCO) {
		LOG_INFO("Selecting high-speed mode");
		target_write_u16(bank->target, SYSTEM_PRCR, 0xa503);
		target_write_u8(bank->target, SYSTEM_OPCCR, SYSTEM_SCKSCR_CKSEL_HOCO);
		target_read_u8 (bank->target, SYSTEM_OPCCR, &operating_mode);
		if (operating_mode != SYSTEM_SCKSCR_CKSEL_HOCO) {
			LOG_ERROR("Failed to select high-speed mode");
			return ERROR_FAIL;
		}
	}

	info->probed = false;
	if (info->probed)
		return ERROR_OK; /* the bank has already been probed */

	return mf3_probe(bank);
}

static const struct command_registration mf3_command_handlers[] = {
	{
		.name = "mf3",
		.mode = COMMAND_ANY,
		.help = "mf3 flash command group",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver sst_mf3_flash = {
	.name = "sst_mf3",
	.commands = mf3_command_handlers,
	.flash_bank_command = flash_bank_command,
	.erase = mf3_erase,
	.protect = mf3_protect,
	.write = mf3_write,
	.read = default_flash_read,
	.probe = mf3_probe,
	.auto_probe = mf3_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
	.usage = "flash bank bank_id 'sst_mf3' base_address"
};
