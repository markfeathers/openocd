// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (c) 2022 Miloslav Semler                                    *
 *   Copyright (c) 2024 Mark Featherston                                   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* FACI registers */
#define FACI_FMEPROT  0x407FE044
#define FACI_FWEPROR  0x4001E416
#define FACI_FENTRYR  0x407FE084
#define FACI_FSTATR   0x407FE080
#define FACI_FASTAT   0x407FE010
#define FACI_FSADDR   0x407FE030
#define FACI_FEADDR   0x407FE034
#define FACI_CMD_AREA 0x407E0000
#define FACI_PNR      0x010080F0

/* register bits */
#define FACI_FSTATR_FLWEERR   BIT(6)
#define FACI_FSTATR_PRGSPD    BIT(8)
#define FACI_FSTATR_ERSSPD    BIT(9)
#define FACI_FSTATR_DBFULL    BIT(10)
#define FACI_FSTATR_SUSRDY    BIT(11)
#define FACI_FSTATR_PRGERR    BIT(12)
#define FACI_FSTATR_ERSERR    BIT(13)
#define FACI_FSTATR_ILGLERR   BIT(14)
#define FACI_FSTATR_FRDY      BIT(15)
#define FACI_FSTATR_OTERR     BIT(20)
#define FACI_FSTATR_SECERR    BIT(21)
#define FACI_FSTATR_FESETERR  BIT(22)
#define FACI_FSTATR_ILGCOMERR BIT(23)

#define FACI_FASTAT_DFAE  BIT(3)
#define FACI_FASTAT_CMDLK BIT(4)
#define FACI_FASTAT_CFAE  BIT(7)

#define FACI_CMD_FORCED_STOP 0xB3
#define FACI_CMD_ERASE1      0x20
#define FACI_CMD_PE_SUSPEND  0xB0
#define FACI_CMD_PE_RESUME   0xD0
#define FACI_CMD_STATUSCLR   0x50

/* Rough estimates from the RA4M2
 * Write 32KB: 848ms
 * erase 32KB: 1040ms
 */
#define TIMEOUT_ERASE_MS 2000
#define TIMEOUT_WRITE_MS 1000

struct renesas_bank {
	/* bank size in bytes */
	unsigned int size;
	/* base address */
	unsigned int base;
	/* 1 for data flash or 0 for code flash */
	unsigned int is_data;
	bool probed;
	bool has_fmeprot;
};

FLASH_BANK_COMMAND_HANDLER(rv40f_flash_bank_command)
{
	struct renesas_bank *info;

	unsigned int base = strtoul(CMD_ARGV[1], NULL, 16);
	unsigned int size = strtoul(CMD_ARGV[2], NULL, 16);

	info = malloc(sizeof(struct renesas_bank));
	if (!info)
		return ERROR_FAIL;
	bank->driver_priv = info;

	info->base = base;
	info->size = size;
	info->is_data = 0; /* assume code flash initially */
	info->probed = false;
	info->has_fmeprot = false;

	return ERROR_OK;
}

static int rv40f_busy_wait(struct target *target, int timeout_ms)
{
	long long endtime;
	uint32_t reg32;
	int retval;

	endtime = timeval_ms() + timeout_ms;
	do {
		/* read status register FSTATR */
		retval = target_read_u32(target, FACI_FSTATR, &reg32);
		if (retval != ERROR_OK)
			return ERROR_FAIL;

		/* No (ERASE)/PROGRAM operation in progress */
		if (reg32 & FACI_FSTATR_FRDY)
			return ERROR_OK;

		alive_sleep(1);
	} while (timeval_ms() < endtime);

	return ERROR_TIMEOUT_REACHED;
}

static int rv40f_unlock(struct target *target, uint32_t reg32)
{
	uint8_t reg8;
	int retval;

	LOG_ERROR("Recovering from command-locked state");

	if (reg32 & FACI_FSTATR_ILGLERR) {
		retval = target_read_u8(target, FACI_FASTAT, &reg8);
		if (retval != ERROR_OK)
			return retval;

		/* reset CFAE and DFAE flags if needed */
		if (reg8 & (FACI_FASTAT_CFAE | FACI_FASTAT_DFAE)) {
			reg8 &= ~(FACI_FASTAT_CFAE | FACI_FASTAT_DFAE);

			retval = target_write_u8(target, FACI_FASTAT, reg8);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	retval = target_read_u32(target, FACI_FSTATR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	if (reg32 & FACI_FSTATR_FLWEERR) {
		retval = target_write_u8(target, FACI_CMD_AREA, FACI_CMD_FORCED_STOP);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = target_write_u8(target, FACI_CMD_AREA, FACI_CMD_STATUSCLR);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = rv40f_busy_wait(target, 2000);
	if (retval != ERROR_OK)
		return retval;

	/* check CMDLK bit from FASTAT register */
	retval = target_read_u8(target, FACI_FASTAT, &reg8);
	if (retval != ERROR_OK)
		return retval;

	/* check CMDLK actually unlocked */
	if ((reg8 & FACI_FASTAT_CMDLK) != 0) {
		LOG_ERROR("Failed to recover from CMDLK");
		retval = 1;
	}

	return retval;
}

/* wait for complete FACI command and do cleanup after lockup if needed */
static int rv40f_complete(struct target *target, int timeout)
{
	int retval;
	uint8_t reg8;
	uint32_t reg32;

	retval = rv40f_busy_wait(target, timeout);
	if (retval != ERROR_OK)
		return retval;

	/* check CMDLK bit from FASTAT register */
	retval = target_read_u8(target, FACI_FASTAT, &reg8);
	if (retval != ERROR_OK)
		return retval;

	/* check CMDLK bit */
	if (reg8 & 0x10) {
		retval = target_read_u32(target, FACI_FSTATR, &reg32);
		if (retval != ERROR_OK)
			return retval;

		if (reg32 & FACI_FSTATR_FLWEERR)
			LOG_ERROR("FSTATR: Flash write/erase protect error");
		if (reg32 & FACI_FSTATR_PRGSPD)
			LOG_ERROR("FSTATR: Programming Suspend");
		if (reg32 & FACI_FSTATR_ERSSPD)
			LOG_ERROR("FSTATR: Erasure suspend");
		if (reg32 & FACI_FSTATR_SUSRDY)
			LOG_ERROR("FSTATR: Suspend ready");
		if (reg32 & FACI_FSTATR_ERSERR)
			LOG_ERROR("FSTATR: Erasure Error");
		if (reg32 & FACI_FSTATR_ILGLERR)
			LOG_ERROR("FSTATR: Illegal Command Error Flag");
		if (reg32 & FACI_FSTATR_OTERR)
			LOG_ERROR("FSTATR: Other Error");
		if (reg32 & FACI_FSTATR_SECERR)
			LOG_ERROR("FSTATR: Security Error");
		if (reg32 & FACI_FSTATR_FESETERR)
			LOG_ERROR("FSTATR: FENTRY Setting Error");
		if (reg32 & FACI_FSTATR_ILGCOMERR)
			LOG_ERROR("FSTATR: Illegal Command");

		rv40f_unlock(target, reg32);
		retval = ERROR_FLASH_OPERATION_FAILED;
	}

	return retval;
}

/* switch to read mode after erasing or programming */
static int rv40f_to_readonly(struct target *target)
{
	int retval;
	uint32_t reg32;

	retval = target_read_u32(target, FACI_FSTATR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	/* operation in progress */
	if ((reg32 & FACI_FSTATR_FRDY) == 0)
		return ERROR_FLASH_OPERATION_FAILED;

	retval = target_write_u16(target, FACI_FENTRYR, 0xAA00);

	return retval;
}

static int rv40f_pe_mode(struct flash_bank *bank)
{
	struct renesas_bank *info = bank->driver_priv;
	int retval;
	uint16_t r16;

	/* unprotect memories from program/erase */
	retval = target_write_u8(bank->target, FACI_FWEPROR, 1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u16(bank->target, FACI_FENTRYR, &r16);
	if (retval != ERROR_OK)
		return retval;

	/* Enter data flash if needed*/
	if (info->is_data && ((r16 & 0xff) != 0x80)) {
		/* data flash */
		retval = target_write_u16(bank->target, FACI_FENTRYR, 0xAA80);
		if (retval != ERROR_OK)
			return retval;
	} else if (!info->is_data && ((r16 & 0xff) != 0x01)) {
		/* code flash */
		if (info->has_fmeprot) {
			/* Some platforms have this additional code flash protection */
			retval = target_write_u16(bank->target, FACI_FMEPROT, 0xD900);
			if (retval != ERROR_OK)
				return retval;
		}
		retval = target_write_u16(bank->target, FACI_FENTRYR, 0xAA01);
		if (retval != ERROR_OK)
			return retval;
	}

	/* check if the things are OK */
	retval = target_read_u16(bank->target, FACI_FENTRYR, &r16);
	if (retval != ERROR_OK)
		return retval;

	if (info->is_data && (r16 & 0xaa80) == 0x80) {
		LOG_INFO("FACI in data flash P/E mode ...");
	} else if ((r16 & 0xaa01) == 0x1) {
		LOG_INFO("FACI in code flash P/E mode ...");
	} else {
		LOG_ERROR("Flash not in P/E mode (0x%X)", r16);
		retval = ERROR_FLASH_OPERATION_FAILED;
	}

	return retval;
}

static int rv40f_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct renesas_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* enter P/E mode */
	retval = rv40f_pe_mode(bank);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int sector = first ; sector <= last ; sector++) {
		unsigned int offset = bank->sectors[sector].offset;

		/* start address */
		retval = target_write_u32(bank->target, FACI_FSADDR, offset + info->base);
		if (retval != ERROR_OK)
			return retval;

		/* fire the erase */
		retval = target_write_u8(target, FACI_CMD_AREA, FACI_CMD_ERASE1);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u8(target, FACI_CMD_AREA, FACI_CMD_PE_RESUME);
		if (retval != ERROR_OK)
			return retval;

		retval = rv40f_complete(target, TIMEOUT_ERASE_MS);
		if (retval != ERROR_OK)
			return retval;
	}

	/* switch back to readonly after operation */
	retval |= rv40f_to_readonly(target);

	return retval;
}

static int rv40f_write_block(struct flash_bank *bank, const uint8_t *buffer,
							 uint32_t offset, uint32_t count)
{
	struct renesas_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 2048;		/* Default minimum value */
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;
	unsigned int thisrun_count;
	uint8_t r8;

	/* Increase buffer_size if needed */
	if (buffer_size < (target->working_area_size / 2))
		buffer_size = (target->working_area_size / 2);

	/* RAMCODE used for rv40f Flash programming:
	 * R0 keeps number of halfwords to write (4,8,16 - data, 64 - program)
	 * R1 keeps source start address         (u32SourceStart)
	 * R2 keeps source end address           (u32SourceEnd)
	 * R3 keeps target start address         (u32Target) */
	unsigned char rv40f_flash_write_code[] = {
		0x89, 0x46, 0x4c, 0x46, 0x0e, 0x4d, 0x0f, 0x4f,
		0x81, 0x46, 0x4e, 0x46, 0x2b, 0x63, 0xe8, 0x20,
		0x38, 0x70, 0x48, 0x46, 0x38, 0x70, 0x20, 0x88,
		0x38, 0x80, 0x02, 0x34, 0x01, 0x3e, 0x09, 0xd0,
		0x20, 0x88, 0x38, 0x80, 0x02, 0x34, 0x80, 0x20,
		0x40, 0x19, 0x01, 0x68, 0x06, 0x48, 0x08, 0x42,
		0xf9, 0xd1, 0xf3, 0xe7, 0xd0, 0x20, 0x38, 0x70,
		0x80, 0x21, 0x49, 0x19, 0x08, 0x68, 0x00, 0xbe,
		0x00, 0xe0, 0x7f, 0x40, 0x00, 0x00, 0x7e, 0x40,
		0x00, 0x04, 0x00, 0x00
	};

	LOG_INFO("Renesas RV40F FLASH Write ...");

	if (info->is_data) {
		/* check alignment - data flash */
		if (offset & 0x7) {
			LOG_WARNING("offset 0x%" PRIx32 " breaks required 8-byte alignment", offset);
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		}
	} else {
		/* check alignment - code flash */
		if (offset & 0x7f) {
			LOG_WARNING("offset 0x%" PRIx32 " breaks required 128-byte alignment", offset);
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		}
	}

	/* enter P/E mode */
	retval = rv40f_pe_mode(bank);
	if (retval != ERROR_OK)
		return retval;

	count = count / 2; /* number bytes -> number halfwords */

	/* allocate working area and variables with flash programming code */
	if (target_alloc_working_area(target, sizeof(rv40f_flash_write_code),
		&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
	sizeof(rv40f_flash_write_code), rv40f_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* free working area, write algorithm already allocated */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("No large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); /* number of halfwords to program */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /* source start address */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); /* source end address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT); /* target start address */

	/* write code buffer and use Flash programming code within rv40f */
	/* Set breakpoint to 0 with time-out of 1000 ms */
	while (count > 0) {
		/* dataflash has a write count 2,4,8 halfwords */
		if (info->is_data) {
			if (count >= 8)
				thisrun_count = 8;
			else if (count >= 4)
				thisrun_count = 4;
			else
				thisrun_count = 2;
			/* code flash has 64 halfwords programming block */
		} else {
			thisrun_count = 64;
		}

		retval = target_write_buffer(target, source->address, thisrun_count * 2, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, thisrun_count);
		buf_set_u32(reg_params[1].value, 0, 32, source->address);
		buf_set_u32(reg_params[2].value, 0, 32, source->address + thisrun_count * 2);
		buf_set_u32(reg_params[3].value, 0, 32, address);
		retval = target_run_algorithm(target, 0, NULL, 4, reg_params,
			write_algorithm->address, 0, 1000, &armv7m_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error executing RV40f Flash programming algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		target_read_u8(bank->target, FACI_FWEPROR, &r8);

		if (retval != ERROR_OK || r8 != 1) {
			LOG_ERROR("Flash locked for write");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		/* wait for write 1000 ms */
		retval = rv40f_busy_wait(target, TIMEOUT_WRITE_MS);
		if (retval != ERROR_OK)
			return retval;

		/* decrement counters */
		if (count >= thisrun_count) {
			buffer  += thisrun_count * 2;
			address += thisrun_count * 2;
			count   -= thisrun_count;
		} else {
			count = 0;
			break;
		}
	}

	/* switch back to readonly after operation */
	retval = rv40f_to_readonly(target);
	if (retval != ERROR_OK)
		return retval;

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	return retval;
}

static int rv40f_probe(struct flash_bank *bank)
{
	struct renesas_bank *info = bank->driver_priv;
	struct flash_sector *s;
	unsigned int n, num_pages;
	char part_number[17] = {0};
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	info->probed = false;
	bank->base = info->base;
	bank->size = info->size; /* bytes */

	if (info->base > 0) {
		/* data flash - same for all types */
		info->is_data = 1;
		num_pages = info->size / 64; /* 1 block = 64 bytes */
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		if (!bank->sectors)
			return ERROR_FAIL;
		for (n = 0; n < num_pages; n++) {
			s = &bank->sectors[n];
			s->offset = n * 64;
			s->size = 64;
			s->is_erased = -1;
			s->is_protected = -1;
		}
	} else {
		/* code flash 8k x 8 + 32k x N */
		info->is_data = 0;
		num_pages = (info->size - 8 * 8192) / 32768 + 8;
		bank->num_sectors = num_pages;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
		if (!bank->sectors)
			return ERROR_FAIL;
		for (n = 0; n < 8; n++) {
			s = &bank->sectors[n];
			s->offset = n * 8192;
			s->size = 8192;
			s->is_erased = -1;
			s->is_protected = -1;
		}
		/* rest of sectors (bigger) */
		for (; n < num_pages; n++) {
			s = &bank->sectors[n];
			s->offset = (n - 8) * 32768 + 8 * 8192;
			s->size = 32768;
			s->is_erased = -1;
			s->is_protected = -1;
		}
	}

	retval = target_read_memory(bank->target, FACI_PNR, 4, 4, (uint8_t *)part_number);
	if (retval != ERROR_OK)
		return retval;

	/* Trim out end spaces */
	for (int i = 15; i != 0; i--)
		if (part_number[i] == ' ')
			part_number[i] = 0;
		else
			break;

	LOG_INFO("Renesas RV40 \"%s\" detected", part_number);

	if (strncmp(part_number, "R7FA4M2", 7) == 0)
		info->has_fmeprot = true;

	info->probed = true;

	return ERROR_OK;
}

static int rv40f_auto_probe(struct flash_bank *bank)
{
	struct renesas_bank *info = bank->driver_priv;

	if (info->probed)
		return ERROR_OK;

	return rv40f_probe(bank);
}

const struct flash_driver renesas_rv40f_flash = {
	.name = "renesas_rv40f",
	.flash_bank_command = rv40f_flash_bank_command,
	.erase = rv40f_erase,
	.write = rv40f_write_block,
	.read = default_flash_read,
	.probe = rv40f_probe,
	.auto_probe = rv40f_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
	.usage = "flash bank bank_id 'renesas_rv40f' base_address size"
};
