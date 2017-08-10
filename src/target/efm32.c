/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015  Richard Meadows <richardeoin>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements EFM32 target specific functions for
 * detecting the device, providing the XML memory map and Flash memory
 * programming.
 *
 * Both EFM32 (microcontroller only) and EZR32 (microcontroller+radio)
 * devices should be supported through this driver.
 *
 * Tested with:
 * * EZR32LG230 (EZR Leopard Gecko M3)
 * *
 */

/* Refer to the family reference manuals:
 *
 *
 * Also refer to AN0062 "Programming Internal Flash Over the Serial Wire Debug Interface"
 * http://www.silabs.com/Support%20Documents/TechnicalDocs/an0062.pdf
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"

#define SRAM_BASE		0x20000000
#define STUB_BUFFER_BASE	ALIGN(SRAM_BASE + sizeof(efm32_flash_write_stub), 4)

static int efm32_flash_erase(struct target_flash *t, target_addr addr, size_t len);
static int efm32_flash_write(struct target_flash *f,
			     target_addr dest, const void *src, size_t len);

static const uint16_t efm32_flash_write_stub[] = {
#include "flashstub/efm32.stub"
};

static bool efm32_cmd_erase_all(target *t);
static bool efm32_cmd_serial(target *t);

const struct command_s efm32_cmd_list[] = {
	{"erase_mass", (cmd_handler)efm32_cmd_erase_all, "Erase entire flash memory"},
	{"serial", (cmd_handler)efm32_cmd_serial, "Prints unique number"},
	{NULL, NULL, NULL}
};



/* -------------------------------------------------------------------------- */
/* Memory System Controller (MSC) Registers */
/* -------------------------------------------------------------------------- */
#define DRACO

#ifdef DRACO
#define EFM32_MSC					0x400e0000
#define EFM32_MSC_LOCK				(EFM32_MSC+0x040)
#define EFM32_MSC_CMD				(EFM32_MSC+0x074)
#else
#define EFM32_MSC					0x400c0000
#define EFM32_MSC_LOCK				(EFM32_MSC+0x03c)
#define EFM32_MSC_CMD				(EFM32_MSC+0x040)
#endif

#define EFM32_MSC_WRITECTRL			(EFM32_MSC+0x008)
#define EFM32_MSC_WRITECMD			(EFM32_MSC+0x00c)
#define EFM32_MSC_ADDRB				(EFM32_MSC+0x010)
#define EFM32_MSC_WDATA				(EFM32_MSC+0x018)
#define EFM32_MSC_STATUS			(EFM32_MSC+0x01c)
#define EFM32_MSC_MASSLOCK			(EFM32_MSC+0x054)

#define EFM32_MSC_LOCK_LOCKKEY		0x1b71
#define EFM32_MSC_MASSLOCK_LOCKKEY	0x631a

#define EFM32_MSC_WRITECMD_LADDRIM		(1<<0)
#define EFM32_MSC_WRITECMD_ERASEPAGE	(1<<1)
#define EFM32_MSC_WRITECMD_WRITEEND		(1<<2)
#define EFM32_MSC_WRITECMD_WRITEONCE	(1<<3)
#define EFM32_MSC_WRITECMD_WRITETRIG	(1<<4)
#define EFM32_MSC_WRITECMD_ERASEABORT	(1<<5)
#define EFM32_MSC_WRITECMD_ERASEMAIN0	(1<<8)

#define EFM32_MSC_STATUS_BUSY			(1<<0)
#define EFM32_MSC_STATUS_LOCKED			(1<<1)
#define EFM32_MSC_STATUS_INVADDR		(1<<2)
#define EFM32_MSC_STATUS_WDATAREADY		(1<<3)


/* -------------------------------------------------------------------------- */
/* Flash Infomation Area */
/* -------------------------------------------------------------------------- */

#define EFM32_INFO			0x0fe00000
#define EFM32_USER_DATA			(EFM32_INFO+0x0000)
#define EFM32_LOCK_BITS			(EFM32_INFO+0x4000)
#define EFM32_DI			(EFM32_INFO+0x8000)


/* -------------------------------------------------------------------------- */
/* Device Information (DI) Area */
/* -------------------------------------------------------------------------- */


#define EFM32_DI_RADIO_REV_MIN 		(EFM32_DI+0x1AC)
#define EFM32_DI_RADIO_REV_MAJ 		(EFM32_DI+0x1AD)
#define EFM32_DI_RADIO_OPN 			(EFM32_DI+0x1AE)

// Gen 1 DI
#define EFM32_DI_DI_CRC 			(EFM32_DI+0x1B0)
#define EFM32_DI_MEM_INFO_PAGE_SIZE (EFM32_DI+0x1E7)
#define EFM32_DI_RADIO_ID 			(EFM32_DI+0x1EE)
#define EFM32_DI_EUI64_0 			(EFM32_DI+0x1F0)
#define EFM32_DI_EUI64_1 			(EFM32_DI+0x1F4)
#define EFM32_DI_MEM_INFO_FLASH 	(EFM32_DI+0x1F8)
#define EFM32_DI_MEM_INFO_RAM 		(EFM32_DI+0x1FA)
#define EFM32_DI_PART_NUMBER 		(EFM32_DI+0x1FC)
#define EFM32_DI_PART_FAMILY 		(EFM32_DI+0x1FE)
#define EFM32_DI_PROD_REV 			(EFM32_DI+0x1FF)

// Gen 2 DI
#define EFM32_G2_DI_CRC 			0x000 	// CRC of DI-page and calibration temperature (RO)
#define EFM32_G2_DI_EUI48L 			0x028 	// EUI48 OUI and Unique identifier (RO)
#define EFM32_G2_DI_EUI48H 			0x02 	// OUI (RO)
#define EFM32_G2_DI_CUSTOMINFO 		0x030 	// Custom information (RO)
#define EFM32_G2_DI_MEMINFO 		0x034 	// Flash page size and misc. chip information (RO)
#define EFM32_G2_DI_UNIQUEL 		0x040 	// Low 32 bits of device unique number (RO)
#define EFM32_G2_DI_UNIQUEH 		0x044 	// High 32 bits of device unique number (RO)
#define EFM32_G2_DI_MSIZE 			0x048 	// Flash and SRAM Memory size in kB (RO)
#define EFM32_G2_DI_PART 			0x04 	// Part description (RO)
#define EFM32_G2_DI_DEVINFOREV 		0x050 	// Device information page revision (RO)

/* top 24 bits of eui */
#define EFM32_DI_EUI_SILABS	0x000b57


// Struct to enumerate different devices
typedef struct {
	uint16_t 	family;  			// Family for device matching
	char* 	 	name;	 			// Friendly device family name
	uint32_t    flash_page_size;	// Flash page size
	uint32_t 	msc_offset;			// Offset for MSC
	bool 		has_radio;			// Indicates a device has attached radio
} efm32_device_t;

efm32_device_t efm32_devices[] = {
	// Second gen devices micro + Radio
	{16, "EFR32MG1P", 	2048,	0x400e0000, true},
	{17, "EFR32MG1B", 	2048,	0x400e0000, true},
	{18, "EFR32MG1V", 	2048,	0x400e0000, true},
	{19, "EFR32BG1P", 	2048,	0x400e0000, true},
	{20, "EFR32BG1B", 	2048,	0x400e0000, true},
	{21, "EFR32BG1V", 	2048,	0x400e0000, true},
	{25, "EFR32FG1P", 	2048,	0x400e0000, true},
	{26, "EFR32FG1B", 	2048,	0x400e0000, true},
	{27, "EFR32FG1V", 	2048,	0x400e0000, true},
	{28, "EFR32MG12P", 	2048,	0x400e0000, true},
	{28, "EFR32MG2P", 	2048,	0x400e0000, true},
	{29, "EFR32MG12B", 	2048,	0x400e0000, true},
	{30, "EFR32MG12V", 	2048,	0x400e0000, true},
	{31, "EFR32BG12P", 	2048,	0x400e0000, true},
	{32, "EFR32BG12B", 	2048,	0x400e0000, true},
	{33, "EFR32BG12V", 	2048,	0x400e0000, true},
	{37, "EFR32FG12P", 	2048,	0x400e0000, true},
	{38, "EFR32FG12B", 	2048,	0x400e0000, true},
	{39, "EFR32FG12V", 	2048,	0x400e0000, true},
	{40, "EFR32MG13P", 	2048,	0x400e0000, true},
	{41, "EFR32MG13B", 	2048,	0x400e0000, true},
	{42, "EFR32MG13V", 	2048,	0x400e0000, true},
	{43, "EFR32BG13P", 	2048,	0x400e0000, true},
	{44, "EFR32BG13B", 	2048,	0x400e0000, true},
	{45, "EFR32BG13V", 	2048,	0x400e0000, true},
	{49, "EFR32FG13P", 	2048,	0x400e0000, true},
	{50, "EFR32FG13B", 	2048,	0x400e0000, true},
	{51, "EFR32FG13V", 	2048,	0x400e0000, true},
	// Second gen micros
	{81, "EFM32PG1B",	2048,	0x400e0000, false},
	{83, "EFM32JG1B",	2048,	0x400e0000, false},
	// First gen micros
	{71, "EFM32G",		512,	0x400c0000, false},
	{72, "EFM32GG",		2048,	0x400c0000, false},
	{73, "EFM32TG",		512,	0x400c0000, false},
	{74, "EFM32LG",		2048,	0x400c0000, false},
	{75, "EFM32WG",		2048,	0x400c0000, false},
	{76, "EFM32ZG",		1024,	0x400c0000, false},
	{77, "EFM32HG",		1024,	0x400c0000, false},
	// First (1.5) gen micro + radios
	{120, "EFR32WG",	2048,	0x400c0000, true},
	{121, "EFR32LG",	2048,	0x400c0000, true},
};

/* -------------------------------------------------------------------------- */
/* Helper functions */
/* -------------------------------------------------------------------------- */

/**
 * Reads the EFM32 Extended Unique Identifier
 */
	uint64_t efm32_read_eui(target *t)
	{
		uint64_t eui;

		eui  = (uint64_t)target_mem_read32(t, EFM32_DI_EUI64_1) << 32;
		eui |= (uint64_t)target_mem_read32(t, EFM32_DI_EUI64_0) <<  0;

		return eui;
	}
/**
 * Reads the EFM32 flash size in kiB
 */
uint16_t efm32_read_flash_size(target *t)
{
	return target_mem_read16(t, EFM32_DI_MEM_INFO_FLASH);
}
/**
 * Reads the EFM32 RAM size in kiB
 */
uint16_t efm32_read_ram_size(target *t)
{
	return target_mem_read16(t, EFM32_DI_MEM_INFO_RAM);
}
/**
 * Reads the EFM32 Part Number
 */
uint16_t efm32_read_part_number(target *t)
{
	return target_mem_read16(t, EFM32_DI_PART_NUMBER);
}
/**
 * Reads the EFM32 Part Family
 */
uint8_t efm32_read_part_family(target *t)
{
	return target_mem_read8(t, EFM32_DI_PART_FAMILY);
}
/**
 * Reads the EFM32 Radio part number (EZR parts only)
 */
uint16_t efm32_read_radio_part_number(target *t)
{
	return target_mem_read16(t, EFM32_DI_RADIO_OPN);
}




static void efm32_add_flash(target *t, target_addr addr, size_t length,
			    size_t page_size)
{
	struct target_flash *f = calloc(1, sizeof(*f));
	f->start = addr;
	f->length = length;
	f->blocksize = page_size;
	f->erase = efm32_flash_erase;
	f->write = target_flash_write_buffered;
	f->done = target_flash_done_buffered;
	f->write_buf = efm32_flash_write;
	f->buf_size = page_size;
	target_add_flash(t, f);
}

char variant_string[40];
bool efm32_probe(target *t)
{
	/* Read the IDCODE register from the SW-DP */
	ADIv5_AP_t *ap = cortexm_ap(t);
	uint32_t ap_idcode = ap->dp->idcode;

	/* Check the idcode is silabs. See AN0062 Section 2.2 */
	if (ap_idcode == 0x2BA01477) {
		/* Cortex M3, Cortex M4 */
	} else if (ap_idcode == 0x0BC11477) {
		/* Cortex M0+ */
	} else {
		return false;
	}

	/* Read the part number and family */
	uint16_t part_number = efm32_read_part_number(t);
	uint8_t part_family = efm32_read_part_family(t);
	uint16_t radio_number;
	uint32_t flash_page_size;

	DEBUG("efm32_probe - part_number: %d part_family: %d\n", part_number, part_family);

	efm32_device_t* device = NULL;
	for (size_t i=0; i<(sizeof(efm32_devices) / sizeof(efm32_device_t)); i++) {
		if (efm32_devices[i].family == part_family) {
			device = &efm32_devices[i];
			break;
		}
	}
	
	if (device == NULL) {
		return false;
	}
	
	flash_page_size = device->flash_page_size;

	if (!device->has_radio) {
		sprintf(variant_string, "%s", device->name);
	} else {
		radio_number = efm32_read_radio_part_number(t); /* on-chip radio */
		sprintf(variant_string, "%s (radio: %d)", device->name, radio_number);
	}

	/* Read memory sizes, convert to bytes */
	uint32_t flash_size = efm32_read_flash_size(t) * 0x400;
	uint32_t ram_size   = efm32_read_ram_size(t)   * 0x400;

	/* Setup Target */
	t->target_options |= CORTEXM_TOPT_INHIBIT_SRST;
	t->driver = variant_string;
	tc_printf(t, "flash size %d page size %d\n", flash_size, flash_page_size);
	target_add_ram (t, SRAM_BASE, ram_size);
	efm32_add_flash(t, 0x00000000, flash_size, flash_page_size);
	target_add_commands(t, efm32_cmd_list, "EFM32");

	return true;
}

/**
 * Erase flash row by row
 */
static int efm32_flash_erase(struct target_flash *f, target_addr addr, size_t len)
{
	target *t = f->t;

	/* Set WREN bit to enabel MSC write and erase functionality */
	target_mem_write32(t, EFM32_MSC_WRITECTRL, 1);

	while (len) {
		/* Write address of first word in row to erase it */
		target_mem_write32(t, EFM32_MSC_ADDRB, addr);
		target_mem_write32(t, EFM32_MSC_WRITECMD, EFM32_MSC_WRITECMD_LADDRIM);

		/* Issue the erase command */
		target_mem_write32(t, EFM32_MSC_WRITECMD, EFM32_MSC_WRITECMD_ERASEPAGE );

		/* Poll MSC Busy */
		while ((target_mem_read32(t, EFM32_MSC_STATUS) & EFM32_MSC_STATUS_BUSY)) {
			if (target_check_error(t))
				return -1;
		}

		addr += f->blocksize;
		len -= f->blocksize;
	}

	return 0;
}

/**
 * Write flash page by page
 */
static int efm32_flash_write(struct target_flash *f,
			     target_addr dest, const void *src, size_t len)
{
	(void)len;
	target *t = f->t;

	/* Write flashloader */
	target_mem_write(t, SRAM_BASE, efm32_flash_write_stub,
			 sizeof(efm32_flash_write_stub));
	/* Write Buffer */
	target_mem_write(t, STUB_BUFFER_BASE, src, len);
	/* Run flashloader */
	return cortexm_run_stub(t, SRAM_BASE, dest, STUB_BUFFER_BASE, len, 0);

	return 0;
}

/**
 * Uses the MSC ERASEMAIN0 command to erase the entire flash
 */
static bool efm32_cmd_erase_all(target *t)
{
	/* Set WREN bit to enabel MSC write and erase functionality */
	target_mem_write32(t, EFM32_MSC_WRITECTRL, 1);

	/* Unlock mass erase */
	target_mem_write32(t, EFM32_MSC_MASSLOCK, EFM32_MSC_MASSLOCK_LOCKKEY);

	/* Erase operation */
	target_mem_write32(t, EFM32_MSC_WRITECMD, EFM32_MSC_WRITECMD_ERASEMAIN0);

	/* Poll MSC Busy */
	while ((target_mem_read32(t, EFM32_MSC_STATUS) & EFM32_MSC_STATUS_BUSY)) {
		if (target_check_error(t))
			return false;
	}

	/* Relock mass erase */
	target_mem_write32(t, EFM32_MSC_MASSLOCK, 0);

	tc_printf(t, "Erase successful!\n");

	return true;
}

/**
 * Reads the 40-bit unique number
 */
static bool efm32_cmd_serial(target *t)
{
	/* Read the extended unique identifier */
	uint64_t eui = efm32_read_eui(t);

	/* 64 bits of unique number */
	tc_printf(t, "Unique Number: 0x%016llx\n", eui);

	return true;
}
