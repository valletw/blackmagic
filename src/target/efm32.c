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
 * detecting the device, providing the memory map and Flash memory
 * programming.
 *
 * EFM32, EZR32 and EFR32 devices are all currently supported through
 * this driver.
 *
 * Tested with:
 * * EZR32LG230 (EZR Leopard Gecko M3)
 * * EFR32BG13P532F512GM32 (EFR Blue Gecko)
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

static bool efm32_cmd_erase_all(target *t, int argc, const char **argv);
static bool efm32_cmd_serial(target *t, int argc, const char **argv);
static bool efm32_cmd_efm_info(target *t, int argc, const char **argv);
static bool efm32_cmd_bootloader(target *t, int argc, const char **argv);

const struct command_s efm32_cmd_list[] = {
	{"erase_mass", (cmd_handler)efm32_cmd_erase_all, "Erase entire flash memory"},
	{"serial", (cmd_handler)efm32_cmd_serial, "Prints unique number"},
	{"efm_info", (cmd_handler)efm32_cmd_efm_info, "Prints information about the device"},
	{"bootloader", (cmd_handler)efm32_cmd_bootloader, "Bootloader status in CLW0"},
	{NULL, NULL, NULL}
};

/* -------------------------------------------------------------------------- */
/* Memory System Controller (MSC) Registers */
/* -------------------------------------------------------------------------- */

#define EFM32_MSC_WRITECTRL(msc)   (msc+(msc == 0x40030000?0x0c:0x08))
#define EFM32_MSC_WRITECMD(msc)    (msc+(msc == 0x40030000?0x10:0x0c))
#define EFM32_MSC_ADDRB(msc)       (msc+(msc == 0x40030000?0x14:0x10))
#define EFM32_MSC_WDATA(msc)       (msc+0x018)
#define EFM32_MSC_STATUS(msc)      (msc+0x01c)
#define EFM32_MSC_IF(msc)          (msc+(msc == 0x40030000?0x20:0x30))
#define EFM32_MSC_LOCK(msc)        (msc+((msc == 0x40030000)\
                                       ||(msc == 0x400c0000)?0x3c:0x40))
#define EFM32_MSC_MASSLOCK(msc)    (msc+(msc == 0x40030000?0x40:0x54))

#define EFM32_MSC_LOCK_LOCKKEY        0x1b71
#define EFM32_MSC_MASSLOCK_LOCKKEY    0x631a

#define EFM32_MSC_WRITECMD_LADDRIM       (1<<0)
#define EFM32_MSC_WRITECMD_ERASEPAGE     (1<<1)
#define EFM32_MSC_WRITECMD_WRITEEND      (1<<2)
#define EFM32_MSC_WRITECMD_WRITEONCE     (1<<3)
#define EFM32_MSC_WRITECMD_WRITETRIG     (1<<4)
#define EFM32_MSC_WRITECMD_ERASEABORT    (1<<5)
#define EFM32_MSC_WRITECMD_ERASEMAIN0    (1<<8)

#define EFM32_MSC_STATUS_BUSY            (1<<0)
#define EFM32_MSC_STATUS_LOCKED          (1<<1)
#define EFM32_MSC_STATUS_INVADDR         (1<<2)
#define EFM32_MSC_STATUS_WDATAREADY      (1<<3)

/* -------------------------------------------------------------------------- */
/* Flash Infomation Area */
/* -------------------------------------------------------------------------- */

#define EFM32_INFO          0x0fe00000
#define EFM32_USER_DATA    (EFM32_INFO+0x00000)
#define EFM32_LOCK_BITS    (EFM32_INFO+0x04000)    /* EFR32xG2x has no Lock Bits page in Flash */
#define EFM32_DI_V1        (EFM32_INFO+0x081B0)    /* EFM32xG */
#define EFM32_DI_V2        (EFM32_INFO+0x081A8)    /* EZR32xG */
#define EFM32_DI_V3        (EFM32_INFO+0x081B0)    /* EFR32xG1x */
#define EFM32_DI_V4        (EFM32_INFO+0x08000)    /* EFR32xG2x */
#define EFM32_BOOTLOADER   (EFM32_INFO+0x10000)

/* -------------------------------------------------------------------------- */
/* Lock Bits (LB) */
/* -------------------------------------------------------------------------- */

#define EFM32_LOCK_BITS_DLW     (EFM32_LOCK_BITS+(4*127))
#define EFM32_LOCK_BITS_ULW     (EFM32_LOCK_BITS+(4*126))
#define EFM32_LOCK_BITS_MLW     (EFM32_LOCK_BITS+(4*125))
#define EFM32_LOCK_BITS_CLW0    (EFM32_LOCK_BITS+(4*122))

#define EFM32_CLW0_BOOTLOADER_ENABLE    (1<<1)
#define EFM32_CLW0_PINRESETSOFT         (1<<2)

/* -------------------------------------------------------------------------- */
/* Device Information (DI) Area */
/* -------------------------------------------------------------------------- */
/* Information extracted from Gecko SDK v2.7 */

#define EFM32_DI_PART_NUMBER_OFST     0
#define EFM32_DI_PART_FAMILY_OFST    16
#define EFM32_DI_PART_NUMBER_MASK    0xFFFF
#define EFM32_DI_PART_FAMILY_MASK    0xFF

#define EFM32_DI_V4_PART_FAMILYNUM_OFST    16
#define EFM32_DI_V4_PART_FAMILYNUM_MASK    0x3F
#define EFM32_DI_V4_PART_FAMILY_OFST       24
#define EFM32_DI_V4_PART_FAMILY_MASK       0x3F

#define EFM32_DI_MSIZE_FLASH_OFST     0
#define EFM32_DI_MSIZE_SRAM_OFST     16
#define EFM32_DI_MSIZE_FLASH_MASK    0xFFFF
#define EFM32_DI_MSIZE_SRAM_MASK     0xFFFF

#define EFM32_DI_MEMINFO_FLASHPAGESIZE_OFST    24
#define EFM32_DI_MEMINFO_FLASHPAGESIZE_MASK    0xFF

#define EFM32_DI_V4_MEMINFO_FLASHPAGESIZE_OFST    0
#define EFM32_DI_V4_MEMINFO_FLASHPAGESIZE_MASK    0xFF

#define EFM32_DI_PKGINFO_TEMPGRADE_OFST         0
#define EFM32_DI_PKGINFO_PKGTYPE_OFST           8
#define EFM32_DI_PKGINFO_PINCOUNT_OFST         16
#define EFM32_DI_PKGINFO_TEMPGRADE_MASK        0xFF
#define EFM32_DI_PKGINFO_PKGTYPE_MASK          0xFF
#define EFM32_DI_PKGINFO_PINCOUNT_MASK         0xFF

/* top 24 bits of eui */
#define EFM32_DI_V0_DI_EUI_SILABS         0x000b57
#define EFM32_DI_V3_DI_EUI_ENERGYMICRO    0xd0cf5e

/* EFM32xG */
typedef struct {
	uint32_t CAL;           /* Calibration temperature and checksum */
	uint32_t ADC0CAL0;      /* ADC0 Calibration register 0 */
	uint32_t ADC0CAL1;      /* ADC0 Calibration register 1 */
	uint32_t ADC0CAL2;      /* ADC0 Calibration register 2 */
	uint32_t RESERVED0[2U];
	uint32_t DAC0CAL0;      /* DAC calibrartion register 0 */
	uint32_t DAC0CAL1;      /* DAC calibrartion register 1 */
	uint32_t DAC0CAL2;      /* DAC calibrartion register 2 */
	uint32_t AUXHFRCOCAL0;  /* AUXHFRCO calibration register 0 */
	uint32_t AUXHFRCOCAL1;  /* AUXHFRCO calibration register 1 */
	uint32_t HFRCOCAL0;     /* HFRCO calibration register 0 */
	uint32_t HFRCOCAL1;     /* HFRCO calibration register 1 */
	uint32_t MEMINFO;       /* Memory information */
	uint32_t RESERVED2[2U];
	uint32_t UNIQUEL;       /* Low 32 bits of device unique number */
	uint32_t UNIQUEH;       /* High 32 bits of device unique number */
	uint32_t MSIZE;         /* Flash and SRAM Memory size in KiloBytes */
	uint32_t PART;          /* Part description */
} di_v1_t;

/* EZR32xG */
typedef struct {
	uint32_t RADIO0;        /* Radio information 0 */
	uint32_t RADIO1;        /* Radio information 1 */
	uint32_t CAL;           /* Calibration temperature and checksum */
	uint32_t ADC0CAL0;      /* ADC0 Calibration register 0 */
	uint32_t ADC0CAL1;      /* ADC0 Calibration register 1 */
	uint32_t ADC0CAL2;      /* ADC0 Calibration register 2 */
	uint32_t RESERVED0[2];
	uint32_t DAC0CAL0;      /* DAC calibrartion register 0 */
	uint32_t DAC0CAL1;      /* DAC calibrartion register 1 */
	uint32_t DAC0CAL2;      /* DAC calibrartion register 2 */
	uint32_t AUXHFRCOCAL0;  /* AUXHFRCO calibration register 0 */
	uint32_t AUXHFRCOCAL1;  /* AUXHFRCO calibration register 1 */
	uint32_t HFRCOCAL0;     /* HFRCO calibration register 0 */
	uint32_t HFRCOCAL1;     /* HFRCO calibration register 1 */
	uint32_t MEMINFO;       /* Memory information */
	uint32_t RESERVED2;
	uint32_t RADIO2;        /* Radio information 2 */
	uint32_t UNIQUEL;       /* Low 32 bits of device unique number */
	uint32_t UNIQUEH;       /* High 32 bits of device unique number */
	uint32_t MSIZE;         /* Flash and SRAM Memory size in KiloBytes */
	uint32_t PART;          /* Part description */
} di_v2_t;

/* EFR32xG1x */
typedef struct {
	uint32_t CAL;              /* CRC of DI-page and calibration temperature */
	uint32_t MODULEINFO;       /* Module trace information */
	uint32_t MODXOCAL;         /* Module Crystal Oscillator Calibration */
	uint32_t RESERVED0[5U];
	uint32_t EXTINFO;          /* External Component description */
	uint32_t RESERVED1[1U];
	uint32_t EUI48L;           /* EUI48 OUI and Unique identifier */
	uint32_t EUI48H;           /* OUI */
	uint32_t CUSTOMINFO;       /* Custom information */
	uint32_t MEMINFO;          /* Flash page size and misc. chip information */
	uint32_t RESERVED2[2U];
	uint32_t UNIQUEL;          /* Low 32 bits of device unique number */
	uint32_t UNIQUEH;          /* High 32 bits of device unique number */
	uint32_t MSIZE;            /* Flash and SRAM Memory size in kB */
	uint32_t PART;             /* Part description */
	uint32_t DEVINFOREV;       /* Device information page revision */
	uint32_t EMUTEMP;          /* EMU Temperature Calibration Information */
	uint32_t RESERVED3[2U];
	uint32_t ADC0CAL0;         /* ADC0 calibration register 0 */
	uint32_t ADC0CAL1;         /* ADC0 calibration register 1 */
	uint32_t ADC0CAL2;         /* ADC0 calibration register 2 */
	uint32_t ADC0CAL3;         /* ADC0 calibration register 3 */
	uint32_t RESERVED4[4U];
	uint32_t HFRCOCAL0;        /* HFRCO Calibration Register (4 MHz) */
	uint32_t RESERVED5[2U];
	uint32_t HFRCOCAL3;        /* HFRCO Calibration Register (7 MHz) */
	uint32_t RESERVED6[2U];
	uint32_t HFRCOCAL6;        /* HFRCO Calibration Register (13 MHz) */
	uint32_t HFRCOCAL7;        /* HFRCO Calibration Register (16 MHz) */
	uint32_t HFRCOCAL8;        /* HFRCO Calibration Register (19 MHz) */
	uint32_t RESERVED7[1U];
	uint32_t HFRCOCAL10;       /* HFRCO Calibration Register (26 MHz) */
	uint32_t HFRCOCAL11;       /* HFRCO Calibration Register (32 MHz) */
	uint32_t HFRCOCAL12;       /* HFRCO Calibration Register (38 MHz) */
	uint32_t RESERVED8[11U];
	uint32_t AUXHFRCOCAL0;     /* AUXHFRCO Calibration Register (4 MHz) */
	uint32_t RESERVED9[2U];
	uint32_t AUXHFRCOCAL3;     /* AUXHFRCO Calibration Register (7 MHz) */
	uint32_t RESERVED10[2U];
	uint32_t AUXHFRCOCAL6;     /* AUXHFRCO Calibration Register (13 MHz) */
	uint32_t AUXHFRCOCAL7;     /* AUXHFRCO Calibration Register (16 MHz) */
	uint32_t AUXHFRCOCAL8;     /* AUXHFRCO Calibration Register (19 MHz) */
	uint32_t RESERVED11[1U];
	uint32_t AUXHFRCOCAL10;    /* AUXHFRCO Calibration Register (26 MHz) */
	uint32_t AUXHFRCOCAL11;    /* AUXHFRCO Calibration Register (32 MHz) */
	uint32_t AUXHFRCOCAL12;    /* AUXHFRCO Calibration Register (38 MHz) */
	uint32_t RESERVED12[11U];
	uint32_t VMONCAL0;         /* VMON Calibration Register 0 */
	uint32_t VMONCAL1;         /* VMON Calibration Register 1 */
	uint32_t VMONCAL2;         /* VMON Calibration Register 2 */
	uint32_t RESERVED13[3U];
	uint32_t IDAC0CAL0;        /* IDAC0 Calibration Register 0 */
	uint32_t IDAC0CAL1;        /* IDAC0 Calibration Register 1 */
	uint32_t RESERVED14[2U];
	uint32_t DCDCLNVCTRL0;     /* DCDC Low-noise VREF Trim Register 0 */
	uint32_t DCDCLPVCTRL0;     /* DCDC Low-power VREF Trim Register 0 */
	uint32_t DCDCLPVCTRL1;     /* DCDC Low-power VREF Trim Register 1 */
	uint32_t DCDCLPVCTRL2;     /* DCDC Low-power VREF Trim Register 2 */
	uint32_t DCDCLPVCTRL3;     /* DCDC Low-power VREF Trim Register 3 */
	uint32_t DCDCLPCMPHYSSEL0; /* DCDC LPCMPHYSSEL Trim Register 0 */
	uint32_t DCDCLPCMPHYSSEL1; /* DCDC LPCMPHYSSEL Trim Register 1 */
} di_v3_t;

/* EFR32xG2x */
typedef struct {
	uint32_t INFO;                  /* DI Information */
	uint32_t PART;                  /* Part Info */
	uint32_t MEMINFO;               /* Memory Info */
	uint32_t MSIZE;                 /* Memory Size */
	uint32_t PKGINFO;               /* Misc Device Info */
	uint32_t CUSTOMINFO;            /* Custom Part Info */
	uint32_t SWFIX;                 /* SW Fix Register */
	uint32_t SWCAPA0;               /* Software Restriction */
	uint32_t SWCAPA1;               /* Software Restriction */
	uint32_t RESERVED0[1U];
	uint32_t EXTINFO;               /* External Component Info */
	uint32_t RESERVED1[2U];
	uint32_t RESERVED2[3U];
	uint32_t EUI48L;                /* EUI 48 Low */
	uint32_t EUI48H;                /* EUI 48 High */
	uint32_t EUI64L;                /* EUI64 Low */
	uint32_t EUI64H;                /* EUI64 High */
	uint32_t CALTEMP;               /* Calibration temperature Information */
	uint32_t EMUTEMP;               /* EMU Temperature Sensor Calibration Information */
	uint32_t HFRCODPLLCAL[18U];     /* HFRCODPLL Calibration */
	uint32_t HFRCOEM23CAL[18U];     /* HFRCOEM23 Calibration */
	uint32_t HFRCOSECAL[18U];       /* HFRCOSECAL Calibration */
	uint32_t MODULENAME0;           /* Module Name Information */
	uint32_t MODULENAME1;           /* Module Name Information */
	uint32_t MODULENAME2;           /* Module Name Information */
	uint32_t MODULENAME3;           /* Module Name Information */
	uint32_t MODULENAME4;           /* Module Name Information */
	uint32_t MODULENAME5;           /* Module Name Information */
	uint32_t MODULENAME6;           /* Module Name Information */
	uint32_t MODULEINFO;            /* Module Information */
	uint32_t MODXOCAL;              /* Module External Oscillator Calibration Information */
	uint32_t RESERVED3[11U];
	uint32_t IADC0GAIN0;            /* IADC Gain Calibration */
	uint32_t IADC0GAIN1;            /* IADC Gain Calibration */
	uint32_t IADC0OFFSETCAL0;       /* IADC Offset Calibration */
	uint32_t IADC0NORMALOFFSETCAL0; /* IADC Offset Calibration */
	uint32_t IADC0NORMALOFFSETCAL1; /* IADC Offset Calibration */
	uint32_t IADC0HISPDOFFSETCAL0;  /* IADC Offset Calibration */
	uint32_t IADC0HISPDOFFSETCAL1;  /* IADC Offset Calibration */
	uint32_t RESERVED4[24U];
	uint32_t LEGACY;                /* Legacy Device Info */
	uint32_t RESERVED5[23U];
	uint32_t RTHERM;                /* Thermistor Calibration */
	uint32_t RESERVED6[81U];
} di_v4_t;

/* -------------------------------------------------------------------------- */
/* Constants */
/* -------------------------------------------------------------------------- */

#define EFM32_UNKNOWN_FAMILY    9999

typedef struct efm32_device_t {
	uint16_t family_id;          /* Family for device matching */
	uint8_t di_version;          /* Device information version */
	char* name;                  /* Friendly device family name */
	uint32_t flash_page_size;    /* Flash page size */
	uint32_t msc_addr;           /* MSC Address */
	bool has_radio;              /* Indicates a device has attached radio */
	uint32_t user_data_size;     /* User Data (UD) region size */
	uint32_t bootloader_size;    /* Bootloader (BL) region size (may be 0 for no BL region) */
	char* description;           /* Human-readable description */
} efm32_device_t;

static efm32_device_t const efm32_devices[] = {
	/*  First gen micros */
	{71, 1, "EFM32G" ,  512, 0x400c0000, false,  512, 0, "Gecko"},
	{72, 1, "EFM32GG", 2048, 0x400c0000, false, 4096, 0, "Giant Gecko"},
	{73, 1, "EFM32TG",  512, 0x400c0000, false,  512, 0, "Tiny Gecko"},
	{74, 1, "EFM32LG", 2048, 0x400c0000, false, 2048, 0, "Leopard Gecko"},
	{75, 1, "EFM32WG", 2048, 0x400c0000, false, 2048, 0, "Wonder Gecko"},
	{76, 1, "EFM32ZG", 1024, 0x400c0000, false, 1024, 0, "Zero Gecko"},
	{77, 1, "EFM32HG", 1024, 0x400c0000, false, 1024, 0, "Happy Gecko"},
	/*  First (1.5) gen micro + radio */
	{120, 2, "EZR32WG", 2048, 0x400c0000, true, 2048, 0, "EZR Wonder Gecko"},
	{121, 2, "EZR32LG", 2048, 0x400c0000, true, 2048, 0, "EZR Leopard Gecko"},
	{122, 2, "EZR32HG", 1024, 0x400c0000, true, 1024, 0, "EZR Happy Gecko"},
	/*  Second gen micros */
	{81, 3, "EFM32PG1B" , 2048, 0x400e0000, false, 2048, 10240, "Pearl Gecko"},
	{83, 3, "EFM32JG1B" , 2048, 0x400e0000, false, 2048, 10240, "Jade Gecko"},
	{85, 3, "EFM32PG12B", 2048, 0x400e0000, false, 2048, 32768,"Pearl Gecko 12"},
	{87, 3, "EFM32JG12B", 2048, 0x400e0000, false, 2048, 32768, "Jade Gecko 12"},
	/*  Second (2.5) gen micros, with re-located MSC */
	{100, 3, "EFM32GG11B", 4096, 0x40000000, false, 4096, 32768, "Giant Gecko 11"},
	{103, 3, "EFM32TG11B", 2048, 0x40000000, false, 2048, 18432, "Tiny Gecko 11"},
	{106, 3, "EFM32GG12B", 2048, 0x40000000, false, 2048, 32768, "Giant Gecko 12"},
	/*  Second gen devices micro + radio */
	{16, 3, "EFR32MG1P" , 2048, 0x400e0000, true, 2048, 10240, "Mighty Gecko"},
	{17, 3, "EFR32MG1B" , 2048, 0x400e0000, true, 2048, 10240, "Mighty Gecko"},
	{18, 3, "EFR32MG1V" , 2048, 0x400e0000, true, 2048, 10240, "Mighty Gecko"},
	{19, 3, "EFR32BG1P" , 2048, 0x400e0000, true, 2048, 10240, "Blue Gecko"},
	{20, 3, "EFR32BG1B" , 2048, 0x400e0000, true, 2048, 10240, "Blue Gecko"},
	{21, 3, "EFR32BG1V" , 2048, 0x400e0000, true, 2048, 10240, "Blue Gecko"},
	{25, 3, "EFR32FG1P" , 2048, 0x400e0000, true, 2048, 10240, "Flex Gecko"},
	{26, 3, "EFR32FG1B" , 2048, 0x400e0000, true, 2048, 10240, "Flex Gecko"},
	{27, 3, "EFR32FG1V" , 2048, 0x400e0000, true, 2048, 10240, "Flex Gecko"},
	{28, 3, "EFR32MG12P", 2048, 0x400e0000, true, 2048, 32768, "Mighty Gecko"},
	{29, 3, "EFR32MG12B", 2048, 0x400e0000, true, 2048, 32768, "Mighty Gecko"},
	{30, 3, "EFR32MG12V", 2048, 0x400e0000, true, 2048, 32768, "Mighty Gecko"},
	{31, 3, "EFR32BG12P", 2048, 0x400e0000, true, 2048, 32768, "Blue Gecko"},
	{32, 3, "EFR32BG12B", 2048, 0x400e0000, true, 2048, 32768, "Blue Gecko"},
	{33, 3, "EFR32BG12V", 2048, 0x400e0000, true, 2048, 32768, "Blue Gecko"},
	{37, 3, "EFR32FG12P", 2048, 0x400e0000, true, 2048, 32768, "Flex Gecko"},
	{38, 3, "EFR32FG12B", 2048, 0x400e0000, true, 2048, 32768, "Flex Gecko"},
	{39, 3, "EFR32FG12V", 2048, 0x400e0000, true, 2048, 32768, "Flex Gecko"},
	{40, 3, "EFR32MG13P", 2048, 0x400e0000, true, 2048, 16384, "Mighty Gecko"},
	{41, 3, "EFR32MG13B", 2048, 0x400e0000, true, 2048, 16384, "Mighty Gecko"},
	{42, 3, "EFR32MG13V", 2048, 0x400e0000, true, 2048, 16384, "Mighty Gecko"},
	{43, 3, "EFR32BG13P", 2048, 0x400e0000, true, 2048, 16384, "Blue Gecko"},
	{44, 3, "EFR32BG13B", 2048, 0x400e0000, true, 2048, 16384, "Blue Gecko"},
	{45, 3, "EFR32BG13V", 2048, 0x400e0000, true, 2048, 16384, "Blue Gecko"},
	{45, 3, "EFR32ZG13P", 2048, 0x400e0000, true, 2048, 16384, "Zero Gecko"},
	{49, 3, "EFR32FG13P", 2048, 0x400e0000, true, 2048, 16384, "Flex Gecko"},
	{50, 3, "EFR32FG13B", 2048, 0x400e0000, true, 2048, 16384, "Flex Gecko"},
	{51, 3, "EFR32FG13V", 2048, 0x400e0000, true, 2048, 16384, "Flex Gecko"},
	{52, 3, "EFR32MG14P", 2048, 0x400e0000, true, 2048, 16384, "Mighty Gecko"},
	{53, 3, "EFR32MG14B", 2048, 0x400e0000, true, 2048, 16384, "Mighty Gecko"},
	{54, 3, "EFR32MG14V", 2048, 0x400e0000, true, 2048, 16384, "Mighty Gecko"},
	{55, 3, "EFR32BG14P", 2048, 0x400e0000, true, 2048, 16384, "Blue Gecko"},
	{56, 3, "EFR32BG14B", 2048, 0x400e0000, true, 2048, 16384, "Blue Gecko"},
	{57, 3, "EFR32BG14V", 2048, 0x400e0000, true, 2048, 16384, "Blue Gecko"},
	{58, 3, "EFR32ZG14P", 2048, 0x400e0000, true, 2048, 16384, "Zero Gecko"},
	{61, 3, "EFR32FG14P", 2048, 0x400e0000, true, 2048, 16384, "Flex Gecko"},
	{62, 3, "EFR32FG14B", 2048, 0x400e0000, true, 2048, 16384, "Flex Gecko"},
	{63, 3, "EFR32FG14V", 2048, 0x400e0000, true, 2048, 16384, "Flex Gecko"},
	/*  Third gen devices micro + radio */
	{128, 4, "EFR32xG21", 8192, 0x40030000, true, 1024, 0, "Flex Gecko"},
	{129, 4, "EFR32xG21", 8192, 0x40030000, true, 1024, 0, "Mighty Gecko"},
	{130, 4, "EFR32xG21", 8192, 0x40030000, true, 1024, 0, "Blue Gecko"},
	{221, 4, "EFR32xG22", 8192, 0x40030000, true, 1024, 0, "Flex Gecko"},
	{222, 4, "EFR32xG22", 8192, 0x40030000, true, 1024, 0, "Mighty Gecko"},
	{223, 4, "EFR32xG22", 8192, 0x40030000, true, 1024, 0, "Blue Gecko"},
};
static const size_t efm32_devices_nb = sizeof(efm32_devices) / sizeof(efm32_device_t);

/* miscchip */
typedef struct {
	uint8_t pincount;
	uint8_t pkgtype;
	uint8_t tempgrade;
} efm32_di_miscchip_t;

/* pkgtype */
typedef struct {
	uint8_t pkgtype;
	char* name;
} efm32_di_pkgtype_t;

static efm32_di_pkgtype_t const efm32_di_pkgtypes[] = {
	{74, "WLCSP"},  /* WLCSP package */
	{76, "BGA"},    /* BGA package */
	{77, "QFN"},    /* QFN package */
	{81, "QFxP"},   /* QFP package */
};

/* tempgrade */
typedef struct {
	uint8_t tempgrade;
	char* name;
} efm32_di_tempgrade_t;

static efm32_di_tempgrade_t const efm32_di_tempgrades[] = {
	{0, "-40 to 85degC"},
	{1, "-40 to 125degC"},
	{2, "-40 to 105degC"},
	{3, "0 to 70degC"}
};

/* -------------------------------------------------------------------------- */
/* Helper functions - Version 1 V1 */
/* -------------------------------------------------------------------------- */

/**
 * Reads the EFM32 Extended Unique Identifier EUI48 (V2)
 */
#if 0
static uint64_t efm32_read_eui48(target *t, uint8_t di_version)
{
	uint64_t eui = 0;
	uint32_t addr_l;
	uint32_t addr_h;
	switch (di_version) {
	case 3:
		addr_l = (uint32_t) &((di_v3_t *) EFM32_DI_V3)->EUI48L;
		addr_h = (uint32_t) &((di_v3_t *) EFM32_DI_V3)->EUI48H;
		break;
	case 4:
		addr_l = (uint32_t) &((di_v4_t *) EFM32_DI_V4)->EUI48L;
		addr_h = (uint32_t) &((di_v4_t *) EFM32_DI_V4)->EUI48H;
		break;
	default:
		/* DI not supported */
		addr_l = 0;
		addr_h = 0;
		break;
	}
	if (addr_l != 0 && addr_h != 0) {
		eui  = (uint64_t) target_mem_read32(t, addr_h) << 32;
		eui |= target_mem_read32(t, addr_l);
	}
	return eui;
}
#endif

/**
 * Reads the Unique Number
 */
static uint64_t efm32_read_unique(target *t, uint8_t di_version)
{
	uint64_t unique = 0;
	uint32_t addr_l;
	uint32_t addr_h;
	switch (di_version) {
	case 1:
		addr_l = (uint32_t) &((di_v1_t *) EFM32_DI_V1)->UNIQUEL;
		addr_h = (uint32_t) &((di_v1_t *) EFM32_DI_V1)->UNIQUEH;
		break;
	case 2:
		addr_l = (uint32_t) &((di_v2_t *) EFM32_DI_V2)->UNIQUEL;
		addr_h = (uint32_t) &((di_v2_t *) EFM32_DI_V2)->UNIQUEH;
		break;
	case 3:
		addr_l = (uint32_t) &((di_v3_t *) EFM32_DI_V3)->UNIQUEL;
		addr_h = (uint32_t) &((di_v3_t *) EFM32_DI_V3)->UNIQUEH;
		break;
	case 4:
		addr_l = (uint32_t) &((di_v4_t *) EFM32_DI_V4)->EUI64L;
		addr_h = (uint32_t) &((di_v4_t *) EFM32_DI_V4)->EUI64H;
		break;
	default:
		/* DI not supported */
		addr_l = 0;
		addr_h = 0;
		break;
	}
	DEBUG("efm32 read unique %lx %lx\n", addr_h, addr_l);
	if (addr_l != 0 && addr_h != 0) {
		unique  = (uint64_t) target_mem_read32(t, addr_h) << 32;
		unique |= target_mem_read32(t, addr_l);
	}
	return unique;
}

/**
 * Reads the EFM32 flash size in kiB
 */
static uint16_t efm32_read_flash_size(target *t, uint8_t di_version)
{
	uint16_t size = 0;
	uint32_t addr;
	const uint32_t mask = EFM32_DI_MSIZE_FLASH_MASK;
	const uint32_t ofst = EFM32_DI_MSIZE_FLASH_OFST;
	switch (di_version) {
	case 1:
		addr = (uint32_t) &((di_v1_t *) EFM32_DI_V1)->MSIZE;
		break;
	case 2:
		addr = (uint32_t) &((di_v2_t *) EFM32_DI_V2)->MSIZE;
		break;
	case 3:
		addr = (uint32_t) &((di_v3_t *) EFM32_DI_V3)->MSIZE;
		break;
	case 4:
		addr = (uint32_t) &((di_v4_t *) EFM32_DI_V4)->MSIZE;
		break;
	default:
		/* DI not supported */
		addr = 0;
		break;
	}
	DEBUG("efm32 read flash size %lx\n", addr);
	if (addr != 0)
		size = (uint16_t)((target_mem_read32(t, addr) >> ofst) & mask);
	return size;
}

/**
 * Reads the EFM32 RAM size in kiB
 */
static uint16_t efm32_read_ram_size(target *t, uint8_t di_version)
{
	uint16_t size = 0;
	uint32_t addr;
	const uint32_t mask = EFM32_DI_MSIZE_SRAM_MASK;
	const uint32_t ofst = EFM32_DI_MSIZE_SRAM_OFST;
	switch (di_version) {
	case 1:
		addr = (uint32_t) &((di_v1_t *) EFM32_DI_V1)->MSIZE;
		break;
	case 2:
		addr = (uint32_t) &((di_v2_t *) EFM32_DI_V2)->MSIZE;
		break;
	case 3:
		addr = (uint32_t) &((di_v3_t *) EFM32_DI_V3)->MSIZE;
		break;
	case 4:
		addr = (uint32_t) &((di_v4_t *) EFM32_DI_V4)->MSIZE;
		break;
	default:
		/* DI not supported */
		addr = 0;
		break;
	}
	DEBUG("efm32 read ram size %lx\n", addr);
	if (addr != 0)
		size = (uint16_t)((target_mem_read32(t, addr) >> ofst) & mask);
	return size;
}

/**
 * Reads the EFM32 reported flash page size in bytes.  Note: This
 * driver ignores this value, and uses a conservative hard-coded
 * value. There are errata on the value reported by the EFM32
 * eg. DI_101
 */
static uint32_t efm32_read_flash_page_size(target *t, uint8_t di_version)
{
	uint32_t size = 0;
	uint32_t addr;
	uint32_t ofst;
	uint32_t mask;
	switch (di_version) {
	case 1:
		addr = (uint32_t) &((di_v1_t *) EFM32_DI_V1)->MEMINFO;
		mask = EFM32_DI_MEMINFO_FLASHPAGESIZE_MASK;
		ofst = EFM32_DI_MEMINFO_FLASHPAGESIZE_OFST;
		break;
	case 2:
		addr = (uint32_t) &((di_v2_t *) EFM32_DI_V2)->MEMINFO;
		mask = EFM32_DI_MEMINFO_FLASHPAGESIZE_MASK;
		ofst = EFM32_DI_MEMINFO_FLASHPAGESIZE_OFST;
		break;
	case 3:
		addr = (uint32_t) &((di_v3_t *) EFM32_DI_V3)->MEMINFO;
		mask = EFM32_DI_MEMINFO_FLASHPAGESIZE_MASK;
		ofst = EFM32_DI_MEMINFO_FLASHPAGESIZE_OFST;
		break;
	case 4:
		addr = (uint32_t) &((di_v4_t *) EFM32_DI_V4)->MEMINFO;
		mask = EFM32_DI_V4_MEMINFO_FLASHPAGESIZE_MASK;
		ofst = EFM32_DI_V4_MEMINFO_FLASHPAGESIZE_OFST;
		break;
	default:
		/* DI not supported */
		addr = 0;
		break;
	}
	DEBUG("efm32 read flash page size %lx\n", addr);
	if (addr != 0) {
		size = (target_mem_read32(t, addr) >> ofst) & mask;
		size = 1 << (size + 10);
	}
	return size;
}

/**
 * Reads the EFM32 Part Number
 */
static uint16_t efm32_read_part_number(target *t, uint8_t di_version)
{
	uint16_t partno = 0;
	uint32_t addr;
	const uint32_t mask = EFM32_DI_PART_NUMBER_MASK;
	const uint32_t ofst = EFM32_DI_PART_NUMBER_OFST;
	switch (di_version) {
	case 1:
		addr = (uint32_t) &((di_v1_t *) EFM32_DI_V1)->PART;
		break;
	case 2:
		addr = (uint32_t) &((di_v2_t *) EFM32_DI_V2)->PART;
		break;
	case 3:
		addr = (uint32_t) &((di_v3_t *) EFM32_DI_V3)->PART;
		break;
	case 4:
		addr = (uint32_t) &((di_v4_t *) EFM32_DI_V4)->PART;
		break;
	default:
		/* DI not supported */
		addr = 0;
		break;
	}
	DEBUG("efm32 read part number %lx\n", addr);
	if (addr != 0)
		partno = (uint16_t)((target_mem_read32(t, addr) >> ofst) & mask);
	return partno;
}

/**
 * Reads the EFM32 Part Family
 */
static uint8_t efm32_read_part_family(target *t, uint8_t di_version)
{
	uint8_t part;
	uint32_t addr;
	uint32_t reg;
	switch (di_version) {
	case 1:
		addr = (uint32_t) &((di_v1_t *) EFM32_DI_V1)->PART;
		reg = target_mem_read32(t, addr);
		part = (uint8_t)((reg >> EFM32_DI_PART_FAMILY_OFST) & EFM32_DI_PART_FAMILY_MASK);
		break;
	case 2:
		addr = (uint32_t) &((di_v2_t *) EFM32_DI_V2)->PART;
		reg = target_mem_read32(t, addr);
		part = (uint8_t)((reg >> EFM32_DI_PART_FAMILY_OFST) & EFM32_DI_PART_FAMILY_MASK);
		break;
	case 3:
		addr = (uint32_t) &((di_v3_t *) EFM32_DI_V3)->PART;
		reg = target_mem_read32(t, addr);
		part = (uint8_t)((reg >> EFM32_DI_PART_FAMILY_OFST) & EFM32_DI_PART_FAMILY_MASK);
		break;
	case 4:
		addr = (uint32_t) &((di_v4_t *) EFM32_DI_V4)->PART;
		reg = target_mem_read32(t, addr);
		part  = (uint8_t)((reg >> EFM32_DI_V4_PART_FAMILYNUM_OFST) & EFM32_DI_V4_PART_FAMILYNUM_MASK);
		part += (uint8_t)((reg >> EFM32_DI_V4_PART_FAMILY_OFST) & EFM32_DI_V4_PART_FAMILY_MASK);
		break;
	default:
		/* DI not supported */
		part = 0;
		break;
	}
	return part;
}

/**
 * Reads the EFM32 Radio part number (EZR parts with V1 DI only)
 */
static uint16_t efm32_read_radio_part_number(target *t, uint8_t di_version)
{
	if (di_version == 2)
		return target_mem_read16(t, (uint32_t) &((di_v2_t *) EFM32_DI_V2)->RADIO1);
	else
		return 0;
}

/**
 * Reads the EFM32 Misc. Chip definitions
 */
static efm32_di_miscchip_t efm32_read_miscchip(target *t, uint8_t di_version)
{
	uint32_t pkginfo;
	uint32_t addr;
	efm32_di_miscchip_t miscchip = { 0 };
	switch (di_version) {
	case 3:
		addr = (uint32_t) &((di_v3_t *) EFM32_DI_V3)->MEMINFO;
		break;
	case 4:
		addr = (uint32_t) &((di_v4_t *) EFM32_DI_V4)->PKGINFO;
		break;
	default:
		/* DI not supported */
		addr = 0;
		break;
	}
	DEBUG("efm32 read chip %lx\n", addr);
	if (addr != 0) {
		pkginfo = target_mem_read32(t, addr);
		miscchip.pincount  = (pkginfo >> EFM32_DI_PKGINFO_PINCOUNT_OFST ) & EFM32_DI_PKGINFO_PINCOUNT_MASK;
		miscchip.pkgtype   = (pkginfo >> EFM32_DI_PKGINFO_PKGTYPE_OFST  ) & EFM32_DI_PKGINFO_PKGTYPE_MASK;
		miscchip.tempgrade = (pkginfo >> EFM32_DI_PKGINFO_TEMPGRADE_OFST) & EFM32_DI_PKGINFO_TEMPGRADE_MASK;
	}
	return miscchip;
}

/* -------------------------------------------------------------------------- */
/* Shared Functions */
/* -------------------------------------------------------------------------- */

static void efm32_add_flash(target *t, target_addr addr, size_t length,
			    size_t page_size)
{
	struct target_flash *f = calloc(1, sizeof(*f));
	if (!f) {			/* calloc failed: heap exhaustion */
		DEBUG("calloc: failed in %s\n", __func__);
		return;
	}

	f->start = addr;
	f->length = length;
	f->blocksize = page_size;
	f->erase = efm32_flash_erase;
	f->write = efm32_flash_write;
	f->buf_size = page_size;
	target_add_flash(t, f);
}

/**
 * Lookup device
 */
static size_t efm32_lookup_device_index(target *t, uint8_t di_version)
{
	uint8_t part_family = efm32_read_part_family(t, di_version);
	DEBUG("EFM32 di_version=%d part_family=%d\n", di_version, part_family);

	/* Search for family */
	for (size_t i = 0; i < efm32_devices_nb; i++) {
		if (efm32_devices[i].family_id == part_family) {
			DEBUG("EFM32 family found i=%u\n", i);
			return i;
		}
	}
	/* Unknown family */
	return EFM32_UNKNOWN_FAMILY;
}

static efm32_device_t const * efm32_get_device(size_t index)
{
	if (index >= efm32_devices_nb) {
		return NULL;
	}
	return &efm32_devices[index];
}

/**
 * Probe
 */
char variant_string[60];
bool efm32_probe(target *t)
{
	uint8_t di_version;

	/* Read the IDCODE register from the SW-DP */
	ADIv5_AP_t *ap = cortexm_ap(t);
	uint32_t ap_idcode = ap->dp->idcode;

	/* Check the idcode. See AN0062 Section 2.2 */
	DEBUG("efm32_probe: ap_idcode=%lx\n", ap_idcode);
	if (ap_idcode == 0x2BA01477) {
		/* Cortex M3, Cortex M4 */
		di_version = 3;
	} else if (ap_idcode == 0x0BC11477) {
		/* Cortex M0+ */
		di_version = 2;
	} else if (ap_idcode == 0x6BA02477) {
		/* Cortex M33 */
		di_version = 4;
	} else {
		return false;
	}

	/* Check the OUI in the EUI is silabs or energymicro.
	 * Use this to identify the Device Identification (DI) version */
	// uint64_t oui24 = ((efm32_v1_read_eui64(t) >> 40) & 0xFFFFFF);
	// DEBUG("efm32_probe: oui24=%lx\n", (uint32_t) oui24);
	// if (oui24 == EFM32_V1_DI_EUI_SILABS) {
	// 	/* Device Identification (DI) version 1 */
	// 	di_version = 1;
	// } else if (oui24 == EFM32_V2_DI_EUI_ENERGYMICRO) {
	// 	/* Device Identification (DI) version 2 */
	// 	di_version = 2;
	// } else {
	// 	/* Unknown OUI - assume version 1 */
	// 	di_version = 1;
	// }
	// uint32_t oui48 = ((efm32_v2_read_eui48(t) >> 23) & 0xFFFFFF);
	// DEBUG("efm_probe: oui48=%lx\n", oui48);
	uint32_t di_addr;
	switch (di_version){
	case 1:
		di_addr = EFM32_DI_V1;
		break;
	case 2:
		di_addr = EFM32_DI_V2;
		break;
	case 3:
		di_addr = EFM32_DI_V3;
		break;
	case 4:
		di_addr = EFM32_DI_V4;
		break;
	}
	uint32_t di_max = di_addr + 0x1000;
	while (di_addr < di_max) {
		if ((di_addr & 0x0f) == 0)
			DEBUG("\nefm32_probe: DI[%03" PRIx32 "] ", di_addr & 0xfff);
		DEBUG("%08" PRIx32" ", target_mem_read32(t, di_addr));
		di_addr += 4;
	}
	DEBUG("\n");

	/* Read the part family, and reject if unknown */
	size_t device_index  = efm32_lookup_device_index(t, di_version);
	if (device_index == EFM32_UNKNOWN_FAMILY) {
		/* unknown device family */
		return false;
	}
	efm32_device_t const* device = efm32_get_device(device_index);
	if (device == NULL) {
		return false;
	}

	uint16_t part_number = efm32_read_part_number(t, di_version);

	/* Read memory sizes, convert to bytes */
	uint16_t flash_kib  = efm32_read_flash_size(t, di_version);
	uint32_t flash_size = flash_kib * 0x400;
	uint16_t ram_kib    = efm32_read_ram_size(t, di_version);
	uint32_t ram_size   = ram_kib   * 0x400;
	uint32_t flash_page_size = device->flash_page_size;

	snprintf(variant_string, sizeof(variant_string), "%c\b%c\b%s %d F%d %s",
			di_version + 48, (uint8_t)device_index + 32,
			device->name, part_number, flash_kib, device->description);

	/* Setup Target */
	t->target_options |= CORTEXM_TOPT_INHIBIT_SRST;
	t->driver = variant_string;
	tc_printf(t, "flash size %d page size %d\n", flash_size, flash_page_size);
	target_add_ram (t, SRAM_BASE, ram_size);
	efm32_add_flash(t, 0x00000000, flash_size, flash_page_size);
	if (device->user_data_size) { /* optional User Data (UD) section */
		efm32_add_flash(t, EFM32_USER_DATA, device->user_data_size, flash_page_size);
	}
	if (device->bootloader_size) { /* optional Bootloader (BL) section */
		efm32_add_flash(t, EFM32_BOOTLOADER, device->bootloader_size, flash_page_size);
	}
	target_add_commands(t, efm32_cmd_list, "EFM32");

	return true;
}

/**
 * Erase flash row by row
 */
static int efm32_flash_erase(struct target_flash *f, target_addr addr, size_t len)
{
	target *t = f->t;
	efm32_device_t const* device = efm32_get_device(t->driver[2] - 32);
	if (device == NULL) {
		return true;
	}
	uint32_t msc = device->msc_addr;

	/* Unlock */
	target_mem_write32(t, EFM32_MSC_LOCK(msc), EFM32_MSC_LOCK_LOCKKEY);

	/* Set WREN bit to enabel MSC write and erase functionality */
	target_mem_write32(t, EFM32_MSC_WRITECTRL(msc), 1);

	while (len) {
		/* Write address of first word in row to erase it */
		target_mem_write32(t, EFM32_MSC_ADDRB(msc), addr);
		target_mem_write32(t, EFM32_MSC_WRITECMD(msc), EFM32_MSC_WRITECMD_LADDRIM);

		/* Issue the erase command */
		target_mem_write32(t, EFM32_MSC_WRITECMD(msc), EFM32_MSC_WRITECMD_ERASEPAGE );

		/* Poll MSC Busy */
		while ((target_mem_read32(t, EFM32_MSC_STATUS(msc)) & EFM32_MSC_STATUS_BUSY)) {
			if (target_check_error(t))
				return -1;
		}

		addr += f->blocksize;
		if (len > f->blocksize)
			len -= f->blocksize;
		else
			len = 0;
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
	efm32_device_t const* device = efm32_get_device(t->driver[2] - 32);
	if (device == NULL) {
		return true;
	}
	/* Write flashloader */
	target_mem_write(t, SRAM_BASE, efm32_flash_write_stub,
			 sizeof(efm32_flash_write_stub));
	/* Write Buffer */
	target_mem_write(t, STUB_BUFFER_BASE, src, len);
	/* Run flashloader */
	int ret = cortexm_run_stub(t, SRAM_BASE, dest, STUB_BUFFER_BASE, len,
							   device->msc_addr);

#ifdef PLATFORM_HAS_DEBUG
	/* Check the MSC_IF */
	uint32_t msc = device->msc_addr;
	uint32_t msc_if = target_mem_read32(t, EFM32_MSC_IF(msc));
	DEBUG("EFM32: Flash write done MSC_IF=%08"PRIx32"\n", msc_if);
#endif
	return ret;
}

/**
 * Uses the MSC ERASEMAIN0 command to erase the entire flash
 */
static bool efm32_cmd_erase_all(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	efm32_device_t const* device = efm32_get_device(t->driver[2] - 32);
	if (device == NULL) {
		return true;
	}
	uint32_t msc = device->msc_addr;

	/* Set WREN bit to enabel MSC write and erase functionality */
	target_mem_write32(t, EFM32_MSC_WRITECTRL(msc), 1);

	/* Unlock mass erase */
	target_mem_write32(t, EFM32_MSC_MASSLOCK(msc), EFM32_MSC_MASSLOCK_LOCKKEY);

	/* Erase operation */
	target_mem_write32(t, EFM32_MSC_WRITECMD(msc), EFM32_MSC_WRITECMD_ERASEMAIN0);

	/* Poll MSC Busy */
	while ((target_mem_read32(t, EFM32_MSC_STATUS(msc)) & EFM32_MSC_STATUS_BUSY)) {
		if (target_check_error(t))
			return false;
	}

	/* Relock mass erase */
	target_mem_write32(t, EFM32_MSC_MASSLOCK(msc), 0);

	tc_printf(t, "Erase successful!\n");

	return true;
}

/**
 * Reads the 40-bit unique number
 */
static bool efm32_cmd_serial(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	uint64_t unique;
	uint8_t di_version = t->driver[0] - 48; /* di version hidden in driver str */
	unique = efm32_read_unique(t, di_version);
	tc_printf(t, "Unique Number: 0x%08lx%08lx\n", (uint32_t)(unique >> 32), (uint32_t) unique);
	return true;
}
/**
 * Prints various information we know about the device
 */
static bool efm32_cmd_efm_info(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	uint8_t di_version  = t->driver[0] - 48; /* hidden in driver str */

	switch (di_version) {
	case 1:
		tc_printf(t, "DI version 1 (EFM32xG) base 0x%08x\n\n", EFM32_DI_V1);
		break;
	case 2:
		tc_printf(t, "DI version 2 (EZR32xG) base 0x%08x\n\n", EFM32_DI_V2);
		break;
	case 3:
		tc_printf(t, "DI version 3 (EFR32xG1x) base 0x%08x\n\n", EFM32_DI_V3);
		break;
	case 4:
		tc_printf(t, "DI version 4 (EFR32xG2x) base 0x%08x\n\n", EFM32_DI_V4);
		break;
	default:
		tc_printf(t, "Bad DI version %d! This driver doesn't know about this DI version\n", di_version);
		return true;			/* finish */
	}

	/* lookup device and part number */
	efm32_device_t const* device = efm32_get_device(t->driver[2] - 32);
	if (device == NULL) {
		return true;
	}
	uint16_t part_number = efm32_read_part_number(t, di_version);

	/* Read memory sizes, convert to bytes */
	uint16_t flash_kib  = efm32_read_flash_size(t, di_version);
	uint16_t ram_kib    = efm32_read_ram_size(t, di_version);
	uint32_t flash_page_size_reported = efm32_read_flash_page_size(t, di_version);
	uint32_t flash_page_size          = device->flash_page_size;

	tc_printf(t, "%s %d F%d = %s %dkiB flash, %dkiB ram\n",
			  device->name,    part_number, flash_kib,
			  device->description, flash_kib, ram_kib);
	tc_printf(t, "Device says flash page size is %d bytes, we're using %d bytes\n",
			  flash_page_size_reported, flash_page_size);
	if (flash_page_size_reported < flash_page_size) {
		tc_printf(t, "This is bad, flash writes may be corrupted\n");
	}
	tc_printf(t, "\n");

	if (di_version == 3 || di_version == 4) {
		efm32_di_miscchip_t miscchip = efm32_read_miscchip(t, di_version);
		efm32_di_pkgtype_t const* pkgtype = NULL;
		efm32_di_tempgrade_t const* tempgrade;

		for (size_t i = 0; i < (sizeof(efm32_di_pkgtypes) /
								sizeof(efm32_di_pkgtype_t)); i++) {
			if (efm32_di_pkgtypes[i].pkgtype == miscchip.pkgtype) {
				pkgtype = &efm32_di_pkgtypes[i];
			}
		}
		for (size_t i = 0; i < (sizeof(efm32_di_tempgrades) /
								sizeof(efm32_di_tempgrade_t)); i++) {
			if (efm32_di_tempgrades[i].tempgrade == miscchip.tempgrade) {
				tempgrade = &efm32_di_tempgrades[i];
			}
		}

		tc_printf(t, "Package %s %d pins\n", pkgtype->name, miscchip.pincount);
		tc_printf(t, "Temperature grade %s\n", tempgrade->name);
		tc_printf(t, "\n");
	}

	if (di_version == 2 && device->has_radio) {
		uint16_t radio_number = efm32_read_radio_part_number(t, di_version); /* on-chip radio */
		tc_printf(t, "Radio si%d\n", radio_number);
		tc_printf(t, "\n");
	}

	return true;
}

/**
 * Bootloader status in CLW0, if applicable.
 *
 * This is a bit in flash, so it is possible to clear it only once.
 */
static bool efm32_cmd_bootloader(target *t, int argc, const char **argv)
{
	/* lookup device and part number */
	efm32_device_t const* device = efm32_get_device(t->driver[2] - 32);
	if (device == NULL) {
		return true;
	}
	uint32_t msc = device->msc_addr;

	if (device->bootloader_size == 0) {
		tc_printf(t, "This device has no bootloader.\n");
		return false;
	}

	uint32_t clw0 = target_mem_read32(t, EFM32_LOCK_BITS_CLW0);
	bool bootloader_status = (clw0 & EFM32_CLW0_BOOTLOADER_ENABLE)?1:0;

	if (argc == 1) {
		tc_printf(t, "Bootloader %s\n",
			  bootloader_status ? "enabled" : "disabled");
		return true;
	} else {
		bootloader_status = (argv[1][0] == 'e');
	}

	/* Modify bootloader enable bit */
	clw0 &= bootloader_status?~0:~EFM32_CLW0_BOOTLOADER_ENABLE;

	/* Unlock */
	target_mem_write32(t, EFM32_MSC_LOCK(msc), EFM32_MSC_LOCK_LOCKKEY);

	/* Set WREN bit to enabel MSC write and erase functionality */
	target_mem_write32(t, EFM32_MSC_WRITECTRL(msc), 1);

	/* Write address of CLW0 */
	target_mem_write32(t, EFM32_MSC_ADDRB(msc), EFM32_LOCK_BITS_CLW0);
	target_mem_write32(t, EFM32_MSC_WRITECMD(msc), EFM32_MSC_WRITECMD_LADDRIM);

	/* Issue the write */
	target_mem_write32(t, EFM32_MSC_WDATA(msc), clw0);
	target_mem_write32(t, EFM32_MSC_WRITECMD(msc), EFM32_MSC_WRITECMD_WRITEONCE);

	/* Poll MSC Busy */
	while ((target_mem_read32(t, EFM32_MSC_STATUS(msc)) & EFM32_MSC_STATUS_BUSY)) {
		if (target_check_error(t))
			return false;
	}

	return true;
}


/*** Authentication Access Port (AAP) **/

/* There's an additional AP on the SW-DP is accessable when the part
 * is almost entirely locked.
 *
 * The AAP can be used to issue a DEVICEERASE command, which erases:
 * * Flash
 * * SRAM
 * * Lock Bit (LB) page
 *
 * It does _not_ erase:
 * * User Data (UD) page
 * * Bootloader (BL) if present
 *
 * Once the DEVICEERASE command has completed, the main AP will be
 * accessable again. If the device has a bootloader, it will attempt
 * to boot from this. If you have just unlocked the device the
 * bootloader could be anything (even garbage, if the bootloader
 * wasn't used before the DEVICEERASE). Therefore you may want to
 * connect under srst and use the bootloader command to disable it.
 *
 * It is possible to lock the AAP itself by clearing the AAP Lock Word
 * (ALW). In this case the part is unrecoverable (unless you glitch
 * it, please try glitching it).
 */

#include "adiv5.h"

/* IDR revision [31:28] jes106 [27:17] class [16:13] res [12:8]
 * variant [7:4] type [3:0] */
#define EFM32_AAP_IDR      0x06E60001
#define EFM32_APP_IDR_MASK 0x0FFFFF0F

#define AAP_CMD      ADIV5_AP_REG(0x00)
#define AAP_CMDKEY   ADIV5_AP_REG(0x04)
#define AAP_STATUS   ADIV5_AP_REG(0x08)

#define AAP_STATUS_LOCKED    (1 << 1)
#define AAP_STATUS_ERASEBUSY (1 << 0)

#define CMDKEY 0xCFACC118

static bool efm32_aap_cmd_device_erase(target *t, int argc, const char **argv);

const struct command_s efm32_aap_cmd_list[] = {
	{"erase_mass", (cmd_handler)efm32_aap_cmd_device_erase, "Erase entire flash memory"},
	{NULL, NULL, NULL}
};

static bool nop_function(void)
{
	return true;
}

/**
 * AAP Probe
 */
char aap_driver_string[42];
void efm32_aap_probe(ADIv5_AP_t *ap)
{
	if ((ap->idr & EFM32_APP_IDR_MASK) == EFM32_AAP_IDR) {
		/* It's an EFM32 AAP! */
		DEBUG("EFM32: Found EFM32 AAP\n");
	} else {
		DEBUG("EFM32: AAP not found idr=%lx\n", ap->idr);
		return;
	}

	/* Both revision 1 and revision 2 devices seen in the wild */
	uint16_t aap_revision = (uint16_t)((ap->idr & 0xF0000000) >> 28);

	/* New target */
	target *t = target_new();
	adiv5_ap_ref(ap);
	t->priv = ap;
	t->priv_free = (void*)adiv5_ap_unref;

	//efm32_aap_cmd_device_erase(t);

	/* Read status */
	DEBUG("EFM32: AAP STATUS=%08"PRIx32"\n", adiv5_ap_read(ap, AAP_STATUS));

	sprintf(aap_driver_string,
			"EFM32 Authentication Access Port rev.%d",
			aap_revision);
	t->driver = aap_driver_string;
	t->attach = (void*)nop_function;
	t->detach = (void*)nop_function;
	t->check_error = (void*)nop_function;
	t->mem_read = (void*)nop_function;
	t->mem_write = (void*)nop_function;
	t->regs_size = 4;
	t->regs_read = (void*)nop_function;
	t->regs_write = (void*)nop_function;
	t->reset = (void*)nop_function;
	t->halt_request = (void*)nop_function;
	t->halt_resume = (void*)nop_function;

	target_add_commands(t, efm32_aap_cmd_list, t->driver);
}

static bool efm32_aap_cmd_device_erase(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	ADIv5_AP_t *ap = t->priv;
	uint32_t status;

	/* Read status */
	status = adiv5_ap_read(ap, AAP_STATUS);
	DEBUG("EFM32: AAP STATUS=%08"PRIx32"\n", status);

	if (status & AAP_STATUS_ERASEBUSY) {
		DEBUG("EFM32: AAP Erase in progress\n");
		DEBUG("EFM32: -> ABORT\n");
		return false;
	}

	DEBUG("EFM32: Issuing DEVICEERASE...\n");
	adiv5_ap_write(ap, AAP_CMDKEY, CMDKEY);
	adiv5_ap_write(ap, AAP_CMD, 1);

	/* Read until 0, probably should have a timeout here... */
	do {
		status = adiv5_ap_read(ap, AAP_STATUS);
	} while (status & AAP_STATUS_ERASEBUSY);

	/* Read status */
	status = adiv5_ap_read(ap, AAP_STATUS);
	DEBUG("EFM32: AAP STATUS=%08"PRIx32"\n", status);

	return true;
}
