/* ------------------------------------------
 * Copyright (c) 2017, Synopsys, Inc. All rights reserved.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1) Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.

 * 3) Neither the name of the Synopsys, Inc., nor the names of its contributors may
 * be used to endorse or promote products derived from this software without
 * specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
--------------------------------------------- */
#ifndef _TARGET_MEM_CONFIG_H_
#define _TARGET_MEM_CONFIG_H_

#ifdef LIB_MCUBOOT

#ifdef USE_APPL_MEM_CONFIG
#error "Cannot use USE_APPL_MEM_CONFIG to modify memory when mcuboot is enabled."
#endif

#if defined(MCUBOOT_TARGET_CONFIG)
/*
 * Target-specific definitions are permitted in legacy cases that
 * don't provide the information via DTS, etc.
 */
#include MCUBOOT_TARGET_CONFIG
#else
/*
 * Otherwise, the Zephyr SoC header and the DTS provide most
 * everything we need.
 */
#ifndef FLASH_ALIGN
#define FLASH_ALIGNMENT 8
#define FLASH_ALIGN FLASH_ALIGNMENT
#endif

#define FLASH_DEV_NAME
#define FLASH_AREA_IMAGE_0_OFFSET			0x10000000
#define FLASH_AREA_IMAGE_0_SIZE				0x03000000
#define FLASH_AREA_IMAGE_1_OFFSET			0x13000000
#define FLASH_AREA_IMAGE_1_SIZE				0x03000000
#define FLASH_AREA_IMAGE_SCRATCH_OFFSET		0x16000000
#define FLASH_AREA_IMAGE_SCRATCH_SIZE		0x01000000
#define FLASH_AREA_IMAGE_MCUBOOT_OFFSET		0x17000000
#define FLASH_AREA_IMAGE_MCUBOOT_SIZE		0x00f00000


#if !defined(FLASH_AREA_IMAGE_SECTOR_SIZE)
#define FLASH_AREA_IMAGE_SECTOR_SIZE FLASH_AREA_IMAGE_SCRATCH_SIZE
#endif

#define CONFIG_FLASH_BASE_ADDRESS 0


/*
 * TODO: remove soc_family_kinetis.h once its flash driver supports
 * FLASH_PAGE_LAYOUT.
 */
#endif /* !defined(MCUBOOT_TARGET_CONFIG) */

/*
 * Upstream Zephyr changed the name from FLASH_DRIVER_NAME to
 * FLASH_DEV_NAME.  For now, let's just convert the Zephyr name to the
 * one expected by MCUboot. This can be cleaned up after the upstream
 * Zephyr tree has been released and settled down.
 */
#if !defined(FLASH_DRIVER_NAME) && defined(FLASH_DEV_NAME)
#define FLASH_DRIVER_NAME FLASH_DEV_NAME
#endif

/*
 * Sanity check the target support.
 */
#if !defined(FLASH_DRIVER_NAME) || \
    !defined(FLASH_ALIGN) ||                  \
    !defined(FLASH_AREA_IMAGE_0_OFFSET) || \
    !defined(FLASH_AREA_IMAGE_0_SIZE) || \
    !defined(FLASH_AREA_IMAGE_1_OFFSET) || \
    !defined(FLASH_AREA_IMAGE_1_SIZE) || \
    !defined(FLASH_AREA_IMAGE_SCRATCH_OFFSET) || \
    !defined(FLASH_AREA_IMAGE_SCRATCH_SIZE)
#warning "Target support is incomplete; cannot build mcuboot."
#endif

#ifdef EMBARC_USE_MCUBOOT
#define EXT_RAM_START	FLASH_AREA_IMAGE_MCUBOOT_OFFSET
#define EXT_RAM_SIZE	FLASH_AREA_IMAGE_MCUBOOT_SIZE
#define IMAGE_HEAD_SIZE 0x0
#else
#define EXT_RAM_START	FLASH_AREA_IMAGE_0_OFFSET
#define EXT_RAM_SIZE	FLASH_AREA_IMAGE_0_SIZE
#define IMAGE_HEAD_SIZE 0x400
#endif

#define REGION_ROM	EXT_RAM
#define REGION_RAM	EXT_RAM

#else

#include "arc_feature_config.h"

#ifdef USE_APPL_MEM_CONFIG
#include "appl_mem_config.h"
#endif

/**
 * The unit of XXXX_SIZE is Byte
 * For REGION_ROM, ICCM, EXT_ROM and EXT_RAM are available
 * For REGION_RAM, DCCM and EXT_RAM are available
 */
#ifdef ARC_FEATURE_ICCM_PRESENT
#ifndef ICCM_SIZE
#define ICCM_SIZE	ARC_FEATURE_ICCM_SIZE
#endif
#ifndef ICCM_START
#define ICCM_START	ARC_FEATURE_ICCM_BASE
#endif
#else
#ifndef ICCM_SIZE
#define ICCM_SIZE	0x40000
#endif
#ifndef ICCM_START
#define ICCM_START	0x0
#endif
#endif

#ifdef ARC_FEATURE_DCCM_PRESENT
#ifndef DCCM_SIZE
#define DCCM_SIZE	ARC_FEATURE_DCCM_SIZE
#endif
#ifndef DCCM_START
#define DCCM_START	ARC_FEATURE_DCCM_BASE
#endif
#else
#ifndef DCCM_SIZE
#define DCCM_SIZE	0x20000
#endif
#ifndef DCCM_START
#define DCCM_START	0x80000000
#endif
#endif

#ifndef EXT_RAM_START
#define EXT_RAM_START	0x10000000
#endif

#ifndef EXT_RAM_SIZE
#define EXT_RAM_SIZE	0x8000000
#endif

#ifndef REGION_ROM
#ifdef ARC_FEATURE_ICACHE_PRESENT
#define REGION_ROM	EXT_RAM
#else
#define REGION_ROM	ICCM
#endif
#endif

#ifndef REGION_RAM
#ifdef ARC_FEATURE_DCACHE_PRESENT
#define REGION_RAM	EXT_RAM
#else
#define REGION_RAM	DCCM
#endif
#endif

#define IMAGE_HEAD_SIZE 0x0

#endif /* !defined(LIB_MCUBOOT) */

#endif /* _TARGET_MEM_CONFIG_H_ */
