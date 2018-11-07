/* ------------------------------------------
 * Copyright (c) 2018, Synopsys, Inc. All rights reserved.

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

#include "embARC.h"
#include "embARC_debug.h"
#include "dev_flash.h"

BOOTSPI_DEFINE(bootspi_test, BOOTSPI_CRTL_BASE);

uint8_t g_test_wr_buf[SMIC_BOOTSPI_SEC_SIZE];
uint8_t g_test_rd_buf[SMIC_BOOTSPI_SEC_SIZE];

int main(void)
{
	uint32_t i, sec;
	uint8_t wb, rb;
	uint8_t id[3];
	smic_bootspi_open(bootspi_test);
	smic_bootspi_control(bootspi_test, SMIC_BOOTSPI_RESET, NULL);
	smic_bootspi_control(bootspi_test, SMIC_BOOTSPI_READ_ID, (void *)id);
	EMBARC_PRINTF("bootspi flash id: %02X%02X%02X\n", id[0], id[1], id[2]);

	for (i=0; i<SMIC_BOOTSPI_SEC_SIZE; i++) {
		g_test_wr_buf[i] = i & 0xFF;
	}
	EMBARC_PRINTF("sector erase write and read test\r\n");
	for (sec=0; sec < SMIC_BOOTSPI_SECS_PER_CHIP; sec++) {
		smic_bootspi_read(bootspi_test, sec*SMIC_BOOTSPI_SEC_SIZE, SMIC_BOOTSPI_SEC_SIZE, g_test_rd_buf);
		//EMBARC_PRINTF("erase page 0x%X, addr = 0x%X\r\n", sec, sec * SMIC_BOOTSPI_SEC_SIZE);

		smic_bootspi_control(bootspi_test, SMIC_BOOTSPI_SEC_ERASE, (void *)(sec*SMIC_BOOTSPI_SEC_SIZE));
		smic_bootspi_read(bootspi_test, sec*SMIC_BOOTSPI_SEC_SIZE, SMIC_BOOTSPI_SEC_SIZE, g_test_rd_buf);

		for (i = 0; i < SMIC_BOOTSPI_SEC_SIZE; i++) {
			if (g_test_rd_buf[i] != 0xFF) {
				EMBARC_PRINTF("pos[0x%x], rd: 0x%x, erase fail\n", i + sec*SMIC_BOOTSPI_SEC_SIZE, g_test_rd_buf[i]);
				break;
			}
		}

		smic_bootspi_write(bootspi_test, sec*SMIC_BOOTSPI_SEC_SIZE, SMIC_BOOTSPI_SEC_SIZE, g_test_wr_buf);
		smic_bootspi_read(bootspi_test, sec*SMIC_BOOTSPI_SEC_SIZE, SMIC_BOOTSPI_SEC_SIZE, g_test_rd_buf);

		for (i = 0; i < SMIC_BOOTSPI_SEC_SIZE; i++) {
			if (g_test_rd_buf[i] != g_test_wr_buf[i]) {
				EMBARC_PRINTF("pos[0x%x], rd: 0x%x, wr: 0x%x, write fail\n", i + sec*SMIC_BOOTSPI_SEC_SIZE, g_test_rd_buf[i], g_test_wr_buf[i]);
				break;
			}
		}
	}
	EMBARC_PRINTF("sector erase write and read test finish\r\n");
	EMBARC_PRINTF("one byte write and read test\r\n");
	for (i = 0; i < 256; i++) {
		wb = i & 0xFF;
		smic_bootspi_write(bootspi_test, i, 1, &wb);
		smic_bootspi_read(bootspi_test, i, 1, &rb);
		if( wb != rb) EMBARC_PRINTF("pos[0x%08X], rd: 0x%02X, wr: 0x%02X\n", i, rb, wb);
	}
	EMBARC_PRINTF("one byte write and read test finish\r\n");

	EMBARC_PRINTF("dev_flash: one byte write and read test\r\n");
	DEV_FLASH_PTR bootspi_ptr = flash_get_dev(IOTDK_BOOT_SPI_FLASH_ID);
	bootspi_ptr->flash_open();
	DEV_FLASH_INFO bootspi_info;
	bootspi_ptr->flash_control(FLASH_CMD_GET_INFO, (void *)&bootspi_info);
	EMBARC_PRINTF("bootspi: begin_addr = 0x%X, total_size = 0x%X\r\n",bootspi_info.begin_addr, bootspi_info.total_size);
	for (i = 0; i < 512; i++) {
		wb = i & 0xFF;
		bootspi_ptr->flash_write(bootspi_info.begin_addr+i, &wb, 1);
		bootspi_ptr->flash_read(bootspi_info.begin_addr+i, &rb, 1);
		if( wb != rb)
			EMBARC_PRINTF("pos[0x%08X], rd: 0x%02X, wr: 0x%02X\n", i, rb, wb);
	}
	EMBARC_PRINTF("dev_flash: one byte write and read test finish\r\n");
	EMBARC_PRINTF("%s\n", __func__);

	return E_OK;
}

