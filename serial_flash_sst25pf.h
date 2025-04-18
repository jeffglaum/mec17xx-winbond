/*******************************************************************************
© 2017 Microchip Technology Inc. and its subsidiaries.  You may use this
software and any derivatives exclusively with Microchip products.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER EXPRESS,
IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES
OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE, OR
ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR
USE IN ANY APPLICATION.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER
RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF
THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWED
BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS
SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY
TO MICROCHIP FOR THIS SOFTWARE.

MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
TERMS.
****************************************************************************/
#ifndef _SERIAL_FLASH_SST25PF_H
#define _SERIAL_FLASH_SST25PF_H

#define SERIAL_FLASH_MEMORY_BASE    0x0
//#define SERIAL_FLASH_MEMORY_TOP     0x800000

#define SERIAL_FLASH_SECTOR_SIZE    (4*1024)
#define SERIAL_FLASH_BLOCK_SIZE     (64*1024)

#define PAGE_SIZE   256
#define PAGE_ADDRESS_ALIGN_MASK     0xFFFFFF00 //256 bytes per page

#define PAGE_PROGRAM_MAX_TIME_MS    5           // Maximum time for page program
#define PAGE_PROGRAM_TYP_TIME_MS    4           // Typ? time for page program
#define PAGE_PROGRAM_CHECK_BUSY_TIME_MS   4     // Polling time for checking busy status
#define PAGE_PROGRAM_CHECK_BUSY_POLL_MS   1     // Interval between subsequent read status, during polling

#define SECTOR_ERASE_MAX_TIME_MS    150          // Maximum time for sector erase
#define SECTOR_ERASE_TYP_TIME_MS    40           // Typ? time for sector erase
#define SECTOR_ERASE_CHECK_BUSY_TIME_MS   120    // Polling time for checking busy status after waiting Typ time
#define SECTOR_ERASE_CHECK_BUSY_POLL_MS   10     // Interval between subsequent read status, during polling

#define BLOCK_ERASE_MAX_TIME_MS    250          // Maximum time for block erase
#define BLOCK_ERASE_TYP_TIME_MS    80           // Typ? time for block erase
#define BLOCK_ERASE_CHECK_BUSY_TIME_MS   180    // Polling time for checking busy status after waiting Typ time
#define BLOCK_ERASE_CHECK_BUSY_POLL_MS   10     // Interval between subsequent read status, during polling

#define CHIP_ERASE_MAX_TIME_MS    2000          // Maximum time for chip erase
#define CHIP_ERASE_TYP_TIME_MS    250           // Typ? time for chip erase
#define CHIP_ERASE_CHECK_BUSY_TIME_MS   2000    // Polling time for checking busy status after waiting Typ time
#define CHIP_ERASE_CHECK_BUSY_POLL_MS   25      // Interval between subsequent read status, during polling

#define STATUS_BUSY_BIT_MASK    (1 << 0)

#define MAX_ERASE_TIME            200000         // Maximum time for 16mb flash chip erase

uint8_t sst25pf_read_status_register(uint8_t *status, uint32_t *qmspi_status);
uint8_t sst25pf_write_status_register(uint8_t value, uint32_t *qmspi_status);
uint8_t sst25pf_memory_write_enable(uint32_t *qmspi_status);
uint8_t sst25pf_read_jedec_id(uint8_t *jedec_id, uint32_t *qmspi_status);
uint8_t sst25pf_read_id(uint8_t *read_id, uint32_t *qmspi_status);
uint8_t sst25pf_write_enable(uint32_t *qmspi_status);
uint8_t sst25pf_write_disable(uint32_t *qmspi_status);
uint8_t sst25pf_chip_erase(uint32_t *qmspi_status);
uint8_t sst25pf_sector_erase(uint32_t address, uint32_t *qmspi_status);
uint8_t sst25pf_block_erase(uint32_t address, uint32_t *qmspi_status);
uint8_t sst25pf_sector_erase_sequence(uint32_t erase_address, uint32_t *qmspi_status);
uint8_t sst25pf_block_erase_sequence(uint32_t erase_address, uint32_t *qmspi_status);
uint8_t sst25pf_chip_erase_sequence(uint32_t *qmspi_status);
uint8_t sst25pf_program(uint32_t spi_addr, uint32_t mem_addr, uint32_t data_len, uint32_t *qmspi_status);
uint8_t sst25pf_check_busy(uint8_t *busy_status, uint16_t timeout_ms, uint16_t read_interval_ms, uint32_t *qmspi_status);


#endif // _SERIAL_FLASH_SST25PF_H
