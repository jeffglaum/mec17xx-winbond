#include <stdint.h>
#include <string.h>
#include <qmspi_fix.h>
#include <rom_qmspi.h>
#include <platform.h>
#include <MCHP_MEC1703_C0.h>
#include <serial_flash_sst25pf.h>

// Retry count for polling 'api_qmspi_is_done_status' in milliseconds
#define QMSPI_STATUS_RETRY_COUNT    200
#define W25M512_SECTOR_SIZE         0x100
#define COMMAND_BUFFER_SIZE         4

typedef struct {    
    uint16_t write_len;    
    uint8_t read_len;
    uint8_t rsvd1;
    uint8_t rsvd2;
    uint8_t *write_buf;
    uint8_t *read_buf;    
} SERIAL_FLASH_COMMAND;

void timer_enable(void);
void platform_qmspi_init(uint8_t spi_port);
uint8_t w25m512_sector_erase_sequence(uint32_t erase_address, uint32_t *qmspi_status);
uint8_t w25m512_program_page_sequence(uint32_t spi_addr, uint32_t mem_addr, uint32_t data_len, uint32_t *qmspi_status);

// NOTE: initialize the global so it lands in the .data section (probe-rs
// doesn't consider the .bss section when computing target memory layout).
uint32_t g_flashbase = 0x50000000;

// -----------------------------------------------------------------------------

/**
 * @brief       Initialize the MCU to handle SPI NOR flash access
 *
 * @param[in]   address  Base address of the SPI NOR flash
 * @param[in]   ...      Note that probe-rs also loads Cortex-M4 registers with other
 *                       parameters (ex: clock frequency) but we don't use them here.
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
__attribute__((section(".api")))
__attribute__((noinline))
__attribute__((used))
uint32_t Init(uint32_t address)
{
    // Capture the flash base address (used later to compute sector and page offsets)
    g_flashbase = address;

    // Enable the timer
    timer_enable();

    // Initialize the SPI controller
    platform_qmspi_init(SHD_SPI);

    return 0;
}

/**
 * @brief       Erase the SPI NOR sector specified
 *
 * @param[in]   address  Base address of the SPI NOR flash sector to erase
 *
 * @return      Returns zero for success or non-zero for failure
 *
 * @warning     This routine assumes the address provided is sector-aligned
 * 
 */
__attribute__((section(".api")))
__attribute__((noinline))
__attribute__((used))
uint32_t EraseSector(uint32_t address)
{
    uint32_t qmspi_status = 0; 

    // Calculate sector offset (note we assume the offset is sector-aligned)
    if (address >= g_flashbase) {
        address -= g_flashbase;
    }

    return w25m512_sector_erase_sequence(address, &qmspi_status);
}

/**
 * @brief       Program the SPI NOR page specified with the data provided
 *
 * @param[in]   page_address    Base address of the SPI NOR flash page to write
 * @param[in]   data_size       Total number of bytes to be written to the page
 * @param[in]   buffer_address  Base address of the data buffer containing the
 *                              data to be written to SPI NOR
 *
 * @return      Returns zero for success or non-zero for failure
 *
 * @warning     This routine assumes the address provided is page-aligned
 * 
 */
__attribute__((section(".api")))
__attribute__((noinline))
__attribute__((used))
uint32_t ProgramPage(uint32_t page_address, uint32_t data_size, uint32_t buffer_address)
{
    uint32_t qmspi_status = 0; 

    // Calculate page offset (note we assume the offset is page-aligned)
    if (page_address >= g_flashbase) {
        page_address -= g_flashbase;
    }

    return w25m512_program_page_sequence(page_address, buffer_address, data_size, &qmspi_status);
}

// -----------------------------------------------------------------------------

/**
 * @brief       Enable the timer
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @note        We use CCTM0 free run counter at divide by 16 (48/16 = 3 MHz, T ~= 333 ns)
 *              Range is +/- 2% [2.94, 3.06] MHz, [0.340, 0.327] us
 *              Max time = 2^32 * 0.327 us ~ 1404 seconds
 *              FPGA has 24MHz master clock running all devices at 1/2 speed. For FPGA we will
 *              program CCT to 6 MHz to get same absolute times.
 * 
 */
void timer_enable(void)
{
    uint16_t ctrl16;
   
    // ASIC 48MHz divide by 16 -> 3 MHz
    ctrl16 = CCT_CTRL_TCLK_DIV_16 + CCT_CTRL_FREE_RUN_EN + CCT_CTRL_ACTIVATE;
    
    CAPTURE_COMPARE_TIMER_INST->CAPTURE_COMPARE_TIMER_CONTROL = ctrl16 | CCT_CTRL_FREE_RUN_RESET;
    
    for (uint16_t i = 0u; i < 32ul; i++) {
        if (0 == (CAPTURE_COMPARE_TIMER_INST->CAPTURE_COMPARE_TIMER_CONTROL & (CCT_CTRL_FREE_RUN_RESET))) {
            break;
        }
    }
    
    CAPTURE_COMPARE_TIMER_INST->CAPTURE_COMPARE_TIMER_CONTROL = ctrl16;
}

/**
 * @brief       Timer delay in timer count units
 *
 * @param[in]   counts_to_delay Number of counts to delay
 *
 * @return      None
 *
 * @note        Input Capture Compare free run counter is operating 
 *              at (3 MHz +/- 2%) [2.94, 3.06] MHz, [0.340, 0.327] us.
 *              Assume +2% to insure delays meet the minimum.
 *              1 us = 1/0.327 = 3.06 counts
 *              num_us = num_us * 3 = num_us * (2+1) = (num_us << 1) + num_us
 * 
 */
void timer_delay(uint32_t counts_to_delay)
{
    uint32_t diff, curr_cnt, start_cnt;
    
    if (counts_to_delay) {
        start_cnt = CAPTURE_COMPARE_TIMER_INST->FREE_RUNNING_TIMER;
        do {
            curr_cnt = CAPTURE_COMPARE_TIMER_INST->FREE_RUNNING_TIMER;
            if (curr_cnt >= start_cnt) {
                diff = curr_cnt - start_cnt;
            } else {
                diff = curr_cnt + (0xfffffffful - start_cnt);
            }
        } while (diff < counts_to_delay);
    }
}

/**
 * @brief       Timer delay in milliseconds
 *
 * @param[in]   timer_delay_ms  Number of milliseconds to delay
 *
 * @return      None
 *
 * @note        Input Capture Compare free run counter is operating at (3 MHz +/- 2%)
 *              [2.94, 3.06] MHz, [0.340, 0.327] us
 *              Assume +2% to insure delays meet the minimum.
 *              1 ms = 3060 (0x0BF4) counts
 *              count = ms * 3060 = ms * (3072 - 12) = ms * (0xC00 - 0x0C)
 *              = ms * (0x800 + 0x400 - (0x08 + 0x04))
 *              = (ms * 0x800) + (ms * 0x400) - (ms * 0x08) - (ms * 0x04)
 *              = (ms << 11) + (ms << 10) - (ms << 3) - (ms << 2)
 * 
 */
void timer_delay_ms(uint32_t num_ms)
{
    uint32_t counts = (num_ms << 11) + (num_ms << 10) - (num_ms << 3) - (num_ms << 2);
    
    timer_delay(counts);
}

// -----------------------------------------------------------------------------

/**
 * @brief       Initialize the quad SPI controller
 *
 * @param[in]   spi_port    SPI controller port to be initialized
 *
 * @return      None
 *
 */
void platform_qmspi_init(uint8_t spi_port){
    
    api_qmspi_port_ctrl(spi_port, SPI_IO_FD_DUAL, DEV_ENABLE);
    api_qmspi_init(QMSPI_SPI_MODE_0, QMSPI_FREQ_24M, NONE);
    DMA_MAIN_INST->DMA_MAIN_CONTROL = 1;
}

/**
 * @brief       Read & mask the quad SPI controller status register
 *
 * @param[in]   mask    Mask value to be applied to the status register
 *
 * @return      Masked result of the status register read
 *
 */
inline uint32_t qmspi_status_mask(uint32_t mask)
{
    return QMSPI0->STATUS.w & mask;
}

/**
 * @brief       Provides the default quad SPI read-write timeout value
 *
 * @param[in]   None
 *
 * @return      Default timeout value
 *
 */
uint32_t qmspi_timeout1b_default(void)
{
    return 0;
}

/**
 * @brief       Main quad SPI read-write access routine
 *
 * @param[in]   mode    Read (rx) or write (tx) mode
 * @param[in]   nbytes  Number of bytes to read or write
 * @param[in]   ptr     Write data
 * @param[in]   nptr    Read data
 * @param[in]   ftmout  Timeout routine
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t qmspi_tx_rx(uint8_t mode, uint32_t nbytes, uint8_t *ptr, uint32_t *nptr, FP_RU32_V ftmout)
{
    uint32_t n, tx_rx_msk;

    if ((NULL == ptr) || (NULL == nptr)) {
        return 0;
    }

    *nptr = 0;

    if (NULL == ftmout) {
        ftmout = qmspi_timeout1b_default;
    }

    if (nbytes >= QMSPI_C_XFR_LEN_MAX) {
        nbytes = QMSPI_C_XFR_LEN_MAX;
    }
    if (mode){
        QMSPI0->CTRL.w = QMSPI_C_IO1 + QMSPI_C_TX_DATA + QMSPI_C_XFR_UNIT_1B;
        tx_rx_msk = QMSPI_TX_BUFF_FULL;
    } else {
        QMSPI0->CTRL.w = QMSPI_C_IO1 + QMSPI_C_RX_EN + QMSPI_C_XFR_UNIT_1B;
        tx_rx_msk = QMSPI_RX_BUFF_EMPTY;
    }
    QMSPI0->CTRL.w += nbytes << QMSPI_C_XFR_LEN_BITPOS;

    if (nbytes) {
        QMSPI0->EXE.b[0] = QMSPI_EXE_START;
    }

    n = 0;
    while (n < nbytes) {
        while (0 != qmspi_status_mask(tx_rx_msk)) {
            if (ftmout()) {
                if (NULL != nptr) {
                    *nptr = n;
                }
                return 0;
            }
        }
        if (mode){
            QMSPI0->TX_FIFO.b[0] = (*ptr++);
        } else {
            *ptr++ = QMSPI0->RX_FIFO.b[0];    
        }
        n++;
    }

    if (NULL != nptr) {
        *nptr = n;
    }

    return 1;
}

/**
 * @brief       Send commands to the SPI NOR flash
 *
 * @param[in]   ntx     Number of bytes to sent to flash
 * @param[in]   ptx     Data to sent to flash
 * @param[in]   nrx     Number of bytes to read from flash
 * @param[in]   prx     Data read from flash
 * @param[in]   ftmout  Timeout routine
 *
 * @return      Returns zero for success or non-zero for failure
 *
 * @note        Transmit up to TX_FIFO length bytes and optionally read up to RX_FIFO length bytes
 *              SPI transaction is closed (chip select de-asserted when done).
 *              Transmit and receive are performed full-duplex.
 *              This routine is useful for all the miscellaneous flash command such as read enable,
 *              read status, write status, etc.
 *              This routine is blocking as it does not use descriptors.
 */
uint8_t api_qmspi_flash_cmd(uint32_t ntx, uint8_t *ptx, uint32_t nrx, uint8_t *prx, FP_RU32_V ftmout)
{
    uint32_t n2;
    uint8_t rc;

    if (ntx && (NULL == ptx)) {
        return 1;
    }
    if (nrx && (NULL == prx)) {
        return 2;
    }

    if (qmspi_status_mask(QMSPI_XFR_ACTIVE)) {
        return 4;
    }

    if (ntx) {
        QMSPI0->EXE.b[0] = QMSPI_EXE_CLR_FIFOS;
        QMSPI0->STATUS.w = 0xfffffffful;
        if(!(qmspi_tx_rx(TX_MODE, ntx, ptx, &n2, ftmout))) {
            rc = 5;
            QMSPI0->CTRL.w |= QMSPI_C_XFR_CLOSE;
            QMSPI0->EXE.b[0] = QMSPI_EXE_STOP;
            return rc;
        }
    }

    if (nrx) {
        QMSPI0->EXE.b[0] = QMSPI_EXE_CLR_FIFOS;
        QMSPI0->STATUS.w = 0xfffffffful;

        if (!(qmspi_tx_rx(RX_MODE, nrx, prx, &n2, ftmout))) {
            rc = 6;
            // stop & close transaction
            QMSPI0->CTRL.w |= QMSPI_C_XFR_CLOSE;
            QMSPI0->EXE.b[0] = QMSPI_EXE_STOP;
            return rc;
        }
    }

    rc = 0;
    // stop & close transaction
    QMSPI0->CTRL.w |= QMSPI_C_XFR_CLOSE;
    QMSPI0->EXE.b[0] = QMSPI_EXE_STOP;

    return rc;
}

/**
 * @brief       Checks whether the last flash command issued is complete
 *
 * @param[out]   qmspi_status   Status reported by the quad-SPI controller
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t flash_check_done_status(uint32_t *qmspi_status)
{
    uint16_t count;
    uint8_t ret_val, done_status;
    
    count = QMSPI_STATUS_RETRY_COUNT;    
    do {
        done_status = api_qmspi_is_done_status(qmspi_status);
        timer_delay_ms(1);
        count--;
        
    } while ((!done_status) && (count > 0));
    
    ret_val = 1;
    if (done_status) {
        ret_val = 0;
    }
    
    return ret_val;
}

/**
 * @brief       Processes a single SPI NOR flash command and waits for completion
 *
 * @param[in]    cmd            Flash command & data
 * @param[out]   qmspi_status   Status reported by the quad-SPI controller
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t flash_process_cmd(SERIAL_FLASH_COMMAND *cmd, uint32_t *qmspi_status)
{   
    if (api_qmspi_flash_cmd(cmd->write_len, cmd->write_buf, cmd->read_len, cmd->read_buf, NULL)) {
        return 1;
    }
    
    return flash_check_done_status(qmspi_status);  
}

/**
 * @brief       Erase a single SPI NOR flash sector
 *
 * @param[in]    address        Sector address to be erased
 * @param[out]   qmspi_status   Status reported by the quad-SPI controller
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t w25m512_sector_erase(uint32_t address, uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    uint8_t cmd_buffer[COMMAND_BUFFER_SIZE]={0}; 
    
	cmd_buffer[0] = 0x20;
    cmd_buffer[1] = (address >> 16) & 0xFF;
    cmd_buffer[2] = (address >>  8) & 0xFF;
    cmd_buffer[3] = (address      ) & 0xFF;
        
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 4;
    cmd.read_buf = NULL;
    cmd.read_len = 0;

    return flash_process_cmd(&cmd, qmspi_status);   
}

/**
 * @brief       Read SPI NOR flash status register
 *
 * @param[out]   status         Status register value read
 * @param[out]   qmspi_status   Status reported by the quad-SPI controller
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t w25m512_read_status_register(uint8_t *status, uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    uint8_t cmd_buffer[COMMAND_BUFFER_SIZE]={0}; 
    
    cmd_buffer[0] = 0x05;
    
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 1;
    cmd.read_buf = status;
    cmd.read_len = 1;
    
    return flash_process_cmd(&cmd, qmspi_status); 
}

/**
 * @brief       Checks whether the SPI NOR flash is busy
 *
 * @param[out]   busy_status        Current busy status
 * @param[in]    timeout_ms         Number of milliseconds to wait when checking busy status
 * @param[in]    read_interval_ms   Number of milliseconds in between attempts to read busy status
 * @param[out]   qmspi_status       Status reported by the quad-SPI controller
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t w25m512_check_busy(uint8_t *busy_status, uint16_t timeout_ms, uint16_t read_interval_ms, uint32_t *qmspi_status)
{
    uint8_t read_status, ret_val=0;
    uint16_t delay_ms;
    
    *busy_status = 1;
    
    do {        
        // Read the status register
        if (w25m512_read_status_register(&read_status, qmspi_status)) {
            ret_val = 1;
            break;
        }    
            
        // Check the busy bit
        if (!(read_status & STATUS_BUSY_BIT_MASK)) {
            *busy_status = 0;
            break;
        }
        
        delay_ms = read_interval_ms;
        if (timeout_ms < read_interval_ms) {
            delay_ms = timeout_ms;
        }
        
        timer_delay_ms(delay_ms);
        timeout_ms -= delay_ms;
        
    } while (timeout_ms);
    
    return ret_val;
}

/**
 * @brief       Enables writes to the SPI NOR flash
 *
 * @param[out]   qmspi_status       Status reported by the quad-SPI controller
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t w25m512_write_enable(uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    uint8_t cmd_buffer[COMMAND_BUFFER_SIZE]={0};

    cmd_buffer[0] = 0x06;
        
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 1;
    cmd.read_buf = NULL;
    cmd.read_len = 0;
    
    return flash_process_cmd(&cmd, qmspi_status);   
}

/**
 * @brief       Disables writes to the SPI NOR flash
 *
 * @param[out]   qmspi_status       Status reported by the quad-SPI controller
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t w25m512_write_disable(uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    uint8_t cmd_buffer[COMMAND_BUFFER_SIZE]={0};  
    
    cmd_buffer[0] = 0x04;
        
    cmd.write_buf = &cmd_buffer[0];
    cmd.write_len = 1;
    cmd.read_buf = NULL;
    cmd.read_len = 0;
    
    return flash_process_cmd(&cmd, qmspi_status);   
}

/**
 * @brief       Writes a single page of data to the SPI NOR flash
 *
 * @param[in]   page_address    Base address of the SPI NOR flash page to be written
 * @param[in]   memory_address  Base address of the memory buffer containing data to be written
 * @param[in]   data_size       Number of bytes to be written (at most one page of data) 
 * @param[out]  qmspi_status    Status reported by the quad-SPI controller
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t flash_write_page(uint32_t page_address, uint32_t memory_address, uint32_t data_size, uint32_t *qmspi_status)
{
    SERIAL_FLASH_COMMAND cmd;
    uint8_t g_page_program_buffer[W25M512_SECTOR_SIZE + COMMAND_BUFFER_SIZE];

    g_page_program_buffer[0] = 0x02;
    g_page_program_buffer[1] = (page_address >> 16) & 0xFF;
    g_page_program_buffer[2] = (page_address >>  8) & 0xFF;
    g_page_program_buffer[3] = (page_address      ) & 0xFF;
  
    memcpy(&g_page_program_buffer[COMMAND_BUFFER_SIZE], (uint8_t *)memory_address, data_size);

    cmd.write_buf = &g_page_program_buffer[0];
    cmd.write_len = sizeof(g_page_program_buffer);
    cmd.read_buf = NULL;
    cmd.read_len = 0;

    return flash_process_cmd(&cmd, qmspi_status);    
}

/**
 * @brief       Full sequence to writes a page of data to the SPI NOR flash
 *
 * @param[in]   page_address    Base address of the SPI NOR flash page to be written
 * @param[in]   memory_address  Base address of the memory buffer containing data to be written
 * @param[in]   data_size       Number of bytes to be written (at most one page of data) 
 * @param[out]  qmspi_status    Status reported by the quad-SPI controller
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t w25m512_program_page_sequence(uint32_t page_address, uint32_t memory_address, uint32_t data_size, uint32_t *qmspi_status)
{    
    uint8_t ret_val = 0;
    uint8_t busy_status = 0;
   
    // Verify data is a page size or less
    if (data_size > PAGE_SIZE) {
        ret_val = 1;
        goto exit;
    }
    
    // Enable writes to flash
    if (w25m512_write_enable(qmspi_status)) {
        ret_val = 1;
        goto exit;
    }            

    // Write page to flash
    if (flash_write_page(page_address, memory_address, data_size, qmspi_status)) {
        ret_val = 1;
        goto exit;
    }                
            
    // Wait for page program to complete
    timer_delay_ms(PAGE_PROGRAM_TYP_TIME_MS);
            
    // Check busy bit by polling
    ret_val = w25m512_check_busy(&busy_status, 
                                 PAGE_PROGRAM_CHECK_BUSY_TIME_MS, 
                                 PAGE_PROGRAM_CHECK_BUSY_POLL_MS, 
                                 qmspi_status);
    if (ret_val || busy_status) {
        ret_val = 1;
        goto exit;
    }
            
    // Disable writes to flash
    w25m512_write_disable(qmspi_status);

exit:
    return ret_val;
}

/**
 * @brief       Full sequence to erase a SPI NOR flash sector
 *
 * @param[in]   sector_address  Base address of the SPI NOR flash sector to be erased
 * @param[out]  qmspi_status    Status reported by the quad-SPI controller
 *
 * @return      Returns zero for success or non-zero for failure
 *
 */
uint8_t w25m512_sector_erase_sequence(uint32_t sector_address, uint32_t *qmspi_status)
{    
    uint8_t ret_val = 0;
    uint8_t busy_status = 0;
    
    do {
        // Enable writes to flash
        if (w25m512_write_enable(qmspi_status))
        {
            ret_val = 1;
            break;
        }        
  
        // Erase flash sector
        if (w25m512_sector_erase(sector_address, qmspi_status)) {
            ret_val = 1;
            break;
        }        
                    
        // Wait for erase to complete
        timer_delay_ms(SECTOR_ERASE_TYP_TIME_MS);
        
        // Check busy bit by polling
        ret_val = w25m512_check_busy(&busy_status, 
                                     SECTOR_ERASE_CHECK_BUSY_TIME_MS, 
                                     SECTOR_ERASE_CHECK_BUSY_POLL_MS, 
                                     qmspi_status);
        if (ret_val || busy_status) {
            ret_val = 1;
            break;
        } 
    } while (0);
    
    return ret_val;    
}
