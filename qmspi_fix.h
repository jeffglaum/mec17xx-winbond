/*****************************************************************************
* Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
* You may use this software and any derivatives exclusively with
* Microchip products.
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".
* NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
* TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
* CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
* FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
* OF THESE TERMS.
*****************************************************************************/

/** @file mec2016_qmspi.h
 *MEC2016 QMSPI Class Definition
 */
/** @defgroup MEC2016 Periph QMSPI
 */

#ifndef _MEC2016_QMSPI_H
#define _MEC2016_QMSPI_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#if   defined (__CC_ARM)
  #pragma anon_unions
#endif
typedef struct qmspir QMSPI_TypeDef;
/* Register Union */
typedef union
{
    volatile uint32_t w;
    volatile uint16_t h[2];
    volatile uint8_t  b[4];
    struct {
        volatile uint8_t byte0;
        volatile uint8_t byte1;
        volatile uint8_t byte2;
        volatile uint8_t byte3;
    };
    struct {
        volatile uint16_t hword0;
        volatile uint16_t hword1;
    };
} REG32_U;

#define QMSPI_MAX_DESCR (15ul)
#define QMSPI0_BASE     (0x40000000UL + 0x70000UL) /*!< (QMSPI 0    ) Base Address */

#define QMSPI0          ((QMSPI_TypeDef *) QMSPI0_BASE)
struct qmspir {
    REG32_U  MODE;           /*!< Offset: 0x0000  QMSPI Mode/ClockDiv/Rst/Activate */
    REG32_U  CTRL;           /*!< Offset: 0x0004  QMSPI Control */
    REG32_U  EXE;            /*!< Offset: 0x0008  QMSPI Execute */
    REG32_U  IF_CTRL;        /*!< Offset: 0x000C  QMSPI Interface Control */
    REG32_U  STATUS;         /*!< Offset: 0x0010  QMSPI Status */
    REG32_U  BUF_CNT_STS;    /*!< Offset: 0x0014  QMSPI FIFO Buffer Count Status, Read-Only */
    REG32_U  IEN;            /*!< Offset: 0x0018  QMSPI Interrupt Enable */
    REG32_U  BUF_CNT_TRIG;   /*!< Offset: 0x001C  QMSPI FIFO Buffer Count Trigger Levels */
    REG32_U  TX_FIFO;        /*!< Offset: 0x0020  QMSPI Transmit FIFO register */
    REG32_U  RX_FIFO;        /*!< Offset: 0x0024  QMSPI Receive FIFO register */
    REG32_U  CS_TIMING_IDX;  /*!< Offset: 0x0028  QMSPI CS Timing Register */
    uint32_t RSVDC;         // +0x2c - 0x2F
    REG32_U  DESCR[QMSPI_MAX_DESCR]; /*!< Offset: 0x0030 - 0x006C  QMSPI Descriptor registers */
};
typedef enum
{
    QMSPI_DESCR0 = 0u,
    QMSPI_DESCR1 = 1u,
    QMSPI_DESCR2 = 2u,
    QMSPI_DESCR3 = 3u,
    QMSPI_DESCR4 = 4u,
    QMSPI_DESCR_MAX = 5u
} QMSPI_DESCR_ID;

typedef enum {
    QMSPI_XFR_COMPLETE  = (1ul << 0),
    QMSPI_DMA_COMPLETE  = (1ul << 1),
    QMSPI_TX_BUFF_ERR   = (1ul << 2),
    QMSPI_RX_BUFF_ERR   = (1ul << 3),
    QMSPI_PROG_ERR      = (1ul << 4),
    QMSPI_TX_BUFF_FULL  = (1ul << 8),
    QMSPI_TX_BUFF_EMPTY = (1ul << 9),
    QMSPI_TX_BUFF_REQ   = (1ul << 10),
    QMSPI_TX_BUFF_STALL = (1ul << 11),
    QMSPI_RX_BUFF_FULL  = (1ul << 12),
    QMSPI_RX_BUFF_EMPTY = (1ul << 13),
    QMSPI_RX_BUFF_REQ   = (1ul << 14),
    QMSPI_RX_BUFF_STALL = (1ul << 15),
    QMSPI_XFR_ACTIVE    = (1ul << 16)
} QMSPI_STATUS_MASK;

#define TX_MODE    1
#define RX_MODE    0

// Control Register
#define QMSPI_CTRL_OFS              (0x04)

#define QMSPI_C_MASK             (0x0001FFFFul)
#define QMSPI_C_XFR_LEN_MAX      (0x7FFFul)
#define QMSPI_C_XFR_LEN_MASK0    (0x7FFFul)
#define QMSPI_C_XFR_LEN_MASK     (0xFFFE0000ul)
#define QMSPI_C_XFR_LEN_BITPOS   (17u)

#define QMSPI_C_IO1                 (0ul)
#define QMSPI_C_IO2                 (1ul)
#define QMSPI_C_IO4                 (2ul)
#define QMSPI_C_IO_WIDTH_MASK       (3ul)
#define QMSPI_C_TX_BITPOS           (2u)
#define QMSPI_C_TX_DIS              (0ul << QMSPI_C_TX_BITPOS)
#define QMSPI_C_TX_DATA             (1ul << QMSPI_C_TX_BITPOS)
#define QMSPI_C_TX_ZEROS            (2ul << QMSPI_C_TX_BITPOS)
#define QMSPI_C_TX_ONES             (3ul << QMSPI_C_TX_BITPOS)
#define QMSPI_C_TX_OUT_MASK         (3ul << QMSPI_C_TX_BITPOS)
#define QMSPI_C_TX_DMA_BITPOS       (4u)
#define QMSPI_C_TX_DMA_DIS          (0ul << QMSPI_C_TX_DMA_BITPOS)
#define QMSPI_C_TX_DMA_1B           (1ul << QMSPI_C_TX_DMA_BITPOS)
#define QMSPI_C_TX_DMA_2B           (2ul << QMSPI_C_TX_DMA_BITPOS)
#define QMSPI_C_TX_DMA_4B           (3ul << QMSPI_C_TX_DMA_BITPOS)
#define QMSPI_C_TX_DMA_MASK0        (3ul)
#define QMSPI_C_TX_DMA_MASK         (3ul << QMSPI_C_TX_DMA_BITPOS)
#define QMSPI_C_RX_BITPOS           (6u)
#define QMSPI_C_RX_DIS              (0ul << QMSPI_C_RX_BITPOS)
#define QMSPI_C_RX_EN               (1ul << QMSPI_C_RX_BITPOS)
#define QMSPI_C_RX_DMA_BITPOS       (7u)
#define QMSPI_C_RX_DMA_DIS          (0ul << (QMSPI_C_RX_DMA_BITPOS))
#define QMSPI_C_RX_DMA_1B           (1ul << (QMSPI_C_RX_DMA_BITPOS))
#define QMSPI_C_RX_DMA_2B           (2ul << (QMSPI_C_RX_DMA_BITPOS))
#define QMSPI_C_RX_DMA_4B           (3ul << (QMSPI_C_RX_DMA_BITPOS))
#define QMSPI_C_RX_DMA_MASK0        (3ul)
#define QMSPI_C_RX_DMA_MASK         (3ul << (QMSPI_C_TX_DMA_BITPOS))
#define QMSPI_C_XFR_CLOSE_BITPOS    (9u)
#define QMSPI_C_XFR_CLOSE           (1ul << QMSPI_C_XFR_CLOSE_BITPOS)
#define QMSPI_C_XFR_NOT_CLOSE       (0ul << QMSPI_C_XFR_CLOSE_BITPOS)
#define QMSPI_C_XFR_UNIT_BITPOS     (10u)
#define QMSPI_C_XFR_UNIT_MASK0      (3ul)
#define QMSPI_C_XFR_UNIT_MASK       (3ul << QMSPI_C_XFR_UNIT_BITPOS)
#define QMSPI_C_XFR_UNIT_BITS       (0ul << QMSPI_C_XFR_UNIT_BITPOS)
#define QMSPI_C_XFR_UNIT_1B         (1ul << QMSPI_C_XFR_UNIT_BITPOS)
#define QMSPI_C_XFR_UNIT_4B         (2ul << QMSPI_C_XFR_UNIT_BITPOS)
#define QMSPI_C_XFR_UNIT_16B        (3ul << QMSPI_C_XFR_UNIT_BITPOS)
#define QMSPI_C_FN_DESCR_BITPOS     (12u)
#define QMSPI_C_FN_DESCR_MASK0      (0xFul)
#define QMSPI_C_FN_DESCR_MASK       (0xFul << QMSPI_C_FN_DESCR_BITPOS)
#define QMSPI_C_NEXT_DESCR_0        (0ul << QMSPI_C_FN_DESCR_BITPOS)
#define QMSPI_C_NEXT_DESCR_1        (1ul << QMSPI_C_FN_DESCR_BITPOS)
#define QMSPI_C_NEXT_DESCR_2        (2ul << QMSPI_C_FN_DESCR_BITPOS)
#define QMSPI_C_NEXT_DESCR_3        (3ul << QMSPI_C_FN_DESCR_BITPOS)
#define QMSPI_C_NEXT_DESCR_4        (4ul << QMSPI_C_FN_DESCR_BITPOS)
#define QMSPI_C_DESCR_EN_BITPOS     (16u)
#define QMSPI_C_DESCR_LAST_BITPOS   (16u)
#define QMSPI_C_DESCR_MODE_EN       (1ul << QMSPI_C_DESCR_EN_BITPOS)
#define QMSPI_C_DESCR_LAST          (1ul << QMSPI_C_DESCR_LAST_BITPOS)
#define QMSPI_C_DESCR_NOT_LAST      (0ul << QMSPI_C_DESCR_LAST_BITPOS)

// Execute register
#define QMSPI_EXE_START       (1ul)
#define QMSPI_EXE_STOP        (2ul)
#define QMSPI_EXE_CLR_FIFOS   (4ul)

#endif // #ifndef _MEC2016_QMSPI_H
/**   @}
 */

