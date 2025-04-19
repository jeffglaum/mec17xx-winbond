/*****************************************************************************
* © 2018 Microchip Technology Inc. and its subsidiaries.
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

/** @file platform.h
 *MEC1322 BootROM platform/cpu abstractions
 */
/** @defgroup bootrom include
 */


#ifndef _PLATFORM_H
#define _PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#define TRUE        (1)
#define FALSE       (0)

#define ON          (1)
#define OFF         (0)


#ifdef __CC_ARM             // Keil ARM MDK

#define USED __attribute__((used))
#define WEAK __attribute__((weak)) 
#define INLINE __inline
#define NORETURN __declspec(noreturn)
#define PACKED __packed
#define CPU_GET_INTERRUPT_STATE() __get_PRIMASK()
#define CPU_SET_INTERRUPT_STATE(x) __set_PRIMASK(x)
#define CPU_DISABLE_INTERRUPTS() __disable_irq()
/*
 * Keil MDK intrisic __disable_irq() returns the value of MSR PRIMASK
 * before disabling interrupts.
*/
#define CPU_GET_DISABLE_INTERRUPTS(x) {x = __disable_irq();}
#define CPU_RESTORE_INTERRUPTS(x) { if (!x) { __enable_irq(); } }
#define CPU_ENABLE_INTERRUPTS() __enable_irq()

#define CPU_NOP() __NOP()
#define CPU_WAIT_FOR_INTR() __WFI()

#define CPU_REV(x) __REV(x)

#elif defined(__ICCARM__)   // IAR ARM Compiler

#define USED __root
#define WEAK __weak
#define INLINE inline   
#define NORETURN __noreturn
#define CPU_INTERRUPT_STATE __istate_t

#define CPU_ENABLE_INTERRUPTS() __enable_irqs()
#define CPU_DISABLE_INTERRUPTS() __disable_irqs()

#define CPU_GET_DISABLE_INTERRUPTS(x) {x = __disable_irq();}
#define CPU_RESTORE_INTERRUPTS(x) { if (!x) { __enable_irq(); } }


#define CPU_NOP() __NOP()
#define CPU_WAIT_FOR_INTR() __WFI()
#define CPU_REV(x) __REV(x)


#elif defined(__GNUC__) && defined(__ARM_EABI__)    // GCC for ARM (arm-none-eabi-gcc)

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Assume CMSIS header files are present */

__attribute__( ( always_inline ) ) static inline void __NOP_THUMB2(void)
{
  __asm volatile ("nop");
}
 
#define CPU_NOP() __NOP_THUMB2()

__attribute__( ( always_inline ) ) static inline void __WFI_THUMB2(void)
{
  __asm volatile ("wfi");
}

#define CPU_WAIT_FOR_INTR() __WFI_THUMB2()

/* We require user have ARM CMSIS header files available and configured
 * core_cmFunc.h includes inlines for global interrupt control. 
 * For some reason, CMSIS __disable_irq for GCC does not return current PRIMASK 
 * value. */

__attribute__( ( always_inline ) ) static inline uint8_t __get_disable_irq(void)
{
    uint8_t pri_mask;
    
    __asm volatile (
        "MRS      %0, PRIMASK         \n\t"
        "CPSID    I                   \n\t"
        : "=r" (pri_mask) :: "memory"
    );
    return pri_mask;
}

__attribute__((always_inline)) static inline void __enable_irqs(void)
{
    __asm volatile ("cpsie i" : : : "memory");
}

__attribute__((always_inline)) static inline void __disable_irqs(void)
{
    __asm volatile ("cpsid i" : : : "memory");
}


#define CPU_GET_DISABLE_INTERRUPTS(x) {x = __get_disable_irq();}
#define CPU_RESTORE_INTERRUPTS(x) { if (!x) { __enable_irq(); } }
#define CPU_ENABLE_INTERRUPTS() __enable_irqs()
#define CPU_DISABLE_INTERRUPTS() __disable_irqs()


__attribute__( ( always_inline ) ) static inline uint32_t __REV_THUMB2(uint32_t u32)
{
    return __builtin_bswap32(u32);
}

#define CPU_REV(x) __REV_THUMB2(x)

__attribute__( ( always_inline, noreturn ) ) static inline void CPU_JMP(uint32_t addr)
{
    addr |= (1ul << 0);
    __asm volatile (
        "\n\t"
        "BX  %0 \n\t"
        "NOP \n\t"
        :   /* no outputs */
        :"r"(addr)
        :);
    while(true);
}


#else                       // Any other compiler

#error "FORCED BUILD ERROR: Unknown compiler"   

#endif

/*
Need to define NULL
*/
#ifndef NULL
    #ifdef __CPLUSPLUS__
    #define NULL            0
    #else
    #define NULL            ((void *)0)
    #endif
#endif

#ifdef __cplusplus
}
#endif
/* Control register at offset 0x00 */
#define CCT_CTRL_ACTIVATE               (1ul << 0)
#define CCT_CTRL_FREE_RUN_EN            (1ul << 1)
#define CCT_CTRL_FREE_RUN_RESET         (1ul << 2)  /* self clearing bit */
#define CCT_CTRL_TCLK_MASK0             (0x07ul)
#define CCT_CTRL_TCLK_MASK              ((CCT_CTRL_TCLK_MASK0) << 4)
#define CCT_CTRL_TCLK_DIV_1             (0ul)
#define CCT_CTRL_TCLK_DIV_2             (1ul << 4)
#define CCT_CTRL_TCLK_DIV_4             (2ul << 4)
#define CCT_CTRL_TCLK_DIV_8             (3ul << 4)
#define CCT_CTRL_TCLK_DIV_16            (4ul << 4)
#define CCT_CTRL_TCLK_DIV_32            (5ul << 4)
#define CCT_CTRL_TCLK_DIV_64            (6ul << 4)
#define CCT_CTRL_TCLK_DIV_128           (7ul << 4)
#define CCT_CTRL_COMP0_EN               (1ul << 8)
#define CCT_CTRL_COMP1_EN               (1ul << 9)
#define CCT_CTRL_COMP1_SET              (1ul << 16) /* R/WS */
#define CCT_CTRL_COMP0_SET              (1ul << 17) /* R/WS */

#endif // #ifndef _PLATFORM_H
/**   @}
 */
