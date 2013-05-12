/**HEADER********************************************************************
*
* Copyright (c) 2011 Freescale Semiconductor;
* All Rights Reserved
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName: bsp_cm.h$
* $Version : 3.8.4.0$
* $Date    : Oct-5-2011$
*
* Comments:
*   Clock manager Kinetis specific definitions and prototypes.
*
*       _bsp_initialize_hardware();
*       _bsp_set_clock_configuration();
*       _bsp_get_clock_configuration();
*       _bsp_get_clock();
*       _bsp_osc_autotrim();
*
*END************************************************************************/

#ifndef __bsp_cm_h__
    #define __bsp_cm_h__


#ifndef PE_LDD_VERSION

#define CPU_CLOCK_CONFIG_NUMBER         0x01U       /* Specifies number of defined clock configurations. */

#define CPU_XTAL_CLK_HZ                 50000000UL  /* Value of the external crystal or oscillator clock frequency in Hz */
#define CPU_XTAL32k_CLK_HZ              32768UL     /* Value of the external 32k crystal or oscillator clock frequency in Hz */
#define CPU_INT_SLOW_CLK_HZ             32768UL     /* Value of the slow internal oscillator clock frequency in Hz  */
#define CPU_INT_FAST_CLK_HZ             4000000UL   /* Value of the fast internal oscillator clock frequency in Hz  */


/* CPU frequencies in clock configuration 0 */
#define CPU_CLOCK_CONFIG_0              0x00U           /* Clock configuration 0 identifier */
#define CPU_CORE_CLK_HZ_CONFIG_0        120000000UL     /* Core clock frequency in clock configuration 0 */
#define CPU_BUS_CLK_HZ_CONFIG_0         60000000UL      /* Bus clock frequency in clock configuration 0 */
#define CPU_FLEXBUS_CLK_HZ_CONFIG_0     60000000UL      /* Flexbus clock frequency in clock configuration 0 */
#define CPU_FLASH_CLK_HZ_CONFIG_0       20000000UL      /* FLASH clock frequency in clock configuration 0 */
#define CPU_USB_CLK_HZ_CONFIG_0         48000000UL      /* USB clock frequency in clock configuration 0 */
#define CPU_PLL_FLL_CLK_HZ_CONFIG_0     120000000UL     /* PLL/FLL clock frequency in clock configuration 0 */
#define CPU_MCGIR_CLK_HZ_CONFIG_0       0               /* MCG internal reference clock frequency in clock configuration 0 */
#define CPU_OSCER_CLK_HZ_CONFIG_0       50000000UL      /* System OSC external reference clock frequency in clock configuration 0 */
#define CPU_ERCLK32K_CLK_HZ_CONFIG_0    32728UL         /* External reference clock 32k frequency in clock configuration 0 */
#define CPU_MCGFF_CLK_HZ_CONFIG_0       24414UL         /* MCG fixed frequency clock */
// TODO add NFC and USB HS clocks

#endif /* PE_LDD_VERSION */


typedef enum {
    BSP_CLOCK_CONFIGURATION_0 = 0,      /* 120 MHZ PEE   */
     BSP_CLOCK_CONFIGURATIONS,           /* Number of available clock configurations */
    BSP_CLOCK_CONFIGURATION_DEFAULT     = BSP_CLOCK_CONFIGURATION_0,
    BSP_CLOCK_CONFIGURATION_120MHZ      = BSP_CLOCK_CONFIGURATION_0
} BSP_CLOCK_CONFIGURATION;

#ifdef __cplusplus
extern "C" {
#endif


void _bsp_initialize_hardware
(
    void
);

BSP_CLOCK_CONFIGURATION _bsp_get_clock_configuration
(
    void
);

uint16_t _bsp_set_clock_configuration
(
    /* [IN] runtime clock configuration */
    const BSP_CLOCK_CONFIGURATION clock_configuration
);


uint32_t _bsp_get_clock
(
    /* [IN] clock configuration */
    const BSP_CLOCK_CONFIGURATION   clock_configuration,
    /* [IN] clock source index */
    const CM_CLOCK_SOURCE           clock_source
);

uint16_t _bsp_osc_autotrim
(
    void
);

#ifdef __cplusplus
}
#endif


#endif /* __bsp_cm_h__ */
