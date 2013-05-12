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
* $FileName: bsp_cm.c$
* $Version : 3.8.6.1$
* $Date    : Nov-10-2011$
*
* Comments:
*   Clock manager Kinetis BSP specific definitions and function prototypes.
*
*       _bsp_initialize_hardware();
*       _bsp_set_clock_configuration();
*       _bsp_get_clock_configuration();
*       _bsp_get_clock();
*       _bsp_osc_autotrim();
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <bsp_prv.h>

/* local function prototypes */

#ifdef PE_LDD_VERSION

extern const TCpuClockConfiguration PE_CpuClockConfigurations[CPU_CLOCK_CONFIG_NUMBER];

#else /* PE_LDD_VERSION */

/*
** ===========================================================================
** The array of clock frequencies in configured clock configurations.
** ===========================================================================
*/
static const TCpuClockConfiguration PE_CpuClockConfigurations[CPU_CLOCK_CONFIG_NUMBER] = {
  /* Clock configuration 0 */
  {
    CPU_CORE_CLK_HZ_CONFIG_0,          /* Core clock frequency in clock configuration 0 */
    CPU_BUS_CLK_HZ_CONFIG_0,           /* Bus clock frequency in clock configuration 0 */
    CPU_FLEXBUS_CLK_HZ_CONFIG_0,       /* Flexbus clock frequency in clock configuration 0 */
    CPU_FLASH_CLK_HZ_CONFIG_0,         /* FLASH clock frequency in clock configuration 0 */
    CPU_USB_CLK_HZ_CONFIG_0,           /* USB clock frequency in clock configuration 0 */
    CPU_PLL_FLL_CLK_HZ_CONFIG_0,       /* PLL/FLL clock frequency in clock configuration 0 */
    CPU_MCGIR_CLK_HZ_CONFIG_0,         /* MCG internal reference clock frequency in clock configuration 0 */
    CPU_OSCER_CLK_HZ_CONFIG_0,         /* System OSC external reference clock frequency in clock configuration 0 */
    CPU_ERCLK32K_CLK_HZ_CONFIG_0,      /* External reference clock 32k frequency in clock configuration 0 */
    CPU_MCGFF_CLK_HZ_CONFIG_0          /* MCG fixed frequency clock */
  },
};

/* Global variables */
static BSP_CLOCK_CONFIGURATION ClockConfigurationID = BSP_CLOCK_CONFIGURATION_DEFAULT; /* Active clock configuration */


int pll_init(unsigned char init_osc, unsigned char osc_select, int crystal_val, unsigned char hgo_val, unsigned char erefs_val, unsigned char pll_select, signed char prdiv_val, signed char vdiv_val, unsigned char mcgout_select);

// Constants for use in pll_init
#define BSP_CM_NO_OSCINIT       0
#define BSP_CM_OSCINIT          1

#define BSP_CM_OSC_0            0
#define BSP_CM_OSC_1            1

#define BSP_CM_LOW_POWER        0
#define BSP_CM_HIGH_GAIN        1

#define BSP_CM_CANNED_OSC       0
#define BSP_CM_CRYSTAL          1

#define BSP_CM_PLL_0            0
#define BSP_CM_PLL_1            1

#define BSP_CM_PLL_ONLY         0
#define BSP_CM_MCGOUT           1


/*
* Input Clock Info
*/
#define BSP_CM_PLL0_PRDIV        5
#define BSP_CM_PLL0_VDIV         24

#define BSP_CM_PLL1_PRDIV        5
#define BSP_CM_PLL1_VDIV         30


void _bsp_initialize_hardware(void)
{
    uint_32 mcg_clk, pll_1_clk;

    _bsp_watchdog_disable();

    /* Set the system dividers */
    /* NOTE: The PLL init will not configure the system clock dividers,
    * so they must be configured appropriately before calling the PLL
    * init function to ensure that clocks remain in valid ranges.
    */
    SIM_CLKDIV1 = ( 0
                    | SIM_CLKDIV1_OUTDIV1(0)
                    | SIM_CLKDIV1_OUTDIV2(1)
                    | SIM_CLKDIV1_OUTDIV3(1)
                    | SIM_CLKDIV1_OUTDIV4(5) );

    SIM_CLKDIV4 |=  SIM_CLKDIV4_NFCDIV(4);

    /* Initialize PLL0 */
    /* PLL0 will be the source for MCG CLKOUT so the core, system, FlexBus, and flash clocks are derived from it */
    mcg_clk = pll_init(BSP_CM_OSCINIT,   /* Initialize the oscillator circuit */
                       BSP_CM_OSC_0,     /* Use CLKIN0 as the input clock */
                       CPU_OSCER_CLK_HZ_CONFIG_0,  /* CLKIN0 frequency */
                       BSP_CM_LOW_POWER,     /* Set the oscillator for low power mode */
                       BSP_CM_CANNED_OSC,     /* Crystal or canned oscillator clock input */
                       BSP_CM_PLL_0,         /* PLL to initialize, in this case PLL0 */
                       BSP_CM_PLL0_PRDIV,    /* PLL predivider value */
                       BSP_CM_PLL0_VDIV,     /* PLL multiplier */
                       BSP_CM_MCGOUT);       /* Use the output from this PLL as the MCGOUT */

    /* Initialize PLL1 */
    /* PLL1 will be the source for the DDR controller, but NOT the MCGOUT */
    pll_1_clk = pll_init(BSP_CM_NO_OSCINIT, /* Don't init the osc circuit, already done */
                         BSP_CM_OSC_0,      /* Use CLKIN0 as the input clock */
                         CPU_OSCER_CLK_HZ_CONFIG_0,  /* CLKIN0 frequency */
                         BSP_CM_LOW_POWER,     /* Set the oscillator for low power mode */
                         BSP_CM_CANNED_OSC,     /* Crystal or canned oscillator clock input */
                         BSP_CM_PLL_1,         /* PLL to initialize, in this case PLL1 */
                         BSP_CM_PLL1_PRDIV,    /* PLL predivider value */
                         BSP_CM_PLL1_VDIV,     /* PLL multiplier */
                         BSP_CM_PLL_ONLY);   /* Don't use the output from this PLL as the MCGOUT */
}

#endif /* PE_LDD_VERSION */


uint16_t _bsp_set_clock_configuration
(
    /* [IN] runtime clock configuration */
    const BSP_CLOCK_CONFIGURATION clock_configuration
)
{
    /* Only one clock configuration is currently implemented for TWR-MK70 BSP, do nothing */
    return MQX_OK;
}


BSP_CLOCK_CONFIGURATION _bsp_get_clock_configuration
(
    void
)
{
    return ClockConfigurationID;
}

uint32_t _bsp_get_clock
(
    const BSP_CLOCK_CONFIGURATION   clock_configuration,
    const CM_CLOCK_SOURCE           clock_source
)
{
    if (    (clock_configuration < CPU_CLOCK_CONFIG_NUMBER)
         && (clock_source        < sizeof(PE_CpuClockConfigurations)/sizeof(uint32_t))
        )
    {
        return *(((uint32_t *)&(PE_CpuClockConfigurations[clock_configuration])) + clock_source);
    }
    else
    {
        return 0;
    }
}


/*********************************************************************************************/
/* Functon name : pll_init
 *
 * Mode transition: Option to move from FEI to PEE mode or to just initialize the PLL
 *
 * This function initializess either PLL0 or PLL1. Either OSC0 or OSC1 can be selected for the
 * reference clock source. The oscillators can be configured to use a crystal or take in an
 * external square wave clock.
 * The PLL outputs a PLLCLK and PLLCLK2X. PLLCLK2X is the actual PLL frequency and PLLCLK is
 * half this frequency. PLLCLK is used for MCGOUT and is also typically used by the
 * peripherals that can select the PLL as a clock source. So the PLL frequency generated will
 * be twice the desired frequency.
 * Using the function parameter names the PLL frequency is calculated as follows:
 * PLL freq = ((crystal_val / prdiv_val) * vdiv_val)
 * Refer to the readme file in the mcg driver directory for examples of pll_init configurations.
 * All parameters must be provided, for example crystal_val must be provided even if the
 * oscillator associated with that parameter is already initialized.
 * The various passed parameters are checked to ensure they are within the allowed range. If any
 * of these checks fail the driver will exit and return a fail/error code. An error code will
 * also be returned if any error occurs during the PLL initialization sequence. Refer to the
 * readme file in the mcg driver directory for a list of all these codes.
 *
 * Parameters: init_osc    - 0 if oscillator does not need to be initialized, non-zero if the
 *                           oscillator needs to be configured.
 *             osc_select  - 0 to select OSC0, non-zero to select OSC1.
 *             crystal_val - external clock frequency in Hz either from a crystal or square
 *                           wave clock source
 *             hgo_val     - selects whether low power or high gain mode is selected
 *                           for the crystal oscillator. This has no meaning if an
 *                           external clock is used.
 *             erefs_val   - selects external clock (=0) or crystal osc (=1)
 *             pll_select  - 0 to select PLL0, non-zero to select PLL1.
 *             prdiv_val   - value to divide the external clock source by to create the desired
 *                           PLL reference clock frequency
 *             vdiv_val    - value to multiply the PLL reference clock frequency by
 *             mcgout_select  - 0 if the PLL is just to be enabled, non-zero if the PLL is used
 *                              to provide the MCGOUT clock for the system.
 *
 * Return value : PLL frequency (Hz) divided by 2 or error code
 */
static int pll_init(unsigned char init_osc, unsigned char osc_select, int crystal_val, unsigned char hgo_val, unsigned char erefs_val, unsigned char pll_select, signed char prdiv_val, signed char vdiv_val, unsigned char mcgout_select)
{
  unsigned char frdiv_val;
  unsigned char temp_reg;
  unsigned char prdiv, vdiv;
  short i;
  int ref_freq;
  int pll_freq;

  // If using the PLL as MCG_OUT must check if the MCG is in FEI mode first
  if (mcgout_select)
  {
    // check if in FEI mode
    if (!((((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x0) && // check CLKS mux has selcted FLL output
        (MCG_S & MCG_S_IREFST_MASK) &&                                  // check FLL ref is internal ref clk
        (!(MCG_S & MCG_S_PLLST_MASK))))                                 // check PLLS mux has selected FLL
    {
      return 0x1;                                                     // return error code
    }
  } // if (mcgout_select)

  // Check if OSC1 is being used as a reference for the MCGOUT PLL
  // This requires a more complicated MCG configuration.
  // At this time (Sept 8th 2011) this driver does not support this option
  if (osc_select && mcgout_select)
  {
    return 0x80; // Driver does not support using OSC1 as the PLL reference for the system clock on MCGOUT
  }

  // check external frequency is less than the maximum frequency
  if  (crystal_val > 60000000) {return 0x21;}

  // check crystal frequency is within spec. if crystal osc is being used as PLL ref
  if (erefs_val)
  {
    if ((crystal_val < 8000000) || (crystal_val > 32000000)) {return 0x22;} // return 1 if one of the available crystal options is not available
  }

  // make sure HGO will never be greater than 1. Could return an error instead if desired.
  if (hgo_val > 0)
  {
    hgo_val = 1; // force hgo_val to 1 if > 0
  }

  // Check PLL divider settings are within spec.
  if ((prdiv_val < 1) || (prdiv_val > 8)) {return 0x41;}
  if ((vdiv_val < 16) || (vdiv_val > 47)) {return 0x42;}

  // Check PLL reference clock frequency is within spec.
  ref_freq = crystal_val / prdiv_val;
  if ((ref_freq < 8000000) || (ref_freq > 32000000)) {return 0x43;}

  // Check PLL output frequency is within spec.
  pll_freq = (crystal_val / prdiv_val) * vdiv_val;
  if ((pll_freq < 180000000) || (pll_freq > 360000000)) {return 0x45;}

  // Determine if oscillator needs to be set up
  if (init_osc)
  {
    // Check if the oscillator needs to be configured
    if (!osc_select)
    {
      // configure the MCG_C2 register
      // the RANGE value is determined by the external frequency. Since the RANGE parameter affects the FRDIV divide value
      // it still needs to be set correctly even if the oscillator is not being used
      temp_reg = MCG_C2;
      temp_reg &= ~(MCG_C2_RANGE0_MASK | MCG_C2_HGO0_MASK | MCG_C2_EREFS0_MASK); // clear fields before writing new values
      if (crystal_val <= 8000000)
      {
        temp_reg |= (MCG_C2_RANGE0(1) | (hgo_val << MCG_C2_HGO0_SHIFT) | (erefs_val << MCG_C2_EREFS0_SHIFT));
      }
      else
      {
        // For this crystal frequency RANGE should be 2 but to assist with debugging, due to an issue on rev. 1.0 silicon
        // a value of 1 is specified here. If running from flash and the debugger is not connected, the correct value of
        // 2 can be used here.
        temp_reg |= (MCG_C2_RANGE0(1) | (hgo_val << MCG_C2_HGO0_SHIFT) | (erefs_val << MCG_C2_EREFS0_SHIFT));
      }
      MCG_C2 = temp_reg;
    }
    else
    {
      // configure the MCG_C10 register
      // the RANGE value is determined by the external frequency. Since the RANGE parameter affects the FRDIV divide value
      // it still needs to be set correctly even if the oscillator is not being used
      temp_reg = MCG_C10;
      temp_reg &= ~(MCG_C10_RANGE1_MASK | MCG_C10_HGO1_MASK | MCG_C10_EREFS1_MASK); // clear fields before writing new values
      if (crystal_val <= 8000000)
      {
        temp_reg |= MCG_C10_RANGE1(1) | (hgo_val << MCG_C10_HGO1_SHIFT) | (erefs_val << MCG_C10_EREFS1_SHIFT);
      }
      else
      {
        // For this crystal frequency RANGE should be 2 but to assist with debugging, due to an issue on rev. 1.0 silicon
        // a value of 1 is specified here. If running from flash and the debugger is not connected, the correct value of
        // 2 can be used here.
        temp_reg |= MCG_C10_RANGE1(1) | (hgo_val << MCG_C10_HGO1_SHIFT) | (erefs_val << MCG_C10_EREFS1_SHIFT);
      }
      MCG_C10 = temp_reg;
    } // if (!osc_select)
  } // if (init_osc)

  if (mcgout_select)
  {
    // determine FRDIV based on reference clock frequency
    // since the external frequency has already been checked only the maximum frequency for each FRDIV value needs to be compared here.
    if (crystal_val <= 1250000) {frdiv_val = 0;}
    else if (crystal_val <= 2500000) {frdiv_val = 1;}
    else if (crystal_val <= 5000000) {frdiv_val = 2;}
    else if (crystal_val <= 10000000) {frdiv_val = 3;}
    else if (crystal_val <= 20000000) {frdiv_val = 4;}
    else {frdiv_val = 5;}

    // Select external oscillator and Reference Divider and clear IREFS to start ext osc
    // If IRCLK is required it must be enabled outside of this driver, existing state will be maintained
    // CLKS=2, FRDIV=frdiv_val, IREFS=0, IRCLKEN=0, IREFSTEN=0
    temp_reg = MCG_C1;
    temp_reg &= ~(MCG_C1_CLKS_MASK | MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK); // Clear values in these fields
    temp_reg = MCG_C1_CLKS(2) | MCG_C1_FRDIV(frdiv_val); // Set the required CLKS and FRDIV values
    MCG_C1 = temp_reg;

    // if the external oscillator is used need to wait for OSCINIT to set
    if (erefs_val)
    {
      for (i = 0 ; i < 10000 ; i++)
      {
        if (MCG_S & MCG_S_OSCINIT0_MASK) break; // jump out early if OSCINIT sets before loop finishes
      }
      if (!(MCG_S & MCG_S_OSCINIT0_MASK)) return 0x23; // check bit is really set and return with error if not set
    }

    // wait for Reference clock Status bit to clear
    for (i = 0 ; i < 2000 ; i++)
    {
      if (!(MCG_S & MCG_S_IREFST_MASK)) break; // jump out early if IREFST clears before loop finishes
    }
    if (MCG_S & MCG_S_IREFST_MASK) return 0x11; // check bit is really clear and return with error if not set

    // Wait for clock status bits to show clock source is ext ref clk
    for (i = 0 ; i < 2000 ; i++)
    {
      if (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x2) break; // jump out early if CLKST shows EXT CLK slected before loop finishes
    }
    if (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2) return 0x1A; // check EXT CLK is really selected and return with error if not

    // Now in FBE
    // It is recommended that the clock monitor is enabled when using an external clock as the clock source/reference.
    // It is enabled here but can be removed if this is not required.
    MCG_C6 |= MCG_C6_CME0_MASK;

    // Select which PLL to enable
    if (!pll_select)
    {
      // Configure PLL0
      // Ensure OSC0 is selected as the reference clock
      MCG_C5 &= ~MCG_C5_PLLREFSEL0_MASK;
      //Select PLL0 as the source of the PLLS mux
      MCG_C11 &= ~MCG_C11_PLLCS_MASK;
      // Configure MCG_C5
      // If the PLL is to run in STOP mode then the PLLSTEN bit needs to be OR'ed in here or in user code.
      temp_reg = MCG_C5;
      temp_reg &= ~MCG_C5_PRDIV0_MASK;
      temp_reg |= MCG_C5_PRDIV0(prdiv_val - 1);    //set PLL ref divider
      MCG_C5 = temp_reg;

      // Configure MCG_C6
      // The PLLS bit is set to enable the PLL, MCGOUT still sourced from ext ref clk
      // The loss of lock interrupt can be enabled by seperately OR'ing in the LOLIE bit in MCG_C6
      temp_reg = MCG_C6; // store present C6 value
      temp_reg &= ~MCG_C6_VDIV0_MASK; // clear VDIV settings
      temp_reg |= MCG_C6_PLLS_MASK | MCG_C6_VDIV0(vdiv_val - 16); // write new VDIV and enable PLL
      MCG_C6 = temp_reg; // update MCG_C6

      // wait for PLLST status bit to set
      for (i = 0 ; i < 2000 ; i++)
      {
        if (MCG_S & MCG_S_PLLST_MASK) break; // jump out early if PLLST sets before loop finishes
      }
      if (!(MCG_S & MCG_S_PLLST_MASK)) return 0x16; // check bit is really set and return with error if not set

      // Wait for LOCK bit to set
      for (i = 0 ; i < 2000 ; i++)
      {
        if (MCG_S & MCG_S_LOCK0_MASK) break; // jump out early if LOCK sets before loop finishes
      }
      if (!(MCG_S & MCG_S_LOCK0_MASK)) return 0x44; // check bit is really set and return with error if not set

      // Use actual PLL settings to calculate PLL frequency
      prdiv = ((MCG_C5 & MCG_C5_PRDIV0_MASK) + 1);
      vdiv = ((MCG_C6 & MCG_C6_VDIV0_MASK) + 16);
    }
    else
    {
      // Configure PLL1
      // Ensure OSC0 is selected as the reference clock
      MCG_C11 &= ~MCG_C11_PLLREFSEL1_MASK;
      //Select PLL1 as the source of the PLLS mux
      MCG_C11 |= MCG_C11_PLLCS_MASK;
      // Configure MCG_C11
      // If the PLL is to run in STOP mode then the PLLSTEN2 bit needs to be OR'ed in here or in user code.
      temp_reg = MCG_C11;
      temp_reg &= ~MCG_C11_PRDIV1_MASK;
      temp_reg |= MCG_C11_PRDIV1(prdiv_val - 1);    //set PLL ref divider
      MCG_C11 = temp_reg;

      // Configure MCG_C12
      // The PLLS bit is set to enable the PLL, MCGOUT still sourced from ext ref clk
      // The loss of lock interrupt can be enabled by seperately OR'ing in the LOLIE2 bit in MCG_C12
      temp_reg = MCG_C12; // store present C12 value
      temp_reg &= ~MCG_C12_VDIV1_MASK; // clear VDIV settings
      temp_reg |=  MCG_C12_VDIV1(vdiv_val - 16); // write new VDIV and enable PLL
      MCG_C12 = temp_reg; // update MCG_C12
      // Enable PLL by setting PLLS bit
      MCG_C6 |= MCG_C6_PLLS_MASK;

      // wait for PLLCST status bit to set
      for (i = 0 ; i < 2000 ; i++)
      {
        if (MCG_S2 & MCG_S2_PLLCST_MASK) break; // jump out early if PLLST sets before loop finishes
      }
      if (!(MCG_S2 & MCG_S2_PLLCST_MASK)) return 0x17; // check bit is really set and return with error if not set

      // wait for PLLST status bit to set
      for (i = 0 ; i < 2000 ; i++)
      {
        if (MCG_S & MCG_S_PLLST_MASK) break; // jump out early if PLLST sets before loop finishes
      }
      if (!(MCG_S & MCG_S_PLLST_MASK)) return 0x16; // check bit is really set and return with error if not set

      // Wait for LOCK bit to set
      for (i = 0 ; i < 2000 ; i++)
      {
        if (MCG_S2 & MCG_S2_LOCK1_MASK) break; // jump out early if LOCK sets before loop finishes
      }
      if (!(MCG_S2 & MCG_S2_LOCK1_MASK)) return 0x44; // check bit is really set and return with error if not set

      // Use actual PLL settings to calculate PLL frequency
      prdiv = ((MCG_C11 & MCG_C11_PRDIV1_MASK) + 1);
      vdiv = ((MCG_C12 & MCG_C12_VDIV1_MASK) + 16);
    } // if (!pll_select)

    // now in PBE

    MCG_C1 &= ~MCG_C1_CLKS_MASK; // clear CLKS to switch CLKS mux to select PLL as MCG_OUT

    // Wait for clock status bits to update
    for (i = 0 ; i < 2000 ; i++)
    {
      if (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x3) break; // jump out early if CLKST = 3 before loop finishes
    }
    if (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3) return 0x1B; // check CLKST is set correctly and return with error if not

    // Now in PEE
  }
  else
  {
    // Setup PLL for peripheral only use
    if (pll_select)
    {
      // Setup and enable PLL1
      // Select ref source
      if (osc_select)
      {
        MCG_C11 |= MCG_C11_PLLREFSEL1_MASK; // Set select bit to choose OSC1
      }
      else
      {
        MCG_C11 &= ~MCG_C11_PLLREFSEL1_MASK; // Clear select bit to choose OSC0
      }
      // Configure MCG_C11
      // If the PLL is to run in STOP mode then the PLLSTEN2 bit needs to be OR'ed in here or in user code.
      temp_reg = MCG_C11;
      temp_reg &= ~MCG_C11_PRDIV1_MASK;
      temp_reg |= MCG_C11_PRDIV1(prdiv_val - 1);    //set PLL ref divider
      MCG_C11 = temp_reg;

      // Configure MCG_C12
      // The loss of lock interrupt can be enabled by separately OR'ing in the LOLIE2 bit in MCG_C12
      temp_reg = MCG_C12; // store present C12 value
      temp_reg &= ~MCG_C12_VDIV1_MASK; // clear VDIV settings
      temp_reg |=  MCG_C12_VDIV1(vdiv_val - 16); // write new VDIV and enable PLL
      MCG_C12 = temp_reg; // update MCG_C12
      // Now enable the PLL
      MCG_C11 |= MCG_C11_PLLCLKEN1_MASK; // Set PLLCLKEN1 to enable PLL1

      // Wait for LOCK bit to set
      for (i = 0 ; i < 2000 ; i++)
      {
        if (MCG_S2 & MCG_S2_LOCK1_MASK) break; // jump out early if LOCK sets before loop finishes
      }
      if (!(MCG_S2 & MCG_S2_LOCK1_MASK)) return 0x44; // check bit is really set and return with error if not set

      // Use actual PLL settings to calculate PLL frequency
      prdiv = ((MCG_C11 & MCG_C11_PRDIV1_MASK) + 1);
      vdiv = ((MCG_C12 & MCG_C12_VDIV1_MASK) + 16);
    }
    else
    {
      // Setup and enable PLL0
      // Select ref source
      if (osc_select)
      {
        MCG_C5 |= MCG_C5_PLLREFSEL0_MASK; // Set select bit to choose OSC1
      }
      else
      {
        MCG_C5 &= ~MCG_C5_PLLREFSEL0_MASK; // Clear select bit to choose OSC0
      }
      // Configure MCG_C5
      // If the PLL is to run in STOP mode then the PLLSTEN bit needs to be OR'ed in here or in user code.
      temp_reg = MCG_C5;
      temp_reg &= ~MCG_C5_PRDIV0_MASK;
      temp_reg |= MCG_C5_PRDIV0(prdiv_val - 1);    //set PLL ref divider
      MCG_C5 = temp_reg;

      // Configure MCG_C6
      // The loss of lock interrupt can be enabled by seperately OR'ing in the LOLIE bit in MCG_C6
      temp_reg = MCG_C6; // store present C6 value
      temp_reg &= ~MCG_C6_VDIV0_MASK; // clear VDIV settings
      temp_reg |=  MCG_C6_VDIV0(vdiv_val - 16); // write new VDIV and enable PLL
      MCG_C6 = temp_reg; // update MCG_C6
      // Now enable the PLL
      MCG_C5 |= MCG_C5_PLLCLKEN0_MASK; // Set PLLCLKEN0 to enable PLL0

      // Wait for LOCK bit to set
      for (i = 0 ; i < 2000 ; i++)
      {
        if (MCG_S & MCG_S_LOCK0_MASK) break; // jump out early if LOCK sets before loop finishes
      }
      if (!(MCG_S & MCG_S_LOCK0_MASK)) return 0x44; // check bit is really set and return with error if not set

      // Use actual PLL settings to calculate PLL frequency
      prdiv = ((MCG_C5 & MCG_C5_PRDIV0_MASK) + 1);
      vdiv = ((MCG_C6 & MCG_C6_VDIV0_MASK) + 16);
    } // if (pll_select)

  } // if (mcgout_select)

  return (((crystal_val / prdiv) * vdiv) / 2); //MCGOUT equals PLL output frequency/2
} // pll_init

