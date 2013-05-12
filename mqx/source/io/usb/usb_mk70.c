/**HEADER********************************************************************
* 
* Copyright (c) 2008 Freescale Semiconductor;
* All Rights Reserved
*
* Copyright (c) 2004-2008 Embedded Access Inc.;
* All Rights Reserved
*
* Copyright (c) 1989-2008 ARC International;
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
* $FileName: usb_mk70.c$
* $Version : 3.8.1.0$
* $Date    : Oct-5-2011$
*
* Comments:
*
*   This file contains board-specific USB initialization functions.
*
*END************************************************************************/

#include "mqx.h"
#include "bsp.h"
#include "usb_bsp.h"

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_usb_dev_init
* Returned Value   : 0 for success, -1 for failure
* Comments         :
*    This function performs BSP-specific initialization related to USB
*
*END*----------------------------------------------------------------------*/
_mqx_int _bsp_usb_dev_init(pointer param)
{
    _mqx_uint dev_num = (_mqx_uint) param;
    _mqx_int result = _bsp_usb_io_init(dev_num);

    if (result != MQX_OK)
        return result;

    if (dev_num == USBCFG_CONTROLLER_KHCI) {
        /* Configure enable USB regulator for device */
        SIM_SOPT1CFG_REG(SIM_BASE_PTR) |= SIM_SOPT1CFG_URWE_MASK;
        SIM_SOPT1_REG(SIM_BASE_PTR) |= SIM_SOPT1_USBREGEN_MASK;
        /* Reset USB peripheral */
        USB_USBTRC0_REG(USB0_BASE_PTR) |= USB_USBTRC0_USBRESET_MASK;
        /* Wait while resetting */
        while (USB_USBTRC0_REG(USB0_BASE_PTR) & USB_USBTRC0_USBRESET_MASK)
        {}

        /* reset USB CTRL register */
        USB_USBCTRL_REG(USB0_BASE_PTR) = 0;

        /* Enable internal pull-up resistor */
        USB_CONTROL_REG(USB0_BASE_PTR) = USB_CONTROL_DPPULLUPNONOTG_MASK;
        USB_USBTRC0_REG(USB0_BASE_PTR) |= 0x40; /* Software must set this bit to 1 */
    
        /* setup interrupt */
        _bsp_int_init(INT_USB0, BSP_USB_INT_LEVEL, 0, TRUE);
    }   
    else if (dev_num == USBCFG_CONTROLLER_EHCI) {
    }
    else {
        /* unknown controller */
        result = IO_ERROR;
    }

    return MQX_OK;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_usb_host_init
* Returned Value   : 0 for success, -1 for failure
* Comments         :
*    This function performs BSP-specific initialization related to USB
*
*END*----------------------------------------------------------------------*/
_mqx_int _bsp_usb_host_init(pointer param)
{
    _mqx_uint dev_num = (_mqx_uint) param;
    _mqx_int result = _bsp_usb_io_init(dev_num);
    
    if (result != MQX_OK)
        return result;

    if (dev_num == USBCFG_CONTROLLER_KHCI) {
        /* Do not configure enable USB regulator for host */
//      SIM_SOPT1CFG_REG(SIM_BASE_PTR) |= SIM_SOPT1CFG_URWE_MASK;
//      SIM_SOPT1_REG(SIM_BASE_PTR) |= SIM_SOPT1_USBREGEN_MASK;

        /* Reset USB peripheral */
        USB_USBTRC0_REG(USB0_BASE_PTR) |= USB_USBTRC0_USBRESET_MASK;
        /* Wait while resetting */
        while (USB_USBTRC0_REG(USB0_BASE_PTR) & USB_USBTRC0_USBRESET_MASK)
        {}

        /* reset USB CTRL register */
        USB_USBCTRL_REG(USB0_BASE_PTR) = 0;

        /* setup interrupt */
        _bsp_int_init(INT_USB0, BSP_USB_INT_LEVEL, 0, TRUE);
    }
    else if (dev_num == USBCFG_CONTROLLER_EHCI) {
        
    }
    else {
        /* unknown controller */
        result = IO_ERROR;
    }

    return result;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_get_usb_base
* Returned Value   : Address upon success, NULL upon failure
* Comments         :
*    This function returns the address of USB OTG module
*
*END*----------------------------------------------------------------------*/
pointer _bsp_get_usb_base(uint_8 dev_num)
{
    if (dev_num == USBCFG_CONTROLLER_KHCI) {
        return (pointer)USB0_BASE_PTR;
    }
    else if (dev_num == USBCFG_CONTROLLER_EHCI) {
        return (pointer)USBHS_BASE_PTR;
    }
   
    return NULL;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_get_usb_vector
* Returned Value   : Vector Number upon success, 0 upon failure
* Comments         :
*    This function returns the vector number of the VUSBHS Host Module
*
*END*----------------------------------------------------------------------*/
uint_8 _bsp_get_usb_vector
(
    uint_8 dev_num
)
{
    if (dev_num == USBCFG_CONTROLLER_KHCI) {
        return INT_USB0;
    }
    else if (dev_num == USBCFG_CONTROLLER_EHCI) {
        return INT_USBHS;
    }
   
    return 0;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_get_usb_capability_register_base
* Returned Value   : Address upon success, NULL upon failure
* Comments         :
*    This function returns the address of the VUSBHS Capability Registers
*
*END*----------------------------------------------------------------------*/
pointer _bsp_get_usb_capability_register_base(uint_8 dev_num)
{
    if (dev_num == USBCFG_CONTROLLER_EHCI) {
        return (pointer) ((uchar *) USBHS_BASE_PTR + 0x100);
    }

    return NULL;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_get_usb_timer_register_base
* Returned Value   : Address upon success, NULL upon failure
* Comments         :
*    This function returns the address of the VUSBHS Timer Registers
*
*END*----------------------------------------------------------------------*/
pointer _bsp_get_usb_timer_register_base(uint_8 dev_num)
{
    if (dev_num == USBCFG_CONTROLLER_EHCI) {
        return (pointer) ((uchar *) USBHS_BASE_PTR + 0x80);
    }

    return NULL;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_get_usb_otg_csr
* Returned Value   : Address upon success, NULL upon failure
* Comments         :
*    This function returns the address of the U0CSR register
*
*END*----------------------------------------------------------------------*/
pointer _bsp_get_usb_otg_csr(uint_8 dev_num)
{
    return NULL;
}
