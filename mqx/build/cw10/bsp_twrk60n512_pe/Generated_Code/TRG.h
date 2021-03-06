/** ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : TRG.h
**     Project   : ProcessorExpert
**     Processor : MK60DN512ZVMD10
**     Component : TimerUnit_LDD
**     Version   : Component 01.122, Driver 01.05, CPU db: 3.00.003
**     Compiler  : CodeWarrior ARM C Compiler
**     Date/Time : 2011-11-24, 16:34, # CodeGen: 1
**     Abstract  :
**          This TimerUnit component provides a low level API for unified hardware access across
**          various timer devices using the Prescaler-Counter-Compare-Capture timer structure.
**     Settings  :
**          Component name                                 : TRG
**          Module name                                    : FTM2
**          Counter                                        : FTM2_CNT
**          Counter direction                              : Up
**          Counter width                                  : 16 bits
**          Value type                                     : uint16_t
**          Input clock source                             : Internal
**            Counter frequency                            : 48 MHz
**          Counter restart                                : On-match
**            Period device                                : FTM2_MOD
**            Period                                       : 64000 Hz
**            Interrupt                                    : Enabled
**              Interrupt                                  : INT_FTM2
**              Interrupt priority                         : medium priority
**          Channel list                                   : 0
**          Initialization                                 : 
**            Enabled in init. code                        : no
**            Auto initialization                          : no
**            Event mask                                   : 
**              OnCounterRestart                           : Enabled
**              OnChannel0                                 : Disabled
**              OnChannel1                                 : Disabled
**              OnChannel2                                 : Disabled
**              OnChannel3                                 : Disabled
**              OnChannel4                                 : Disabled
**              OnChannel5                                 : Disabled
**              OnChannel6                                 : Disabled
**              OnChannel7                                 : Disabled
**          CPU clock/configuration selection              : 
**            Clock configuration 0                        : This component enabled
**            Clock configuration 1                        : This component disabled
**            Clock configuration 2                        : This component disabled
**     Contents  :
**         Init                  - LDD_TDeviceData* TRG_Init(LDD_TUserData *UserDataPtr);
**         Deinit                - void TRG_Deinit(LDD_TDeviceData *DeviceDataPtr);
**         Enable                - LDD_TError TRG_Enable(LDD_TDeviceData *DeviceDataPtr);
**         Disable               - LDD_TError TRG_Disable(LDD_TDeviceData *DeviceDataPtr);
**         SetEventMask          - LDD_TError TRG_SetEventMask(LDD_TDeviceData *DeviceDataPtr, LDD_TEventMask...
**         GetEventMask          - LDD_TEventMask TRG_GetEventMask(LDD_TDeviceData *DeviceDataPtr);
**         GetEventStatus        - LDD_TEventMask TRG_GetEventStatus(LDD_TDeviceData *DeviceDataPtr);
**         GetInputFrequencyReal - LDD_TimerUnit_Tfloat TRG_GetInputFrequencyReal(LDD_TDeviceData *DeviceDataPtr);
**         GetInputFrequency     - uint32_t TRG_GetInputFrequency(LDD_TDeviceData *DeviceDataPtr);
**         SetPeriodTicks        - LDD_TError TRG_SetPeriodTicks(LDD_TDeviceData *DeviceDataPtr, TRG_TValueType...
**         GetPeriodTicks        - LDD_TError TRG_GetPeriodTicks(LDD_TDeviceData *DeviceDataPtr, TRG_TValueType...
**         ResetCounter          - LDD_TError TRG_ResetCounter(LDD_TDeviceData *DeviceDataPtr);
**         GetCounterValue       - TRG_TValueType TRG_GetCounterValue(LDD_TDeviceData *DeviceDataPtr);
**         SetOperationMode      - LDD_TError TRG_SetOperationMode(LDD_TDeviceData *DeviceDataPtr,...
**         GetDriverState        - LDD_TDriverState TRG_GetDriverState(LDD_TDeviceData *DeviceDataPtr);
**
**     Copyright : 1997 - 2011 Freescale Semiconductor, Inc. All Rights Reserved.
**     
**     http      : www.freescale.com
**     mail      : support@freescale.com
** ###################################################################*/

#ifndef __TRG_H
#define __TRG_H

/* MODULE TRG. */

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* Include inherited beans */

#include "PE_LDD.h"
#include "FTM_PDD.h"
#include "Cpu.h"


#ifndef __BWUserType_TRG_TValueType
#define __BWUserType_TRG_TValueType
  typedef uint16_t TRG_TValueType ;    /* Type for data parameters of methods */
#endif
#define TRG_CNT_INP_FREQ_U_0 0x02DC6C00UL /* Counter input frequency in Hz */
#define TRG_CNT_INP_FREQ_U_0_CFG_0 0x02DC6C00UL /* Counter input frequency in Hz for Clock configuration 0 */
#define TRG_CNT_INP_FREQ_R_0 48000768.0122881966F /* Counter input frequency in Hz */
#define TRG_CNT_INP_FREQ_R_0_CFG_0 48000768.0122881966F /* Counter input frequency in Hz for Clock configuration 0 */
#define TRG_CNT_INP_FREQ_COUNT 0U      /* Count of predefined counter input frequencies */
#define TRG_PERIOD_TICKS   0x02EEUL    /* Initialization value of period in 'counter ticks' */
#define TRG_NUMBER_OF_CHANNELS 0x00U   /* Count of predefined channels */
#define TRG_COUNTER_WIDTH  0x10U       /* Counter width in bits  */
#define TRG_COUNTER_DIR    DIR_UP      /* Direction of counting */
/* Methods configuration constants - generated for all enabled component's methods */
#define TRG_Init_METHOD_ENABLED
#define TRG_Deinit_METHOD_ENABLED
#define TRG_Enable_METHOD_ENABLED
#define TRG_Disable_METHOD_ENABLED
#define TRG_SetEventMask_METHOD_ENABLED
#define TRG_GetEventMask_METHOD_ENABLED
#define TRG_GetEventStatus_METHOD_ENABLED
#define TRG_GetInputFrequencyReal_METHOD_ENABLED
#define TRG_GetInputFrequency_METHOD_ENABLED
#define TRG_SetPeriodTicks_METHOD_ENABLED
#define TRG_GetPeriodTicks_METHOD_ENABLED
#define TRG_ResetCounter_METHOD_ENABLED
#define TRG_GetCounterValue_METHOD_ENABLED
#define TRG_SetOperationMode_METHOD_ENABLED
#define TRG_GetDriverState_METHOD_ENABLED

/* Events configuration constants - generated for all enabled component's events */
#define TRG_OnCounterRestart_EVENT_ENABLED



LDD_TDeviceData* TRG_Init(LDD_TUserData *UserDataPtr);
/*
** ===================================================================
**     Method      :  TRG_Init (component TimerUnit_LDD)
**
**     Description :
**         Initializes the device. Allocates memory for the device data
**         structure, allocates interrupt vectors and sets interrupt
**         priority, sets pin routing, sets timing, etc. If the
**         property <"Enable in init. code"> is set to "yes" value then
**         the device is also enabled (see the description of the
**         <Enable> method). In this case the <Enable> method is not
**         necessary and needn't to be generated. This method can be
**         called only once. Before the second call of Init the <Deinit>
**         must be called first.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer will be
**                           passed as an event or callback parameter.
**     Returns     :
**         ---             - Pointer to the dynamically allocated
**                           private structure or NULL if there was an
**                           error.
** ===================================================================
*/

void TRG_Deinit(LDD_TDeviceData *DeviceDataPtr);
/*
** ===================================================================
**     Method      :  TRG_Deinit (component TimerUnit_LDD)
**
**     Description :
**         Deinitializes the device. Switches off the device, frees the
**         device data structure memory, interrupts vectors, etc.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by Init method
**     Returns     : Nothing
** ===================================================================
*/

LDD_TError TRG_Enable(LDD_TDeviceData *DeviceDataPtr);
/*
** ===================================================================
**     Method      :  TRG_Enable (component TimerUnit_LDD)
**
**     Description :
**         Enables the component - it starts the signal generation.
**         Events may be generated (see SetEventMask). The method is
**         not available if the counter can't be disabled/enabled by HW.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
** ===================================================================
*/

LDD_TError TRG_Disable(LDD_TDeviceData *DeviceDataPtr);
/*
** ===================================================================
**     Method      :  TRG_Disable (component TimerUnit_LDD)
**
**     Description :
**         Disables the component - it stops signal generation and
**         events calling. The method is not available if the counter
**         can't be disabled/enabled by HW.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
** ===================================================================
*/

LDD_TError TRG_SetEventMask(LDD_TDeviceData *DeviceDataPtr, LDD_TEventMask EventMask);
/*
** ===================================================================
**     Method      :  TRG_SetEventMask (component TimerUnit_LDD)
**
**     Description :
**         Enables/disables event(s). The events contained within the
**         mask are enabled. Events not contained within the mask are
**         disabled. The component event masks are defined in the
**         PE_LDD.h file. Note: Event that are not generated (See the
**         "Events" tab in the Component inspector) are not handled by
**         this method. In this case the method returns ERR_PARAM_MASK
**         error code. See also method <GetEventMask>.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**         EventMask       - Event mask
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
**                           ERR_PARAM_MASK - Event mask is not valid
** ===================================================================
*/

LDD_TEventMask TRG_GetEventMask(LDD_TDeviceData *DeviceDataPtr);
/*
** ===================================================================
**     Method      :  TRG_GetEventMask (component TimerUnit_LDD)
**
**     Description :
**         Returns current events mask. Note: Event that are not
**         generated (See the "Events" tab in the Component inspector)
**         are not handled by this method. See also method
**         <SetEventMask>.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by Init method.
**     Returns     :
**         ---             - Current EventMask.
**                           The component event masks are defined in
**                           the PE_LDD.h file.
** ===================================================================
*/

LDD_TEventMask TRG_GetEventStatus(LDD_TDeviceData *DeviceDataPtr);
/*
** ===================================================================
**     Method      :  TRG_GetEventStatus (component TimerUnit_LDD)
**
**     Description :
**         Returns current pending flags and clears them.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by Init method.
**     Returns     :
**         ---             - Current status flags
** ===================================================================
*/

LDD_TimerUnit_Tfloat TRG_GetInputFrequencyReal(LDD_TDeviceData *DeviceDataPtr);
/*
** ===================================================================
**     Method      :  TRG_GetInputFrequencyReal (component TimerUnit_LDD)
**
**     Description :
**         Returns current input frequency of the counter in Hz as
**         float number. This method can be used only if <"Input clock
**         source"> property is set to "internal".
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by Init method.
**     Returns     :
**         ---             - Input frequency
** ===================================================================
*/

uint32_t TRG_GetInputFrequency(LDD_TDeviceData *DeviceDataPtr);
/*
** ===================================================================
**     Method      :  TRG_GetInputFrequency (component TimerUnit_LDD)
**
**     Description :
**         Returns current input frequency of the counter in Hz as
**         32-bit unsigned integer number. This method can be used only
**         if <"Input clock source"> property is set to "internal".
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by Init method.
**     Returns     :
**         ---             - Input frequency
** ===================================================================
*/

LDD_TError TRG_SetPeriodTicks(LDD_TDeviceData *DeviceDataPtr, TRG_TValueType Ticks);
/*
** ===================================================================
**     Method      :  TRG_SetPeriodTicks (component TimerUnit_LDD)
**
**     Description :
**         The method sets timer re-initialization period (in timer
**         ticks). This method is available only if the property
**         <"Counter restart"> is switched to 'on-match' value.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**         Ticks           - Number of counter ticks before counter
**                           re-initialization. Value 0 means maximal
**                           period value the same as "free-running
**                           mode", e.g. counter overflow or underflow
**                           without any explicit re-initialization.
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK 
**                           ERR_PARAM_TICKS - Ticks parameter is out of
**                           possible range.
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
** ===================================================================
*/

LDD_TError TRG_GetPeriodTicks(LDD_TDeviceData *DeviceDataPtr, TRG_TValueType *TicksPtr);
/*
** ===================================================================
**     Method      :  TRG_GetPeriodTicks (component TimerUnit_LDD)
**
**     Description :
**         Returns the number of counter ticks before re-initialization.
**         See also method <SetPeriodTicks>. This function is available
**         only if the property <"Counter restart"> is switched to
**         'on-match' value.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**       * TicksPtr        - Pointer to return value of the
**                           number of counter ticks before
**                           re-initialization
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK 
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
** ===================================================================
*/

LDD_TError TRG_ResetCounter(LDD_TDeviceData *DeviceDataPtr);
/*
** ===================================================================
**     Method      :  TRG_ResetCounter (component TimerUnit_LDD)
**
**     Description :
**         Resets counter. If counter is counting up then it is set to
**         zero. If counter is counting down then counter is updated to
**         the reload value.
**         The method is not available if HW doesn't allow resetting of
**         the counter.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK 
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
** ===================================================================
*/

TRG_TValueType TRG_GetCounterValue(LDD_TDeviceData *DeviceDataPtr);
/*
** ===================================================================
**     Method      :  TRG_GetCounterValue (component TimerUnit_LDD)
**
**     Description :
**         Returns the content of counter register. This method can be
**         used both if counter is enabled and if counter is disabled.
**         The method is not available if HW doesn't allow reading of
**         the counter.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by Init method.
**     Returns     :
**         ---             - Counter value (number of counted ticks).
** ===================================================================
*/

void TRG_SetClockConfiguration(LDD_TDeviceData *DeviceDataPtr, LDD_TClockConfiguration ClockConfiguration);
/*
** ===================================================================
**     Method      :  TRG_SetClockConfiguration (component TimerUnit_LDD)
**
**     Description :
**         This method changes the clock configuration. During a clock 
**         configuration change the component changes it's setting 
**         immediately upon a request.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

LDD_TError TRG_SetOperationMode(LDD_TDeviceData *DeviceDataPtr, LDD_TDriverOperationMode OperationMode, LDD_TCallback ModeChangeCallback, LDD_TCallbackParam *ModeChangeCallbackParamPtr);
/*
** ===================================================================
**     Method      :  TRG_SetOperationMode (component TimerUnit_LDD)
**
**     Description :
**         This method requests to change the component's operation
**         mode. Upon a request to change the operation mode, the
**         component will finish a pending job first and then notify a
**         caller that an operation mode has been changed. When no job
**         is pending (ERR_OK), the component changes an operation mode
**         immediately and notify a caller about this change. 
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**         OperationMode   - Requested driver
**                           operation mode.
**         ModeChangeCallback - Callback to
**                           notify the upper layer once a mode has been
**                           changed.
**       * ModeChangeCallbackParamPtr -
**                           Pointer to callback parameter to
**                           notify the upper layer once a mode has been
**                           changed.
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration.
**                           ERR_DISABLED - The component is disabled by
**                           user.
**                           ERR_PARAM_MODE - Invalid operation mode.
**                           ERR_BUSY - A job is pending.
** ===================================================================
*/

LDD_TDriverState TRG_GetDriverState(LDD_TDeviceData *DeviceDataPtr);
/*
** ===================================================================
**     Method      :  TRG_GetDriverState (component TimerUnit_LDD)
**
**     Description :
**         This method returns the current driver status.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**     Returns     :
**         ---             - The current driver status mask.
**                           Following status masks defined in PE_LDD.h
**                           can be used to check the current driver
**                           status.
**                           PE_LDD_DRIVER_DISABLED_IN_CLOCK_CONFIGURATION - 1 -
**                           Driver is disabled in the current mode; 0 -
**                           Driver is enabled in the current mode.  
**                           PE_LDD_DRIVER_DISABLED_BY_USER - 1 - Driver
**                           is disabled by the user; 0 - Driver is
**                           enabled by the user.        
**                           PE_LDD_DRIVER_BUSY - 1 - Driver is the BUSY
**                           state; 0 - Driver is in the IDLE state.
** ===================================================================
*/

/* {MQX RTOS Adapter} ISR function prototype */
void TRG_Interrupt(LDD_RTOS_TISRParameter _isrParameter);
/*
** ===================================================================
**     Method      :  TRG_Interrupt (component TimerUnit_LDD)
**
**     Description :
**         The method services the interrupt of the selected peripheral(s)
**         and eventually invokes event(s) of the component.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

/* END TRG. */

#endif
/* ifndef __TRG_H */
/*
** ###################################################################
**
**     This file was created by Processor Expert 5.2 [04.49]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
