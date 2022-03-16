#include <PilotIncludes.h>

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

// Compilation on Windows only for static analysis purpose
#if (PILOT_TARGET == PT_HARDWARE) || (PILOT_TARGET == PT_WIN32)

/** \name Function wraps the RS485 interface of an interrupt handler for the NIOS platform.
* Real handler is a static function in the Rs485ADNios class which cannot be called by an address.
* Handler is called after the interruption has been requested by a device.
* \param 'context' - pointer to the Rs485ADNios object which handles specified instantion of a device
* \param 'id' - interruption identifier
* \note Handler MUST resets the reason of an interruption request in the other case the system lock occurs (continuous execution of a handler)
*/
void intHandlerRs485ADNios (void* context, u32 id) // parasoft-suppress  JSF-124 "It cannot be an inline - calling by an address"
{
    Rs485ADNios::intHandler(context, id);
}

#endif  // PILOT_TARGET
