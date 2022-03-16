#ifndef RS485ADNIOSHANDLERWRAPPER_H
#define RS485ADNIOSHANDLERWRAPPER_H

#ifndef PILOT_TARGET
    #error "Define PILOT_TARGET in pilot_cfg.h"
#endif

// Compilation on Windows only for static analysis purpose
#if (PILOT_TARGET == PT_HARDWARE) || (PILOT_TARGET == PT_WIN32)

/** \name Function wraps the RS485 interface of an interrupt handler for the NIOS platform.
*/
void intHandlerRs485ADNios (void* context, u32 id);

#endif  // PILOT_TARGET
#endif  // RS485ADNIOSHANDLERWRAPPER_H
