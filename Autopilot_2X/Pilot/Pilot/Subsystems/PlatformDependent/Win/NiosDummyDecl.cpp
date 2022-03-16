// Declarations required to compile NIOS sources under Windows
// Included as "include" to all files

#include <PilotIncludes.h>

#if PILOT_TARGET == PT_WIN32
/************************************************************************************/
/*                                                                                  */
/*                                  Windows                                         */
/*                                                                                  */
/************************************************************************************/

/* parasoft off */
// Temporarily disabling static analysis
int alt_avalon_spi_command(alt_u32 base, alt_u32 slave,
                           alt_u32 write_length, const alt_u8 * write_data,
                           alt_u32 read_length, alt_u8 * read_data,
                           alt_u32 flags) {return 0;};

int alt_irq_register (alt_u32 id, 
                             void*   context, 
                             void (*irq_handler)(void*, alt_u32)) {return 0;};

int __builtin_ldbio (volatile const void *) {return 0;};
int __builtin_ldbuio (volatile const void *) {return 0;};
int __builtin_ldhio (volatile const void *) {return 0;};
int __builtin_ldhuio (volatile const void *) {return 0;};
int __builtin_ldwio (volatile const void *) {return 0;};
void __builtin_stbio (volatile void *, int) {};
void __builtin_sthio (volatile void *, int) {};
void __builtin_stwio (volatile void *, int) {};
void __builtin_sync (void) {};
int __builtin_rdctl (int) {return 0;};
void __builtin_wrctl (int, int) {};

/* parasoft on */
// Enable static analysis

#endif  // PILOT_TARGET
