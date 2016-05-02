
#ifndef SI47xx_h
#define SI47xx_h

#include <Arduino.h>

//-----------------------------------------------------------------------------
// Helper function that is used to write to the part which abstracts what
// bus mode is currently being used.
//-----------------------------------------------------------------------------
void si47xx_lowWrite(u8 number_bytes, u8 *data_out);

//-----------------------------------------------------------------------------
// Helper function that is used to read from the part which abstracts what
// bus mode is currently being used.
//-----------------------------------------------------------------------------
void si47xx_lowRead(u8 number_bytes, u8 *data_in);

u8 si47xx_readStatus();

//-----------------------------------------------------------------------------
// Command that will wait for CTS before returning
//-----------------------------------------------------------------------------
void si47xx_waitForCTS();

void si47xx_command(int cmd_size, byte *cmd, int reply_size, byte *reply);

#endif
