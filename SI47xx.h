
#ifndef SI47xx_h
#define SI47xx_h

#include <Arduino.h>
#include <stdint.h>

//-----------------------------------------------------------------------------
// Helper function that is used to write to the part which abstracts what
// bus mode is currently being used.
//-----------------------------------------------------------------------------
void si47xx_lowWrite(uint8_t number_bytes, uint8_t *data_out);

//-----------------------------------------------------------------------------
// Helper function that is used to read from the part which abstracts what
// bus mode is currently being used.
//-----------------------------------------------------------------------------
void si47xx_lowRead(uint8_t number_bytes, uint8_t *data_in);

uint8_t si47xx_readStatus();

//-----------------------------------------------------------------------------
// Command that will wait for CTS before returning
//-----------------------------------------------------------------------------
void si47xx_waitForCTS();

void si47xx_command(int cmd_size, byte *cmd, int reply_size, byte *reply);

#endif
