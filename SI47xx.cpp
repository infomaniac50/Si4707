#include <Arduino.h>
#include <Wire.h>
#include "SI4707_DEFINITIONS.h"
#include "SI47xx.h"

//-----------------------------------------------------------------------------
// Helper function that is used to write to the part which abstracts what
// bus mode is currently being used.
//-----------------------------------------------------------------------------
void si47xx_lowWrite(u8 number_bytes, u8 *data_out)
{
    Wire.beginTransmission(RADIO_ADDRESS);
    for (int index = 0; index < number_bytes; index++) {
        Wire.write(data_out[index]);
    }
    Wire.endTransmission();
}

//-----------------------------------------------------------------------------
// Helper function that is used to read from the part which abstracts what
// bus mode is currently being used.
//-----------------------------------------------------------------------------
void si47xx_lowRead(u8 number_bytes, u8 *data_in)
{
    Wire.requestFrom(RADIO_ADDRESS, (int)number_bytes);
    for (int index = 0; index < number_bytes; index++) {
        data_in[index] = Wire.read();
    }
}

u8 si47xx_readStatus()
{
    u8 status;

    si47xx_lowRead(1, &status);

    return status;
}

//-----------------------------------------------------------------------------
// Command that will wait for CTS before returning
//-----------------------------------------------------------------------------
void si47xx_waitForCTS()
{
    u16 i=1000;

    // Loop until CTS is found or stop due to the counter running out.
    while (--i && !(si47xx_readStatus() & CTSINT))
    {
        delayMicroseconds(500);
    }

    // If the i is equal to 0 then something must have happened.
    // It is recommended that the controller do some type of error
    // handling in this case.
}

void si47xx_command(int cmd_size, byte *cmd, int reply_size, byte *reply) {
    // It is always a good idea to check for cts prior to sending a command to
    // the part.
    si47xx_waitForCTS();

    // Write the command to the part
    si47xx_lowWrite(cmd_size, cmd);

    // Wait for CTS after sending the command
    si47xx_waitForCTS();

    // If the calling function would like to have results then read them.
    if(reply_size > 0 && reply != NULL)
    {
        si47xx_lowRead(reply_size, reply);
    }
}
