/*
  Si4707.h - Arduino library for controling the Silicon Labs Si4707 in I2C mode.

  Copyright 2013 by Ray H. Dees
  Copyright 2013 by Richard Vogel

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef SI4707_h
#define SI4707_h
//
#include "Arduino.h"
#include "SI4707_DEFINITIONS.h"

#define SI4707_RST                               4

#ifndef SI4707_INT
// Both the Uno and Mega have external interrupts available on pin 2
#define SI4707_INT                               2      //  Arduino Interrupt input pin.
#endif

#define SI4707_ON 		                        0x01      //  Used for Power/Mute On.
#define SI4707_OFF 	                          0x00      //  Used for Power/Mute Off.
#define SI4707_CMD_DELAY	                       2      //  Inter-Command delay (301 usec).
#define SI4707_PROP_DELAY                       10      //  Set Property Delay (10.001 msec)
#define SI4707_PUP_DELAY	                     200      //  Power Up Delay.  (110.001 msec)
#define SI4707_TUNE_DELAY                      250      //  Tune Delay. (250.001 msec)
/*
Although the Si4707 will respond to only a single device
address, this address can be changed with the SEN pin
(note that the SEN pin is not used for signaling in 2-wire
mode). When SEN = 0, the 7-bit device address is
0010001b. When SEN = 1, the address is 1100011b.
*/
#ifndef SI4707_RADIO_ADDRESS
#define SI4707_RADIO_ADDRESS            0b00010001      //  I2C address of the Si4707, shifted one bit.
#endif
#define SI4707_RADIO_VOLUME                 0x003F      //  Default Volume.
//
//  SAME Definitions.
//
#define SI4707_SAME_CONFIDENCE_THRESHOLD         1      //  Must be 1, 2 or 3, nothing else!
#define SI4707_SAME_BUFFER_SIZE                255      //  The maximum number of receive bytes.
#define SI4707_SAME_MIN_LENGTH                  36      //  The SAME message minimum acceptable length.
#define SI4707_SAME_LOCATION_CODES              30      //  Subtract 1, because we count from 0.
#define SI4707_SAME_TIME_OUT                     6      //  Time before buffers are flushed.
//
//  Program Control Status Bits.
//
#define SI4707_INTAVL                         0x10      //  A status interrupt is available.
//
#define SI4707_MSGAVL                         0x01      //  A SAME message is Available to be printed/parsed.
#define SI4707_MSGPAR                         0x02      //  The SAME message was successfully Parsed.
#define SI4707_MSGUSD                         0x04      //  When set, this SAME message has been used.
#define SI4707_MSGPUR                         0x08      //  The SAME message should be Purged (Third Header received).
//
//  Global Status Bytes.
//
extern uint8_t intStatus;
extern uint8_t rsqStatus;
extern uint8_t sameStatus;
extern uint8_t asqStatus;
extern uint8_t agcStatus;
extern uint8_t msgStatus;
//
//  Global Radio Variables.
//
extern uint16_t channel;
extern float frequency;
extern uint16_t volume;
extern uint8_t mute;
extern uint8_t rssi;
extern uint8_t snr;
extern int freqoff;
extern uint8_t power;
//
//  Global SAME Variables.
//
extern char sameOriginatorName[];
extern char sameEventName[];
extern char sameCallSign[];
//
extern uint8_t sameHeaderCount;
extern uint8_t sameLength;
extern uint8_t sameState;
extern uint8_t samePlusIndex;
extern uint8_t sameLocations;
extern uint32_t sameLocationCodes[];
extern uint16_t sameDuration;
extern uint16_t sameDay;
extern uint8_t sameHour;
extern uint8_t sameMinute;
extern uint8_t sameWat;
//
extern uint8_t response[];
//
//  SI4707 Class.
//
class SI4707
{
  public:

    void begin(uint16_t reset);
    void begin(void);
    void on(void);
    void getRevision(void);
    void patch(void);

    void enableInterrupt(void);
    void disableInterrupt(void);

    void off(void);
    void end(void);

    void tune(uint32_t direct);
    void tune(void);
    void scan(void);

    uint8_t getIntStatus(void);
    void getTuneStatus(uint8_t mode);
    void getRsqStatus(uint8_t mode);
    void getSameStatus(uint8_t mode);
    void getAsqStatus(uint8_t mode);
    void getAgcStatus(void);

    void setVolume(uint16_t volume);
    void setMute(uint8_t value);

    void setProperty(uint16_t property, uint16_t value);
    uint16_t getProperty(uint16_t property);

    void gpioControl(uint8_t value);
    void gpioSet(uint8_t value);

    int  sameAvailable(void);
    char sameRead(void);
    void sameParse(void);
    void sameFlush(void);
    void sameFill(const String &s);

  private:

    static uint8_t sameConf[];
    static char sameData[];
    static uint8_t rxConfidence[];
    static char rxBuffer[];
    static uint8_t rxBufferIndex;
    static uint8_t rxBufferLength;

    uint16_t reset;
    void writeCommand(uint8_t command);
    void writeByte(uint8_t command, uint8_t value);
    void writeWord(uint8_t command, uint16_t value);
    void writeAddress(uint8_t address, uint8_t mode);

    void readStatus(void);
    void readBurst(int quantity);
};

extern SI4707 Radio;

#endif  //  End of SI4707.h

