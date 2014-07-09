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
//
//  Arduino definitions.
//
#ifndef NO_INTERRUPTS()
#define NO_INTERRUPTS()           uint8_t sreg = SREG; cli()
#define INTERRUPTS()              SREG = sreg
#endif
//
#ifndef TIMER1_START()
#define TIMER1_START()            TCNT1 = 0x00; TCCR1B |= (1 << WGM12 | 1 << CS12 | 1 << CS10); timer = 0;
#define TIMER1_STOP()             TCCR1B = 0x00
#endif
//
#define INT                               2      //  Arduino Interrupt input pin.
#define RST                               4      //  Arduino pin used to reset the Si4707.
#define ON 		                         0x01      //  Used for Power/Mute On.
#define OFF 	                         0x00      //  Used for Power/Mute Off.
#define CMD_DELAY	                        2      //  Inter-Command delay (301 usec).
#define PROP_DELAY                       10      //  Set Property Delay (10.001 msec)
#define PUP_DELAY	                      200      //  Power Up Delay.  (110.001 msec)
#define TUNE_DELAY                      250      //  Tune Delay. (250.001 msec)
#define RADIO_ADDRESS                  0x22 >> 1 //  I2C address of the Si4707, shifted one bit.
#define RADIO_VOLUME                 0x003F      //  Default Volume.
//
//  SAME Definitions.
//
#define SAME_CONFIDENCE_THRESHOLD         1      //  Must be 1, 2 or 3, nothing else!
#define SAME_BUFFER_SIZE                255      //  The maximum number of receive bytes.
#define SAME_MIN_LENGTH                  36      //  The SAME message minimum acceptable length.
#define SAME_LOCATION_CODES              30      //  Subtract 1, because we count from 0.
#define SAME_TIME_OUT                     6      //  Time before buffers are flushed.
//
//  Program Control Status Bits.
//
#define INTAVL                         0x10      //  A status interrupt is available.
//
#define MSGAVL                         0x01      //  A SAME message is Available to be printed/parsed.
#define MSGPAR                         0x02      //  The SAME message was successfully Parsed.
#define MSGUSD                         0x04      //  When set, this SAME message has been used.
#define MSGPUR                         0x08      //  The SAME message should be Purged (Third Header received).
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
extern uint16_t sameTime;
extern uint8_t sameWat;
//
extern uint8_t response[];
extern volatile uint8_t sreg;
extern volatile uint8_t timer;
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

