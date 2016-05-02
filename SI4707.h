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

extern volatile uint8_t sreg;
extern volatile uint8_t timer;

inline void waitCommand();
inline void waitPowerUp();
inline void waitPropertySet();

struct InterruptStatus {
    unsigned char clearToSend : 1; // 0x80
    unsigned char error : 1; // 0x40
    unsigned char : 2; // 0x20, 0x10 Reserved
    unsigned char rsq : 1; // 0x08
    unsigned char same : 1; // 0x04
    unsigned char asq : 1; // 0x02
    unsigned char tuneComplete : 1;  // 0x01
};

struct SameStatus {
    unsigned char reserved : 4;
    unsigned char eomdet : 1;
    unsigned char somdet : 1;
    unsigned char predet : 1;
    unsigned char hdrrdy : 1;
};

struct AlertStatus {
    unsigned char reserved : 6;
    unsigned char alertoff_int : 1;
    unsigned char alerton_int : 1;
    unsigned char tonePresent : 8;
};

struct SignalStatus {
    unsigned char reserved1 : 4;
    unsigned char snrh_int : 1;
    unsigned char snrl_int : 1;
    unsigned char rssih_int : 1;
    unsigned char rssil_int : 1;
    unsigned char reserved2 : 6;
    unsigned char afcrl : 1;
    unsigned char valid : 1;
};
//
//  SI4707 Class.
//
class SI4707 {
public:
    //
    //  Global Status Bytes.
    //
    struct InterruptStatus interruptStatus;
    volatile unsigned char available;

    struct SameStatus sameStatus;
    struct AlertStatus alertStatus;
    struct SignalStatus signalStatus;
    // uint8_t intStatus;
    // uint8_t rsqStatus;
    // uint8_t sameStatus;
    // uint8_t asqStatus;
    uint8_t agcStatus;
    uint8_t msgStatus;
    //
    //  Global Radio Variables.
    //
    uint16_t channel;
    float frequency;
    uint16_t volume;
    uint8_t mute;
    uint8_t rssi;
    uint8_t snr;
    int freqoff;
    uint8_t power;
    //
    //  Global SAME Variables.
    //
    char sameOriginatorName[4];
    char sameEventName[4];
    char sameCallSign[9];
    //
    uint8_t sameHeaderCount;
    uint8_t sameState;
    uint8_t sameLength;
    uint8_t samePlusIndex;
    uint8_t sameLocations;
    uint32_t sameLocationCodes[SAME_LOCATION_CODES];
    uint16_t sameDuration;
    uint16_t sameDay;
    uint16_t sameTime;
    uint8_t response[15];

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
    void getSignalStatus(uint8_t mode);
    void getSameStatus(uint8_t mode);
    void getAlertStatus(uint8_t mode);
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

    uint8_t sameConf[8];
    char sameData[8];
    uint8_t rxConfidence[SAME_BUFFER_SIZE];
    char rxBuffer[SAME_BUFFER_SIZE];
    uint8_t rxBufferIndex;
    uint8_t rxBufferLength;

    uint16_t reset;
    void beginReadStatus(int quantity);
    int readInto(uint8_t * response, int quantity);
    uint8_t readByte();
    uint16_t readWord();
    void beginCommand(uint8_t command);
    void endCommand(void);
    void writeCommand(uint8_t command);
    void writeByte(uint8_t command, uint8_t value);
    void writeWord(uint8_t command, uint16_t value);
    void writeAddress(uint8_t address, uint8_t mode);

    void readBurst(int quantity);

    static void toInterruptStatus(struct InterruptStatus *status, uint8_t value);
    static void toSameStatus(struct SameStatus *same, uint8_t value);
};

extern SI4707 Radio;

#endif  //  End of SI4707.h
