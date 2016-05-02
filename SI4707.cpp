/*
   SI4707.cpp - Arduino library for controling the Silicon Labs Si4707 in I2C mode.

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

#include <Wire.h>

#include "SI4707.h"
#include "SI4707_PATCH.h"
#include "SI47xx.h"

// http://stackoverflow.com/questions/5320439/how-do-i-swap-endian-ness-byte-order-of-a-variable-in-javascript
static int swap16( int val) {
    return ((val & 0xFF) << 8)
           | ((val >> 8) & 0xFF);
}

inline void waitCommand() {
    delayMicroseconds(300);
}

inline void waitPowerUp() {
    delay(110);
}

inline void waitPropertySet() {
    delay(10);
}
//
//  Timer 1 Variables.
//
volatile uint8_t sreg;
volatile uint8_t timer;
volatile bool interruptFired;

#define TIMER1_START()            TCNT1 = 0x00; TCCR1B |= (1 << WGM12 | 1 << CS12 | 1 << CS10); timer = 0;
#define TIMER1_STOP()             TCCR1B = 0x00


//
//  Static Class Variables.
//
//
// Begin using the Si4707.
//
void SI4707::begin(void) {
    begin(RST);
}

void SI4707::begin(uint16_t _reset) {
    noInterrupts();                               //  Disable interrupts.
    // intStatus =  0x00;
    // rsqStatus =  0x00;
    // sameStatus = 0x00;
    // asqStatus =  0x00;
    agcStatus =  0x00;
    msgStatus =  0x00;

    channel = WB_MIN_FREQUENCY;
    volume = RADIO_VOLUME;
    mute = OFF;
    power = OFF;

    reset = _reset;
    pinMode(reset, OUTPUT);                          //  Setup the reset pin.
    digitalWrite(reset, LOW);                        //  Reset the Si4707.
    waitCommand();
    digitalWrite(reset, HIGH);

    pinMode(INT, INPUT);                           //  Setup the interrupt pin.
    digitalWrite(INT, HIGH);

    ADCSRA = 0x00;                                 //  Disable the analog comparator.

#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)
    EICRA = 0x00;
    EICRA |= (1 << ISC01);                         //  Setup Interrupt 0 for FALLING edge.
    EIFR  |= (1 << INTF0);                         //  Clear pending interrupts.
    EIMSK |= (1 << INT0);                          //  Enable Interrupt 0.
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
    EICRB = 0x00;
    EICRB |= (1 << ISC41);                         //  Setup Interrupt 4 for FALLING edge.
    EIFR  |= (1 << INTF4);                         //  Clear pending interrupts.
    EIMSK |= (1 << INT4);                          //  Enable Interrupt 4.
#endif

    TCCR1A = 0x00;                                 //  Reset TCCR1A to Normal mode.
    TCCR1B = 0x00;                                 //  Reset TCCR1B.
    TIFR1 = (1 << OCF1B | 1 << OCF1A | 1 << TOV1); //  Clear pending interrupts.

#if F_CPU == 16000000UL                            //  16 MHz clock.
    OCR1A = 0x3D08;                                //  Compare Match at 1 second.
#elif F_CPU == 8000000UL                           //  8 MHz clock.
    OCR1A = 0x1E83;                                //  Compare Match at 1 second.
#endif

    TIMSK1 = 0x00;                                 //  Reset TIMSK1.
    TIMSK1 |= (1 << OCIE1A);                       //  Timer 1 Compare Match A Interrupt Enable.

    TCCR2B = 0x00;                                 //  Stop Timer 2.

    interrupts();                                  // Enable Global Interrupts.

    Wire.begin();
}

//
//  Powers up the Si4707.
//
void SI4707::on(void) {
    if (power)
        return;

    Wire.beginTransmission(RADIO_ADDRESS);
    Wire.write(POWER_UP);
    Wire.write(GPO2EN | XOSCEN | WB);
    Wire.write(OPMODE);
    Wire.endTransmission();

    waitPowerUp();

    getIntStatus();

    power = ON;
}

//
//  Gets the revision of the Si4707.
//
void SI4707::getRevision(void) {
    writeCommand(GET_REV);
    readBurst(8);
}

//
//  Powers up the Si4707 and uploads a patch.
//
void SI4707::patch(void) {
    if (power)
        return;

    uint16_t i, j;

    beginCommand(POWER_UP);
    Wire.write(GPO2EN | PATCH | XOSCEN | WB);
    Wire.write(OPMODE);
    endCommand();

    for (i = 0; i < sizeof(SI4707_PATCH_DATA); i += 8) {
        Wire.beginTransmission(RADIO_ADDRESS);

        for (j = 0; j < 8; j++)
            Wire.write(pgm_read_byte(&(SI4707_PATCH_DATA[i + j])));

        Wire.endTransmission();
        waitPropertySet();
    }

    power = ON;
}

//
//  Powers down the Si4707.
//
void SI4707::off() {
    if (!power)
        return;

    writeCommand(POWER_DOWN);
    power = OFF;
}

//
//  End using the Si4707.
//
void SI4707::end(void) {
    off();
    digitalWrite(reset, LOW);
    waitCommand();
    digitalWrite(reset, HIGH);
}

//
//  Tunes using direct entry.
//
void SI4707::tune(uint32_t direct) {
    if (direct < 162400 || direct > 162550)
        return;

    channel = direct / 2.5;
    tune();
}

//
//  Tunes based on current channel value.
//
void SI4707::tune(void) {
    beginCommand(WB_TUNE_FREQ);
    Wire.write(0);
    Wire.write(highByte(channel));
    Wire.write(lowByte(channel));
    endCommand();

    getIntStatus();
}

//
//  Scans for the best frequency based on RSSI.
//
void SI4707::scan(void) {
    uint16_t i;
    uint16_t best_channel;
    uint8_t best_rssi = 0x00;

    setMute(ON);

    for (i = WB_MIN_FREQUENCY; i <= WB_MAX_FREQUENCY; i += WB_CHANNEL_SPACING) {
        channel = i;
        tune();
        getTuneStatus(INTACK);

        if (rssi > best_rssi) {
            best_rssi = rssi;
            best_channel = channel;
        }
    }

    channel = best_channel;
    tune();
    setMute(OFF);
    available = true;
}

//
//  Returns the current Interrupt Status.
//
uint8_t SI4707::getIntStatus(void) {
    writeCommand(GET_INT_STATUS);
    beginReadStatus(0);

    return available;
}

//
//  Gets the current Tune Status.
//
void SI4707::getTuneStatus(uint8_t mode) {
    // Returns the status of WB_TUNE_FREQ. The commands returns the current frequency, and RSSI/SNR at the
    // moment of tune. The command clears the STCINT interrupt bit when INTACK bit of ARG1 is set. The CTS bit (and
    // optional interrupt) is set when it is safe to send the next command. This command may only be sent when in
    // powerup mode.
    writeByte(WB_TUNE_STATUS, mode);

    readBurst(5);
    channel = word(response[2], response[3]);

    frequency = channel * .0025;
    rssi = response[4];
    snr = response[5];
}

//
//  Queries the status of the Received Signal Quality (RSQ) of the current channel
//
void SI4707::getSignalStatus(uint8_t mode) {
    // Returns status information about the received signal quality. The commands returns the RSSI, SNR, and frequency
    // offset. It also indicates whether the frequency is a currently valid frequency as indicated by VALID, and whether the
    // AFC is railed or not as indicated by AFCRL. This command can be used to check if the received signal is above the
    // RSSI high threshold as reported by RSSIHINT, or below the RSSI low threshold as reported by RSSILINT. It can
    // also be used to check if the received signal is above the SNR high threshold as reported by SNRHINT, or below the
    // SNR low threshold as reported by SNRLINT. The command clears the STCINT interrupt bit when INTACK bit of
    // ARG1 is set. The CTS bit (and optional interrupt) is set when it is safe to send the next command. This command
    // may only be sent when in powerup mode.

    // Bit  | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0
    // CMD  | 0 | 1 | 0 | 1 | 0 | 0 | 1 | 1
    // ARG1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | INTACK

    // Arg | Bit | Name | Function
    // 1 | 0 | INTACK | Interrupt Acknowledge
    //    0 = Interrupt status preserved.
    //    1 = Clears RSQINT, SNRHINT, SNRLINT, RSSIHINT, RSSILINT
    writeByte(WB_RSQ_STATUS, mode);

    // Bit | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0
    // STATUS | CTS | ERR | X | X | RSQINT | SAMEINT | ASQINT | STCINT
    // RESP1 | X | X | X | X | SNRHINT | SNRLINT | RSSIHINT | RSSIILINT
    // RESP2 | X | X | X | X | X | X | AFCRL | VALID
    // RESP3 | X | X | X | X | X | X | X | X
    // RESP4 RSSI[7:0]
    // RESP5 ASNR[7:0]
    // RESP6 | X | X | X | X | X | X | X | X
    // RESP7 FREQOFF[7:0]
    beginReadStatus(7);

    // Data | Bit | Name | Function
    // 1 | 3 | SNRHINT | SNR Detect High.
    //    0 = Received SNR has not exceeded above SNR high threshold.
    //    1 = Received SNR has exceeded above SNR high threshold.
    // 1 | 2 | SNRLINT | SNR Detect Low.
    //    0 = Received SNR has not exceeded below SNR low threshold.
    //    1 = Received SNR has exceeded below SNR low threshold.
    // 1 | 1 | RSSIHINT | RSSI Detect High.
    //    0 = RSSI has not exceeded above RSSI high threshold.
    //    1 = RSSI has exceeded above RSSI high threshold.
    // 1 | 0 | RSSILINT | RSSI Detect Low.
    //    0 = RSSI has not exceeded below RSSI low threshold.
    //    1 = RSSI has exceeded below RSSI low threshold.
    // 2 | 1 | AFCRL | AFC Rail Indicator.
    //    This bit will be set if the AFC rails.
    // 2 | 0 | VALID | Valid Channel.
    //    Confirms if the channel is currently valid.
    // 4 | 7:0 | RSSI[7:0] | Received Signal Strength Indicator.
    //    This byte will contain the receive signal strength at the tuned frequency.
    // 5 | 7:0 | SNR[7:0] | SNR.
    //    This byte will contain the SNR metric at the tuned frequency.
    // 7 | 7:0 | FREQOFF[7:0] | Frequency Offset.
    //    Signed frequency offset in kHz.
    readInto((uint8_t*)&signalStatus, 2);
    readByte();  // RESP3 No Data
    rssi = readByte();
    snr = readByte();
    readByte(); // RESP6 No Data
    freqoff = readByte();

    if (freqoff >= 128)
        freqoff = (freqoff - 256) >> 1;
    else
        freqoff = (freqoff >> 1);
}

//
//  Gets the current SAME Status.
//
void SI4707::getSameStatus(uint8_t mode) {
    uint8_t i, j;

    writeAddress(0x00, mode);

    beginReadStatus(3);

    readInto((uint8_t*)&sameStatus, 1);
    readInto((uint8_t*)&sameState, 1);
    readInto((uint8_t*)&sameLength, 1);

    if (!(sameStatus.hdrrdy))                    //  If no HDRRDY, return.
        return;

    // TIMER1_START();                                //  Start/Re-start the 6 second timer.

    sameHeaderCount++;

    if (sameHeaderCount >= 3)                      //  If this is the third Header, set msgStatus to show that it needs to be purged after usage.
        msgStatus |= MSGPUR;

    if (sameLength < SAME_MIN_LENGTH)              //  Don't process messages that are too short to be valid.
        return;

    for (i = 0; i < sameLength && i < SAME_BUFFER_SIZE; i += 8) {
        writeAddress(i, CHECK);

        readBurst(14);

        sameConf[0] = (response[5] & SAME_STATUS_OUT_CONF0) >> SAME_STATUS_OUT_CONF0_SHFT;
        sameConf[1] = (response[5] & SAME_STATUS_OUT_CONF1) >> SAME_STATUS_OUT_CONF1_SHFT;
        sameConf[2] = (response[5] & SAME_STATUS_OUT_CONF2) >> SAME_STATUS_OUT_CONF2_SHFT;
        sameConf[3] = (response[5] & SAME_STATUS_OUT_CONF3) >> SAME_STATUS_OUT_CONF3_SHFT;
        sameConf[4] = (response[4] & SAME_STATUS_OUT_CONF4) >> SAME_STATUS_OUT_CONF4_SHFT;
        sameConf[5] = (response[4] & SAME_STATUS_OUT_CONF5) >> SAME_STATUS_OUT_CONF5_SHFT;
        sameConf[6] = (response[4] & SAME_STATUS_OUT_CONF6) >> SAME_STATUS_OUT_CONF6_SHFT;
        sameConf[7] = (response[4] & SAME_STATUS_OUT_CONF7) >> SAME_STATUS_OUT_CONF7_SHFT;

        sameData[0] = response[6];
        sameData[1] = response[7];
        sameData[2] = response[8];
        sameData[3] = response[9];
        sameData[4] = response[10];
        sameData[5] = response[11];
        sameData[6] = response[12];
        sameData[7] = response[13];

        for (j = 0; j + i < sameLength && j < 8; j++) {
            rxBuffer[j + i] = sameData[j];
            rxConfidence[j + i] = sameConf[j];
            if (rxBuffer[j + i] < 0x2B  || rxBuffer[j + i] > 0x7F) {
                sameLength = j + i;
                break;
            }
        }
    }

    msgStatus |= MSGAVL;

    for (i = 0; i < sameLength; i++) {
        if (rxConfidence[i] > SAME_CONFIDENCE_THRESHOLD)
            rxConfidence[i] = SAME_CONFIDENCE_THRESHOLD;

        if (rxConfidence[i] < SAME_CONFIDENCE_THRESHOLD) {
            msgStatus &= ~MSGAVL;
            break;
        }
    }

    if (!(msgStatus & MSGAVL))
        return;

    rxBufferIndex = 0;
    rxBufferLength = sameLength;
}

//
// Queries the status of the 1050 kHz alert tone in Weather Band.
//
void SI4707::getAlertStatus(uint8_t mode) {
    // Returns status information about the 1050kHz alert tone in Weather Band. The commands returns the alert on/off
    // Interrupt and the present state of the alert tone. The command clears the ASQINT bit when INTACK bit of ARG1 is
    // set. The CTS bit (and optional interrupt) is set when it is safe to send the next command. This command may only
    // be sent when in powerup mode.

    // Bit  7 | 6 | 5 | 4 | 3 | 2 | 1 | 0
    // CMD  0 | 1 | 0 | 1 | 0 | 1 | 0 | 1
    // ARG1 0 | 0 | 0 | 0 | 0 | 0 | 0 | INTACK

    // Arg | Bit | Name | Function
    // 1 | 7:1 | Reserved | Always write to 0.
    // 1 | 0 | INTACK | Interrupt Acknowledge
    //    0 = Interrupt status preserved.
    //    1 = Clears ASQINT, ALERTOFF_INT, ALERTON_INT
    writeByte(WB_ASQ_STATUS, mode);

    // Bit    | 7  6  5  4  3  2  1  0
    // STATUS | CTS ERR X X RSQINT SAMEINT ASQINT STCINT
    // RESP1  | X X X X X X ALERTOFF_INT ALERTON_INT
    // RESP2  | X X X X X X X ALERT
    beginReadStatus(2);


    // Data | Bit | Name | Function
    // 1 | 1 | ALERTOFF_INT | ALERTOFF_INT.
    //    0 = 1050 Hz alert tone has not been detected to be absent since the last WB_TUNE_FREQ or WB_RSQ_STATUS with INTACK = 1.
    //    1 = 1050 Hz alert tone has been detected to be absent since the last WB_TUNE_FREQ or WB_RSQ_STATUS with INTACK = 1.
    // 1 | 0 | ALERTON_INT | ALERTON_INT.
    //    0 = 1050 Hz alert tone has not been detected to be present since the last WB_TUNE_FREQ or WB_RSQ_STATUS with INTACK = 1.
    //    1 = 1050 Hz alert tone has been detected to be present since the last WB_TUNE_FREQ or WB_RSQ_STATUS with INTACK = 1.
    // 2 | 0 | ALERT | ALERT.
    //    0 = 1050 Hz alert tone is currently not present.
    //    1 = 1050 Hz alert tone is currently present.
    readInto((uint8_t*)&alertStatus, 2);
}

//
//  Gets the current AGC Status.
//
void SI4707::getAgcStatus(void) {
    writeCommand(WB_AGC_STATUS);

    beginReadStatus(1);
    readInto(&agcStatus, 1);
}

//
//  Sets the audio volume level.
//
void SI4707::setVolume(uint16_t newVolume) {
    if (newVolume < 0x0000 || newVolume > 0x003F)
        return;
    volume = newVolume;

    setProperty(RX_VOLUME, volume);
}

//
//  Sets the current Mute state.
//
void SI4707::setMute(uint8_t value) {
    switch (value) {
    case OFF:
        setProperty(RX_HARD_MUTE, 0x0000);
        mute = OFF;
        break;

    case ON:
        setProperty(RX_HARD_MUTE, 0x0003);
        mute = ON;
        break;

    default:
        break;
    }
}

//
//  Sets a specified property value.
//
void SI4707::setProperty(uint16_t property, uint16_t value) {
    beginCommand(SET_PROPERTY);
    Wire.write(uint8_t(0x00)); // Reserved
    Wire.write(highByte(property)); // PROP H
    Wire.write(lowByte(property)); // PROP L
    Wire.write(highByte(value)); // PROPV H
    Wire.write(lowByte(value)); // PROPV L
    endCommand();
}

//
//  Returns a specified property value.
//
uint16_t SI4707::getProperty(uint16_t property) {
    uint16_t value = 0;
    beginCommand(GET_PROPERTY);
    Wire.write(uint8_t(0x00)); // ARG1 Reserved
    Wire.write(highByte(property)); // PROPG H
    Wire.write(lowByte(property)); // PROPG L
    endCommand();

    beginReadStatus(3);
    Wire.read(); // RESP1 Reserved
    value = readWord(); // RESP2, RESP3

    value = swap16(value);

    return value;
}

//
//  Controls a specified GPIO.
//
void SI4707::gpioControl(uint8_t value) {
    writeByte(GPIO_CTL, value);
}

//
//  Sets a specified GPIO.
//
void SI4707::gpioSet(uint8_t value) {
    writeByte(GPIO_SET, value);
}

//
//  Return available character count.
//
int SI4707::sameAvailable(void) {
    if (rxBufferIndex == rxBufferLength)
        return -1;

    else
        return rxBufferLength - rxBufferIndex;
}

//
//  Return received characters.
//
char SI4707::sameRead(void) {
    char value = 0x00;

    if (rxBufferIndex < rxBufferLength) {
        value = rxBuffer[rxBufferIndex];
        rxBufferIndex++;
    }

    else {
        rxBufferIndex = rxBufferLength = 0;
        msgStatus |= MSGUSD;
    }

    return value;
}

//
//  The SAME message is parsed here.
//
void SI4707::sameParse(void) {
    if (!(msgStatus & MSGAVL))                     //  If no message is Available, return
        return;

    samePlusIndex = 0;
    sameLocations = 0;
    sameDuration = 0;
    sameDay = 0;
    sameTime = 0;

    uint8_t i = 0;

    sameOriginatorName[0] = rxBuffer[i + 1];
    sameOriginatorName[1] = rxBuffer[i + 2];
    sameOriginatorName[2] = rxBuffer[i + 3];
    sameOriginatorName[3] = 0x00;

    sameEventName[0] = rxBuffer[i + 5];
    sameEventName[1] = rxBuffer[i + 6];
    sameEventName[2] = rxBuffer[i + 7];
    sameEventName[3] = 0x00;

    for (i = 0; i < sizeof(rxBuffer); i++) {       //  Look for the Plus Sign.
        if (rxBuffer[i] == 0x2B)
            samePlusIndex = i;                       //  Found it.

        if (rxBuffer[i] >= 0x30 && rxBuffer[i] <= 0x39) //  If the value is ascii, strip off the upper bits.
            rxBuffer[i] = rxBuffer[i] & 0x0F;
    }

    if (samePlusIndex == 0)                        //  No Plus Sign found.
        return;

    for (i = 6; i < samePlusIndex; i++) {          //  There are no sameLocationCodes past the samePlusIndex.
        if (rxBuffer[i] == 0x2D) {
            sameLocationCodes[sameLocations] = 0;  //  Clear out any remaining data.
            sameLocationCodes[sameLocations] += rxBuffer[i + 1] * 100000UL;
            sameLocationCodes[sameLocations] += rxBuffer[i + 2] *  10000UL;
            sameLocationCodes[sameLocations] += rxBuffer[i + 3] *   1000UL;
            sameLocationCodes[sameLocations] += rxBuffer[i + 4] *    100UL;
            sameLocationCodes[sameLocations] += rxBuffer[i + 5] *     10UL;
            sameLocationCodes[sameLocations] += rxBuffer[i + 6] *      1UL;

            if (sameLocations > SAME_LOCATION_CODES) //  SAME_LOCATION_CODES (31) is the maximum allowed.
                return;

            else
                sameLocations++;
        }
    }

    sameDuration += rxBuffer[samePlusIndex + 1] * 600;
    sameDuration += rxBuffer[samePlusIndex + 2] *  60;
    sameDuration += rxBuffer[samePlusIndex + 3] *  10;
    sameDuration += rxBuffer[samePlusIndex + 4] *   1;

    sameDay += rxBuffer[samePlusIndex + 6] * 100;
    sameDay += rxBuffer[samePlusIndex + 7] *  10;
    sameDay += rxBuffer[samePlusIndex + 8] *   1;

    sameTime += rxBuffer[samePlusIndex +  9] * 1000;
    sameTime += rxBuffer[samePlusIndex + 10] *  100;
    sameTime += rxBuffer[samePlusIndex + 11] *   10;
    sameTime += rxBuffer[samePlusIndex + 12] *    1;

    for (i = 0; i < 8; i++) {
        if (rxBuffer[i + samePlusIndex + 14] == 0x2D || rxBuffer[i + samePlusIndex + 14] == 0x00) {
            sameCallSign[i] = 0x00;
            break;
        }

        sameCallSign[i] = rxBuffer[i + samePlusIndex + 14];
    }

    msgStatus |= (MSGUSD | MSGPAR);                // Set the status to show the message was successfully Parsed.
}

//
//  Flush the SAME receive data.
//
void SI4707::sameFlush(void) {
    TIMER1_STOP();

    getSameStatus(CLRBUF | INTACK);

    for (uint8_t i = 0; i < SAME_BUFFER_SIZE; i++) {
        rxBuffer[i] = 0x00;
        rxConfidence[i] = 0x00;
    }

    msgStatus = 0x00;
    sameHeaderCount = sameLength = 0;
    rxBufferIndex = rxBufferLength = 0;
}

//
//  Fill SAME rxBuffer for testing purposes.
//
void SI4707::sameFill(const String &s) {
    sameFlush();

    for (uint8_t i = 0; i < s.length(); i++) {
        rxBuffer[i] = s[i];
        rxConfidence[i] = SAME_CONFIDENCE_THRESHOLD;
        sameLength++;
        if (sameLength == SAME_BUFFER_SIZE)
            break;
    }
}

//
//  Start a single command.
//
void SI4707::beginCommand(uint8_t command) {
    Wire.beginTransmission(RADIO_ADDRESS);
    Wire.write(command);
}

//
//  Finish a single command.
//
void SI4707::endCommand() {
    Wire.endTransmission();
    si47xx_waitForCTS();
}

//
//  Write a single command.
//
void SI4707::writeCommand(uint8_t command) {
    beginCommand(command);
    endCommand();
}

//
//  Write a single command byte.
//
void SI4707::writeByte(uint8_t command, uint8_t value) {
    beginCommand(command);
    Wire.write(value);
    endCommand();
}

//
//  Write a single command word.
//
void SI4707::writeWord(uint8_t command, uint16_t value) {
    beginCommand(command);
    Wire.write(0x00);
    Wire.write(highByte(value));
    Wire.write(lowByte(value));
    endCommand();
}

//
//  Write an address and mode byte.
//
void SI4707::writeAddress(uint8_t address, uint8_t mode) {
    beginCommand(WB_SAME_STATUS);
    Wire.write(mode);
    Wire.write(address);
    endCommand();
    waitCommand(); //  A CLRBUF takes a fair amount of time! CMD_DELAY + CMD_DELAY * 3 == CMD_DELAY * 4
    waitCommand();
    waitCommand();
}


void SI4707::beginReadStatus(int numBytes) {
    Wire.requestFrom(RADIO_ADDRESS, numBytes + 1);
    readInto((uint8_t*)&interruptStatus, 1);
}

int SI4707::readInto(uint8_t * response, int numBytes) {
    int i;

    for (i = 0; i < numBytes; i++) {
        if (!Wire.available()) break;
        response[i] =  Wire.read();
    }

    return i;
}

uint8_t SI4707::readByte() {
    if (Wire.available()) {
        return Wire.read();
    }

    return 0;
}

uint16_t SI4707::readWord() {
    if (Wire.available() >= 2) {
        return word(Wire.read(), Wire.read());
    }
    return 0;
}

//
//  Reads the number of bytes specified by quantity.
//
void SI4707::readBurst(int quantity) {
    beginReadStatus(quantity);
    readInto((uint8_t*)response, quantity);
}


/* Begin Static Methods */

// void SI4707::toInterruptStatus(struct InterruptStatus *interruptStatus, uint8_t value) {
//     interruptStatus->clearToSend  = value & CTSINT;
//     interruptStatus->error        = value & ERRINT;
//     interruptStatus->reserved     = 0x00;
//     interruptStatus->rsq          = value & RSQINT;
//     interruptStatus->same         = value & SAMEINT;
//     interruptStatus->asq          = value & ASQINT;
//     interruptStatus->tuneComplete = value & STCINT;
// }


void SI4707::toSameStatus(struct SameStatus *same, uint8_t value) {
    same->reserved = 0x00;
    same->eomdet = value & 0x08;
    same->somdet = value & 0x04;
    same->predet = value & 0x02;
    same->hdrrdy = value & 0x01;
}

/* End Static Methods */

//
//  Interrupt 0 or 4 Service Routine - Triggered on the Falling edge.
//
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)     //  Interrupt 0.
ISR(INT0_vect)
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)  //  Interrupt 4.
ISR(INT4_vect)
#endif
{
    interruptFired = true;
}

//
//  Timer 1 Compare Match A Interrupt Service Routine -  Increments every 1 second.
//
ISR(TIMER1_COMPA_vect, ISR_NOBLOCK) {
    sreg = SREG;

    timer++;

    if (timer >= SAME_TIME_OUT)
        Radio.sameFlush();

    SREG = sreg;
}

SI4707 Radio;
