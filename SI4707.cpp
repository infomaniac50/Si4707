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
#include "SI4707.h"
#include "SI4707_PATCH.h"
#include "Wire.h"

//
//  Global Status Bytes.
//
uint8_t intStatus =  0x00;
uint8_t rsqStatus =  0x00;
uint8_t sameStatus = 0x00;
uint8_t asqStatus =  0x00;
uint8_t agcStatus =  0x00;
uint8_t msgStatus =  0x00;
//
//  Global Radio Variables.
//
uint16_t channel = SI4707_WB_MIN_FREQUENCY;
float frequency;
uint16_t volume = SI4707_RADIO_VOLUME;
uint8_t mute = SI4707_OFF;
uint8_t rssi;
uint8_t snr;
int freqoff;
uint8_t power = SI4707_OFF;
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
uint32_t sameLocationCodes[SI4707_SAME_LOCATION_CODES];
uint16_t sameDuration;
uint16_t sameDay;
uint8_t sameHour;
uint8_t sameMinute;
uint8_t sameWat = 0x02;
uint8_t response[15];

//
//  Static Class Variables.
//
uint8_t SI4707::sameConf[8];
char SI4707::sameData[8];
uint8_t SI4707::rxConfidence[SI4707_SAME_BUFFER_SIZE];
char SI4707::rxBuffer[SI4707_SAME_BUFFER_SIZE];
//
uint8_t SI4707::rxBufferIndex;
uint8_t SI4707::rxBufferLength;

//
//  Interrupt 0 or 4 Service Routine - Triggered on the Falling edge.
//
void setInterruptAvailable()
{
  intStatus |= SI4707_INTAVL;
}

//
// Begin using the Si4707.
//
void SI4707::begin(void)
{
  //  Arduino pin used to reset the Si4707.
  begin(SI4707_RST);
}

void SI4707::begin(uint16_t _reset)
{
  reset = _reset;
  pinMode(SI4707_INT, OUTPUT);
  digitalWrite(SI4707_INT, LOW);
  pinMode(reset, OUTPUT);                          //  Setup the reset pin.
  digitalWrite(reset, LOW);                        //  Reset the Si4707.
  delay(SI4707_CMD_DELAY);
  digitalWrite(reset, HIGH);

  Wire.begin();
}

void SI4707::enableInterrupt() {
  pinMode(SI4707_INT, INPUT);                           //  Setup the interrupt pin.

  attachInterrupt(digitalPinToInterrupt(SI4707_INT), setInterruptAvailable, FALLING);
}

void SI4707::disableInterrupt() {
  detachInterrupt(digitalPinToInterrupt(SI4707_INT));
}

//
//  Powers up the Si4707.
//
void SI4707::on(void)
{
  if (power)
    return;

  Wire.beginTransmission(SI4707_RADIO_ADDRESS);
  Wire.write(SI4707_POWER_UP);
  Wire.write(SI4707_GPO2EN | SI4707_XOSCEN | SI4707_WB);
  Wire.write(SI4707_OPMODE);
  Wire.endTransmission();

  delay(SI4707_PUP_DELAY);

  readStatus();

  power = SI4707_ON;
}
//
//  Gets the revision of the Si4707.
//
void SI4707::getRevision(void)
{
  writeCommand(SI4707_GET_REV);
  readBurst(9);
}
//
//  Powers up the Si4707 and uploads a patch.
//
void SI4707::patch(void)
{
  if (power)
    return;

  uint16_t i, j;

  Wire.beginTransmission(SI4707_RADIO_ADDRESS);
  Wire.write(SI4707_POWER_UP);
  Wire.write(SI4707_GPO2EN | SI4707_PATCH | SI4707_XOSCEN | SI4707_WB);
  Wire.write(SI4707_OPMODE);
  Wire.endTransmission();

  delay(SI4707_PUP_DELAY);

  readStatus();

  for (i = 0; i < sizeof(SI4707_PATCH_DATA_BYTES); i += 8)
    {
      Wire.beginTransmission(SI4707_RADIO_ADDRESS);

      for (j = 0; j < 8; j++)
        Wire.write(pgm_read_byte(&(SI4707_PATCH_DATA_BYTES[i + j])));

      Wire.endTransmission();
      delay(SI4707_PROP_DELAY);
      readStatus();
    }

  power = SI4707_ON;
}
//
//  Powers down the Si4707.
//
void SI4707::off()
{
  if (!power)
    return;

  writeCommand(SI4707_POWER_DOWN);
  power = SI4707_OFF;
  delay(SI4707_CMD_DELAY);
}
//
//  End using the Si4707.
//
void SI4707::end(void)
{
  off();
  digitalWrite(reset, LOW);
  delay(SI4707_CMD_DELAY);
  digitalWrite(reset, HIGH);
}
//
//  Tunes using direct entry.
//
void SI4707::tune(uint32_t direct)
{
  if (direct < 162400 || direct > 162550)
    return;

  channel = direct / 2.5;
  tune();
}
//
//  Tunes based on current channel value.
//
void SI4707::tune(void)
{
  writeWord(SI4707_WB_TUNE_FREQ, channel);
  delay(SI4707_TUNE_DELAY);
  intStatus |= SI4707_INTAVL;
}
//
//  Scans for the best frequency based on RSSI.
//
void SI4707::scan(void)
{
  uint16_t i;
  uint16_t best_channel;
  uint8_t  best_rssi = 0x00;

  setMute(SI4707_ON);

  for (i = SI4707_WB_MIN_FREQUENCY; i <= SI4707_WB_MAX_FREQUENCY; i += SI4707_WB_CHANNEL_SPACING)
    {
      channel = i;
      tune();
      getTuneStatus(SI4707_INTACK);

      if (rssi > best_rssi)
        {
          best_rssi = rssi;
          best_channel = channel;
        }
    }

  channel = best_channel;
  tune();
  setMute(SI4707_OFF);
  intStatus |= SI4707_INTAVL;
}
//
//  Returns the current Interrupt Status.
//
uint8_t SI4707::getIntStatus(void)
{
  writeCommand(SI4707_GET_INT_STATUS);
  readStatus();
  intStatus = response[0];

  return intStatus;
}
//
//  Gets the current Tune Status.
//
void SI4707::getTuneStatus(uint8_t mode)
{
  writeByte(SI4707_WB_TUNE_STATUS, mode);

  readBurst(6);

  channel = (0x0000 | response[2] << 8 | response[3]);
  frequency = channel * .0025;
  rssi = response[4];
  snr = response[5];
}
//
//  Gets the current RSQ Status.
//
void SI4707::getRsqStatus(uint8_t mode)
{
  writeByte(SI4707_WB_RSQ_STATUS, mode);

  readBurst(8);

  rsqStatus = response[1];
  rssi = response[4];
  snr = response[5];
  freqoff = response[7];

  if (freqoff >= 128)
    freqoff = (freqoff - 256) >> 1;
  else
    freqoff = (freqoff >> 1);
}
//
//  Gets the current SAME Status.
//
void SI4707::getSameStatus(uint8_t mode)
{
  uint8_t i, j;

  writeAddress(0x00, mode);

  readBurst(4);

  sameStatus = response[1];
  sameState  = response[2];
  sameLength = response[3];

  if (!(sameStatus & SI4707_HDRRDY))                    //  If no SI4707_HDRRDY, return.
    return;

  // TIMER1_START();                                //  Start/Re-start the 6 second timer.

  sameHeaderCount++;

  if (sameHeaderCount >= 3)                      //  If this is the third Header, set msgStatus to show that it needs to be purged after usage.
    msgStatus |= SI4707_MSGPUR;

  if (sameLength < SI4707_SAME_MIN_LENGTH)              //  Don't process messages that are too short to be valid.
    return;

  for (i = 0; i < sameLength && i < SI4707_SAME_BUFFER_SIZE; i += 8)
    {
      writeAddress(i, SI4707_CHECK);

      readBurst(14);

      sameConf[0] = (response[5] & SI4707_SAME_STATUS_OUT_CONF0) >> SI4707_SAME_STATUS_OUT_CONF0_SHFT;
      sameConf[1] = (response[5] & SI4707_SAME_STATUS_OUT_CONF1) >> SI4707_SAME_STATUS_OUT_CONF1_SHFT;
      sameConf[2] = (response[5] & SI4707_SAME_STATUS_OUT_CONF2) >> SI4707_SAME_STATUS_OUT_CONF2_SHFT;
      sameConf[3] = (response[5] & SI4707_SAME_STATUS_OUT_CONF3) >> SI4707_SAME_STATUS_OUT_CONF3_SHFT;
      sameConf[4] = (response[4] & SI4707_SAME_STATUS_OUT_CONF4) >> SI4707_SAME_STATUS_OUT_CONF4_SHFT;
      sameConf[5] = (response[4] & SI4707_SAME_STATUS_OUT_CONF5) >> SI4707_SAME_STATUS_OUT_CONF5_SHFT;
      sameConf[6] = (response[4] & SI4707_SAME_STATUS_OUT_CONF6) >> SI4707_SAME_STATUS_OUT_CONF6_SHFT;
      sameConf[7] = (response[4] & SI4707_SAME_STATUS_OUT_CONF7) >> SI4707_SAME_STATUS_OUT_CONF7_SHFT;

      sameData[0] = response[6];
      sameData[1] = response[7];
      sameData[2] = response[8];
      sameData[3] = response[9];
      sameData[4] = response[10];
      sameData[5] = response[11];
      sameData[6] = response[12];
      sameData[7] = response[13];

      for (j = 0; j + i < sameLength && j < 8; j++)
        {
          rxBuffer[j + i] = sameData[j];
          rxConfidence[j + i] = sameConf[j];
          if (rxBuffer[j + i] < 0x2B  || rxBuffer[j + i] > 0x7F)
            {
              sameLength = j + i;
              break;
            }
        }
    }

  msgStatus |= SI4707_MSGAVL;

  for (i = 0; i < sameLength; i++)
    {
      if (rxConfidence[i] > SI4707_SAME_CONFIDENCE_THRESHOLD)
        rxConfidence[i] = SI4707_SAME_CONFIDENCE_THRESHOLD;

      if (rxConfidence[i] < SI4707_SAME_CONFIDENCE_THRESHOLD)
        {
          msgStatus &= ~SI4707_MSGAVL;
          break;
        }
    }

  if (!(msgStatus & SI4707_MSGAVL))
    return;

  rxBufferIndex = 0;
  rxBufferLength = sameLength;
}
//
//  Gets the current ASQ Status.
//
void SI4707::getAsqStatus(uint8_t mode)
{
  writeByte(SI4707_WB_ASQ_STATUS, mode);

  readBurst(3);

  asqStatus = response[1];
}
//
//  Gets the current AGC Status.
//
void SI4707::getAgcStatus(void)
{
  writeCommand(SI4707_WB_AGC_STATUS);

  readBurst(2);

  agcStatus = response[1];
}
//
//  Sets the audio volume level.
//
void SI4707::setVolume(uint16_t volume)
{
  if (volume < 0x0000 || volume > 0x003F)
    return;

  setProperty(SI4707_RX_VOLUME, volume);
}
//
//  Sets the current Mute state.
//
void SI4707::setMute(uint8_t value)
{
  switch (value)
    {
      case SI4707_OFF:
                setProperty(SI4707_RX_HARD_MUTE, 0x0000);
                mute = SI4707_OFF;
                break;

      case SI4707_ON:
                setProperty(SI4707_RX_HARD_MUTE, 0x0003);
                mute = SI4707_ON;
                break;

      default:
                break;
    }
}
//
//  Sets a specified property value.
//
void SI4707::setProperty(uint16_t property, uint16_t value)
{
  Wire.beginTransmission(SI4707_RADIO_ADDRESS);
  Wire.write(SI4707_SET_PROPERTY);
  Wire.write(uint8_t(0x00));
  Wire.write(highByte(property));
  Wire.write(lowByte(property));
  Wire.write(highByte(value));
  Wire.write(lowByte(value));
  Wire.endTransmission();
  delay(SI4707_PROP_DELAY);
}
//
//  Returns a specified property value.
//
uint16_t SI4707::getProperty(uint16_t property)
{
  uint16_t value = 0;

  writeWord(SI4707_GET_PROPERTY, property);

  readBurst(3);

  value |= (response[2] << 8 | response[3]);

  return value;
}
//
//  Controls a specified GPIO.
//
void SI4707::gpioControl(uint8_t value)
{
  writeByte(SI4707_GPIO_CTL, value);
}
//
//  Sets a specified GPIO.
//
void SI4707::gpioSet(uint8_t value)
{
  writeByte(SI4707_GPIO_SET, value);
}
//
//  Return available character count.
//
int SI4707::sameAvailable(void)
{
  if (rxBufferIndex == rxBufferLength)
    return -1;

  else
    return rxBufferLength - rxBufferIndex;
}
//
//  Return received characters.
//
char SI4707::sameRead(void)
{
  char value = 0x00;

  if (rxBufferIndex < rxBufferLength)
    {
      value = rxBuffer[rxBufferIndex];
      rxBufferIndex++;
    }

  else
    {
      rxBufferIndex = rxBufferLength = 0;
      msgStatus |= SI4707_MSGUSD;
    }

  return value;
}
//
//  The SAME message is parsed here.
//
void SI4707::sameParse(void)
{
  if (!(msgStatus & SI4707_MSGAVL))                     //  If no message is Available, return
    return;

  samePlusIndex = 0;
  sameLocations = 0;
  sameDuration = 0;
  sameDay = 0;
  sameHour = 0;
  sameMinute = 0;

  uint8_t i = 0;

  sameOriginatorName[0] = rxBuffer[i + 1];
  sameOriginatorName[1] = rxBuffer[i + 2];
  sameOriginatorName[2] = rxBuffer[i + 3];
  sameOriginatorName[3] = 0x00;

  sameEventName[0] = rxBuffer[i + 5];
  sameEventName[1] = rxBuffer[i + 6];
  sameEventName[2] = rxBuffer[i + 7];
  sameEventName[3] = 0x00;

  for (i = 0; i < sizeof(rxBuffer); i++)         //  Look for the Plus Sign.
    {
      if (rxBuffer[i] == 0x2B)
        samePlusIndex = i;                       //  Found it.

      if (rxBuffer[i] >= 0x30 && rxBuffer[i] <= 0x39) //  If the value is ascii, strip off the upper bits.
        rxBuffer[i] = rxBuffer[i] & 0x0F;
    }

  if (samePlusIndex == 0)                        //  No Plus Sign found.
    return;

  for (i = 6; i < samePlusIndex; i++)            //  There are no sameLocationCodes past the samePlusIndex.
    {
      if (rxBuffer[i] == 0x2D)
        {
          sameLocationCodes[sameLocations] = 0;  //  Clear out any remaining data.
          sameLocationCodes[sameLocations] += rxBuffer[i + 1] * 100000UL;
          sameLocationCodes[sameLocations] += rxBuffer[i + 2] *  10000UL;
          sameLocationCodes[sameLocations] += rxBuffer[i + 3] *   1000UL;
          sameLocationCodes[sameLocations] += rxBuffer[i + 4] *    100UL;
          sameLocationCodes[sameLocations] += rxBuffer[i + 5] *     10UL;
          sameLocationCodes[sameLocations] += rxBuffer[i + 6] *      1UL;

          if (sameLocations > SI4707_SAME_LOCATION_CODES) //  SI4707_SAME_LOCATION_CODES (31) is the maximum allowed.
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

  sameHour +=   rxBuffer[samePlusIndex +  9] * 10;
  sameHour +=   rxBuffer[samePlusIndex + 10];
  sameMinute += rxBuffer[samePlusIndex + 11] * 10;
  sameMinute += rxBuffer[samePlusIndex + 12];

  for (i = 0; i < 8; i++)
    {
      if (rxBuffer[i + samePlusIndex + 14] == 0x2D || rxBuffer[i + samePlusIndex + 14] == 0x00)
        {
          sameCallSign[i] = 0x00;
          break;
        }

      sameCallSign[i] = rxBuffer[i + samePlusIndex + 14];
    }

  msgStatus |= (SI4707_MSGUSD | SI4707_MSGPAR);                // Set the status to show the message was successfully Parsed.
}
//
//  Flush the SAME receive data.
//
void SI4707::sameFlush(void)
{
  getSameStatus(SI4707_CLRBUF | SI4707_INTACK);

  for (uint8_t i = 0; i < SI4707_SAME_BUFFER_SIZE; i++)
    {
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
void SI4707::sameFill(const String &s)
{
  sameFlush();

  for (uint8_t i = 0; i < s.length(); i++)
    {
      rxBuffer[i] = s[i];
      rxConfidence[i] = SI4707_SAME_CONFIDENCE_THRESHOLD;
      sameLength++;
      if (sameLength == SI4707_SAME_BUFFER_SIZE)
        break;
    }
}
//
//  Write a single command.
//
void SI4707::writeCommand(uint8_t command)
{
  Wire.beginTransmission(SI4707_RADIO_ADDRESS);
  Wire.write(command);
  Wire.endTransmission();
  delay(SI4707_CMD_DELAY);
}
//
//  Write a single command byte.
//
void SI4707::writeByte(uint8_t command, uint8_t value)
{
  Wire.beginTransmission(SI4707_RADIO_ADDRESS);
  Wire.write(command);
  Wire.write(value);
  Wire.endTransmission();
  delay(SI4707_CMD_DELAY);
}
//
//  Write a single command word.
//
void SI4707::writeWord(uint8_t command, uint16_t value)
{
  Wire.beginTransmission(SI4707_RADIO_ADDRESS);
  Wire.write(command);
  Wire.write(0x00);
  Wire.write(highByte(value));
  Wire.write(lowByte(value));
  Wire.endTransmission();
  delay(SI4707_CMD_DELAY);
}
//
//  Write an address and mode byte.
//
void SI4707::writeAddress(uint8_t address, uint8_t mode)
{
  Wire.beginTransmission(SI4707_RADIO_ADDRESS);
  Wire.write(SI4707_WB_SAME_STATUS);
  Wire.write(mode);
  Wire.write(address);
  Wire.endTransmission();
  delay(SI4707_CMD_DELAY * 4);                          //  A SI4707_CLRBUF takes a fair amount of time!
}
//
//  Reads the current Status byte.
//
void SI4707::readStatus(void)
{
  readBurst(1);
}
//
//  Reads the number of bytes specified by quantity.
//
void SI4707::readBurst(int quantity)
{
  Wire.requestFrom(SI4707_RADIO_ADDRESS, quantity);
  delay(SI4707_CMD_DELAY);
  Wire.readBytes(response, quantity);
  intStatus = response[0];
  delay(SI4707_CMD_DELAY);
}

//
//
//
SI4707 Radio;
