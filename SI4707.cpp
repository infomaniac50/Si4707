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
uint16_t channel = WB_MIN_FREQUENCY;
float frequency;
uint16_t volume = RADIO_VOLUME;
uint8_t mute = OFF;
uint8_t rssi;
uint8_t snr;
int freqoff;
uint8_t power = OFF;
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
uint8_t sameWat = 0x02;
uint8_t response[15];
//
//  Timer 1 Variables.
//
volatile uint8_t sreg;
volatile uint8_t timer;
//
//  Static Class Variables.
//
uint8_t SI4707::sameConf[8];
char SI4707::sameData[8];
uint8_t SI4707::rxConfidence[SAME_BUFFER_SIZE];
char SI4707::rxBuffer[SAME_BUFFER_SIZE];  
//
uint8_t SI4707::rxBufferIndex;
uint8_t SI4707::rxBufferLength;
//
// Begin using the Si4707.
//
void SI4707::begin(void)
{
  begin(RST);
}

void SI4707::begin(uint16_t _reset)
{
  NO_INTERRUPTS();                               //  Disable interrupts.
    
  reset = _reset;
  pinMode(reset, OUTPUT);                          //  Setup the reset pin.
  digitalWrite(reset, LOW);                        //  Reset the Si4707. 
  delay(CMD_DELAY); 
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

#if F_CPU == 16000000UL                          //  16 MHz clock.
  OCR1A = 0x3D08;                                //  Compare Match at 1 second.
#elif F_CPU == 8000000UL                         //  8 MHz clock.
  OCR1A = 0x1E83;                                //  Compare Match at 1 second.
#endif

  TIMSK1 = 0x00;                                 //  Reset TIMSK1.
  TIMSK1 |= (1 << OCIE1A);                       //  Timer 1 Compare Match A Interrupt Enable. 
    
  TCCR2B = 0x00;                                 //  Stop Timer 2.  
    
  INTERRUPTS();                                  // Enable Global Interrupts.
  
  Wire.begin();
}  
//
//  Powers up the Si4707.
//  
void SI4707::on(void)
{
  if (power)
    return;
  
  Wire.beginTransmission(RADIO_ADDRESS);
  Wire.write(POWER_UP);
  Wire.write(GPO2EN | XOSCEN | WB);
  Wire.write(OPMODE);
  Wire.endTransmission();

  delay(PUP_DELAY);  
    
  readStatus();
   
  power = ON;  
}    
//
//  Gets the revision of the Si4707.
//
void SI4707::getRevision(void)
{
  writeCommand(GET_REV);
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
      
  Wire.beginTransmission(RADIO_ADDRESS);
  Wire.write(POWER_UP);
  Wire.write(GPO2EN | PATCH | XOSCEN | WB);
  Wire.write(OPMODE);
  Wire.endTransmission();

  delay(PUP_DELAY);
  
  readStatus();

  for (i = 0; i < sizeof(SI4707_PATCH_DATA); i += 8)
    {
      Wire.beginTransmission(RADIO_ADDRESS);
      
      for (j = 0; j < 8; j++)
        Wire.write(pgm_read_byte(&(SI4707_PATCH_DATA[i + j])));
      
      Wire.endTransmission();
      delay(PROP_DELAY);
      readStatus();
    }
  
  power = ON;    
}
//
//  Powers down the Si4707.
//
void SI4707::off()
{
  if (!power)
    return;
  
  writeCommand(POWER_DOWN);
  power = OFF;
  delay(CMD_DELAY);
}
//
//  End using the Si4707.
//
void SI4707::end(void)
{
  off();
  digitalWrite(reset, LOW);
  delay(CMD_DELAY); 
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
  writeWord(WB_TUNE_FREQ, channel);
  delay(TUNE_DELAY);
  intStatus |= INTAVL;
}
//
//  Scans for the best frequency based on RSSI.
//
void SI4707::scan(void)
{
  uint16_t i;
  uint16_t best_channel;
  uint8_t  best_rssi = 0x00;
  
  setMute(ON);
  
  for (i = WB_MIN_FREQUENCY; i <= WB_MAX_FREQUENCY; i += WB_CHANNEL_SPACING)
    {
      channel = i;
      tune();
      getTuneStatus(INTACK);
    
      if (rssi > best_rssi)
        {    
          best_rssi = rssi;
          best_channel = channel;
        }
    }
  
  channel = best_channel;
  tune();
  setMute(OFF);
  intStatus |= INTAVL;
}
//
//  Returns the current Interrupt Status.
//
uint8_t SI4707::getIntStatus(void)
{
  writeCommand(GET_INT_STATUS);
  readStatus();
  intStatus = response[0];
  
  return intStatus;
}	 
//
//  Gets the current Tune Status.
//
void SI4707::getTuneStatus(uint8_t mode)
{
  writeByte(WB_TUNE_STATUS, mode);
  
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
  writeByte(WB_RSQ_STATUS, mode);
  
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
      
  if (!(sameStatus & HDRRDY))                    //  If no HDRRDY, return.
    return;
    
  TIMER1_START();                                //  Start/Re-start the 6 second timer.
  
  sameHeaderCount++;
      
  if (sameHeaderCount >= 3)                      //  If this is the third Header, set msgStatus to show that it needs to be purged after usage.
    msgStatus |= MSGPUR;
  
  if (sameLength < SAME_MIN_LENGTH)              //  Don't process messages that are too short to be valid.
    return;
  
  for (i = 0; i < sameLength && i < SAME_BUFFER_SIZE; i += 8)  
    {
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
  
  msgStatus |= MSGAVL;
  
  for (i = 0; i < sameLength; i++)
    {
      if (rxConfidence[i] > SAME_CONFIDENCE_THRESHOLD)
        rxConfidence[i] = SAME_CONFIDENCE_THRESHOLD;
                       
      if (rxConfidence[i] < SAME_CONFIDENCE_THRESHOLD)
        {
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
//  Gets the current ASQ Status.
//
void SI4707::getAsqStatus(uint8_t mode)
{
  writeByte(WB_ASQ_STATUS, mode);
  
  readBurst(3);
  
  asqStatus = response[1];
}
//
//  Gets the current AGC Status.
//
void SI4707::getAgcStatus(void)
{
  writeCommand(WB_AGC_STATUS);
  
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
     
  setProperty(RX_VOLUME, volume);
}
//
//  Sets the current Mute state.
//
void SI4707::setMute(uint8_t value)
{
  switch (value)
    {
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
void SI4707::setProperty(uint16_t property, uint16_t value)
{
  Wire.beginTransmission(RADIO_ADDRESS);
  Wire.write(SET_PROPERTY);
  Wire.write(uint8_t(0x00));
  Wire.write(highByte(property));
  Wire.write(lowByte(property));
  Wire.write(highByte(value));
  Wire.write(lowByte(value));
  Wire.endTransmission();
  delay(PROP_DELAY);
}
//
//  Returns a specified property value.
//
uint16_t SI4707::getProperty(uint16_t property)
{
  uint16_t value = 0;
  
  writeWord(GET_PROPERTY, property);
  
  readBurst(3);
  
  value |= (response[2] << 8 | response[3]);
  
  return value;
}
//
//  Controls a specified GPIO.
//
void SI4707::gpioControl(uint8_t value)
{
  writeByte(GPIO_CTL, value);
}
//
//  Sets a specified GPIO.
//
void SI4707::gpioSet(uint8_t value)
{
  writeByte(GPIO_SET, value);
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
      msgStatus |= MSGUSD;
    }  
    
  return value;
}
//
//  The SAME message is parsed here.
//
void SI4707::sameParse(void)
{
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
  
  for (i = 0; i < 8; i++)
    {
      if (rxBuffer[i + samePlusIndex + 14] == 0x2D || rxBuffer[i + samePlusIndex + 14] == 0x00)
        {
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
void SI4707::sameFlush(void)
{
  TIMER1_STOP();
  
  getSameStatus(CLRBUF | INTACK);
  
  for (uint8_t i = 0; i < SAME_BUFFER_SIZE; i++)
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
      rxConfidence[i] = SAME_CONFIDENCE_THRESHOLD;
      sameLength++;
      if (sameLength == SAME_BUFFER_SIZE)
        break;
    }  
}
//
//  Write a single command.
//
void SI4707::writeCommand(uint8_t command)
{
  Wire.beginTransmission(RADIO_ADDRESS);
  Wire.write(command);
  Wire.endTransmission();
  delay(CMD_DELAY);
}
//
//  Write a single command byte.
//
void SI4707::writeByte(uint8_t command, uint8_t value)
{
  Wire.beginTransmission(RADIO_ADDRESS);
  Wire.write(command);
  Wire.write(value);
  Wire.endTransmission();
  delay(CMD_DELAY);
}
//
//  Write a single command word.
//
void SI4707::writeWord(uint8_t command, uint16_t value)
{
  Wire.beginTransmission(RADIO_ADDRESS);
  Wire.write(command);
  Wire.write(0x00);
  Wire.write(highByte(value));
  Wire.write(lowByte(value));
  Wire.endTransmission();
  delay(CMD_DELAY);
}
//
//  Write an address and mode byte.
//
void SI4707::writeAddress(uint8_t address, uint8_t mode)
{
  Wire.beginTransmission(RADIO_ADDRESS);
  Wire.write(WB_SAME_STATUS);
  Wire.write(mode);
  Wire.write(address);
  Wire.endTransmission();
  delay(CMD_DELAY * 4);                          //  A CLRBUF takes a fair amount of time!    
}
//
//  Reads the current Status byte.
//
void SI4707::readStatus(void)
{
  Wire.requestFrom(RADIO_ADDRESS, 1);
  delay(CMD_DELAY);
  if(Wire.available() > 0)
    response[0] = Wire.read();
  delay(CMD_DELAY);  
}
//
//  Reads the number of bytes specified by quantity.
//
void SI4707::readBurst(int quantity)
{
  uint8_t i = 0x00;
  
  Wire.requestFrom(RADIO_ADDRESS, quantity);
  delay(CMD_DELAY);
  
  while(Wire.available() > 0)
    {
      response[i] =  Wire.read(); 
      i++;
    }
  
  intStatus = response[0];
  delay(CMD_DELAY);
}
//
//  Interrupt 0 or 4 Service Routine - Triggered on the Falling edge.
//
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)     //  Interrupt 0.
ISR(INT0_vect)
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)  //  Interrupt 4.
ISR(INT4_vect)
#endif
{
  intStatus |= INTAVL;
}
//
//  Timer 1 Compare Match A Interrupt Service Routine -  Increments every 1 second.
//
ISR(TIMER1_COMPA_vect, ISR_NOBLOCK)
{
  sreg = SREG;
  
  timer++;
  
  if (timer >= SAME_TIME_OUT)
    Radio.sameFlush();
  
  SREG = sreg;
}  
//
//
//
SI4707 Radio;
