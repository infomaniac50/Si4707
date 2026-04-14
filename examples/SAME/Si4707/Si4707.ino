/*

  Si4707 Basic Demonstration Program.

  16 JUN 2013

  Note:

  You must set your own startup frequency in setup().
  You must enable the interrupts that you want in setup().

*/
#include "SI4707.h"
#include "Wire.h"
//
//  Global Variables.
//
byte function = 0x00;           //  Function to be performed.

//
//  Prints the Function Menu.
//
void showMenu()
{
  Serial.println();
  Serial.println(F("Display this menu =\t 'h' or '?'"));
  Serial.println(F("Channel down =\t\t 'd'"));
  Serial.println(F("Channel up =\t\t 'u'"));
  Serial.println(F("Scan =\t\t\t 's'"));
  Serial.println(F("Volume - =\t\t '-'"));
  Serial.println(F("Volume + =\t\t '+'"));
  Serial.println(F("Mute / Unmute =\t\t 'm'"));
  Serial.println(F("On / Off =\t\t 'o'"));
  Serial.println();
}
//
//  Status bits are processed here.
//
void getStatus()
{
  Radio.getIntStatus();

  if (intStatus & SI4707_STCINT)
  {
    Radio.getTuneStatus(SI4707_INTACK);  //  Using SI4707_INTACK clears SI4707_STCINT, SI4707_CHECK preserves it.
    Serial.print(F("FREQ: "));
    Serial.print(frequency, 3);
    Serial.print(F("  RSSI: "));
    Serial.print(rssi);
    Serial.print(F("  SNR: "));
    Serial.println(snr);
    Radio.sameFlush();             //  This should be done after any tune function.
    //intStatus |= SI4707_RSQINT;         //  We can force it to get rsqStatus on any tune.
  }

  if (intStatus & SI4707_RSQINT)
  {
    Radio.getRsqStatus(SI4707_INTACK);
    Serial.print(F("RSSI: "));
    Serial.print(rssi);
    Serial.print(F("  SNR: "));
    Serial.print(snr);
    Serial.print(F("  FREQOFF: "));
    Serial.println(freqoff);
  }

  if (intStatus & SI4707_SAMEINT)
  {
    Radio.getSameStatus(SI4707_INTACK);

    if (sameStatus & SI4707_EOMDET)
    {
      Radio.sameFlush();
      Serial.println(F("EOM detected."));
      Serial.println();
      //  More application specific code could go here. (Mute audio, turn something on/off, etc.)
      return;
    }

    if (msgStatus & SI4707_MSGAVL && (!(msgStatus & SI4707_MSGUSD)))  // If a message is available and not already used,
      Radio.sameParse();                                // parse it.

    if (msgStatus & SI4707_MSGPAR)
    {
      msgStatus &= ~SI4707_MSGPAR;                         // Clear the parse status, so that we don't print it again.
      Serial.print(F("Originator: "));
      Serial.println(sameOriginatorName);
      Serial.print(F("Event: "));
      Serial.println(sameEventName);
      Serial.print(F("Locations: "));
      Serial.println(sameLocations);
      Serial.print(F("Location Codes: "));

      for (int i = 0; i < sameLocations; i++)
      {
        Serial.print(sameLocationCodes[i]);
        Serial.print(' ');
      }

      Serial.println();
      Serial.print(F("Duration: "));
      Serial.println(sameDuration);
      Serial.print(F("Day: "));
      Serial.println(sameDay);
      Serial.print(F("Time: "));
      if (sameHour < 10) {
        Serial.print('0');
      }
      Serial.print(sameHour);
      Serial.print(':');
      if (sameMinute < 10) {
        Serial.print('0');
      }
      Serial.print(F("Callsign: "));
      Serial.println(sameCallSign);
      Serial.println();
    }

    if (msgStatus & SI4707_MSGPUR)  //  Signals that the third header has been received.
      Radio.sameFlush();
  }

  if (intStatus & SI4707_ASQINT)
  {
    Radio.getAsqStatus(SI4707_INTACK);

    if (sameWat == asqStatus)
      return;

    if (asqStatus == 0x01)
    {
      Radio.sameFlush();
      Serial.println(F("WAT is on."));
      Serial.println();
      //  More application specific code could go here.  (Unmute audio, turn something on/off, etc.)
    }

    if (asqStatus == 0x02)
    {
      Serial.println(F("WAT is off."));
      Serial.println();
      //  More application specific code could go here.  (Mute audio, turn something on/off, etc.)
    }

    sameWat = asqStatus;
  }

  if (intStatus & SI4707_ERRINT)
  {
    intStatus &= ~SI4707_ERRINT;
    Serial.println(F("An error occured!"));
    Serial.println();
  }
}
//
//  Functions are performed here.
//
void getFunction()
{
  function = Serial.read();

  switch (function)
  {
    case 'h':
    case '?':
      showMenu();
      break;

    case 'd':
      if (channel <= SI4707_WB_MIN_FREQUENCY)
        break;
      Serial.println(F("Channel down."));
      channel -= SI4707_WB_CHANNEL_SPACING;
      Radio.tune();
      break;

    case 'u':
      if (channel >= SI4707_WB_MAX_FREQUENCY)
        break;
      Serial.println(F("Channel up."));
      channel += SI4707_WB_CHANNEL_SPACING;
      Radio.tune();
      break;

    case 's':
      Serial.println(F("Scanning....."));
      Radio.scan();
      break;

    case '-':
      if (volume <= 0x0000)
        break;
      volume--;
      Radio.setVolume(volume);
      Serial.print(F("Volume: "));
      Serial.println(volume);
      break;

    case '+':
      if (volume >= 0x003F)
        break;
      volume++;
      Radio.setVolume(volume);
      Serial.print(F("Volume: "));
      Serial.println(volume, DEC);
      break;

    case 'm':
      if (mute)
      {
        Radio.setMute(SI4707_OFF);
        Serial.println(F("Mute: Off"));
        break;
      }
      else
      {
        Radio.setMute(SI4707_ON);
        Serial.println(F("Mute: On"));
        break;
      }

    case 'o':
      if (power)
      {
        Radio.off();
        Serial.println(F("Radio powered off."));
        break;
      }
      else
      {
        Radio.on();
        Serial.println(F("Radio powered on."));
        Radio.tune();
        break;
      }

    default:
      break;
  }

  Serial.flush();
  function = 0x00;
}
//
//  Simple Hex print utility - Prints a Byte with a leading zero and trailing space.
//
void printHex(byte value)
{
  Serial.print(F("0x"));
  Serial.print(value >> 4 & 0x0F, HEX);
  Serial.print(value >> 0 & 0x0F, HEX);
  Serial.print("  ");
}

//
//  Setup Loop.
//
void setup()
{
  delay(100);
  Serial.begin(115200);
  delay(100);
  Serial.println(F("Starting up the Si4707......."));
  Serial.println();
  delay(1000);
  showMenu();
  delay(1000);
  Radio.begin();
  Radio.patch();          //  Use this one to to include the 1050 Hz patch.
  //Radio.on();           //  Use this one if not using the patch.
  //Radio.getRevision();  //  Only captured on the logic analyzer - not displayed.
  //
  //  All useful interrupts are enabled here.
  //
  Radio.setProperty(SI4707_GPO_IEN, (SI4707_CTSIEN | SI4707_ERRIEN | SI4707_RSQIEN | SI4707_SAMEIEN | SI4707_ASQIEN | SI4707_STCIEN));
  //
  //  RSQ Interrupt Sources.
  //
  Radio.setProperty(SI4707_WB_RSQ_SNR_HIGH_THRESHOLD, 0x007F);   // 127 dBuV for testing..want it high
  Radio.setProperty(SI4707_WB_RSQ_SNR_LOW_THRESHOLD, 0x0001);    // 1 dBuV for testing
  Radio.setProperty(SI4707_WB_RSQ_RSSI_HIGH_THRESHOLD, 0x004D);  // -30 dBm for testing
  Radio.setProperty(SI4707_WB_RSQ_RSSI_LOW_THRESHOLD, 0x0007);   // -100 dBm for testing
  //Radio.setProperty(SI4707_WB_RSQ_INT_SOURCE, (SI4707_SNRHIEN | SI4707_SNRLIEN | SI4707_RSSIHIEN | SI4707_RSSILIEN));
  //
  //  SAME Interrupt Sources.
  //
  Radio.setProperty(SI4707_WB_SAME_INTERRUPT_SOURCE, (SI4707_EOMDETIEN | SI4707_HDRRDYIEN));
  //
  //  ASQ Interrupt Sources.
  //
  Radio.setProperty(SI4707_WB_ASQ_INT_SOURCE, (SI4707_ALERTOFIEN | SI4707_ALERTONIEN));
  //
  //  Tune to the desired frequency.
  //
  Radio.tune(162550);  //  6 digits only.
}

//
//  Main Loop.
//
void loop() // run over and over
{
  if (intStatus & SI4707_INTAVL)
    getStatus();

  if (Serial.available() > 0)
    getFunction();
}