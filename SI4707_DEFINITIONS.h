/*
  SI4707_DEFINITIONS.h

  Si4707 Weather Band Receiver Commands and Property Definitions.

  Adapted for use by Arduino by Ray H. Dees & Richard Vogel.

*/
#ifndef SI4707_DEFINITIONS_h
#define SI4707_DEFINITIONS_h
//
//  Weather Band definitions.
//
#define SI4707_WB_CHANNEL_SPACING              0x0A        //  25 kHz.
#define SI4707_WB_MIN_FREQUENCY              0xFDC0        //  162.400 mHz.
#define SI4707_WB_MAX_FREQUENCY              0xFDFC        //  162.550 mHz.
//
//  Si4707 Command definitions.
//
#define SI4707_POWER_UP                        0x01        //  Powerup device.
#define SI4707_GET_REV                         0x10        //  Returns revision information on the device.
#define SI4707_POWER_DOWN                      0x11        //  Powerdown device.
#define SI4707_SET_PROPERTY                    0x12        //  Sets the value of a property.
#define SI4707_GET_PROPERTY                    0x13        //  Retrieves a property's value.
#define SI4707_GET_INT_STATUS                  0x14        //  Read interrupt status bits.
#define SI4707_PATCH_ARGS                      0x15        //  Reserved command used for firmware file downloads.
#define SI4707_PATCH_DATA                      0x16        //  Reserved command used for firmware file downloads.
#define SI4707_WB_TUNE_FREQ                    0x50        //  Selects the SI4707_WB tuning frequency.
#define SI4707_WB_TUNE_STATUS                  0x52        //  Queries the status of the previous SI4707_WB_TUNE_FREQ command.
#define SI4707_WB_RSQ_STATUS                   0x53        //  Queries the status of the Received Signal Quality (RSQ) of the current channel.
#define SI4707_WB_SAME_STATUS                  0x54        //  Returns SAME information for the current channel.
#define SI4707_WB_ASQ_STATUS                   0x55        //  Queries the status of the 1050 Hz alert tone.
#define SI4707_WB_AGC_STATUS                   0x57        //  Queries the status of the AGC.
#define SI4707_WB_AGC_OVERRIDE                 0x58        //  Enable or disable the AGC.
#define SI4707_GPIO_CTL                        0x80        //  Configures GPO as output or Hi-Z.
#define SI4707_GPIO_SET                        0X81        //  Sets GPO output level (low or high).
//
//  Si4707 Property definitions.
//
#define SI4707_GPO_IEN                       0x0001      //  Enables GPO2 interrupt sources.
#define SI4707_REFCLK_FREQ                   0x0201      //  Sets frequency of reference clock in Hz.
#define SI4707_REFCLK_PRESCALE               0x0202      //  Sets the prescaler value for RCLK input.
#define SI4707_RX_VOLUME                     0x4000      //  Sets the output volume.
#define SI4707_RX_HARD_MUTE                  0x4001      //  Mutes the audio output.
#define SI4707_WB_MAX_TUNE_ERROR             0x5108      //  Maximum change from the SI4707_WB_TUNE_FREQ to which the AFC will lock.
#define SI4707_WB_RSQ_INT_SOURCE             0x5200      //  Configures interrupts related to RSQ metrics.
#define SI4707_WB_RSQ_SNR_HIGH_THRESHOLD     0x5201      //  Sets high threshold for SNR interrupt.
#define SI4707_WB_RSQ_SNR_LOW_THRESHOLD      0x5202      //  Sets low threshold for SNR interrupt.
#define SI4707_WB_RSQ_RSSI_HIGH_THRESHOLD    0x5203      //  Sets high threshold for RSSI interrupt.
#define SI4707_WB_RSQ_RSSI_LOW_THRESHOLD     0x5204      //  Sets low threshold for RSSI interrupt.
#define SI4707_WB_VALID_SNR_THRESHOLD        0x5403      //  Sets SNR threshold to indicate a valid channel.
#define SI4707_WB_VALID_RSSI_THRESHOLD       0x5404      //  Sets RSSI threshold to indicate a valid channel.
#define SI4707_WB_SAME_INTERRUPT_SOURCE      0x5500      //  Configures SAME interrupt sources.
#define SI4707_WB_ASQ_INT_SOURCE             0x5600      //  Configures 1050 Hz alert tone interrupts.
//
//  Si4707 Power Up Command Arguments.
//
#define SI4707_WB                              0x03      //  Function, 3 = SI4707_WB receive.
#define SI4707_QUERY                           0x0F      //  Function, 15 = Query Library ID.
#define SI4707_XOSCEN                          0x10      //  Crystal Oscillator Enable.
#define SI4707_PATCH                           0x20      //  Patch Enable.
#define SI4707_GPO2EN                          0x40      //  GPO2 Output Enable.
#define SI4707_CTSEN                           0x80      //  CTS Interrupt Enable.
//
#define SI4707_OPMODE                          0x05      //  Application Setting, 5 = Analog L & R outputs.
//
// Si4707 Returned Interrupt Status Bits.
//
#define SI4707_STCINT                          0x01      //  Seek/Tune Complete Interrupt.
#define SI4707_ASQINT                          0x02      //  1050 Hz Alert Tone Interrupt.
#define SI4707_SAMEINT                         0x04      //  SAME Interrupt.
#define SI4707_RSQINT                          0x08      //  Received Signal Quality Interrupt.
#define SI4707_ERRINT                          0x40      //  Error Interrupt.
#define SI4707_CTSINT                          0x80      //  Clear To Send Interrupt.
//
//  Si4707 Status Register Masks.
//
#define SI4707_VALID                           0x01      //  Valid Channel.
#define SI4707_AFCRL                           0x02      //  AFC Rail Indicator.
//
#define SI4707_RSSILINT                        0x01      //  RSSI was Detected Low.
#define SI4707_RSSIHINT                        0x02      //  RSSI was Detected High.
#define SI4707_SNRLINT                         0x04      //  SNR was Detected Low.
#define SI4707_SNRHINT                         0x08      //  SNR was Detected High.
//
#define SI4707_HDRRDY                          0x01      //  SAME Header Ready was detected.
#define SI4707_PREDET                          0x02      //  SAME Preamble was Detected.
#define SI4707_SOMDET                          0x04      //  SAME Start Of Message was Detected.
#define SI4707_EOMDET                          0x08      //  SAME End Of Message was Detected.
//
#define SI4707_ALERTON                         0x01      //  Alert Tone has not been detected on since last SI4707_WB_TUNE_FREQ.
#define SI4707_ALERTOF                         0x02      //  Alert Tone has not been detected off since last SI4707_WB_TUNE_FREQ.
#define SI4707_ALERT                           0x01      //  Alert Tone is currently present.
//
//  Si4707 Interrupt Acknowledge Commands.
//
#define SI4707_CHECK                           0x00      //  Allows checking of status without clearing interrupt.
#define SI4707_INTACK                          0x01      //  If set, this bit clears the current interrupt.
#define SI4707_CLRBUF                          0x02      //  If set, the SAME buffer is cleared.
//
//  Si4707 Sources for GPO2/INT Interrupt pin.
//
#define SI4707_STCIEN                        0x0001      //  Seek/Tune Complete Interrupt Enable.
#define SI4707_ASQIEN                        0x0002      //  ASQ Interrupt Enable.
#define SI4707_SAMEIEN                       0x0004      //  SAME Interrupt Enable.
#define SI4707_RSQIEN                        0x0008      //  RSQ Interrupt Enable.
#define SI4707_ERRIEN                        0x0040      //  Error Interrupt Enable.
#define SI4707_CTSIEN                        0x0080      //  CTS Interrupt Enable.
#define SI4707_STCREP                        0X0100      //  Repeat SI4707_STCINT even if it is already set.
#define SI4707_ASQREP                        0x0200      //  Repeat SI4707_ASQINT even if it is already set.
#define SI4707_SAMEREP                       0x0400      //  Repeat SI4707_SAMEINT even if it is already set.
#define SI4707_RSQREP                        0x0800      //  Repeat SI4707_RSQINT even if it is already set.
//
#define SI4707_RSSILIEN                      0x0001      //  RSSI detect Low Interrupt Enable.
#define SI4707_RSSIHIEN                      0x0002      //  RSSI detect High Interrupt Enable.
#define SI4707_SNRLIEN                       0x0004      //  SNR detect Low Interrupt Enable.
#define SI4707_SNRHIEN                       0x0008      //  SNR detect High Interrupt Enable.
//
#define SI4707_HDRRDYIEN                     0x0001      //  SAME Header Ready Interrupt Enable.
#define SI4707_PREDETIEN                     0x0002      //  SAME Preamble Detected Interrupt Enable.
#define SI4707_SOMDETIEN                     0x0004      //  SAME Start Of Message Detected Interrupt Enable.
#define SI4707_EOMDETIEN                     0x0008      //  SAME End Of Message Detected Interrupt Enable.
//
#define SI4707_ALERTONIEN                    0x0001      //  Sets 1050 Hz tone on as source of ASQ Interrupt.
#define SI4707_ALERTOFIEN                    0x0002      //  Sets 1050 Hz tone off as source of ASQ Interrupt.
//
//  Si4707 GPO Control / Set Functions.
//
#define SI4707_GPO1OEN                         0x02      //  GPO1 Output Enable.
#define SI4707_GPO2OEN                         0x04      //  GPO2 Output Enable.  The use of GPO2 as an interrupt pin will override this.
#define SI4707_GPO3OEN                         0x08      //  GPO3 Output Enable.
#define SI4707_GPO1LEVEL                       0x02      //  Sets GPO1 High.
#define SI4707_GPO2LEVEL                       0x04      //  Sets GPO2 High.
#define SI4707_GPO3LEVEL                       0x08      //  Sets GPO3 High.
//
//  SAME Confidence Level Masks and Bit Shift Positions.
//
#define SI4707_SAME_STATUS_OUT_CONF0           0x03
#define SI4707_SAME_STATUS_OUT_CONF1           0x0C
#define SI4707_SAME_STATUS_OUT_CONF2           0x30
#define SI4707_SAME_STATUS_OUT_CONF3           0xC0
#define SI4707_SAME_STATUS_OUT_CONF4           0x03
#define SI4707_SAME_STATUS_OUT_CONF5           0x0C
#define SI4707_SAME_STATUS_OUT_CONF6           0x30
#define SI4707_SAME_STATUS_OUT_CONF7           0xC0
#define SI4707_SAME_STATUS_OUT_CONF0_SHFT         0
#define SI4707_SAME_STATUS_OUT_CONF1_SHFT         2
#define SI4707_SAME_STATUS_OUT_CONF2_SHFT         4
#define SI4707_SAME_STATUS_OUT_CONF3_SHFT         6
#define SI4707_SAME_STATUS_OUT_CONF4_SHFT         0
#define SI4707_SAME_STATUS_OUT_CONF5_SHFT         2
#define SI4707_SAME_STATUS_OUT_CONF6_SHFT         4
#define SI4707_SAME_STATUS_OUT_CONF7_SHFT         6
//
#endif