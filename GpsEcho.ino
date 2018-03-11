// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code just echos whatever is coming from the GPS unit to the
// serial monitor, handy for debugging!
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>

// Connect the GPS Power pin to 5V (The USB pin will suffice if you have decent USB ports
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to RXD0/Pin 16
// Connect the GPS RX (receive) pin to TXD0/Pin 15

Adafruit_GPS GPS(&Serial1);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()  
{    
  // connect at 115200 so we can print the GPS output fast enough
  Serial.begin(115200);
  Serial.println("Adafruit GPS library M0 test!");

  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);
  
  // You can adjust which sentences to have the module emit, below
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  
  // Set the update rate
  // Note you must send both commands below to change both the output rate (how often the position
  // is written to the serial line), and the position fix rate.
  
  // 1 Hz update rate
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  // Note the position can only be updated at most 5 times a second so it will lag behind serial output.
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
  
  delay(1000);
}

// Interrupt code for ARM Cortex M0 (Feather M0)
//
// Largely cribbed from
// https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989

void TC3_Handler()
{
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) 
  {
    TC->INTFLAG.bit.MC0 = 1;
    // do GPS stuffs
    char c = GPS.read();
    if (GPSECHO && c) Serial.print(c);
  }
}

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

#define SyncWAIT while (GCLK->STATUS.bit.SYNCBUSY == 1)

void useInterrupt(boolean v) 
{
  if (v) 
  {
    // Setup 16-bit timer on TC3 for our use:
    REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3);
    SyncWAIT;
      
    TcCount16* TC = (TcCount16*) TC3;
    TC->CTRLA.reg &= ~TC_CTRLA_ENABLE; SyncWAIT;
    TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16; SyncWAIT;
    TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; SyncWAIT; // Reset timer counter on match
    TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024; SyncWAIT; // Prescalar CLK/1024
    
    // Interrupt every millisecond
    int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * 1000)) - 1;

    // Make sure the count is in a proportional position to where it was
    // to prevent any jitter or disconnect when changing the compare value.
    TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
    TC->CC[0].reg = compareValue;
    
    Serial.println(TC->COUNT.reg); Serial.println(TC->CC[0].reg); SyncWAIT;
    
    // Enable the compare interrupt
    TC->INTENSET.reg = 0;
    TC->INTENSET.bit.MC0 = 1;
  
    NVIC_EnableIRQ(TC3_IRQn);
  
    TC->CTRLA.reg |= TC_CTRLA_ENABLE; SyncWAIT;
  }
  else
  {
    // Logically, I should turn the timer off here...
  }
}


void loop()
{
   // do nothing! all reading and printing is done in the interrupt
}
