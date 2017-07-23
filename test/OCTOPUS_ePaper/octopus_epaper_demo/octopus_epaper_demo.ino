#include <stdint.h>
#include "SparkFunBME280.h"
//Library allows either I2C or SPI, so include both.
#include "Wire.h"

#include <SPI.h>
#include "EPD_drive.h"
#include "EPD_drive_gpio.h"
#include "Display_Lib.h"

// wiring WAVESHARE 1.54" to OCTOPUS
// BUSY - 12
// RST - 0
// DC - 15
// CS - 2
// CLK - 14
// DIN - 13
// GND - GND
// 3.3V - 3.3V
// NOTE: cut the trace in the middle of tracepad (2 pads on top of RGB1) - this will disconnect NeoPixels from GPIO13, which is needed for the ePaper
// you can resolder the trace anytime, when you want to reactivate the NeoPixel while not using the ePaper display

BME280 mySensor;


WaveShare_EPD EPD = WaveShare_EPD();
void setup() {

  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x77;
  mySensor.settings.runMode = 3; //Normal mode
  mySensor.settings.tStandby = 0;
  mySensor.settings.filter = 0;
  mySensor.settings.tempOverSample = 1;
  mySensor.settings.pressOverSample = 1;
  mySensor.settings.humidOverSample = 1;
  
  // put your setup code here, to run once:
   pinMode(CS,OUTPUT);
   pinMode(DC,OUTPUT);
   pinMode(RST,OUTPUT);
   pinMode(BUSY,INPUT);
   
  Serial.begin(115200);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  
  Serial.println("------------Clear full screen-----------------");
  EPD.Dis_Clear_full();
  //2.display init
  EPD.EPD_init_Part();
  driver_delay_xms(DELAYTIME);

  
  //Clear screen
  Serial.println("--------------Clear part screen------------");
  EPD.Dis_Clear_part();
  
  
  driver_delay_xms(DELAYTIME);
  
  //String
  
 EPD.Dis_String(10, 10, "Hello World!",12);
 EPD.Dis_String(10, 50, "#IoT-OCTOPUS",12);
 EPD.Dis_String(10, 70, "www.Fab-Lab.eu",12);
 EPD.Dis_String(10, 110, "first epaper wing",12);
 driver_delay_xms(DELAYTIME);

/*
 EPD.Dis_Drawing(0,0,(unsigned char *)klaus,200,200); //Line
 driver_delay_xms(DELAYTIME);
*/

 EPD.Dis_Drawing(0,0,(unsigned char *)octo,200,200); //Line
 driver_delay_xms(DELAYTIME);

}

void loop() {
  // put your main code here, to run repeatedly:
  // just doing something here:
  
  Serial.print("%RH: ");
  Serial.print(mySensor.readFloatHumidity(), 2);
  Serial.println(" %");

  delay(1000);
}
