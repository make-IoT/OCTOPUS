/* Disclaimer IoT-Werkstatt CC 4.0 BY NC SA 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. For Ardublock see the
 GNU General Public License for more details. */
#include <lmic.h>
#include <hal/hal.h>
#define LORA_TX_INTERVAL 10
#include <bsec.h>
#include <Wire.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#define LORA_DEEPSLEEP

// LoraWAN Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
// (c) 2018 Terry Moore, MCCI
// https://github.com/mcci-catena/arduino-lmic
// -------- LoRa PinMapping FeatherWing Octopus
const lmic_pinmap lmic_pins = {  
  .nss = 2,                            // Connected to pin D
  .rxtx = LMIC_UNUSED_PIN,             // For placeholder only, Do not connected on RFM92/RFM95
  .rst = LMIC_UNUSED_PIN,              // Needed on RFM92/RFM95? (probably not) D0/GPIO16 
  .dio = {
    15, 15, LMIC_UNUSED_PIN         }
};

static const u1_t PROGMEM DEVEUI[8]={
  0xE3,0x03,0x05,0xD0,0x7E,0xD5,0xB3,0x70};
void os_getDevEui (u1_t* buf) { 
  memcpy_P(buf, DEVEUI, 8);
}

static const u1_t PROGMEM APPEUI[8]={
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void os_getArtEui (u1_t* buf) { 
  memcpy_P(buf, APPEUI, 8);
}

static const u1_t PROGMEM APPKEY[16]={
  0xDD,0xF5,0x97,0x0C,0x90,0x10,0x24,0x46,0xEB,0x7F,0x94,0xDF,0x10,0x74,0xC6,0x17};
void os_getDevKey (u1_t* buf) {  
  memcpy_P(buf, APPKEY, 16);
};

volatile int LoRaWAN_Tx_Ready   = 0; // Flag for Tx Send 
long         LoRaWAN_ms_Wakeup  = 0; // ms at start message
long         LoRaWAN_ms_EmExit  = 0; // max. ms  

int LoRaWAN_Rx_Payload = 0 ;
int LoRaWAN_Rx_Port = 0 ;
// Berechne CRC-Prüfsumme für RTC-RAM 
uint32_t RTCcalculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}
//--------------------------  Load/Store LoRa LMIC to RTC-Mem 
void LoadLMICFromRTC() {
  lmic_t RTC_LMIC;
  uint32_t crcOfData;
  if (sizeof(lmic_t) <= 512) {
    ESP.rtcUserMemoryRead(1, (uint32_t*) &RTC_LMIC, sizeof(RTC_LMIC));
    ESP.rtcUserMemoryRead(0, (uint32_t*) &crcOfData, sizeof(crcOfData));
    uint32_t crcOfData_RTC = RTCcalculateCRC32((uint8_t*) &RTC_LMIC, sizeof(RTC_LMIC));
    if (crcOfData != crcOfData_RTC) {
      Serial.println("CRC32 in RTC memory doesn't match CRC32 of data. Data is probably invalid!");
    } 
    else {
      Serial.print(F("load LMIC from RTC, FrameCounter =  "));
      LMIC = RTC_LMIC;
      Serial.println(LMIC.seqnoUp);     
    }  
  } 
  else {
    Serial.println(F("sizelimit RTC-Mem, #define LMIC_ENABLE_long_messages in config.h"));
  }
} 

void SaveLMICToRTC(int deepsleep_sec) {
  if (sizeof(lmic_t) <= 512) {
    Serial.println(F("Save LMIC to RTC and deepsleep"));
    unsigned long now = millis();
    // EU Like Bands
#if defined(CFG_LMIC_EU_like)
    // Serial.println(F("Reset CFG_LMIC_EU_like band avail"));
    for (int i = 0; i < MAX_BANDS; i++)
    {
      ostime_t correctedAvail = LMIC.bands[i].avail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
      if (correctedAvail < 0)
      {
        correctedAvail = 0;
      }
      LMIC.bands[i].avail = correctedAvail;
    }

    LMIC.globalDutyAvail = LMIC.globalDutyAvail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
    if (LMIC.globalDutyAvail < 0)
    {
      LMIC.globalDutyAvail = 0;
    }
#else
    //Serial.println(F("No DutyCycle recalculation function!"));
#endif
    // Write to RTC
    uint32_t crcOfData = RTCcalculateCRC32((uint8_t*) &LMIC, sizeof(LMIC));
    ESP.rtcUserMemoryWrite(1, (uint32_t*) &LMIC, sizeof(LMIC));
    ESP.rtcUserMemoryWrite(0, (uint32_t*) &crcOfData, sizeof(crcOfData));
  } 
  else {
    Serial.println(F("sizelimit RTC-Mem, #define LMIC_ENABLE_long_messages in config.h"));
  }
} 
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    LoRaWAN_Tx_Ready =  !(LMIC.opmode & OP_TXDATA); // otherwise joined without TX blocks queue
    break;
    /*
        || This event is defined but not used in the code. No
     || point in wasting codespace on it.
     ||
     || case EV_RFU1:
     ||     Serial.println(F("EV_RFU1"));
     ||     break;
     */
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen) {
      Serial.println(F("Received "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
      LoRaWAN_Rx_Payload = 0; // #kgo Payload IoT-Werkstatt
      LoRaWAN_Rx_Port    = LMIC.frame[LMIC.dataBeg-1];              
      for (int i = 0;i<LMIC.dataLen;i++) { 
        Serial.println(LMIC.frame[i+ LMIC.dataBeg],HEX);
        LoRaWAN_Rx_Payload = 256*LoRaWAN_Rx_Payload+LMIC.frame[i+ LMIC.dataBeg];
      }
#ifdef LORA_DOWNLINK_ENABLE 
      LoRaWAN_DownlinkCallback();
#endif 
    }
    LoRaWAN_Tx_Ready =  !(LMIC.opmode & OP_TXDATA);
    // Schedule next transmission
    // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
    /*
        || This event is defined but not used in the code. No
     || point in wasting codespace on it.
     ||
     || case EV_SCAN_FOUND:
     ||    Serial.println(F("EV_SCAN_FOUND"));
     ||    break;
     */
  case EV_TXSTART:
    Serial.println(F("EV_TXSTART"));
    break;
  case EV_TXCANCELED:
    Serial.println(F("EV_TXCANCELED"));
    break;
  case EV_RXSTART:
    /* do not print anything -- it wrecks timing */
    break;
  case EV_JOIN_TXCOMPLETE:
    Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
    break;
  default:
    Serial.print(F("Unknown event: "));
    Serial.println((unsigned) ev);
    break;
  }
}
// -- initialize LoraWAN LMIC structure
void LoRaWAN_Start(int fromRTCMem) { // using OTA-Communication 
  os_init();             // LMIC LoraWAN
  LMIC_reset();          // Reset the MAC state 
  LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100); // timing difference esp clock
  if  (fromRTCMem) { 
#ifdef LORA_DEEPSLEEP 
    LoadLMICFromRTC(); // restart from deepsleep, get LMIC state from RTC 
#endif
  } // continue runing state-maschine
}

/* 
 Bosch BSEC Lib, https://github.com/BoschSensortec/BSEC-Arduino-library
 The BSEC software is only available for download or use after accepting the software license agreement.
 By using this library, you have agreed to the terms of the license agreement: 
 https://ae-bst.resource.bosch.com/media/_tech/media/bsec/2017-07-17_ClickThrough_License_Terms_Environmentalib_SW_CLEAN.pdf */
Bsec iaqSensor;     // Create an object of the class Bsec 
Ticker Bsec_Ticker; // schedule cyclic update via Ticker 
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_28d_2d_iaq_50_200/bsec_iaq.txt"
};

// ------------------------   Helper functions Bosch Bsec - Lib 
void checkIaqSensorStatus(void)
{ 
  String output; 
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      for (;;) {
        Serial.println(output);
        delay(500);
      } // Halt in case of failure 
    } 
    else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      for (;;){
        Serial.println(output);
        delay(500);
      }  // Halt in case of failure 
    } 
    else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

// Housekeeping: scheduled update using ticker-lib
void iaqSensor_Housekeeping(){  // get new data 
  iaqSensor.run();
}

extern "C" {  // zur Nutzung der speziellen ESP-Befehle wie Deep Sleep
#include "user_interface.h"
}


void setup(){ // Einmalige Initialisierung
  WiFi.forceSleepBegin(); // Wifi off
  pinMode( 0 , OUTPUT);
  Serial.begin(115200);
  LoRaWAN_Start(true); // Prepare LMIC-Engine

    Wire.begin(); // ---- Initialisiere den I2C-Bus 

  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  String output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();  
  iaqSensor_Housekeeping();
  Bsec_Ticker.attach_ms(3000, iaqSensor_Housekeeping);

}

void loop() { // Kontinuierliche Wiederholung 
  digitalWrite( 0 , LOW );

  { //Block------------------------------ send data to network
    int port = 10;
    static uint8_t mydata[6];
    int wert=round(analogRead(0)*1000);
    mydata[0] = wert >> 16; 
    mydata[1] = wert >> 8; 
    mydata[2] = wert ;
    wert=round(iaqSensor.temperature*1000);
    mydata[3] = wert >> 16; 
    mydata[4] = wert >> 8; 
    mydata[5] = wert ;
    int Retry = 3, tout = 5000,  ok=-1;
    while (Retry > 0) {
      // Check if there is not a current TX/RX job running, wait until finished
      if (!((LMIC.opmode & OP_TXRXPEND) || (LMIC.opmode & OP_TXDATA) || (LMIC.opmode & OP_POLL) || (LMIC.opmode & OP_JOINING))) {
        //LoRaWAN_Tx_Ready = 0;
        ok = LMIC_setTxData2(port, mydata, sizeof(mydata), 0);     // Sende  
        if (ok!=0) {
          Serial.println(F("------------------------  setTxData: "));
          Serial.print(ok);
        } 
        else {
          Serial.println(F("Packet queued "));
          while (LMIC.opmode & OP_TXDATA) { //(!LoRaWAN_Tx_Ready) {            
            yield();
            os_runloop_once();
          }      
          Serial.println(F("Packet send "));
          Retry = 0;
        }
      }
      if (ok!=0) {
        Retry=Retry-1;
        Serial.println(F("Retry after timeout"));
        long m = millis();
        while ((millis()-m) < tout) {
          yield();
          os_runloop_once();
        }
      }
    } 

    if ((LMIC.opmode & OP_TXRXPEND) || (LMIC.opmode & OP_TXDATA) || (LMIC.opmode & OP_POLL) || (LMIC.opmode & OP_JOINING)) {
      Serial.print(F("some MAC-TXRX activ, mode = "));
      Serial.println(LMIC.opmode,HEX);
      while ((LMIC.opmode & OP_TXRXPEND) || (LMIC.opmode & OP_TXDATA) || (LMIC.opmode & OP_POLL) || (LMIC.opmode & OP_JOINING)) {
        yield();
        os_runloop_once();
      }
    }
    Serial.println(F("Tx Job finished"));
  } // Block 
  delay( 5000 );
  digitalWrite( 0 , HIGH );
  if (os_queryTimeCriticalJobs(ms2osticks(60000))) { 
    Serial.println("busywaiting for criticalJobs");
    while (os_queryTimeCriticalJobs(ms2osticks(60000))) { 
      yield();  
      os_runloop_once();
    }
  }
  SaveLMICToRTC(60000/1000); // Save LMIC-State 
  WiFi.disconnect( true );
  delay(2); // Wifi Off 
  ESP.deepSleep( (long)60000*1000UL,WAKE_RF_DISABLED);//Tiefschlaf, danach Reset und von vorn
  os_runloop_once(); // LORA LMIC Housekeeping
} //end loop
