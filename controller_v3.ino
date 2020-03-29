#include <ESP8266WiFi.h>
#include <Adafruit_BME680.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>

float dpPEEP = 0.0 ;
float dpMax = 0.0 ;
int HubMax = 0 ;
int Inspirationszeit = 0 ;
int Expirationszeit = 0 ;
bool motoron= false ;
String matrixausgabe_text  = " "; // Ausgabetext als globale Variable

volatile int matrixausgabe_index = 0;// aktuelle Position in Matrix

float pEnviro = 0.0 ;
// BME680 Lib written by Limor Fried & Kevin Townsend for Adafruit Industries, http://www.adafruit.com/products/3660
Adafruit_BME680 boschBME680; // Objekt Bosch Umweltsensor
int boschBME680_ready = 0;

int Startzeit = 0 ;
float p = 0.0 ;
float pmin = 0.0 ;
int HubSchritte = 0 ;
// Feather Adafruit Motor Shield v2 http:\www.adafruit.com/products/1438
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Treiber-Objekt
Adafruit_StepperMotor *myStepMotor1 = AFMS.getStepper(100,1); // Motor 1-Objekt, 100 Steps/Umdrehung
Adafruit_StepperMotor *myStepMotor2 = AFMS.getStepper(100,2); // Motor 2-Objekt
// Control Feather Motor Shield
void setMotorSteps(int mot, int steps, int speed) {
  int dir= RELEASE; 	
  if (steps > 0)
    dir = FORWARD;
  else {
    dir = BACKWARD;
    steps = - steps;
  }
  switch (mot) {
  case 1:
    myStepMotor1->setSpeed(speed);
    myStepMotor1->step(steps, dir, DOUBLE);
    if (steps==0) myStepMotor1->release();
    break;
  case 2:
    myStepMotor2->setSpeed(speed);
    myStepMotor2->step(steps, dir, DOUBLE);
    if (steps==0) myStepMotor2->release();
    break;
  }
}

float pmax = 0.0 ;
//--------------------------------------- https-GET
int httpsGET(String host, String cmd, String &antwort, const uint8_t fingerprint[] ) {
  std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
  String message = "https://"+host+cmd;
  client->setFingerprint(fingerprint);
  HTTPClient https;

  //Serial.print("[HTTPS] begin...\n");
  Serial.println(message);
  if (https.begin(*client, message)){  // HTTPS
    //Serial.print("[HTTPS] GET...\n");
    // start connection and send HTTP header
    int httpCode = https.GET();
    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      //Serial.printf("[HTTPS] GET... code: %d\n", httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
        String payload = https.getString();
        antwort = payload;
        Serial.println(payload);
      }
    } 
    else {
      Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
    }

    https.end();
  } 
  else {
    Serial.printf("[HTTPS] Unable to connect\n");
  }
}


void setup(){ // Einmalige Initialisierung
  Serial.begin(115200);
  Wire.begin(); // ---- Initialisiere den I2C-Bus 

  if (Wire.status() != I2C_OK) Serial.println("Something wrong with I2C");

  boschBME680_ready = boschBME680.begin(118);

  if (boschBME680_ready == 0) {
    while(1) { 
      Serial.println("BME680 nicht vorhanden - der alte Octopus nutzt BME280, ggf. Puzzleteile tauschen");
      delay(500);
    }
  }

  // Set up Bosch BME 680
  boschBME680.setTemperatureOversampling(BME680_OS_8X);
  boschBME680.setHumidityOversampling(BME680_OS_2X);
  boschBME680.setPressureOversampling(BME680_OS_4X);
  boschBME680.setIIRFilterSize(BME680_FILTER_SIZE_3);
  boschBME680.setGasHeater(320, 150); // 320*C for 150 ms

  Serial.println();
  AFMS.begin(); // Setup Feather-Shield 1.6KHz PWM

  Wire.setClock(400000L); // speed up i2c 
  randomSeed(analogRead(A0) + analogRead(A0) + analogRead(A0));
  dpPEEP = 1 ;

  dpMax = 20 ;

  HubMax = 100 ;

  Inspirationszeit = 5000 ;

  Expirationszeit = 3000 ;

  motoron = LOW ;

  //------------ WLAN initialisieren 
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  delay(100);
  Serial.print ("\nWLAN connect to:");
  Serial.print("YOURSSID");
  WiFi.begin("YOURSSID","YOURPWD");
  while (WiFi.status() != WL_CONNECTED) { // Warte bis Verbindung steht 
    delay(500); 
    Serial.print(".");
  };
  Serial.println ("\nconnected, meine IP:"+ WiFi.localIP().toString());
  matrixausgabe_text = " Meine IP:" + WiFi.localIP().toString();
  matrixausgabe_index=0;

  delay( 1000 );

  pEnviro = (boschBME680.readPressure()/100.+(0.0)) ;

}

void loop() { // Kontinuierliche Wiederholung 
  Inspiration();
  delay( 500 );
  Expiration();
  delay( 500 );
}

void Expiration()
{
  Startzeit = millis() ;
  p = ( (boschBME680.readPressure()/100.+(0.0)) - pEnviro ) ;
  pmin = 0 ;
  Serial.print("Expiration -start");
  Serial.println();
  Serial.print("P:"+String(String(p)));
  Serial.print(",BEEP:"+String(String(dpPEEP)));
  Serial.println();
  delay( p );
  while ( ( ( ( p ) >= ( dpPEEP ) ) && ( ( HubSchritte ) > ( 0 ) ) ) )
  {
    yield(); // Aufruf Scheduler, bedient WLAN-Stack
    if (motoron)
    {
      setMotorSteps(1,-5,5);    // Ausgabe an Motor 
    }
    HubSchritte = ( HubSchritte - 5 ) ;
    p = ( (boschBME680.readPressure()/100.+(0.0)) - pEnviro ) ;
    if (( ( p ) < ( pmin ) ))
    {
      pmin = p ;
    }
    Serial.print("T:"+String(String(( millis() - Startzeit ))));
    Serial.print(",S:"+String(String(HubSchritte)));
    Serial.println();
    Serial.print(",P:"+String(String(pmin)));
    Serial.println();
  }

  Serial.print("T:"+String(String(( millis() - Startzeit ))));
  Serial.print(",S:"+String(String(HubSchritte)));
  Serial.println();
  while ( ( ( ( millis() - Startzeit ) ) < ( Expirationszeit ) ) )
  {
    yield(); // Aufruf Scheduler, bedient WLAN-Stack
    delay( 1 );
  }

  while ( ( ( HubSchritte ) < ( 0 ) ) )
  {
    yield(); // Aufruf Scheduler, bedient WLAN-Stack
    if (motoron)
    {
      setMotorSteps(1,-5,5);    // Ausgabe an Motor 
    }
    HubSchritte = ( HubSchritte - 5 ) ;
  }

  Serial.print("Expiration -completed");
  Serial.println();
}

void Inspiration()
{
  Startzeit = millis() ;
  HubSchritte = 0 ;
  p = ( (boschBME680.readPressure()/100.+(0.0)) - pEnviro ) ;
  pmax = 0 ;
  Serial.print("Inspiration -start");
  Serial.println();
  while ( ( ( ( ( p ) < ( dpMax ) ) && ( ( HubSchritte ) < ( HubMax ) ) ) && ( ( ( millis() - Startzeit ) ) < ( Inspirationszeit ) ) ) )
  {
    yield(); // Aufruf Scheduler, bedient WLAN-Stack
    if (motoron)
    {
      setMotorSteps(1,5,5);    // Ausgabe an Motor 
    }
    HubSchritte = ( HubSchritte + 5 ) ;
    p = ( (boschBME680.readPressure()/100.+(0.0)) - pEnviro ) ;
    if (( ( p ) > ( pmax ) ))
    {
      pmax = p ;
    }
    Serial.print("T:"+String(String(( millis() - Startzeit ))));
    Serial.print(",P:"+String(String(p)));
    Serial.println();
    delay( p );
  }

  Serial.print("Inspriration -complete: maxP:"+String(String(pmax)));
  Serial.print("maxHub:"+String(String(HubSchritte)));
  Serial.println();
  while ( ( ( ( millis() - Startzeit ) ) < ( Inspirationszeit ) ) )
  {
    yield(); // Aufruf Scheduler, bedient WLAN-Stack
    delay( 1 );
  }

  delay( 1000 );

  { //Block------------------------------ sende Daten an Heruko (mit http GET) 
    Serial.println("\nHeruko update ");
    const uint8_t fingerprint[20] ={
      YOUR SSL FINGERPRINT   };
    String cmd = "/beatmungsgeraet?serialnumber="+ String("device123");
    String host = "YOURSALESFORCEORG.herokuapp.com";
    String antwort= " ";
    cmd = cmd +String("&volume="+String(( HubSchritte * 10 ))
      +"&alarm="+String(0)
      +"&SPO2="+String(( 98 - 	random( 12 ) ))
      +"&CO2="+String(( 550 - 	random( 70 ) ))
      +"&Humidity="+String(( 32 - 	random( 5 ) ))
      +"&heartbeat="+String(( 73 - 	random( 5 ) )));
    httpsGET(host,cmd,antwort,fingerprint);// und absenden 
  } // Blockende
}
