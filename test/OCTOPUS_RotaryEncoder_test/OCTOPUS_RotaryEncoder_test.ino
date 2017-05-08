// RotaryEncoder
// This demonstrates the use of the custom Task object feature of Task library
// It will instance one custom RotaryEncoderTask to monitor three pins that a 
// rotary encoder is connected to and make call backs when they change state;
// the press button will have debouce and auto repeat support;
// the rotary will tell direction and keep track of "clicks" rotated
// This requires a standard rotary encoder be attached to the defined pins,
//
// Usefull information will be sent to the serial monitor 
//
// make sure that you connect your rotary encoder correctly, include gnd and Vcc (+)

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            13

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      2

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);


// nothing special about these pins, just make sure they don't collide with
// other active features
#define ClockPin 14  // labeled either as CLK or as A
#define DataPin 12   // labeled either as DT or as B
#define ButtonPin 2 // labeled as SW

// include libraries
#include <Task.h>

// include sub files
#include "RotaryEncoderTask.h" // this implements the rotary encoder task

TaskManager taskManager;


// Enable one of these two #includes and comment out the other.
// Conditional #include doesn't work due to Arduino IDE shenanigans.
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
//#include <TinyWireM.h> // Enable this line if using Adafruit Trinket, Gemma, etc.

#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_7segment matrix = Adafruit_7segment();

// foreward delcare functions passed to task constructors now required
void HandleButtonChanged(EncoderButtonState state);
void HandleRotationChanged(int8_t rotationDelta);


RotaryEncoderTask RotaryTask(HandleRotationChanged,
    HandleButtonChanged,
    ClockPin,
    DataPin,
    ButtonPin);

void setup()
{
    Serial.begin(57600);
    while (!Serial); // wait for serial monitor to connect


    taskManager.StartTask(&RotaryTask);

    Serial.println("Running...");
    
    matrix.begin(0x70);
    
    pixels.begin(); // This initializes the NeoPixel library.
}

void loop()
{
    taskManager.Loop();
}

void HandleButtonChanged(EncoderButtonState state)
{
    if (state == EncoderButtonState_Pressed)
    {
        Serial.println("Pressed - ");


    pixels.setPixelColor(1, pixels.Color(150,0,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    pixels.setPixelColor(0, pixels.Color(0, 150,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    delay (200);
    
    matrix.println(analogRead(A0));
    matrix.writeDisplay();

    pixels.setPixelColor(0, pixels.Color(150,0,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    pixels.setPixelColor(1, pixels.Color(0, 150,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
    
    delay(200);
 matrix.println(analogRead(A0));
    matrix.writeDisplay();

    pixels.setPixelColor(1, pixels.Color(150,0,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    pixels.setPixelColor(0, pixels.Color(0, 150,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    
    delay(200);
 matrix.println(analogRead(A0));
    matrix.writeDisplay();

        pixels.setPixelColor(0, pixels.Color(150,0,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    pixels.setPixelColor(1, pixels.Color(0, 150,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(200);
 matrix.println(analogRead(A0));
    matrix.writeDisplay();

    pixels.setPixelColor(1, pixels.Color(150,0,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    pixels.setPixelColor(0, pixels.Color(0, 150,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    
    delay(200);
    matrix.println(analogRead(A0));
    matrix.writeDisplay();
    
    pixels.setPixelColor(0, pixels.Color(150,0,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    pixels.setPixelColor(1, pixels.Color(0, 150,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(200);
        
    }
    else if (state == EncoderButtonState_Released)
    {
        Serial.println("Released - ");

 for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.
 
  }
  
    matrix.println(RotaryTask.RotationValue());
    matrix.writeDisplay();
    delay(10);
    
    } 
    else
    {
        Serial.println("Auto-repeat - ");
    }
}

void HandleRotationChanged(int8_t rotationDelta)
{
    if (rotationDelta > 0)
    {
        Serial.print("clockwise = ");
    }
    else
    {
        Serial.print("counter-clockwise = ");
    }

    Serial.println(RotaryTask.RotationValue());

    matrix.println(RotaryTask.RotationValue());
    matrix.writeDisplay();
    delay(10);
  
}
