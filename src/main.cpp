/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  the pid is designed to Output an analog value,
 * but the relay can only be On/Off.
 *
 *   to connect them together we use "time proportioning
 * control"  it's essentially a really slow version of PWM.
 * first we decide on a window size (5000mS say.) we then
 * set the pid to adjust its output between 0 and that window
 * size.  lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the
 * window being "Relay Off Time"
 ********************************************************/


#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <OneWire.h>
#include <Adafruit_NeoPixel.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>
//#include <U8x8lib.h>
#include <Wire.h>
//#include "U8glib.h"



#define ONE_WIRE_BUS 2
#define NUM_LEDS 2
#define DATA_PIN 1 //LED pin data

#define PIN_INPUT 0
#define RELAY_PIN 6

#define BUTTON_PIN 4
#define reset_P 5


// Define the array of leds
//CRGB leds[NUM_LEDS];

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.

//OneWire TemperatureSensor(ONE_WIRE_BUS);//version no dallas lib

DallasTemperature sensors(&oneWire);


int pot;


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;


//U8G2_SSD1306_128X32_UNIVISION_1_SW_I2C(U8G2_R0, /* clock=*/ 9, /* data=*/ 7, /* reset=*/ U8X8_PIN_NONE) [page buffer, size = 128 bytes];

//U8G2_SSD1306_128X32_UNIVISION_2_SW_I2C(U8G2_R0, clock, data [, reset]) [page buffer, size = 256 bytes]
//U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C(rotation, clock, data [, reset]) [full framebuffer, size = 512 bytes]
//U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C(rotation, [reset [, clock, data]]) [page buffer, size = 128 bytes]
//U8G2_SSD1306_128X32_UNIVISION_2_HW_I2C(rotation, [reset [, clock, data]]) [page buffer, size = 256 bytes]
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C(rotation, [reset [, clock, data]]) [full framebuffer, size = 512 bytes]
//U8G2_SSD1306_128X32_UNIVISION_1_2ND_HW_I2C(rotation, [reset]) [page buffer, size = 128 bytes]
//U8G2_SSD1306_128X32_UNIVISION_2_2ND_HW_I2C(U8G2_R0, [reset]) [page buffer, size = 256 bytes]
//U8G2_SSD1306_128X32_UNIVISION_F_2ND_HW_I2C(rotation, [reset]) [full framebuffer, size = 512 bytes]

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 9, /* data=*/ 8);   // pin remapping with ESP8266 HW I2C
//U8G2_SSD1306_128X32_WINSTAR_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C

//U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI

//U8X8_SSD1306_128X32_UNIVISION_2ND_HW_I2C ([reset_P]);


///////////////////neopixel

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, DATA_PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.




/////////////////////end neopixel






void setup()
{

strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'


/////////////////PID///////////////////
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 100;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

////////////////END PID///////////////////

// Start up the dallas library
 // sensors.begin();

pinMode(RELAY_PIN, OUTPUT);

}

void loop()
{
/*
//////////////// DALLAS SHIT NO DALLAS LIB /////////////////
byte i;
byte data[12];

int16_t raw;

float t;

TemperatureSensor.reset(); // reset one wire buss
TemperatureSensor.skip(); // select only device

TemperatureSensor.write(0x44); // start conversion

delay(500); // wait for the conversion

TemperatureSensor.reset();
TemperatureSensor.skip();

TemperatureSensor.write(0xBE); // Read Scratchpad

for ( i = 0; i < 9; i++) { // 9 bytes

data[i] = TemperatureSensor.read();

}

raw = (data[1] << 8) | data[0];
t = (float)raw / 16.0;

//////////////// END DALLAS SHIT NO DALLAS LIB /////////////////
*/


//////////////// DALLAS SHIT /////////////////
sensors.requestTemperatures(); // Send the command to get temperatures
// We use the function ByIndex, and as an example get the temperature from the first sensor only.
  float tempC = sensors.getTempCByIndex(0);

  // Check if reading was successful
  if (tempC != DEVICE_DISCONNECTED_C)
  {
    Serial.print("Temperature for the device 1 (index 0) is: ");
    Serial.println(tempC);
  }
  else
  {
    Serial.println("Error: Could not read temperature data");
  }

//////////////// END DALLAS SHIT /////////////////




  Input = analogRead(PIN_INPUT);
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime) digitalWrite(RELAY_PIN, HIGH);
  else digitalWrite(RELAY_PIN, LOW);

}


