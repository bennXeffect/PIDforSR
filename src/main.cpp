
#include <Arduino.h>
#include <PID_v1.h>
//#include <SPI.h>
#include <OneWire.h>
//#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "OneButton.h"



#define ONE_WIRE_BUS D5
#define NUM_LEDS 3
#define DATA_PIN D2 //LED pin data

#define PIN_INPUT A0
#define RELAY_PIN D4

#define BUTTON_PIN D6
//#define reset_P 5

OneButton button1(BUTTON_PIN, true);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.

//OneWire TemperatureSensor(ONE_WIRE_BUS);//version no dallas lib

DallasTemperature sensors(&oneWire);

int pot;
bool WORKING_STATE = 0; //system ON or OFF ?

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

float tempC = 20; //the temp from the sensor

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned int WindowSize = 3500;
unsigned long windowStartTime;

unsigned char beat = 0;
int blinkPace = 15; //blinking pace, the bigger the slower.

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 9, /* data=*/ 8);   // pin remapping with ESP8266 HW I2C

static unsigned char u8g_heat[] = {
  0x30, 0xC6, 0x00, 0x30, 0xC6, 0x00, 0x10, 0x42, 0x00, 0x18, 0x63, 0x00,
  0x18, 0x63, 0x00, 0x18, 0x63, 0x00, 0x38, 0xE7, 0x00, 0x30, 0xC6, 0x00,
  0x30, 0xC6, 0x00, 0x10, 0x42, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x03,
  0xFF, 0xFF, 0x0F, 0xDB, 0xB6, 0x0D, 0xDB, 0xB6, 0x0D, 0xDB, 0xB6, 0x0D,
  0xDB, 0xB6, 0x0D, 0xDB, 0xB6, 0x0D, 0xFF, 0xFF, 0x0F, 0xFC, 0xFF, 0x03,
};


CRGB leds[NUM_LEDS];

void clic1(){//to do when simple clic
  WORKING_STATE = !WORKING_STATE;
   Serial.print("Working state is ");
  Serial.println(WORKING_STATE);
  delay(150);

}
void longPress1(){
}
void longPressStart1(){
}
void doubleClic1(){
}

String tempInt;
String SetTemp;

void setup(){

  Serial.begin(115200);

   LEDS.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS)
  //LEDS.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS)
      .setTemperature(0xB5D6FF)
  .setCorrection(TypicalSMD5050);
  LEDS.setBrightness(255);

  u8g2.begin();
  u8g2.clearBuffer(); // clear the internal memory

  /////////////////PID///////////////////
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 20;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

////////////////END PID///////////////////

// Start up the dallas library
 // sensors.begin();

pinMode(RELAY_PIN, OUTPUT);


  button1.attachClick(clic1);
  //button1.attachDoubleClick(doubleClic1);
  //button1.attachLongPressStart(longPressStart1);
//  button1.attachLongPressStop(longPressStop1);
  //button1.attachDuringLongPress(longPress1);
  //button1.setClickTicks(300);


//sensors.setResolution(9);
}

void loop(){
 button1.tick(); // check the button state
 SetTemp = "";
 SetTemp += Setpoint;
    beat = sin8(millis() / blinkPace); //put the blinking pace
  u8g2.clearBuffer(); // clear the internal memory
  u8g2.setFont(u8g2_font_5x7_tf);
  if (WORKING_STATE == 1){
    u8g2.drawStr(102, 7, "ON!");
}
else{
    u8g2.drawStr(102, 7, "OFF");
}

 u8g2.drawLine(92, u8g2.getDisplayHeight() / 2 - 10, 92, u8g2.getDisplayHeight() / 2 + 10);
 u8g2.setFont(u8g2_font_5x7_tf);
 //u8g2.drawStr(1, 7, "Target");
 u8g2.drawStr(52, 7, "Actual");
 

 //  u8g2.setDrawColor(0);       // errase mode
 // u8g2.drawBox(100, 16, 128, 22); // clear the txt
 // u8g2.setDrawColor(1); // write mode
 u8g2.setFont(u8g2_font_fur30_tn);
 u8g2.setCursor(0, 31);
 //  u8g2.drawStr(70,32,tempInt.c_str());
 u8g2.print(Setpoint, 0);
 u8g2.setFont(u8g2_font_fur20_tn);
 u8g2.setCursor(51, 31);
 u8g2.print(tempC,0);
  
//    u8g2.setCursor(55, 32);
 // u8g2.print(Setpoint.c_str());
    //u8g2.drawStr(0, 32, SetTemp.c_str());
    

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
    EVERY_N_SECONDS(1)
    {
      sensors.setResolution(12);

sensors.setWaitForConversion(false);  // makes it async
  sensors.requestTemperatures();
  sensors.setWaitForConversion(true);

//      sensors.requestTemperatures(); // Send the command to get temperatures
}
// We use the function ByIndex, and as an example get the temperature from the first sensor only.
  tempC = sensors.getTempCByIndex(0);
  tempC = 30;

  tempInt = "";
  tempInt += floor(tempC);
  // Check if reading was successful
  if (tempC != DEVICE_DISCONNECTED_C)
  {
  //  Serial.print("Temperature for the device 1 (index 0) is: ");
  //  Serial.println(tempC);
  }
  else
  {
    Serial.println("Error: Could not read temperature data");
  }

//////////////// END DALLAS SHIT /////////////////

Setpoint=map(analogRead(PIN_INPUT),0,1023,25,80);
Input = tempC;
myPID.Compute();

if(WORKING_STATE==1){ //system up and running, ON state
    blinkPace = 5;
 for (int i = 0; i < NUM_LEDS; i++) {
        leds[i]  = CHSV(215, 255, map(beat,0,255,40,255));
      }

    /************************************************
     * turn the output pin on/off based on pid output
     ************************************************/
    if (millis() - windowStartTime > WindowSize)
    { // time to shift the Relay Window
      windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime) {
    digitalWrite(RELAY_PIN, LOW);
  }
  else{
    digitalWrite(RELAY_PIN, HIGH);
    leds[NUM_LEDS/2]  = CHSV(0, 255, 255);
    u8g2.setFont(u8g2_font_unifont_t_symbols);
   // u8g2.drawGlyph(102, 30, 0x2615);
    u8g2.drawXBM(99, 12, 20, 20, u8g_heat);
    u8g2.setFont(u8g2_font_5x7_tf);
  }
}
else{
  blinkPace = 15; //slow the blink pace, cuz the system is sleeping
 for (int i = 0; i < NUM_LEDS; i++) {
        leds[i]  = CHSV(185, 200, map(beat,0,255,15,255));
      }
 
  //to do when off
}
Serial.println(beat);
  //Serial.println(digitalRead(RELAY_PIN));
  FastLED.show();
  u8g2.sendBuffer();
}
