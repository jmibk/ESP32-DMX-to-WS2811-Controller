#include <Arduino.h>
#include <FastLED.h>
#include <dmx.h>

//COMMON
TaskHandle_t Task1, Task2;

//LED
#define NUM_LEDS  100
#define DATA_PIN  3
CRGB leds[NUM_LEDS];

//DMX
uint16_t dmxAddress = 1;
byte dmxChannels[512];
uint16_t dmxChannelIndex;
struct deviceChannelsStruct{
  byte red;           //0 ... 255
  byte green;         //0 ... 255
  byte blue;          //0 ... 255
  byte intensity;     //0 ... 255
  byte effect;        //0 ... 9 = RGB, 10 ... 19 = Effect1, 20 ... 29 = Effect2
  byte speed;         //0 ... 255
  };
deviceChannelsStruct deviceChannels;
byte hue;

void task_leds( void * parameter ) {
  Serial.print("started LED process on core number "+String(xPortGetCoreID()));
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);    //GRB ordering is assumed
  for (;;) {
    //NO EFFECT, RGB MODE (effect = 0 ... 9)
    if (deviceChannels.effect <= 9) {
      for (uint16_t i = 0; i < NUM_LEDS; i++){
        leds[i] = CRGB(deviceChannels.red,deviceChannels.green,deviceChannels.blue);
        }
      }
    //EFFECT1 (effect = 10 ... 19)
    else if (deviceChannels.effect <= 19) {
      for (uint16_t i = 0; i < NUM_LEDS; i++){
        leds[i] = CHSV(hue++,255,255);
        }
      delay(map(deviceChannels.speed,0,255,50,0));
      }

    //DIM LEDS
    FastLED.setBrightness(deviceChannels.intensity);

    //SHOW LED DATA
    FastLED.show(); 
    }
  }

void task_dmx( void * parameter ) {
  Serial.print("started DMX process on core number "+String(xPortGetCoreID()));
  DMX::Initialize();
  for (;;) {
    if(DMX::IsHealthy()) {
      dmxChannels[dmxChannelIndex] = DMX::Read(dmxChannelIndex+1);
      dmxChannelIndex++;
      if (dmxChannelIndex >= 512)
        dmxChannelIndex = 0;
      }

    //move values to the device struct
    if (dmxAddress < (512-sizeof(deviceChannels))) {
      deviceChannels.red =        dmxChannels[dmxAddress+0];
      deviceChannels.green =      dmxChannels[dmxAddress+1];
      deviceChannels.blue =       dmxChannels[dmxAddress+2];
      deviceChannels.intensity =  dmxChannels[dmxAddress+3];
      deviceChannels.effect =     dmxChannels[dmxAddress+4];
      deviceChannels.speed =      dmxChannels[dmxAddress+5];
      }

    //
    }
  }

void setup() { 
  //COMMON
  Serial.begin(115200);

  //MULTITASKING
  xTaskCreatePinnedToCore(
    task_leds,
    "task_leds",
    1000,
    NULL,
    1,
    &Task1,
    1);

  xTaskCreatePinnedToCore(
    task_dmx,
    "task_dmx",
    1000,
    NULL,
    1,
    &Task2,
    0);
  }
        
void loop() { 
  }