#include <Arduino.h>
#include <FastLED.h>
#include <esp_dmx.h>

//LED
#define NUM_LEDS  100
#define DATA_PIN  13
CRGB leds[NUM_LEDS];

//DMX
#define DMXtransmitPin 4
#define DMXreceivePin 36
#define DMXenablePin 16
QueueHandle_t DMXqueue;
bool dmxIsConnected = false;
dmx_port_t dmxPort = 1;
uint16_t dmxAddress = 1;
byte dmxChannels[DMX_MAX_PACKET_SIZE];
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

//

void task_leds(void*) {
Serial.println("started LED process on core number "+String(xPortGetCoreID()));
  //FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);    //GRB ordering is assumed
  FastLED.addLeds<WS2811,DATA_PIN, BRG>(leds, NUM_LEDS);
  for(;;) {
    //NO EFFECT, RGB MODE (effect = 0 ... 9)
    if (deviceChannels.effect <= 9) {
      for (uint16_t i = 0; i < NUM_LEDS; i++){
        leds[i] = CRGB(deviceChannels.red,deviceChannels.green,deviceChannels.blue);
        }
      }
    //EFFECT1 (effect = 10 ... 19)
    else if (deviceChannels.effect <= 19) {
      for (int j = 0; j < 255; j++) {
        for (uint16_t i = 0; i < NUM_LEDS; i++) {
          leds[i] = CHSV(i - (j * 2), 255, 255); /* The higher the value 4 the less fade there is and vice versa */ 
          }
        FastLED.show();
        delay(map(deviceChannels.speed,0,255,500,0));
        if ( deviceChannels.effect >= 10 || deviceChannels.effect < 19)
          break;
        }
     
      }

    //DIM LEDS
    FastLED.setBrightness(deviceChannels.intensity);

    //SHOW LED DATA
    FastLED.show(); 
    }
  }

void task_dmx(void*) {
Serial.println("started DMX process on core number "+String(xPortGetCoreID()));

  dmx_config_t dmxConfig = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmxPort, &dmxConfig);
  dmx_set_pin(dmxPort, DMXtransmitPin, DMXreceivePin, DMXenablePin);
  int queueSize = 1;
  int interruptPriority = 1;
  dmx_driver_install(dmxPort, DMX_MAX_PACKET_SIZE, queueSize, &DMXqueue,
                     interruptPriority);

  for (;;) {
    dmx_event_t packet;
    if (xQueueReceive(DMXqueue, &packet, DMX_PACKET_TIMEOUT_TICK)) {
      if (packet.status == DMX_OK) {
       if (!dmxIsConnected) {
        Serial.println("DMX connected!");
        dmxIsConnected = true;
        }
      dmx_read_packet(dmxPort, dmxChannels, packet.size);
      } 
    else {
      Serial.println("DMX error!");
      }
    } 
  else if (dmxIsConnected) {
    Serial.println("DMX timed out! Uninstalling DMX driver...");
    dmx_driver_delete(dmxPort);
    while (true) yield();
    }

    //dmxChannels[dmxChannelIndex] = dmx.read(dmxChannelIndex+1);
    //dmxChannelIndex++;
    if (dmxChannelIndex >= 512)
      dmxChannelIndex = 0;

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
    task_leds,      //Function to implement the task 
    "led",          //Name of the task
    6000,           //Stack size in words 
    NULL,           //Task input parameter 
    0,              //Priority of the task 
    NULL,           //Task handle.
    0);             //Core where the task should run 

  xTaskCreatePinnedToCore(
    task_dmx,      //Function to implement the task 
    "dmx",          //Name of the task
    6000,           //Stack size in words 
    NULL,           //Task input parameter 
    0,              //Priority of the task 
    NULL,           //Task handle.
    1);             //Core where the task should run   
  }

void loop() {
  delay(10000);
  }