/**
  *******************************
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 *
 * REVISION HISTORY
 * Version 1.0 - Compilation of DhtTemperature & Parking Sensor
 * Version 1.1 - Replace counter by time to update every x seconds values
 * DESCRIPTION
 * Parking sensor using a neopixel led ring and distance sensor (HC-SR04).
 * Configure the digital pins used for distance sensor and neopixels below.
 * NOTE! Remeber to feed leds and distance sensor serparatly from your Arduino. 
 * It will probably not survive feeding more than a couple of LEDs. You 
 * can also adjust intesity below to reduce the power requirements.
 * 
 * Sends parking status to the controller as a DOOR sensor if SEND_STATUS_TO_CONTROLLER 
 * is defined below. You can also use this _standalone_ without any radio by 
 * removing the SEND_STATUS_TO_CONTROLLER define.
 */

#define SEND_STATUS_TO_CONTROLLER  // Put a comment on this line for standalone mode

// Enable debug prints to serial monitor
//#define MY_DEBUG 

#ifdef SEND_STATUS_TO_CONTROLLER
// Enable and select radio type attached Disabled
#define MY_RADIO_NRF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_NODE_ID 11
#define MY_REPEATER_FEATURE
#include <MySensors.h>
#endif

#include <Adafruit_NeoPixel.h>
#include <NewPing.h>
#include <SPI.h>
#include <DHT.h> 

// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 3

#define NEO_PIN      8 // NeoPixels input pin
#define TRIGGER_PIN  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define NUMPIXELS      24 // Number of nexpixels in ring/strip
#define MAX_INTESITY   5 // Intesity of leds (in percentage). Remember more intesity requires more power.

// The maximum rated measuring range for the HC-SR04 is about 400-500cm.
#define MAX_DISTANCE 400 // Max distance we want to start indicating green (in cm)
#define PANIC_DISTANCE 15 // Mix distance we red warning indication should be active (in cm)
#define PARKED_DISTANCE 45 // Distance when "parked signal" should be sent to controller (in cm)

#define PARK_OFF_TIMEOUT 80000 // Number of milliseconds until turning off light when parked.

// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

#define CHILD_ID_HUM 0  
#define CHILD_ID_TEMP 1
#define CHILD_ID 2

float lastTempWinP;
float lastTempWinN;
char lastHumWinP;
char lastHumWinN;
boolean metric = true;
unsigned long sendInterval = 5000;  // Send park status at maximum every 5 second.
unsigned long lastSend;

int oldParkedStatus=-1;

unsigned long blinkInterval = 100; // blink interval (milliseconds)
unsigned long lastBlinkPeriod;
bool blinkColor = true;

// To make a fading motion on the led ring/tape we only move one pixel/distDebounce time
unsigned long distDebounce = 30; 
unsigned long lastDebouncePeriod;
int numLightPixels=0;
int skipZero=0;

unsigned long timeTemp = millis();
unsigned long timeHum = millis(); 
unsigned long timeParking = millis();  // add refresh for parking status
const static uint32_t counter = 600000; 

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msg(CHILD_ID,V_TRIPPED);
DHT dht;


void presentation()  {
  sendSketchInfo("ParkingTemperatureAndHumiditySensor", "1.0");
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID, S_DOOR, "Parking Status");

   metric = getControllerConfig().isMetric;
  }

void setup() 
{
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht.getMinimumSamplingPeriod());
  pixels.begin(); // This initializes the NeoPixel library.
  
    }


void loop() 
{
  // Get temperature from DHT library
  float temperature = dht.getTemperature();
  if (temperature >= lastTempWinP || temperature <= lastTempWinN || millis() - timeTemp > counter) {
    // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
    lastTempWinP = temperature + 0.2;
    lastTempWinN = temperature - 0.2;
    temperature += SENSOR_TEMP_OFFSET;
    send(msgTemp.set(temperature, 1));
    timeTemp = millis(); // save the new time
      } 
      

  // Get humidity from DHT library
  char humidity = dht.getHumidity();
  if (humidity >= lastHumWinP || humidity <= lastHumWinN || (millis() - timeHum) > (counter + 1000)) {
    // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
    lastHumWinP = humidity + 2;
    lastHumWinN = humidity - 2;
    // Reset no updates counter
    send(msgHum.set(humidity, 0));
    timeHum = millis(); // save the new time
      } 
     
  unsigned long now = millis(); 
  unsigned int fullDist = (sonar.ping_median() / US_ROUNDTRIP_CM);
  
  int displayDist = min(fullDist, MAX_DISTANCE);
  if (displayDist == 0 && skipZero<10) {
    // Try to filter zero readings
    skipZero++;
  return;
  }

  // Check if it is time to alter the leds
  if (now-lastDebouncePeriod > distDebounce) {
    lastDebouncePeriod = now;

    // Update parked status
    int parked = displayDist != 0 && displayDist<PARKED_DISTANCE; // why not bool ?
    if (parked != oldParkedStatus && now-lastSend > sendInterval) {  // add counter to refresh if parked or not
      
      send(msg.set(parked)); 

      oldParkedStatus = parked;
      lastSend = now;
      }

    // add counter to refresh if parked or not
    if (now - timeParking > counter) { 
      send(msg.set(parked));
      timeParking = now;
      }

    if ((parked || !parked ) && now-lastSend > PARK_OFF_TIMEOUT) {
      // We've been parked for a while now. Turn off all pixels
      for(int i=0;i<NUMPIXELS;i++){
        pixels.setPixelColor(i, pixels.Color(0,0,0)); 
      }
    } else {
      if (displayDist == 0) {
        // No reading from sensor, assume no object found
        numLightPixels--;
      } else {
        skipZero = 0;
        int newLightPixels = NUMPIXELS - (NUMPIXELS*(displayDist-PANIC_DISTANCE)/(MAX_DISTANCE-PANIC_DISTANCE));
        if (newLightPixels>numLightPixels) {
          // Fast raise
          numLightPixels += max((newLightPixels - numLightPixels) / 2, 1);
        } else if (newLightPixels<numLightPixels) {
          // Slow decent
          numLightPixels--;
        }
      }
  
      if (numLightPixels>=NUMPIXELS) {
        // Do some intense red blinking 
        if (now-lastBlinkPeriod > blinkInterval) {
          blinkColor = !blinkColor;
          lastBlinkPeriod = now;
        }
        for(int i=0;i<numLightPixels;i++){
          pixels.setPixelColor(i, pixels.Color(blinkColor?255*MAX_INTESITY/100:0,0,0)); 
        }              
      } else {
        for(int i=0;i<numLightPixels;i++){
          int r = 255 * i/NUMPIXELS;
          int g = 255 - r;     
          // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
          pixels.setPixelColor(i, pixels.Color(r*MAX_INTESITY/100,g*MAX_INTESITY/100,0)); 
        }
        // Turn off the rest
        for(int i=numLightPixels;i<NUMPIXELS;i++){
          pixels.setPixelColor(i, pixels.Color(0,0,0)); 
        }
      }
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
  }

      }

