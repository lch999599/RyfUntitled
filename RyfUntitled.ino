#include "rainbow.gif.h"
//#include "gear1.gif.h"
//#include "gear2.gif.h"
//#include "robot.gif.h"
//#include "cookiemonster.gif.h"
//#include "ninja.gif.h"
//include "mew.gif.h"
//#include "N.gif.h"
//#include "N2.gif.h"
//#include "stickman.gif.h"
//#include "stickman2.gif.h"
//#include "stickman3.gif.h"
//#include "enlighted.gif.h"
//#include "enlighted2.gif.h"
//#include "symmgear.gif.h"

//pins
#define hall_input_pin 14

//cannot use pin 16 cos no PWM
#define motor_pin (15)

//user settings
//whether to stream webcam data or the gif file
//#define WIFI 

//whether 360 hi res sampling or grid sampling
//#define HIRES

//libraries
#ifdef WIFI
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#endif

#include <FastLED.h>

#define LEN(x) (sizeof(x) / sizeof(x[0]))

static const int OUTWARD = 0; // Strip directions
static const int INWARD = 1;

// Settings
static const float OFFSET_ANGLE       = 0.7* 2*PI;
static const int   NUM_CHANNELS       = 3;
static const int   NUM_STRIPS         = 4;
static const int   NUM_LEDS_PER_STRIP = 24;
static const int   BRIGHTNESS         = 64; // 0 - 255
static const int   STRIP_DIRECTIONS[NUM_STRIPS] = { OUTWARD, OUTWARD, OUTWARD, OUTWARD };

#ifdef WIFI
IPAddress serverIP(192, 168, 43, 116); //Android AP
WiFiUDP udp;
static const char _SSID[] = "AndroidAP";
static const char PASS[] = "hyxa7911";
static const int LOCAL_PORT = 2390;             
const int PACKET_SIZE = NUM_LEDS_PER_STRIP * NUM_STRIPS * NUM_CHANNELS;
byte packetBuffer[PACKET_SIZE];
#endif

static const int SERVER_PORT = 7777;

const long TRIGGERINTERVAL = 10000; //time needed to operate the bicycle to trigger the installation
int IDLEINTERVAL = 5000; //if timeInterval takes more than this amount of time, then it is idle

//Hall sensor
int counter; //for testing only
long prevReadTime, //previous hall detection time
     timeInitialised, //time since first hall detection, i.e. user started pedalling
     isTriggeredTime,//time when triggered
     oneRevTimeInterval, //interval between hall detections, i.e. one revolution
     timeInterval, //time since prevReadTime for calculating angle based on rpm
     progStartTime; //time since power on
int currVal, prevVal; //digital values for hall readings
float rpm; //rounds per minute
boolean isInit, //upon first hall detection
        isIdle = true, //user has left
        isTriggered, //user has triggered installation
        hasActivatedMotor;
float angle; //real time angle of the wheel led line wrt to 0 position

//LEDs
const uint8_t DATA_PIN = 12; //green
const uint8_t CLOCK_PIN = 13; //yellow
CRGB leds[NUM_LEDS_PER_STRIP * NUM_STRIPS];

void setup() {
    
  Serial.begin(115200);

  pinMode(hall_input_pin, INPUT_PULLUP);
  pinMode(motor_pin, OUTPUT);

  init_LEDs(); //turn off and initialise colours

  init_motor_cruise(); //set to cruise mode, takes ~12 seconds

  #ifdef WIFI
  init_wifi();
  #endif
}

void loop() {

  currVal = digitalRead(hall_input_pin);

  if (currVal == LOW && prevVal == HIGH) { //positive hall detection

    counter++;

    angle = 0; //assuming the sensor is placed at clock position 3pm, else just do an offset

    calc_rpm();
    
    Serial.print("counter: ");
    Serial.println(counter);
  }

  calc_angle();

  check_idle(); //check whether user has left

  check_trigger(); //check whether user has triggered the installation 

  prevVal = currVal;

  update_leds();
  //update_leds_test();

  update_motor();

//  delay(1); //prevent program from crashing
}


