//#include "rainbow.gif.h"
//#include "gear1.gif.h"
//#include "gear2.gif.h"
//#include "robot.gif.h"
//#include "cookiemonster.gif.h"
//#include "ninja.gif.h"
//#include "mew.gif.h"
#include "N.gif.h"
//#include "N2.gif.h"

#include <avr/pgmspace.h>

#define hall_input_pin 4

#include <FastGPIO.h>
#define APA102_USE_FAST_GPIO
#include <APA102.h>

#include <SoftwareSerial.h> //for communicating with the ESP8266

#define motor_pin A0

//Hall sensor
int counter; //for testing only
long prevReadTime, //previous hall detection time
timeInitialised, //time since first hall detection, i.e. user started pedalling
oneRevTimeInterval, //interval between hall detections, i.e. one revolution
timeInterval, //time since prevReadTime for calculating angle based on rpm
progStartTime; //time since power on
const long TRIGGERINTERVAL = 100000; //time needed to operate the bicycle to trigger the installation
int IDLEINTERVAL = 20000; //if one revolution takes more than this amount of time, then it is idle
int currVal, prevVal; //digital values for hall readings
float rpm; //rounds per minute
boolean isInit, //upon first hall detection
isIdle = true, //user has left
isTriggered; //user has triggered installation
float angle; //real time angle of the wheel led line wrt to 0 position

//LEDs
const uint8_t dataPin = 10;
const uint8_t clockPin = 11;

// Create an object for writing to the LED strip.
APA102<dataPin, clockPin> ledStrip;

// Set the number of LEDs to control.
const uint16_t ledCount = 24; //24 leds need about 1A

// Create a buffer for holding the colors (3 bytes per color).
rgb_color colors[ledCount];

SoftwareSerial mySerial(8, 9); //RX, TX

void setup() {

  Serial.begin(9600);
  while (!Serial) {
    ; //wait for USB serial to connect first
  }
  
  mySerial.begin(115200); //115200 for the ESP8266
  
  pinMode(hall_input_pin, INPUT_PULLUP);

  init_LEDs(); //turn off and initialise colours

  //init_motor(); //set to cruise mode, takes ~12 seconds
}

void loop() {

  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }

  currVal = digitalRead(hall_input_pin);

  if (currVal == LOW && prevVal == HIGH) { //positive hall detection
    
//    counter++;

    angle = 0; //assuming the sensor is placed at clock position 3pm, else just do an offset

    calc_rpm();
    //        Serial.print("counter: ");
    //        Serial.println(counter);
  }

  calc_angle();

  check_idle(); //check whether user has left 

  check_trigger(); //check whether user has triggered the installation to turn on 

  prevVal = currVal;

  update_leds();
  //update_leds_test();

  update_motor();
}

void calc_rpm() { 
/*
 * calculates the rpm based on subsequent hall detections
 */
 
  if (!isInit) {

    isInit = true;

    prevReadTime = millis();
    timeInitialised = millis();

  } else { //already initialised and updates rpm upon a subsequent detection

    oneRevTimeInterval = millis() - prevReadTime; //time for one revolution

    prevReadTime = millis(); //updated

    rpm = 60000 / oneRevTimeInterval;
    Serial.print("RPM: ");
    Serial.println(rpm);
  }
}

void calc_angle() {
/*    
 *     calc the real time angle position of the light strip wrt to the 0 position and based on rpm
 */
    if (isInit) {
       
        timeInterval = millis() - prevReadTime;

        angle = ( (float)timeInterval / oneRevTimeInterval ) * 2*PI;
    }
}

void check_idle() { 
 /*
  * check whether user has stopped pedalling
  */
  
  if (millis() - prevReadTime > IDLEINTERVAL && isInit) { //takes more than this time for one revolution

    reset();
    Serial.println("IDLE!!!!");

  } else {

    isIdle = false; //something is happening
  }
}

void check_trigger() { 
/*
 * check whether to trigger installation
 */
 
  if (!isIdle && isInit && !isTriggered && millis() - timeInitialised > TRIGGERINTERVAL) {

    isTriggered = true;
    Serial.println("TRIGGERED!!!!!!!!!!");

    reset();
  }
}

void reset() {
/*
 * resets upon idling
 */
  isIdle = true;
  timeInitialised = 0;
  isInit = false;
  isTriggered = false;
  rpm = 0; //only for user bicyle
  oneRevTimeInterval = 0;
}

void init_LEDs() {
/*
 * turns off LEDs upon power up
 */
  rgb_color initialColors[72];

  for (uint16_t i = 0; i < 72; i++) {
    initialColors[i].red = 0;
    initialColors[i].green = 0;
    initialColors[i].blue = 0;
  }

  ledStrip.write(initialColors, 72, 0); //turns them off

  for (uint16_t i = 0; i < ledCount; i++) { //initialises to default white color
    colors[i].red = 255;
    colors[i].green = 255;
    colors[i].blue = 255;
  }
}


void update_leds_test() { 
/*
 * test for brightness adjusting according to RPM
 */
  int brightLevel = map(rpm, 0, 500, 0, 31);

  ledStrip.write(colors, ledCount, brightLevel);
}


void update_leds() {
/*
 * light painting based on Jacky's algorithm
 */
    draw_line(angle, ledCount, 16); //half brightness, 24 LEDs
}


void init_motor() {
/*
 * 12 second start up routine for motor controller
 */
    progStartTime = millis();

    while ( millis() - progStartTime < 1000 ) {
        analogWrite(motor_pin, 409); //2V
    }
    while ( millis() - progStartTime < 11000 ) {
        analogWrite(motor_pin, 614); //3V to activate cruise mode
    }
    analogWrite(motor_pin, 0); //turn off so it remains at cruise mode
}

void update_motor() {
/*
 * tells motor whether to go faster or not
 */
    if (isTriggered && !isIdle && isInit) {
        analogWrite(motor_pin, 1023); //5V full speed
    } else {
        analogWrite(motor_pin, 0); //set back to cruise mode
    }
}

void draw_line(float angle, int num_leds, int brightness) {
/*
 * light painting based on one strip and the rpm 
 */
	static const int NUM_CHANNELS = 3;
	rgb_color leds[num_leds];
  
	for (int i = 0; i < num_leds; i++) {
		int x = num_leds * 0.5f + cos(angle) * i * 0.5f;
		int y = num_leds * 0.5f + sin(angle) * i * 0.5f;
		int gif_index = (x + y * num_leds) * NUM_CHANNELS;
        leds[i].red   = pgm_read_byte_near(&gif[gif_index + 0]);
        leds[i].green = pgm_read_byte_near(&gif[gif_index + 1]);
        leds[i].blue  = pgm_read_byte_near(&gif[gif_index + 2]);
		//memcpy(&leds[led_index], &gif[gif_index], NUM_CHANNELS);
	}

	ledStrip.write(leds, num_leds, brightness);
}
