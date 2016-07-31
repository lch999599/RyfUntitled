#include "rainbow.gif.h"
//#include "gear1.gif.h"
//#include "gear2.gif.h"
//#include "robot.gif.h"
//#include "cookiemonster.gif.h"
//#include "ninja.gif.h"
//#include "mew.gif.h"
//#include "N.gif.h"
//#include "N2.gif.h"

#define hall_input_pin 14
#define motor_pin 4

#include <FastLED.h>

//#define HIRES

#define LEN(x) (sizeof(x) / sizeof(x[0]))

// Strip directions
static const int OUTWARD = 0;
static const int INWARD = 0;

// Settings
static const float OFFSET_ANGLE       = 0;
static const int   NUM_STRIPS         = 3;
static const int   NUM_LEDS_PER_STRIP = 24;
static const int   BRIGHTNESS         = 32; // 0 - 255
static const int   STRIP_DIRECTIONS[NUM_STRIPS] = { OUTWARD, INWARD, OUTWARD };

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
const uint8_t DATA_PIN = 12;
const uint8_t CLOCK_PIN = 13;
CRGB leds[NUM_LEDS_PER_STRIP * NUM_STRIPS];

void setup() {
  Serial.begin(115200);
  
  pinMode(hall_input_pin, INPUT_PULLUP);

  init_LEDs(); //turn off and initialise colours

  //init_motor(); //set to cruise mode, takes ~12 seconds
}

void loop() {

  currVal = digitalRead(hall_input_pin);

//    Serial.print("hall pin: ");
//  Serial.println(currVal);

  if (currVal == LOW && prevVal == HIGH) { //positive hall detection
    
    counter++;

    angle = 0; //assuming the sensor is placed at clock position 3pm, else just do an offset

    calc_rpm();
            Serial.print("counter: ");
            Serial.println(counter);
  }

  calc_angle();

  check_idle(); //check whether user has left 

  check_trigger(); //check whether user has triggered the installation to turn on 

  prevVal = currVal;

  update_leds();
  //update_leds_test();

  update_motor();

  //delay(1);
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

void init_LEDs() {
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS_PER_STRIP * NUM_STRIPS);
  FastLED.setBrightness(BRIGHTNESS);
/*
 * turns off LEDs upon power up
 */
  for (uint16_t i = 0; i < NUM_LEDS_PER_STRIP * NUM_STRIPS; i++) {
    leds[i].red = 0;
    leds[i].green = 0;
    leds[i].blue = 0;
  }

  FastLED.show(); //turns them off

  for (uint16_t i = 0; i < NUM_LEDS_PER_STRIP * NUM_STRIPS; i++) { //initialises to default white color
    leds[i].red = 255;
    leds[i].green = 255;
    leds[i].blue = 255;
  }
}


void update_leds_test() {
/*
 * test for brightness adjusting according to RPM
 */
  int brightLevel = map(rpm, 0, 500, 0, 31);

  FastLED.show();
}


void update_leds() {
/*
 * light painting based on Jacky's algorithm
 */
     const float ANGLE_BETWEEN_STRIPS = 2 * PI / NUM_STRIPS;
    float angles[NUM_STRIPS];

    for (int i = 0; i < NUM_STRIPS; i++)
        angles[i] = angle + i * ANGLE_BETWEEN_STRIPS;

    draw_line(&angles);
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


#if defined(HIRES)

void draw_line(float (*angles)[NUM_STRIPS]) {
    static const int NUM_CHANNELS = 3;

    for (int i = 0; i < LEN(*angles); i++) {
        const int angle = (int) (*angles)[i];

        for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
            int k = j;
            
            if (STRIP_DIRECTIONS[i] == INWARD)
                k = NUM_LEDS_PER_STRIP - 1 - j;
                
            const int gif_index = (angle * NUM_LEDS_PER_STRIP + k) * NUM_CHANNELS;

            leds[i * NUM_LEDS_PER_STRIP + k].red   = pgm_read_byte_near(&gif[gif_index + 0]);
            leds[i * NUM_LEDS_PER_STRIP + k].green = pgm_read_byte_near(&gif[gif_index + 1]);
            leds[i * NUM_LEDS_PER_STRIP + k].blue  = pgm_read_byte_near(&gif[gif_index + 2]);
        }
    }

    FastLED.show();
}

#else

void draw_line(float (*angles)[NUM_STRIPS]) {
    static const int NUM_CHANNELS = 3;

    for (int i = 0; i < LEN(*angles); i++) {
        const float angle = (*angles)[i];

        for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
            int k = j;
            
            if (STRIP_DIRECTIONS[i] == INWARD)
                k = NUM_LEDS_PER_STRIP - 1 - j;

            const int x = (NUM_LEDS_PER_STRIP + cos(angle) * k) * 0.5f;
            const int y = (NUM_LEDS_PER_STRIP + sin(angle) * k) * 0.5f;
            const int gif_index = (x + y * NUM_LEDS_PER_STRIP) * NUM_CHANNELS;

            leds[i * NUM_LEDS_PER_STRIP + k].red   = pgm_read_byte_near(&gif[gif_index + 0]);
            leds[i * NUM_LEDS_PER_STRIP + k].green = pgm_read_byte_near(&gif[gif_index + 1]);
            leds[i * NUM_LEDS_PER_STRIP + k].blue  = pgm_read_byte_near(&gif[gif_index + 2]);
        }
    }

    FastLED.show();
}

#endif
