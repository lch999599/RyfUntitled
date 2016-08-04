//#include "rainbow.gif.h"
//#include "gear1.gif.h"
//#include "gear2.gif.h"
//#include "robot.gif.h"
//#include "cookiemonster.gif.h"
//#include "ninja.gif.h"
#include "mew.gif.h"
//#include "N.gif.h"
//#include "N2.gif.h"

#define hall_input_pin 14

#define motor_pin 16

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <FastLED.h>

//#define HIRES //whether 360 hi res sampling or grid sampling

#define LEN(x) (sizeof(x) / sizeof(x[0]))

static const int OUTWARD = 0; // Strip directions
static const int INWARD = 0;

// Settings
static const float OFFSET_ANGLE       = 0;
static const int   NUM_STRIPS         = 3;
static const int   NUM_LEDS_PER_STRIP = 24;
static const int   BRIGHTNESS         = 16; // 0 - 255
static const int   STRIP_DIRECTIONS[NUM_STRIPS] = { OUTWARD, INWARD, OUTWARD };
static const char _SSID[] = "AndroidAP";
static const char PASS[] = "hyxa7911";
static const int LOCAL_PORT = 7776;
static const int SERVER_PORT = 7777;

const long TRIGGERINTERVAL = 10000; //time needed to operate the bicycle to trigger the installation
int IDLEINTERVAL = 5000; //if timeInterval takes more than this amount of time, then it is idle

//Hall sensor
int counter; //for testing only
long prevReadTime, //previous hall detection time
     timeInitialised, //time since first hall detection, i.e. user started pedalling
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

WiFiUDP udp;
IPAddress serverIP(192, 168, 1, 117);

void setup() {
    
  Serial.begin(115200);

  pinMode(hall_input_pin, INPUT_PULLUP);
  pinMode(motor_pin, OUTPUT);

  init_LEDs(); //turn off and initialise colours

  init_motor_cruise(); //set to cruise mode, takes ~12 seconds
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

  delay(1); //prevent program from crashing

}

void calc_rpm() {
  /*
     calculates the rpm based on subsequent hall detections
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
         calc the real time angle position of the light strip wrt to the 0 position and based on rpm
  */
  if (isInit) {

    timeInterval = millis() - prevReadTime; 

    angle = ( (float)timeInterval / oneRevTimeInterval ) * 2 * PI; //radians
  }
}

void reset() {
  /*
     resets upon idling
  */
  isIdle = true;
  timeInitialised = 0;
  isInit = false;
  isTriggered = false;
  rpm = 0; //only for user bicyle
  oneRevTimeInterval = 0;
  hasActivatedMotor = false;
  
  init_motor_cruise(); //set motor to cruise mode 
} 

void check_idle() {
  /*
     check whether user has stopped pedalling
  */

  if (millis() - prevReadTime > IDLEINTERVAL && isInit) { //takes more than this time for one revolution

    Serial.println("IDLE!!!!");
    reset();
    
  } else {

    isIdle = false; //something is happening
  }
}

void check_trigger() {
  /*
     check whether to trigger installation
  */

  if (!isIdle && isInit && !isTriggered && millis() - timeInitialised > TRIGGERINTERVAL) {

    isTriggered = true;
    Serial.println("TRIGGERED!!!!!!!!!!");

    reset();
  }
}

void init_LEDs() {
  /*
     turns off LEDs upon power up
  */
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS_PER_STRIP * NUM_STRIPS);
  FastLED.setBrightness(BRIGHTNESS);
  
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
     test for brightness adjusting according to RPM
  */
  int brightLevel = map(rpm, 0, 500, 0, 31);

  FastLED.show();
}


void update_leds() {
  /*
     light painting based on Jacky's algorithm
  */
  /*
  const float ANGLE_BETWEEN_STRIPS = 2 * PI / NUM_STRIPS;
    
  float angles[NUM_STRIPS];

  for (int i = 0; i < NUM_STRIPS; i++)
    angles[i] = angle + i * ANGLE_BETWEEN_STRIPS;

  draw_line(&angles);
  */

  
    int retryCount = 3;
    int cb;
    
    requestPixelsForAngle(angle);
    
    while (retryCount--) {
        int cb = udp.parsePacket();
        if (cb)
            break;
    }

    // Read pixels data and update LEDs
    Serial.print("packet received, length=");
    Serial.println(cb);
    udp.read((uint8_t *) leds, sizeof(leds));
    FastLED.show();
}


void init_motor_cruise() {
  /*
     12 second start up routine for motor controller. Note the inverse relationship between the 
     voltage at the base of the npn STS8050 transistor and the output voltage at the collector. 

     Motor cruise control startup sequence: 2V 1sec, 3V 10sec, then 0V to activate cruise mode
     Motor max speed: 5V consistent

     This function will freeze the program for 11 seconds
  */
  progStartTime = millis();

  while ( millis() - progStartTime < 1000 ) {
    analogWrite(motor_pin, 614); //2V
    delay(10);
  }
  while ( millis() - progStartTime < 11000 ) {
    analogWrite(motor_pin, 409); //3V
    delay(10);
  }
  analogWrite(motor_pin, 1023); //0V, turn off so it remains at cruise mode
  delay(10);
}

void init_wifi() {
    Serial.begin(115200);

    Serial.print("Connecting to ");
    Serial.println(_SSID);
    WiFi.begin(_SSID, PASS);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Starting UDP");
    udp.begin(LOCAL_PORT);
    Serial.print("Local port: ");
    Serial.println(udp.localPort());
}

void update_motor() {
  /*
     tells motor whether to go faster or not
  */
  if (isTriggered && !isIdle && isInit && !hasActivatedMotor) { 
    analogWrite(motor_pin, 0); //5V full speed
    hasActivatedMotor = true;
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

void requestPixelsForAngle(float angle)
{
    udp.beginPacket(serverIP, SERVER_PORT);
    udp.write((char *) &angle, sizeof(angle));
    udp.endPacket();
}


