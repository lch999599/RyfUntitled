void init_motor_cruise() {
  /*
     12 second start up routine for motor controller. Note the inverse relationship between the 
     voltage at the base of the npn STS8050 transistor and the output voltage at the collector. 

     cruise mode - 2V - 60rpm - 600 analog value
     full speed mode - 3.5V - 220rpm - 300 analog value
  */
  progStartTime = millis();
    
    analogWrite(motor_pin, 600); //2V cruise speed
    delay(100);
}

void update_motor() {
  /*
     tells motor whether to go faster or not
  */
  if (isTriggered && !isIdle && isInit && !hasActivatedMotor) { 
    analogWrite(motor_pin, 300); //3.5V full speed
    hasActivatedMotor = true;
    delay(20);
  } 
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
  counter = 0;
  
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
    isTriggeredTime = millis();
    Serial.println("TRIGGERED!!!!!!!!!!");
  }
}
