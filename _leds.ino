void init_LEDs() {
  /*
     turns off LEDs upon power up
  */
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS_PER_STRIP * NUM_STRIPS);
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

#if defined(HIRES)
void draw_line(float (*angles)[NUM_STRIPS]) {
  static const int NUM_CHANNELS = 3;

  for (int i = 0; i < LEN(*angles); i++) {
    const int angle = (int) (((*angles)[i] + OFFSET_ANGLE) / (2 * PI) * 360) % 360;

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
  for (int i = 0; i < LEN(*angles); i++) {
    const float angle = (*angles)[i] + OFFSET_ANGLE;

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


void update_leds() {
  /*
     light painting based on Jacky's algorithm
  */

#ifdef WIFI

  request_pov_pixels(serverIP, angle + OFFSET_ANGLE);
    
    int cb = udp.parsePacket();

    if (!cb) {
        Serial.println("no packet yet");
    } else {
        Serial.print("packet received, length=");
        Serial.println(cb);
        // We've received a packet, read the data from it
        udp.read((unsigned char *) leds, sizeof(leds)); // read the packet into the buffer
    }
    
  // reverse the pixels if the direction is INWARD
    for (int i = 0; i < NUM_STRIPS; i++) {
        if (STRIP_DIRECTIONS[i] != INWARD)
            continue;
            
        for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
            for (int k = 0; k < NUM_CHANNELS; k++) {
                const int index = i * NUM_LEDS_PER_STRIP + j;
                const int reversed_index = (i + 1) * NUM_LEDS_PER_STRIP - j - 1;
                char tmp[NUM_CHANNELS];

                memcpy(tmp, &leds[index], sizeof(tmp));
                memcpy(&leds[index], &leds[reversed_index], sizeof(tmp));
                memcpy(&leds[reversed_index], tmp, sizeof(tmp));
            }
        }
    }

    FastLED.show();

#else
    const float ANGLE_BETWEEN_STRIPS = 2 * PI / NUM_STRIPS;
    
    float angles[NUM_STRIPS];
    
    for (int i = 0; i < NUM_STRIPS; i++)
        angles[i] = angle + i * ANGLE_BETWEEN_STRIPS;
    
    draw_line(&angles);
    
#endif
}

