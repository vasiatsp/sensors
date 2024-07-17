#include <FastLED.h>       
FASTLED_USING_NAMESPACE

#define NUM_LEDS         60 
#define DATA_PIN          7 
#define LED_TYPE     WS2811 
#define COLOR_ORDER     RGB 
#define ENVELOPE_PIN     A0 
#define BRIGHTNESS      200 
#define SATURATION      150 
#define MIN_VAL          10 
#define MAX_VAL         150 
#define HUE_CHANGE        2 

/*============= SELECT STYLE =============*/
/*                                        */
/*    0   -->   LinearFlowing (BEST)      */
/*    1   -->   LinearReactive            */
/*    2   -->   BrightnessReactive        */
/*    3   -->   CentreProgressive         */
/*    4   -->   EdgeProgressive           */
/*                                        */                                        
/* */          int STYLE = 0;          /* */
/*                                        */
/*========================================*/

CRGB leds[NUM_LEDS];
byte dynamicHue = HUE_INIT;
int analogVal = 0;
int val = 0;

void setup() { 
  pinMode(ENVELOPE_PIN, INPUT);
  
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }

  // Update the LED Strip
  FastLED.show(); 
}

void loop() {
  analogVal = analogRead(ENVELOPE_PIN);

  if(analogVal > MAX_VAL)
    analogVal = MAX_VAL;

  if(analogVal < MIN_VAL)
    analogVal = MIN_VAL;

  switch (STYLE) {
    case 1:
      LinearReactive();
      break;
    case 2:
      BrightnessReactive();
      break;
    case 3:
      CentreProgressive();
      break;
    case 4:
      EdgeProgressive();
      break;
    default:
      LinearFlowing();
      break;
  }
  
  // Update the LED Strip
  FastLED.show();
}

void LinearFlowing() {
  val = map(analogVal, MIN_VAL, MAX_VAL, 0, BRIGHTNESS);
  int dynamicDelay = map(analogVal, MIN_VAL, MAX_VAL, 20, 1);
  
  for (int i = 0; i < NUM_LEDS-1; i++) {
    leds[i] = leds[i+1];
  }

  leds[NUM_LEDS-1] = CHSV(dynamicHue += HUE_CHANGE, SATURATION, val);

  delay(dynamicDelay);
}

void LinearReactive() {
  val = map(analogVal, 0, MAX_VAL+1, 0, NUM_LEDS);

  for(int i = 0; i < NUM_LEDS; i++) {
    if (i <= val)
      leds[i] = CHSV(HUE_INIT+(HUE_CHANGE*i), SATURATION, BRIGHTNESS);
    else
      leds[i].nscale8(10);
  }
}

void BrightnessReactive() {
  val = map(analogVal, MIN_VAL, MAX_VAL, 0, BRIGHTNESS);
  
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(HUE_INIT+(HUE_CHANGE*i), SATURATION, val);
  }
}

void CentreProgressive() {
  val = map(analogVal, MIN_VAL, MAX_VAL, 0, NUM_LEDS/2);

  for(int i = 0; i < NUM_LEDS/2; i++) {
    if (i <= val) {
      leds[(NUM_LEDS/2)+i] = CHSV(HUE_INIT+(HUE_CHANGE*i), SATURATION, BRIGHTNESS);
      leds[(NUM_LEDS/2)-i] = CHSV(HUE_INIT+(HUE_CHANGE*i), SATURATION, BRIGHTNESS);
    } else {
      leds[(NUM_LEDS/2)+i].nscale8(10);
      leds[(NUM_LEDS/2)-i].nscale8(10);
    }
  }
}

void EdgeProgressive() {
  val = map(analogVal, 0, MAX_VAL, 0, NUM_LEDS/2);

  for(int i = 0; i < NUM_LEDS/2; i++) {
    if (i <= val) {
      leds[i] = CHSV(HUE_INIT+(HUE_CHANGE*i), SATURATION, BRIGHTNESS);
      leds[NUM_LEDS-i] = CHSV(HUE_INIT+(HUE_CHANGE*i), SATURATION, BRIGHTNESS);
    } else {
      leds[i].nscale8(10);
      leds[NUM_LEDS-i].nscale8(10);
    }
  }
}
