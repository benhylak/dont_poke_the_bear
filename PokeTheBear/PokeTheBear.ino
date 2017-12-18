#include "FastLED.h"
#include "Servo.h"

/**********LED CODE ********************/
FASTLED_USING_NAMESPACE
#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

//#define CLK_PIN   4
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    145
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120
/****************************************/

enum States {ALL_OFF, APPROACHING, IN_RANGE, RAGE};

/************* Desired Behavior **************

State 1 (ALL_OFF):
*******************
  Condition: User is out of range for x loops

  Behavior: All off, reset all counts/states.

  Transitions:
    User first enters range

State 2 (APPROACHING):
*******************
  Condition: User enters range, but is further than y distance

  Behavior: Gets brighter (more white) proportional to distance

  Transitions:
    User is < y away
    User inititates poke #1
    User is out of range

State 3 (IN_RANGE):
*******************
  Condition: User is <y away

  Behavior: Dance Party, Wave a little, (maybe trigger groovy music?)

  Transitions: User completes poke #1

State 4 (RAGE):
*******************
  Condition: User completed poke #1

  Behavior: Quickly flashing red, all alarms, waving and stuff

  Transitions: User is out of range or user pokes again
/********************************************/

/******************* Pins *******************/

#define SPEAKER_PIN 11
#define ULTRASONIC_PIN 3
#define BUTTON_PIN 12
/*-Use built in pull up resistor
  -high on open, low when closed*/

#define LEFT_ARM_SERVO_PIN 9
#define RIGHT_ARM_SERVO_PIN 10
#define LED_DATA_PIN 7

/********************************************/

#define PARTY_RANGE_DIST 2000
#define IN_RANGE_DIST 6500

#define MAX_OUT_OF_RANGE_LOOPS 0 //max loops out of range before transitioning... 0 for now, median filter is effective
#define MAX_ULTRASONIC_READINGS 5

#define ULTRASONIC_POLL_INTERVAL_MS 20
#define BUTTON_TRIGGER_MIN_INTERVAL 650

/*****state variables***********/
States currentState = ALL_OFF;

int brightness = 96; //brightness of the LEDs
int outOfRangeLoopCount = 0; //how many times ultrasonic has read out of range...
int pokeCount;

Servo leftArm;
Servo rightArm; 
int servoPos = 0;    // variable to store the servo position

bool moveUp = true;
int lastServoUpdate = -1;

void setup() {
  delay(1000); // 3 second delay for recovery

  leftArm.attach(LEFT_ARM_SERVO_PIN);
  rightArm.attach(RIGHT_ARM_SERVO_PIN);

  Serial.begin(57600);
  ///pins setup

  pinMode(ULTRASONIC_PIN, INPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SPEAKER_PIN, OUTPUT);

  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,LED_DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,LED_DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

int lastUpdateTime = -1;
int ultrasonicReadingCount = 0;
int ultrasonicData[MAX_ULTRASONIC_READINGS];

int currentDist = 0;
int buttonPressCount = 0;

int lastButtonTrigger =0; 
bool buttonWentHigh = true;

void loop()
{
  if(millis()-lastUpdateTime > ULTRASONIC_POLL_INTERVAL_MS)
  {
    updateSensors();
    evaluateState(); //update the state using sensor values (currentDist and buttonPressCount)

    lastUpdateTime = millis();
  }

  switch(currentState)
  {
      case ALL_OFF:
      {
        turnOffLights();
        turnOffSound();
        resetArms();
        
        Serial.println("ALL OFF");
        break;
      }

      case APPROACHING:
      {
        setLightProportional(currentDist);
        Serial.println("Approaching");
        break;
      }

      case IN_RANGE:
      {
        partyMode();
        Serial.println("Party");
        break;
      }

      case RAGE:
      { 
        digitalWrite(SPEAKER_PIN, HIGH);
        flashRed();
        moveArms();
        Serial.println("Rage");
        break;
      }
  }
}

/**sorts an array in ascending order, in place**/
void sort(int a[], int size) {
    for(int i=0; i<(size-1); i++) {
        for(int o=0; o<(size-(i+1)); o++) {
                if(a[o] > a[o+1]) {
                    int t = a[o];
                    a[o] = a[o+1];
                    a[o+1] = t;
                }
        }
    }
}


/**
Reads ultrasonic data from the sensor

//once every 20 ms, i should get a new ultrasonic value
//and add it to the array of values
//when the count equals 5, i should sort it, find the value,

Returns data about distance (int)
**/
void readUltrasonicSensor()
{
  if(millis()-lastUpdateTime > ULTRASONIC_POLL_INTERVAL_MS)
  {
    if(ultrasonicReadingCount >= MAX_ULTRASONIC_READINGS)
    {
        ultrasonicReadingCount=0;
        sort(ultrasonicData, MAX_ULTRASONIC_READINGS);

        //update current distance using median 
        currentDist = ultrasonicData[MAX_ULTRASONIC_READINGS/2];
    }
    else
    {
      ultrasonicData[ultrasonicReadingCount] = pulseIn(ULTRASONIC_PIN, HIGH);
      ultrasonicReadingCount++;
    }
  }
}

void checkButton()
{
   if(buttonWentHigh && digitalRead(BUTTON_PIN)==LOW && 
    ((millis() - lastButtonTrigger) > BUTTON_TRIGGER_MIN_INTERVAL))
  {
    buttonPressCount++;
    buttonWentHigh = false;
    
    if (buttonPressCount > 1)
    {
      buttonPressCount=0;
    }

    lastButtonTrigger = millis();
  }
  else if (digitalRead(BUTTON_PIN)==HIGH && buttonWentHigh == false)
  {
    buttonWentHigh = true;
  }
}

void updateSensors()
{
  readUltrasonicSensor();
  checkButton();
}

void evaluateState()
{
    switch(currentState)
    {
      case ALL_OFF:
      {        
        if(currentDist<IN_RANGE_DIST)
        {
          currentState = APPROACHING;
        }

        break;
      }

      case APPROACHING:
      {
        if(buttonPressCount > 0)
        {
          currentState = RAGE;
        }
        else if(currentDist < PARTY_RANGE_DIST)
        {
          currentState = IN_RANGE;
          outOfRangeLoopCount=0;
        }
        else if(currentDist>IN_RANGE_DIST && outOfRangeLoopCount>= MAX_OUT_OF_RANGE_LOOPS)
        {
          currentState = ALL_OFF;
          outOfRangeLoopCount=0;
        }
        else if(currentDist>IN_RANGE_DIST)
        {
          outOfRangeLoopCount++;
        }

        break;
      }

      case IN_RANGE:
      {
        if(buttonPressCount > 0)
        {
          currentState = RAGE;
        }
        else if(currentDist>PARTY_RANGE_DIST && outOfRangeLoopCount>= MAX_OUT_OF_RANGE_LOOPS)
        {
          currentState = APPROACHING;
        }
        else if (currentDist>PARTY_RANGE_DIST)
        {
          outOfRangeLoopCount++;
        }

        break;
      }

      case RAGE:
      {
        if(currentDist>IN_RANGE_DIST)
        {
          buttonPressCount=0;
          currentState = ALL_OFF;
        }
        else if(buttonPressCount==0)
        {
          currentState = ALL_OFF;
        }

        break;
      }
    }
}

void moveArms()
{
  if(millis()-lastServoUpdate > 15)
  {
    if(moveUp)
    {
      servoPos+=20;
      leftArm.write(servoPos);
      rightArm.write(servoPos);

      if(servoPos>=140)
      {
        moveUp = false;
      }
    }
    else
    {
      servoPos-=20;
      leftArm.write(servoPos);
      rightArm.write(servoPos);

      if(servoPos<=0)
      {
        moveUp = true;
      }
    }

    lastServoUpdate = millis();
  }
}


void resetArms()
{
  leftArm.write(180);
  rightArm.write(0);
}

void setLightProportional(int distance)
{
  int brightness = map(distance, 6500, 2000, 30, 255);

  for(int i=0; i< NUM_LEDS; i++)
  {
    leds[i] = CRGB(brightness, brightness, brightness);
  }

  FastLED.show();
}

void turnOffSound()
{
   digitalWrite(SPEAKER_PIN, LOW);
}

void turnOffLights()
{
  for(int i=0; i< NUM_LEDS; i++)
  {
    leds[i] = 0;
  }

  FastLED.show();
}


void setAllLedsWhite()
{
  for(int i=0; i< NUM_LEDS; i++)
  {
    leds[i] = CHSV(0, 0, 100);
  }

  FastLED.show();
}

void flashRed()
{
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = CRGB(255, 0, 0);
  }
  
  FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND);

  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = CRGB(0, 0, 0);
  }
  
  FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND);
}

/************
 * Stock LED functions below
 */
void partyMode()
{
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND);

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically
}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter()
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter)
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}
