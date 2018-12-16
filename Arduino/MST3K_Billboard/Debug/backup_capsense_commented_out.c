#include <CapacitiveSensorImproved.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

#define LED_A_PIN 3
#define LED_B_PIN 4
#define BUTTON_PIN 2
#define TX_PIN 0
#define RX_PIN 1

//#define CAP_SENSE_AUTOCAL_MS 5000
//#define CAP_SENSE_AUTOCAL_OFF 0xFFFFFFFF
//#define CAP_TOUCH_THRESHOLD 32

#define ENABLE_CAP_SENSE 0

#define LONG_PRESS_DURATION_MS 1000
#define DEBOUNCE_DELAY_CYCLES 2500
#define BTN_NO_PRESS 0
#define BTN_SHORT_PRESS 1
#define BTN_LONG_PRESS 2

#define FIRST_BLINK_DURATION_MS 4000
#define BLINK_DURATION_MS 2500
#define DELAY_XLONG 30000
#define DELAY_LONG 10000
#define DELAY_SHORT 5000

#define EEPROM_BRIGHTNESS_ADDR 0

#define BRIGHTNESS_STEPS 32
#define FULL_BRIGHTNESS (BRIGHTNESS_STEPS)
#define HALF_BRIGHTNESS (BRIGHTNESS_STEPS >> 1)
#define QUARTER_BRIGHTNESS (BRIGHTNESS_STEPS >> 2)
#define EIGHTH_BRIGHTNESS (BRIGHTNESS_STEPS >> 3)
#define MIN_BRIGHTNESS 1
#define OFF 0

//CapacitiveSensor touchSensor = CapacitiveSensor(TX_PIN, RX_PIN);
uint8_t globalBrightness = EEPROM.read(EEPROM_BRIGHTNESS_ADDR);
bool globalButtonReady = true;
//bool useCapSense = ENABLE_CAP_SENSE;

/*
   Function:  delayCycles
   --------------------
   Delays for a given number of cycles.

   n: number of cycles to wait.
*/
void delayCycles(uint16_t n) {
  while (n--) {
    asm("nop");
  }
}

ISR(PCINT0_vect) {
  // Define the ISR so that the micro doesn't reset when it's triggered, but it doesn't need to do anything
}

void sleep() {
  // From https://bigdanzblog.wordpress.com/2014/08/10/attiny85-wake-from-sleep-on-pin-state-change-code-example/
  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT2);                   // Use PB2 as interrupt pin
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // Go to power down mode when sleeping
  
  sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                  // Enable interrupts
  sleep_cpu();                            // Sleep
  // Code execution will resume here when an interrupt is triggered
  cli();                                  // Disable interrupts
  PCMSK &= ~_BV(PCINT2);                  // Turn off PB3 as interrupt pin
  sleep_disable();                        // Clear SE bit
  sei();                                  // Enable interrupts so timing functions work
}

inline bool buttonPressed() {
  return !digitalRead(BUTTON_PIN);
}

uint8_t getButtonPress() {
  uint32_t firstPressTime = millis();
  if (buttonPressed()) {
    // Don't accept another button press until this one ends
    if (!globalButtonReady) {
      return BTN_NO_PRESS;
    }
    globalButtonReady = false;
    delayCycles(DEBOUNCE_DELAY_CYCLES);
    while (buttonPressed()) {
      if (millis() - firstPressTime > LONG_PRESS_DURATION_MS) {
        return BTN_LONG_PRESS;
      }
    }
    return BTN_SHORT_PRESS;
  }
  // If we got here, the button isn't pressed, so we're ready to read another
  globalButtonReady = true;
  return BTN_NO_PRESS;
}

void setLeds(uint8_t brightnessA, uint8_t brightnessB, uint16_t cycles) {
  do {
    for (uint8_t i = 0; i < BRIGHTNESS_STEPS; i++) {
      digitalWrite(LED_A_PIN, i < brightnessA);
      digitalWrite(LED_B_PIN, i < brightnessB);
    }
  }
  while (cycles--);
}

uint8_t blinkEffect(uint8_t brightness, uint16_t durationMs) {
  uint32_t endTime = millis() + durationMs;
  uint8_t buttonPressMode = BTN_NO_PRESS;
  while (millis() < endTime) {
    setLeds(brightness, OFF, 6);
    setLeds(OFF, OFF, 6);
    setLeds(OFF, brightness, 6);
    setLeds(OFF, OFF, 6);
    
    // Check the button to end early
    buttonPressMode = getButtonPress();
    if (buttonPressMode != BTN_NO_PRESS) {
      return buttonPressMode;
    }
  }
  return buttonPressMode;
}

void pokeMode() {
  uint8_t buttonPressType = BTN_NO_PRESS;
  // Don't allow a brightness of 'OFF' in this mode
  if (globalBrightness == OFF) {
    globalBrightness = QUARTER_BRIGHTNESS;
  }
  while (1) {
    // Put the CPU to sleep until an interrupt (button press) wakes it up
    sleep();
    buttonPressType = getButtonPress();
    // Using a while loop because the button may be pressed again during the blink effect
    //  which will restart the effect
    while (buttonPressType == BTN_SHORT_PRESS) {
      buttonPressType = blinkEffect(globalBrightness, BLINK_DURATION_MS);
    }
    if (buttonPressType == BTN_LONG_PRESS) {
      // We're about to leave this mode, so give the user a brief indication
      setLeds(FULL_BRIGHTNESS, FULL_BRIGHTNESS, 1);
      setLeds(OFF, OFF, 100);
      return;
    }
  }
}

void manualMode() {
  #define EEPROM_SAVE_DELAY_MS 5000
  int16_t delayCount = DELAY_LONG;
  bool incrementBrightness = false;
  bool saveNewBrightness = false;
  uint32_t brightnessChangeMs = millis();
  uint8_t buttonPressType = BTN_NO_PRESS;
  while (1) {
    if (incrementBrightness) {
      switch (globalBrightness) {
        case FULL_BRIGHTNESS:
          globalBrightness = MIN_BRIGHTNESS;
          break;
        case MIN_BRIGHTNESS:
          globalBrightness = QUARTER_BRIGHTNESS;
          break;
        case QUARTER_BRIGHTNESS:
          globalBrightness = HALF_BRIGHTNESS;
          break;
        case HALF_BRIGHTNESS:
        default:
          globalBrightness = FULL_BRIGHTNESS;
          break;
      }
      incrementBrightness = false;
      saveNewBrightness = true;
      brightnessChangeMs = millis();
    }
    buttonPressType = blinkEffect(globalBrightness, BLINK_DURATION_MS);
    if(saveNewBrightness && millis() > brightnessChangeMs + EEPROM_SAVE_DELAY_MS) {
      // Write the brightness value to EEPROM if it has changed and it's been more than 5 seconds
      saveNewBrightness = false;
      if(EEPROM.read(EEPROM_BRIGHTNESS_ADDR) != globalBrightness) {
        EEPROM.write(EEPROM_BRIGHTNESS_ADDR, globalBrightness);
      }
    }

    if (buttonPressType == BTN_SHORT_PRESS) {
      incrementBrightness = true;
    }
    else if (buttonPressType == BTN_LONG_PRESS) {
      // We're about to leave this mode, so give the user a brief indication
      setLeds(FULL_BRIGHTNESS, FULL_BRIGHTNESS, 1);
      setLeds(OFF, OFF, 20);
      if(saveNewBrightness) {
        // If the brightness has changed, save the new value regardless of how long ago it happened, since we're about to return
        saveNewBrightness = false;
        if(EEPROM.read(EEPROM_BRIGHTNESS_ADDR) != globalBrightness) {
          EEPROM.write(EEPROM_BRIGHTNESS_ADDR, globalBrightness);
        }
      }
      return;
    }
  }
}

void setup() {
  // Power saving functions.
  ACSR |= _BV(ACD); // Disable ADC.
  ADCSRA &= ~_BV(ADEN); // Disable ADC.
  // Unlock watchdog to allow changes (see datasheet section 8.5.2).
  WDTCR |= (_BV(WDCE) | _BV(WDE));
  // Disable watchdog.
  WDTCR &= ~_BV(WDE);
  WDTCR &= ~_BV(WDIE);

  // Pin setup
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  /*
    // Capacitive Sensor
    touchSensor.set_CS_Autocal_Millis(CAP_SENSE_AUTOCAL_MS);

    // If the button is pressed at start-up, go into cap-sense mode
    if(buttonPressed()) {
      useCapSense = true;
    }
  */

  // Fade in and out on power-up (helps to ensure the programmer is attached)
  for(uint8_t fade = 0; fade <= FULL_BRIGHTNESS; fade++) {
    setLeds(fade, fade, 5);
  }
  for(uint8_t fade = 0; fade <= FULL_BRIGHTNESS; fade++) {
    setLeds(FULL_BRIGHTNESS-fade, FULL_BRIGHTNESS-fade, 5);
  }
}

void loop() {
  // Start with "poke mode" - LEDs blink for a bit when the button is pressed
  pokeMode();
  // Next mode is "manual" - Button press controls brightness, but LEDs are always blinking
  manualMode();
}

/*
  void testCapSense() {
  #define SAMPLING_COUNT 50

  float baselineReading = 0;
  float touchReading = 0;
  uint16_t diff = 0;

  delayCycles(DELAY_XLONG);

  while(1) {
    if(!digitalRead(BUTTON_PIN)) {
      delayCycles(DELAY_XLONG);
      for(uint8_t i = 0; i < BRIGHTNESS_STEPS; i++) {
        setLeds(true, false, i, 3);
        delayCycles(DELAY_SHORT);
      }
      delayCycles(DELAY_XLONG);
      baselineReading = touchSensor.capacitiveSensorRaw(SAMPLING_COUNT);
      setLeds(true, false, FULL_BRIGHTNESS, 3);
      delayCycles(DELAY_XLONG);
      setLeds(false, false, OFF, 3);
    }

    touchReading = touchSensor.capacitiveSensorRaw(SAMPLING_COUNT);
    if(touchReading > baselineReading) {
      diff = touchReading - baselineReading;
      if(diff > BRIGHTNESS_STEPS) {
        setLedsDual(diff & 0xFF, (diff - 32) & 0xFF, 1);
      }
      else {
        setLedsDual(diff & 0xFF, 0, 1);
      }
    }
    digitalWrite(LED_A_PIN, LOW);
    digitalWrite(LED_B_PIN, LOW);
  }
  }

  void testCapSense_working_ish() {
  // Working values: Total increase 20, total decrease -15 (no singles) had no false positives. A little hard to trigger
  #define MIN_SINGLE_INCREASE 7
  #define MIN_TOTAL_INCREASE 22
  #define MIN_SINGLE_DECREASE (0)
  #define MIN_TOTAL_DECREASE (-17)
  #define MAX_TOUCH_DURATION 4
  #define SAMPLING_COUNT 50

  long currentMs = 0;
  long endTime = 0;
  long lastLastReading = 0;
  long lastReading = 0;
  long curReading = 0;
  uint8_t bufferIndex = 0;
  uint8_t increaseHappened = 0;
  int32_t derivative = 0;

  bool touchIsValid = false;
  bool needsReset = true;

  while(1) {
    if(needsReset) {
      // Reset state
      touchIsValid = false;
      needsReset = false;
      increaseHappened = 0;
      curReading = touchSensor.capacitiveSensorRaw(SAMPLING_COUNT);
      lastReading = curReading;
      lastLastReading = lastReading;
    }

    curReading = touchSensor.capacitiveSensorRaw(SAMPLING_COUNT);
    derivative = (curReading - lastLastReading) + (lastReading - lastLastReading);
    lastLastReading = lastReading;
    lastReading = curReading;
    if(increaseHappened == 0) {
      // Waiting for a touch
      if(derivative >= MIN_TOTAL_INCREASE
        && curReading - lastLastReading >= MIN_SINGLE_INCREASE
        && lastReading - lastLastReading >= MIN_SINGLE_INCREASE)
      {
        increaseHappened = MAX_TOUCH_DURATION;
      }
    }
    else {
      // Touch recently happened, check for a non-touch
      if(derivative <= MIN_TOTAL_DECREASE
        && curReading - lastLastReading <= MIN_SINGLE_DECREASE
        && lastReading - lastLastReading <= MIN_SINGLE_DECREASE)
      {
        touchIsValid = true;
      }
      increaseHappened--;
    }

    // Pressing the button also counts as a valid "touch"
    if(!digitalRead(BUTTON_PIN)) {
      delayCycles(DEBOUNCE_DELAY_CYCLES);
      if(!digitalRead(BUTTON_PIN)) {
        touchIsValid = true;
      }
    }

    if(touchIsValid) {
      // Touch or button press occurred, blink the LEDs for a bit
      currentMs = millis();
      endTime = currentMs + BLINK_DURATION_MS;
      while(endTime > currentMs) {
        setLeds(true, false, FULL_BRIGHTNESS, 3);
        setLeds(false, false, OFF, 9);
        setLeds(false, true, FULL_BRIGHTNESS, 3);
        setLeds(false, false, OFF, 9);
        currentMs = millis();
      }
      needsReset = true;
    }
  }
  }

  void notGoodSmartCapSense() {
  // Feature 1 = Sum of last N values > THRESHOLD1
  // Feature 2 = Last N values > THRESHOLD2
  // Feature 3 = No more than N consecutive values > THRESHOLD 3
  // True if 3 happens within N cycles of 1 && 2

  #define INTEGRAL_THRESHOLD 55
  #define INTEGRAL_RANGE 5
  #define INTEGRAL_COUNT 1
  #define MIN_CONSECUTIVE_THRESHOLD 16
  #define MIN_CONSECUTIVE_COUNT 2
  #define MAX_CONSECUTIVE_THRESHOLD 10
  #define MAX_CONSECUTIVE_COUNT 10
  #define MAX_CONSECUTIVE_RESET_COUNT 25
  #define TRIGGER_VALID_DURATION 4

  long currentMs = 0;
  long endTime = 0;
  long capVal = 0;
  long sampleBuffer[INTEGRAL_RANGE];
  uint8_t bufferIndex = 0;
  long integral = 0;
  uint8_t cyclesAboveIntegral = 0;
  uint8_t integralIsValid = 0;
  uint8_t cyclesAboveMinThreshold = 0;
  uint8_t minThresholdIsValid = 0;
  uint8_t cyclesAboveMaxThreshold = 0;
  bool touchIsValid = false;
  bool needsReset = false;

  // Clear the buffer
  for(bufferIndex = 0; bufferIndex < INTEGRAL_RANGE; bufferIndex++) {
    sampleBuffer[bufferIndex] = 0;
  }

  while(1) {
    // Increment the circular buffer index
    bufferIndex++;
    if(bufferIndex >= INTEGRAL_RANGE) { bufferIndex = 0; }
    // Remove the oldest reading from the current integral
    if(integral >= sampleBuffer[bufferIndex]) {
      integral -= sampleBuffer[bufferIndex];
    }
    else {
      integral = 0;
    }
    // Read a new capacitive sensor value
    capVal = touchSensor.capacitiveSensor(15);
    // Increment the integral and save the reading into the buffer
    integral += capVal;
    sampleBuffer[bufferIndex] = capVal;

    // Check the features
    // Integral feature
    if(integral >= INTEGRAL_THRESHOLD) {
      cyclesAboveIntegral++;
      if(cyclesAboveIntegral >= INTEGRAL_COUNT) {
        integralIsValid = TRIGGER_VALID_DURATION;
        // Prevent overflow
        cyclesAboveIntegral = INTEGRAL_COUNT;
      }
    }
    else {
      setLeds(false, false, OFF, 6);
      cyclesAboveIntegral = 0;
    }
    // Minimum threshold feature
    if(capVal >= MIN_CONSECUTIVE_THRESHOLD) {
      cyclesAboveMinThreshold++;
      if(cyclesAboveMinThreshold >= MIN_CONSECUTIVE_COUNT) {
        minThresholdIsValid = TRIGGER_VALID_DURATION;
        // Prevent overflow
        cyclesAboveMinThreshold = MIN_CONSECUTIVE_COUNT;
      }
    }
    else {
      cyclesAboveMinThreshold = 0;
    }
    // Maximum threshold feature
    if(capVal >= MAX_CONSECUTIVE_THRESHOLD) {
      cyclesAboveMaxThreshold++;
      if(cyclesAboveMaxThreshold >= MAX_CONSECUTIVE_RESET_COUNT) {
        // If the threshold has been triggered for too long, re-calibrate the touch sensor and reset the state
        setLeds(true, true, QUARTER_BRIGHTNESS, 6);
        delayCycles(DELAY_LONG);
        setLeds(false, false, OFF, 6);
        touchSensor.reset_CS_AutoCal();
        needsReset = true;
      }
    }
    else {
      // When the threshold drops back down, check if the other conditions for a valid touch were met
      if(cyclesAboveMaxThreshold <= MAX_CONSECUTIVE_COUNT
        &&  integralIsValid > 0
        &&  minThresholdIsValid > 0)
      {
        touchIsValid = true;
      }
      cyclesAboveMaxThreshold = 0;
    }

    // Count down since the last triggers
    if(integralIsValid > 0) {
      integralIsValid--;
    }
    if(minThresholdIsValid > 0) {
      minThresholdIsValid--;
    }

    if(touchIsValid) {
      currentMs = millis();
      endTime = currentMs + BLINK_DURATION_MS;
      while(endTime > currentMs) {
        setLeds(true, false, FULL_BRIGHTNESS, 6);
        setLeds(false, false, OFF, 6);
        setLeds(false, true, FULL_BRIGHTNESS, 6);
        setLeds(false, false, OFF, 6);
        currentMs = millis();
      }
      needsReset = true;
    }

    if(needsReset) {
      // Reset state
      touchSensor.reset_CS_AutoCal();
      touchIsValid = false;
      needsReset = false;
      cyclesAboveIntegral = 0;
      cyclesAboveMinThreshold = 0;
      cyclesAboveMaxThreshold = 0;
      integralIsValid = 0;
      minThresholdIsValid = 0;
      integral = 0;
      for(bufferIndex = 0; bufferIndex < INTEGRAL_RANGE; bufferIndex++) {
        sampleBuffer[bufferIndex] = 0;
      }
    }
  }
  }

  void cyclePins() {
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH);
  digitalWrite(RX_PIN, HIGH);
  digitalWrite(TX_PIN, LOW);
  digitalWrite(RX_PIN, LOW);
  }

  void quickBlink() {
  setLeds(true, true, FULL_BRIGHTNESS, 1);
  delayCycles(DELAY_XLONG);
  delayCycles(DELAY_XLONG);
  setLeds(false, false, OFF, 1);
  delayCycles(DELAY_XLONG);
  delayCycles(DELAY_XLONG);
  }

  void dumbCapSense() {
  #define CAP_TOUCH_THRESHOLD 48
  #define BUFFER_SIZE 6
  #define SAMPLING_COUNT 50
  static uint16_t delayCount = DELAY_LONG;
  long endTime = 0;
  long curTime;
  long capVal;
  int32_t integral = 0;
  long sampleBuffer[BUFFER_SIZE];
  uint8_t bufferIndex = 0;
  bool touchIsValid = false;
  bool needsReset = true;

  while(1) {
    if(needsReset) {
      // Reset state
      touchSensor.reset_CS_AutoCal();
      touchIsValid = false;
      needsReset = false;
      // Clear the buffer
      for(bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
        sampleBuffer[bufferIndex] = 0;
      }
      bufferIndex = 0;
      integral = 0;
    }
    capVal = touchSensor.capacitiveSensor(SAMPLING_COUNT);

    // Remove the oldest value from the integral
    integral -= sampleBuffer[bufferIndex];
    // Save the new value...
    sampleBuffer[bufferIndex] = capVal;
    // ...and add it to the integral
    integral += sampleBuffer[bufferIndex];
    // Increment rolling buffer index...
    bufferIndex++;
    // ...and wrap around if needed
    if(bufferIndex >= BUFFER_SIZE) { bufferIndex = 0; }

    if(integral >= CAP_TOUCH_THRESHOLD) {
      touchIsValid = true;
    }

    if(!digitalRead(BUTTON_PIN)) {
      for(uint32_t fp = 0; fp < falsePositives; fp++) {
        quickBlink();
      }
    }

    if(touchIsValid) {
      falsePositives++;
      needsReset = true;
      curTime = millis();
      endTime = curTime + BLINK_DURATION_MS;
      while(endTime > curTime) {
        setLeds(true, false, FULL_BRIGHTNESS, 3);
        setLeds(false, false, OFF, 9);
        setLeds(false, true, FULL_BRIGHTNESS, 3);
        setLeds(false, false, OFF, 9);
        curTime = millis();
      }
    }
  }
  }
*/