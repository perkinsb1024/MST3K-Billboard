#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

#define LED_A_PIN 3
#define LED_B_PIN 4
#define BUTTON_PIN 2

#define LONG_PRESS_DURATION_MS 1000
#define DEBOUNCE_DELAY_MS 100
#define BTN_NO_PRESS 0
#define BTN_SHORT_PRESS 1
#define BTN_LONG_PRESS 2

#define FIRST_BLINK_DURATION_MS 4000
#define BLINK_DURATION_MS 2500
#define DELAY_XLONG 30000
#define DELAY_LONG 10000
#define DELAY_SHORT 5000

#define EEPROM_BRIGHTNESS_ADDR 0
#define EEPROM_SAVE_DELAY_MS 5000

#define BRIGHTNESS_STEPS 32
#define FULL_BRIGHTNESS (BRIGHTNESS_STEPS)
#define HALF_BRIGHTNESS (BRIGHTNESS_STEPS >> 1)
#define QUARTER_BRIGHTNESS (BRIGHTNESS_STEPS >> 2)
#define EIGHTH_BRIGHTNESS (BRIGHTNESS_STEPS >> 3)
#define MIN_BRIGHTNESS 1
#define OFF 0

uint8_t globalBrightness = EEPROM.read(EEPROM_BRIGHTNESS_ADDR);
bool globalButtonReady = true;

ISR(PCINT0_vect) {
  // Define the ISR so that the ATTiny doesn't reset when it's triggered, but it doesn't need to do anything
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
    delay(DEBOUNCE_DELAY_MS);
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
    globalBrightness = MIN_BRIGHTNESS;
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
  bool incrementBrightness = false;
  bool saveNewBrightness = false;
  uint32_t brightnessChangeMs = millis();
  uint8_t buttonPressType = BTN_NO_PRESS;
  while (1) {
    // Check if we need to change the brightness due to a button press
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
      // This means that if the user is cycling through brightness levels, we only write the last one to EEPROM
      saveNewBrightness = false;
      // It probably doesn't matter, but EEPROM can be read more than it can be written, so don't bother writing if it's the same value
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
