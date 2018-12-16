void testCapSense() {
  // Feature 1 = Sum of last N values > THRESHOLD1
  // Feature 2 = Last N values > THRESHOLD2
  // Feature 3 = No more than N consecutive values > THRESHOLD 3
  // True if 3 happens within N cycles of 1 && 2

  #define INTEGRAL_THRESHOLD 50
  #define INTEGRAL_RANGE 4
  #define INTEGRAL_COUNT 1
  #define MIN_CONSECUTIVE_THRESHOLD 5
  #define MIN_CONSECUTIVE_COUNT 4
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
    // Remove the oldest reading from the current integral
    integral -= sampleBuffer[bufferIndex];
    // Increment the circular buffer index
    bufferIndex++;
    if(bufferIndex >= INTEGRAL_RANGE) { bufferIndex = 0; }
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
      endTime = currentMs + POWER_DURATION_MS;
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