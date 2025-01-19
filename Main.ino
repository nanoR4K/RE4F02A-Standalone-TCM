// Define pin numbers for buttons and outputs
const int buttonUpPin = 3;      // + button
const int buttonDownPin = 2;    // - button
const int buttonOnOffPin = 4;   // "on/off" switch
const int buttonODPin = 8;      // "OD" switch
const int outputAPin = 5;       // Output Solenoid A
const int outputBPin = 6;       // Output Solenoid B
const int outputDPin = 7;       // Output on/off
const int outputTPin = 9;       // Output Timing Solenoid
const int outputTCCPin = 10;    // Output Torque Converter Clutch Solenoid
const int potPin = A1;          // Input TPS
const int pwmOutputPin = 11;    // PWM output Line Pressure Solenoid
const int aInPin = 12;          // TCM Solenoid A 
const int bInPin = 13;          // TCM Solenoid B 

// Define states
enum State {
  STATE_1,
  STATE_2,
  STATE_3,
  STATE_4,
  STATE_DEACTIVATE
};

State currentState = STATE_1; // Start at state 1
State previousState = STATE_1; // Variable to store the previous state
bool d7State = LOW;            // Initial state for D7
bool lastButtonState = HIGH;   // Last state of the on/off button
bool lastButtonState2 = HIGH;  // Last state of the second on/off button
bool isOn = false;             // State of the on/off switch (inverted)
bool isOn2 = false;            // State of the second on/off switch
bool rampInProgress = false;   // Flag to track if ramp is in progress

// TPS calibration adjustments
float minVoltage = 0.6;        
float maxVoltage = 3.9;    

int rampDuration = 1000; // 2 seconds for ramp-up

void setup() {
  // Initialize button pins as input
  pinMode(buttonUpPin, INPUT_PULLUP);
  pinMode(buttonDownPin, INPUT_PULLUP);
  pinMode(buttonOnOffPin, INPUT_PULLUP);
  pinMode(buttonODPin, INPUT_PULLUP);
  
  // Initialize output pins
  pinMode(outputAPin, OUTPUT);
  pinMode(outputBPin, OUTPUT);
  pinMode(outputDPin, OUTPUT);
  pinMode(outputTPin, OUTPUT); // Initialize output T
  pinMode(outputTCCPin, OUTPUT); // Initialize output TCC
  pinMode(pwmOutputPin, OUTPUT); // Initialize PWM output (LP)

  // Set initial output state
  updateOutputs();
  digitalWrite(outputDPin, d7State); // Set initial state for D7
}

void loop() {
  // Read button states once
  bool buttonUpState = digitalRead(buttonUpPin);
  bool buttonDownState = digitalRead(buttonDownPin);
  bool buttonOnOffState = digitalRead(buttonOnOffPin);
  bool buttonODState = digitalRead(buttonODPin);
  int aInState = digitalRead(aInPin); // Read A in state
  int bInState = digitalRead(bInPin); // Read B in state

  // Check for simultaneous button presses
  if (buttonUpState == LOW && buttonDownState == LOW) {
    previousState = currentState; // Store the current state before deactivating
    currentState = STATE_DEACTIVATE; // Set to deactivate state
    updateOutputs();
    delay(200); // Debounce delay
  } else {
    // Handle button up
    if (buttonUpState == LOW && isOn && currentState < STATE_4) {
      previousState = currentState; // Store the previous state
      currentState = (State)(currentState + 1);
      updateOutputs();
      delay(200); // Debounce delay
    }

    // Handle button down
    if (buttonDownState == LOW && isOn && currentState > STATE_1) {
      previousState = currentState; // Store the previous state
      currentState = (State)(currentState - 1);
      updateOutputs();
      delay(200); // Debounce delay
    }
  }

  // Handle the first on/off button state
  if (lastButtonState == HIGH && buttonOnOffState == LOW) {
    isOn = !isOn; // Toggle the first on/off state
    digitalWrite(outputDPin, isOn ? HIGH : LOW); // Update D7 output (inverted)
    
    // If the first switch is ON, set the current state based on A in and B in
    if (isOn) {
      // Determine the current state based on A in and B in
      if (aInState == HIGH && bInState == HIGH) {
        previousState = currentState; 
        currentState = STATE_1;
      } else if (aInState == LOW && bInState == HIGH) {
        previousState = currentState; 
        currentState = STATE_2;
      } else if (aInState == LOW && bInState == LOW) {
        previousState = currentState; 
        currentState = STATE_3;
      } else if (aInState == HIGH && bInState == LOW) {
        previousState = currentState; 
        currentState = STATE_4;
      }
      
      updateOutputs(); // Update outputs based on the new state
    }
    delay(200); // Debounce delay
  }
  lastButtonState = buttonOnOffState; // Update last button state

  // Handle the second on/off button state
  if (lastButtonState2 == HIGH && buttonODState == LOW) {
    isOn2 = !isOn2; // Toggle the second on/off state
    updateOutputs(); // Update outputs TCC based on the new state
    delay(200); // Debounce delay
  }
  lastButtonState2 = buttonODState; // Update last button state

  // Read the potentiometer value and map it to the specified voltage range
  int potValue = analogRead(potPin); // Read the potentiometer
  float voltageRange = maxVoltage - minVoltage; // Calculate the voltage range
  float mappedVoltage = (potValue / 1023.0) * voltageRange + minVoltage; // Map to the specified voltage range
  int pwmValue = map(mappedVoltage * 255.0 / maxVoltage, 0, 255, 0, 255); // Map to PWM range (0-255)
  analogWrite(pwmOutputPin, pwmValue); // Set the PWM output (LP)

  // Update outputs based on the current state
  updateOutputs();
}

// Function to ramp up PWM signal
void rampUpPWM(int pin, int targetValue, int duration) {
  int currentValue = 0;
  int stepDelay = duration / targetValue; // Calculate delay per step

  // Gradually increase the PWM value
  for (currentValue = 0; currentValue <= targetValue; currentValue++) {
    analogWrite(pin, currentValue); // Set the PWM value
    delay(stepDelay); // Wait for the specified duration
  }
}

void updateOutputs() {
  // Set outputs based on the current state
  if (currentState == STATE_DEACTIVATE) {
    // Deactivate all outputs
    digitalWrite(outputAPin, LOW);
    digitalWrite(outputBPin, LOW);
    digitalWrite(outputTPin, LOW); // Deactivate output T
    digitalWrite(outputTCCPin, LOW); // Deactivate output TCC
    rampInProgress = false; // Reset ramp flag
    currentState = previousState; // Return to the previous state
  } else if (!isOn) {
    // If the first on/off switch is OFF, deactivate outputs A, B, T, and TCC
    digitalWrite(outputAPin, LOW);
    digitalWrite(outputBPin, LOW);
    digitalWrite(outputTPin, LOW); // Deactivate output T
    digitalWrite(outputTCCPin, LOW); // Deactivate output TCC
    rampInProgress = false; // Reset ramp flag
  } else {
    // Set outputs based on the current state
    switch (currentState) {
      case STATE_1:
        digitalWrite(outputAPin, HIGH);
        digitalWrite(outputBPin, HIGH);
        digitalWrite(outputTPin, HIGH); // Activate output T
        digitalWrite(outputTCCPin, LOW); // Deactivate output TCC
        rampInProgress = false; // Reset ramp flag
        break;
      case STATE_2:
        digitalWrite(outputAPin, LOW);
        digitalWrite(outputBPin, HIGH);
        digitalWrite(outputTPin, LOW); // Deactivate output T
        digitalWrite(outputTCCPin, LOW); // Deactivate output TCC
        rampInProgress = false; // Reset ramp flag
        break;
      case STATE_3:
        digitalWrite(outputAPin, LOW);
        digitalWrite(outputBPin, LOW);
        digitalWrite(outputTPin, LOW); // Deactivate output T
        if (isOn2) { // Check if TCC should be activated
          digitalWrite(outputTCCPin, HIGH); // Activate output TCC
          if (!rampInProgress) { // Ramp only if not already in progress
            rampUpPWM(outputTCCPin, 255, rampDuration); // Ramp up to full PWM for TCC (0 to 5V)
            rampInProgress = true; // Set ramp flag to true
          }
        } else {
          digitalWrite(outputTCCPin, LOW); // Deactivate output TCC
          rampInProgress = false; // Reset ramp flag if TCC is deactivated
        }
        break;
      case STATE_4:
        digitalWrite(outputAPin, HIGH);
        digitalWrite(outputBPin, LOW);
        digitalWrite(outputTPin, HIGH); // Activate output T
        if (isOn2) { // Check if TCC should be activated
          digitalWrite(outputTCCPin, HIGH); // Activate output TCC
          if (!rampInProgress) { // Ramp only if not already in progress
            rampUpPWM(outputTCCPin, 255, rampDuration); // Ramp up to full PWM for TCC (0 to 5V)
            rampInProgress = true; // Set ramp flag to true
          }
        } else {
          digitalWrite(outputTCCPin, LOW); // Deactivate output TCC
          rampInProgress = false; // Reset ramp flag if TCC is deactivated
        }
        break;
    }
  }
}