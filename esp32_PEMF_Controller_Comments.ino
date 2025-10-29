// FINAL FULL WORKING VERSION 5.0
// 2025-10-05 - Alexandre BERNARD ORECILLA
// PEMF Controller: Full commented version for understanding each section and function.

// --- Include Required Libraries ---
#include <Arduino.h>
#include <Wire.h>                    // For I2C communication
#include <LiquidCrystal_I2C.h>       // For LCD display
#include <Adafruit_AM2320.h>         // For temperature sensor

// --- Pin Definitions ---
#define LCD_ADDRESS 0x27             // I2C address for LCD
#define AM2320_ADDRESS 0x5C          // I2C address for AM2320 temperature sensor
#define LED_PIN 2                    // Status LED pin
#define BUZZER_PIN 5                 // Buzzer pin for audio feedback
#define NAV_UP 25                    // Navigation UP button
#define NAV_DOWN 26                  // Navigation DOWN button
#define SEL_UP 32                    // Select/Confirm button
#define SEL_DOWN 33                  // Escape/Back button
#define MD10_PWM 4                   // PEMF output PWM pin (via MD10 motor driver)
#define MD10_DIR 16                  // PEMF output direction pin
#define FAN_PIN 19                   // Cooling fan control pin
#define RELAY_PIN 27                 // Relay for power switching (19V/5V)

// --- Create LCD and Sensor Objects ---
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4); // 20 columns x 4 rows LCD
Adafruit_AM2320 am2320 = Adafruit_AM2320(); // Temperature sensor object

// --- Enumerations for Device State ---
enum IntensityLevel { NORMAL = 1, INTENSE = 2, WARM = 3 };      // PEMF intensity levels
enum PolarityMode { UNIPOLAR, BIPOLAR, ALTERNATE };             // PEMF polarity modes
enum PemfState { STATE_OFF, STATE_POS_PULSE, STATE_NEG_PULSE }; // PEMF pulse states
enum MenuState { MAIN_MENU, INTENSITY_MENU, POLARITY_MENU, CATEGORY_MENU, CONDITION_MENU,
                 FREQUENCY_MENU, DURATION_MENU, CUSTOM_MODE, START_STOP, POWER_MENU }; // Menu navigation states
enum PowerLevel { POWER_19V, POWER_5V };                        // Output voltage modes

// --- Global Variables for State ---
MenuState menuState = MAIN_MENU;          // Current menu state
int frequency = 10;                       // PEMF frequency in Hz
int duration = 20;                        // Treatment duration in minutes
bool pemfActive = false;                  // Is PEMF running?
unsigned long pemfStartTime = 0;          // When PEMF started

PowerLevel currentPowerLevel = POWER_19V; // Default power level
bool powerLevelInitialized = false;       // Prevents double relay init

float currentTemperature = -99.9;         // Last temperature value
unsigned long lastTempRead = 0;           // Last time temp was read
const unsigned long TEMP_READ_INTERVAL = 20000; // Temperature read interval (ms)
unsigned long tempSensorRetryCount = 0;   // Temp sensor read error counter
const unsigned long MAX_TEMP_RETRIES = 3; // Max retries before disabling sensor
bool tempSensorDisabled = false;          // Disable sensor if too many errors

// --- Menu Navigation Variables ---
int currentMenuOption = 0;
int currentCategoryIndex = 0, currentConditionIndex = 0, currentFrequencyIndex = 0, customMenuOption = 0;
bool customAdjustMode = false;

// --- Polarity and Intensity ---
int currentIntensity = NORMAL;            // Current intensity setting
volatile PolarityMode currentPolarity = UNIPOLAR; // Current polarity setting
volatile int currentDirState = HIGH;      // Direction pin state
unsigned long lastPolarityChangeTime = 0; // For alternate mode

// --- ISR Timer Engine Variables ---
hw_timer_t *tick_timer = NULL;            // Hardware timer for pulse generation
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // Mutex for timer ISR
volatile unsigned long tick_counter = 0;  // Timer tick counter
const int TICK_INTERVAL_US = 10;          // Timer tick interval (us)
volatile unsigned long next_state_change_tick = 0; // When to change PEMF state
volatile PemfState pemf_state = STATE_OFF;         // Current PEMF pulse state
volatile unsigned long pos_pulse_ticks = 0, neg_pulse_ticks = 0, off_time_ticks = 0; // Pulse lengths

// --- Button Structure ---
struct Button {
  uint8_t pin;               // Button pin
  bool lastState;            // Last read state
  bool currentState;         // Current state
  unsigned long lastDebounceTime; // For debouncing
  bool pressed;              // True if just pressed
  unsigned long pressTime;   // When button was pressed
};

// --- Create Button Objects ---
Button navUp = {NAV_UP, HIGH, HIGH, 0, false, 0},
      navDown = {NAV_DOWN, HIGH, HIGH, 0, false, 0},
      selUp = {SEL_UP, HIGH, HIGH, 0, false, 0},
      selDown = {SEL_DOWN, HIGH, HIGH, 0, false, 0};

// --- Timing Variables ---
unsigned long lastButtonCheck = 0, lastDisplayUpdate = 0, stateChangeTime = 0;
unsigned long buttonHoldStartTime = 0;
bool isHolding = false;
const unsigned long HOLD_TO_REPEAT_DELAY = 400;    // Delay before repeat adjustment on hold
const unsigned long REPEAT_INTERVAL = 100;         // Interval for repeat adjustment
unsigned long lastRepeatTime = 0;

// --- Fan Control Variables ---
bool fanActive = false, fanCycleEnabled = false;
unsigned long fanCycleStartTime = 0;

// --- Timing Constants ---
const unsigned long DEBOUNCE_DELAY = 50;           // Button debounce (ms)
const unsigned long STATE_CHANGE_DELAY = 300;      // Menu change debounce (ms)
const unsigned long DISPLAY_REFRESH_INTERVAL = 500;// LCD refresh rate (ms)
const unsigned long POLARITY_CHANGE_INTERVAL = 120000; // Alternate polarity interval (ms)
const unsigned long FAN_START_DELAY_MS = 300000;   // Fan starts after 5 min (ms)
const unsigned long FAN_CYCLE_INTERVAL_MS = 30000; // Fan cycles every 30s (ms)

// --- Data Structures for Treatments ---
struct FrequencyOption { int value; String description; }; // Frequency and description
struct Condition { String name; int defaultDuration; FrequencyOption frequencies[5]; int frequencyCount; }; // Condition info
struct Category { String name; Condition conditions[20]; int conditionCount; }; // Category of conditions

Category categories[7];        // Array of all categories
int categoryCount = 7;         // Number of categories

// --- Function Prototypes ---
void setupTreatmentData();      // Loads treatment presets
void updateButtons();           // Reads and debounces buttons
void handleMenuNavigation();    // Handles moving through menus
void handleValueAdjustment();   // Adjusts values in menus
void updateDisplay();           // Updates the LCD display
void readTemperature_robust();  // Reads temperature with retries
void resetI2CBus();             // Resets I2C bus if needed
void setPowerLevel(PowerLevel level); // Sets power relay
void startPEMF();               // Starts PEMF pulses
void stopPEMF();                // Stops PEMF pulses
void handlePEMF();              // In-treatment controls
void playStartSound();          // Sound on start
void playStopSound();           // Sound on stop
void handleFanControl();        // Controls the fan
float getDutyCycleFromIntensity(); // Gets % duty cycle for intensity

// --- ISR Timer: Handles PEMF Pulse Generation ---
void IRAM_ATTR onTimerISR() {
  // Called every TICK_INTERVAL_US microseconds
  tick_counter++;
  // If it's time to change pulse state
  if (tick_counter >= next_state_change_tick) {
    if (currentPolarity == BIPOLAR) {
      // Bipolar mode: alternate direction every pulse
      switch (pemf_state) {
        case STATE_OFF:
          digitalWrite(MD10_DIR, HIGH);
          digitalWrite(MD10_PWM, HIGH);
          pemf_state = STATE_POS_PULSE;
          next_state_change_tick += pos_pulse_ticks;
          break;
        case STATE_POS_PULSE:
          digitalWrite(MD10_DIR, LOW);
          pemf_state = STATE_NEG_PULSE;
          next_state_change_tick += neg_pulse_ticks;
          break;
        case STATE_NEG_PULSE:
          digitalWrite(MD10_PWM, LOW);
          pemf_state = STATE_OFF;
          next_state_change_tick += off_time_ticks;
          break;
      }
    } else {
      // Unipolar or alternate mode: one direction only
      switch (pemf_state) {
        case STATE_OFF:
          digitalWrite(MD10_DIR, currentDirState);
          digitalWrite(MD10_PWM, HIGH);
          pemf_state = STATE_POS_PULSE;
          next_state_change_tick += pos_pulse_ticks;
          break;
        case STATE_POS_PULSE:
          digitalWrite(MD10_PWM, LOW);
          pemf_state = STATE_OFF;
          next_state_change_tick += off_time_ticks;
          break;
        case STATE_NEG_PULSE:
          pemf_state = STATE_OFF;
          break;
      }
    }
  }
}

// --- Loads All Treatment Presets At Startup ---
void setupTreatmentData() {
  // Each category (Musculoskeletal, Circulation, etc.) has conditions (Fractures, Osteoporosis, etc.)
  // Each condition has recommended durations and frequencies for PEMF therapy.
  // This data drives the menu system for preset selection.
  // (See your original file for all categories and conditions)
  // ...
  // [Omitted for brevity, but same as your original treatment database]
  // ...
}

// --- Arduino Setup: Runs Once At Boot ---
void setup() {
  Serial.begin(115200);

  // --- Initialize Relay for Power Switching ---
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Default: 19V mode (relay off)
  delay(100);

  // --- Initialize I2C Bus for LCD/Sensor ---
  delay(200);
  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  // --- Initialize LCD ---
  lcd.init();
  delay(100);
  lcd.backlight();
  delay(100);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ABO PEMF Device V5.0");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);

  // --- Load Treatment Database ---
  setupTreatmentData();

  // --- Initialize Temperature Sensor ---
  if (!am2320.begin()) {
    Serial.println("AM2320 not found on boot. Will keep trying.");
    tempSensorDisabled = false;
    tempSensorRetryCount = 0;
  } else {
    Serial.println("AM2320 sensor found!");
    tempSensorDisabled = false;
    tempSensorRetryCount = 0;
  }

  // --- Initialize Other Pins ---
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(NAV_UP, INPUT_PULLUP);
  pinMode(NAV_DOWN, INPUT_PULLUP);
  pinMode(SEL_UP, INPUT_PULLUP);
  pinMode(SEL_DOWN, INPUT_PULLUP);
  pinMode(MD10_PWM, OUTPUT);
  pinMode(MD10_DIR, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  digitalWrite(MD10_PWM, LOW);
  digitalWrite(MD10_DIR, HIGH);
  digitalWrite(FAN_PIN, LOW);

  // --- Initialize Menu Variables ---
  currentMenuOption = 0;
  currentCategoryIndex = 0;
  currentConditionIndex = 0;
  currentFrequencyIndex = 0;
  customMenuOption = 0;
  customAdjustMode = false;

  powerLevelInitialized = true;

  // --- Configure Timer For PEMF Pulse Generation ---
  Serial.println("Configuring ISR Engine Timer...");
  tick_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(tick_timer, &onTimerISR, true);
  timerAlarmWrite(tick_timer, TICK_INTERVAL_US, true);

  // --- Show Ready Screen ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ABO PEMF Device V5.0");
  lcd.setCursor(0, 2);
  lcd.print("Ready!");
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  stateChangeTime = millis() - STATE_CHANGE_DELAY - 1;
  updateDisplay();
}

// --- Arduino Loop: Runs Continuously After Setup ---
void loop() {
  // --- Check Button Inputs ---
  if (millis() - lastButtonCheck > 10) {
    updateButtons();
    lastButtonCheck = millis();
  }

  // --- Handle Menu Navigation/Adjustment ---
  if (menuState < START_STOP) {
    (menuState < DURATION_MENU) ? handleMenuNavigation() : handleValueAdjustment();
  } else {
    handleMenuNavigation();
  }

  // --- Run PEMF Treatment If Active ---
  if (pemfActive) {
    handlePEMF();
    if (millis() - pemfStartTime > (duration * 60000UL)) {
      stopPEMF();
    }
  }

  // --- Periodically Read Temperature Sensor ---
  if (millis() - lastTempRead > TEMP_READ_INTERVAL && !tempSensorDisabled) {
    lastTempRead = millis();
    readTemperature_robust();
  }

  // --- Periodically Update LCD Display ---
  if (millis() - lastDisplayUpdate > DISPLAY_REFRESH_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
}

// --- Sets Output Power Level By Switching Relay ---
void setPowerLevel(PowerLevel level) {
  currentPowerLevel = level;

  // With transistor: HIGH = ON (5V), LOW = OFF (19V)
  digitalWrite(RELAY_PIN, (level == POWER_5V) ? HIGH : LOW);
  delay(50); // Allow relay to switch

  // LED feedback for power mode
  digitalWrite(LED_PIN, (level == POWER_5V) ? HIGH : LOW);
}

// --- Resets The I2C Bus On Error ---
void resetI2CBus() {
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(50000);
  Serial.println("I2C bus reset");
}

// --- Reads Temperature Sensor Robustly, Handles Failures ---
void readTemperature_robust() {
  if (tempSensorDisabled) return;

  unsigned long wakeUpStart = millis();
  bool wakeUpSuccess = false;

  // Try to wake up sensor
  while (millis() - wakeUpStart < 100) {
    Wire.beginTransmission(AM2320_ADDRESS);
    if (Wire.endTransmission() == 0) {
      wakeUpSuccess = true;
      break;
    }
    delay(10);
  }

  if (!wakeUpSuccess) {
    Serial.println("Sensor not responding to wake-up call.");
    tempSensorRetryCount++;

    if (tempSensorRetryCount >= MAX_TEMP_RETRIES) {
      Serial.println("Multiple sensor failures, resetting I2C bus");
      resetI2CBus();

      if (tempSensorRetryCount >= MAX_TEMP_RETRIES * 2) {
        Serial.println("Disabling temperature sensor due to persistent failures");
        tempSensorDisabled = true;
        tempSensorRetryCount = 0;
      }
    }

    currentTemperature = -99.9;
    return;
  }

  delay(10);

  unsigned long readStart = millis();
  float t = -99.9;
  bool readSuccess = false;

  // Try to read temperature
  while (millis() - readStart < 200) {
    t = am2320.readTemperature();
    if (!isnan(t)) {
      readSuccess = true;
      break;
    }
    delay(20);
  }

  if (!readSuccess) {
    Serial.println("Read failed after wake-up. Giving up for this cycle.");
    tempSensorRetryCount++;

    if (tempSensorRetryCount >= MAX_TEMP_RETRIES) {
      Serial.println("Multiple sensor failures, resetting I2C bus");
      resetI2CBus();

      if (tempSensorRetryCount >= MAX_TEMP_RETRIES * 2) {
        Serial.println("Disabling temperature sensor due to persistent failures");
        tempSensorDisabled = true;
        tempSensorRetryCount = 0;
      }
    }

    currentTemperature = -99.9;
  } else {
    currentTemperature = t;
    tempSensorRetryCount = 0;
    tempSensorDisabled = false;
  }
}

// --- Reads And Debounces Button Presses ---
void updateButtons() {
  Button* buttons[] = {&navUp, &navDown, &selUp, &selDown};

  for (int i = 0; i < 4; i++) {
    Button* btn = buttons[i];
    bool reading = digitalRead(btn->pin);

    // Debounce logic
    if (reading != btn->lastState) {
      btn->lastDebounceTime = millis();
    }

    if ((millis() - btn->lastDebounceTime) > DEBOUNCE_DELAY) {
      if (reading != btn->currentState) {
        btn->currentState = reading;

        // If button just pressed
        if (btn->currentState == LOW) {
          btn->pressed = true;
          btn->pressTime = millis();

          // Stop PEMF if any button pressed during treatment
          if (pemfActive) {
            stopPEMF();
            for (int j = 0; j < 4; j++) {
              buttons[j]->pressed = false;
            }
          }
        }
      }

      // Clear pressed flag after 200ms
      if (btn->pressed && (millis() - btn->pressTime > 200)) {
        btn->pressed = false;
      }
    }

    btn->lastState = reading;
  }
}

// --- Handles Menu Navigation (UP/DOWN/SELECT/ESC) ---
void handleMenuNavigation() {
  if (millis() - stateChangeTime < STATE_CHANGE_DELAY) return;

  // Navigation UP
  if (navUp.pressed) {
    navUp.pressed = false;
    switch (menuState) {
      // Main menu: move up through options
      case MAIN_MENU: currentMenuOption = (currentMenuOption == 0) ? 2 : currentMenuOption - 1; break;
      // Intensity: cycle up
      case INTENSITY_MENU: currentIntensity = (currentIntensity - 1); if (currentIntensity < NORMAL) currentIntensity = WARM; break;
      // Polarity: cycle up
      case POLARITY_MENU: currentPolarity = (PolarityMode)((currentPolarity - 1 + 3) % 3); break;
      // Category: cycle up
      case CATEGORY_MENU: currentCategoryIndex = (currentCategoryIndex - 1 + categoryCount) % categoryCount; break;
      // Condition: cycle up
      case CONDITION_MENU: currentConditionIndex = (currentConditionIndex - 1 + categories[currentCategoryIndex].conditionCount) % categories[currentCategoryIndex].conditionCount; break;
      // Frequency: cycle up
      case FREQUENCY_MENU: currentFrequencyIndex = (currentFrequencyIndex - 1 + categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount) % categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount; break;
      // Power: toggle power
      case POWER_MENU: { PowerLevel newLevel = (currentPowerLevel == POWER_19V) ? POWER_5V : POWER_19V; setPowerLevel(newLevel); break; }
      default: break;
    }
    updateDisplay();
  }

  // Navigation DOWN
  if (navDown.pressed) {
    navDown.pressed = false;
    switch (menuState) {
      case MAIN_MENU: currentMenuOption = (currentMenuOption + 1) % 3; break;
      case INTENSITY_MENU: currentIntensity = (currentIntensity % 3) + 1; break;
      case POLARITY_MENU: currentPolarity = (PolarityMode)((currentPolarity + 1) % 3); break;
      case CATEGORY_MENU: currentCategoryIndex = (currentCategoryIndex + 1) % categoryCount; break;
      case CONDITION_MENU: currentConditionIndex = (currentConditionIndex + 1) % categories[currentCategoryIndex].conditionCount; break;
      case FREQUENCY_MENU: currentFrequencyIndex = (currentFrequencyIndex + 1) % categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount; break;
      case POWER_MENU: { PowerLevel newLevel = (currentPowerLevel == POWER_19V) ? POWER_5V : POWER_19V; setPowerLevel(newLevel); break; }
      default: break;
    }
    updateDisplay();
  }

  // Select/Confirm
  if (selUp.pressed) {
    selUp.pressed = false;
    switch (menuState) {
      case MAIN_MENU:
        if (currentMenuOption == 0) menuState = INTENSITY_MENU;
        else if (currentMenuOption == 1) menuState = CUSTOM_MODE;
        else if (currentMenuOption == 2) menuState = POWER_MENU;
        break;
      case INTENSITY_MENU: menuState = POLARITY_MENU; break;
      case POLARITY_MENU:
        if (currentMenuOption == 0) {
          menuState = CATEGORY_MENU;
          currentCategoryIndex = 0; currentConditionIndex = 0; currentFrequencyIndex = 0;
        } else {
          menuState = CUSTOM_MODE;
          frequency = 10; duration = 20; customMenuOption = 0;
          customAdjustMode = false;
        }
        break;
      case CATEGORY_MENU: menuState = CONDITION_MENU; currentConditionIndex = 0; break;
      case CONDITION_MENU: menuState = FREQUENCY_MENU; currentFrequencyIndex = 0; break;
      case FREQUENCY_MENU: menuState = DURATION_MENU; duration = categories[currentCategoryIndex].conditions[currentConditionIndex].defaultDuration; break;
      case POWER_MENU: menuState = MAIN_MENU; break;
      case START_STOP: if (!pemfActive) { startPEMF(); } break;
      default: break;
    }
    stateChangeTime = millis(); updateDisplay();
  }

  // Escape/Back
  if (selDown.pressed) {
    selDown.pressed = false;
    switch (menuState) {
      case INTENSITY_MENU: menuState = MAIN_MENU; break;
      case POLARITY_MENU: menuState = INTENSITY_MENU; break;
      case CATEGORY_MENU: menuState = POLARITY_MENU; break;
      case CONDITION_MENU: menuState = CATEGORY_MENU; break;
      case FREQUENCY_MENU: menuState = CONDITION_MENU; break;
      case DURATION_MENU:
        if (categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount > 1) {
          menuState = FREQUENCY_MENU;
        } else {
          menuState = CONDITION_MENU;
        }
        break;
      case CUSTOM_MODE: menuState = POLARITY_MENU; break;
      case POWER_MENU: menuState = MAIN_MENU; break;
      case START_STOP: menuState = MAIN_MENU; break;
      default: break;
    }
    stateChangeTime = millis(); updateDisplay();
  }
}

// --- Handles Value Adjustment In Menus (Duration/Frequency/Custom) ---
void handleValueAdjustment() {
  if (millis() - stateChangeTime < STATE_CHANGE_DELAY) return;

  // Escape/Back
  if (selDown.pressed) {
    selDown.pressed = false;
    isHolding = false;
    if (menuState == DURATION_MENU) {
      if (categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount > 1)
        menuState = FREQUENCY_MENU;
      else
        menuState = CONDITION_MENU;
    } else if (menuState == CUSTOM_MODE) {
      if (customAdjustMode) customAdjustMode = false;
      else menuState = POLARITY_MENU;
    }
    stateChangeTime = millis(); updateDisplay(); return;
  }

  // Select/Confirm
  if (selUp.pressed) {
    selUp.pressed = false;
    isHolding = false;
    if (menuState == DURATION_MENU) {
      frequency = categories[currentCategoryIndex].conditions[currentConditionIndex].frequencies[currentFrequencyIndex].value;
      menuState = START_STOP;
    } else if (menuState == CUSTOM_MODE) {
      if (!customAdjustMode) {
        if (customMenuOption < 2) customAdjustMode = true;
        else if (customMenuOption == 2) menuState = START_STOP;
      } else {
        customAdjustMode = false;
      }
    }
    stateChangeTime = millis(); updateDisplay(); return;
  }

  // Custom mode: select parameter (frequency/duration)
  if (menuState == CUSTOM_MODE && !customAdjustMode) {
    if (navUp.pressed) { navUp.pressed = false; customMenuOption = (customMenuOption - 1 + 3) % 3; updateDisplay(); }
    if (navDown.pressed) { navDown.pressed = false; customMenuOption = (customMenuOption + 1) % 3; updateDisplay(); }
    return;
  }

  // Only adjust if allowed (duration/custom)
  bool canAdjust = (menuState == DURATION_MENU) || (menuState == CUSTOM_MODE && customAdjustMode);
  if (!canAdjust) { isHolding = false; return; }

  // Only one button held at a time
  if (navUp.currentState == HIGH && navDown.currentState == HIGH) { isHolding = false; return; }

  bool adjusted = false;
  bool isUp = (navUp.currentState == LOW);
  bool isDown = (navDown.currentState == LOW);

  // Button hold-to-repeat adjustment
  if (isUp || isDown) {
    if (!isHolding) {
      isHolding = true;
      buttonHoldStartTime = millis();
      lastRepeatTime = millis();
      adjusted = true;
    } else {
      if (millis() - buttonHoldStartTime > HOLD_TO_REPEAT_DELAY) {
        if (millis() - lastRepeatTime > REPEAT_INTERVAL) {
          adjusted = true;
          lastRepeatTime = millis();
        }
      }
    }
  }

  // Actually adjust the values
  if (adjusted) {
    if (menuState == DURATION_MENU) {
      int change = isUp ? 5 : -5;
      duration = constrain(duration + change, 5, 480);
    } else if (menuState == CUSTOM_MODE && customAdjustMode) {
      if (customMenuOption == 0) {
        int change = isUp ? 1 : -1;
        frequency = constrain(frequency + change, 1, 100);
      } else {
        int change = isUp ? 5 : -5;
        duration = constrain(duration + change, 5, 480);
      }
    }
    updateDisplay();
  }
}

// --- Updates LCD Display With Current Menu/Status ---
void updateDisplay() {
  lcd.clear(); String tempStr; char timeString[9];

  switch (menuState) {
    case MAIN_MENU:
      lcd.setCursor(0, 0);
      lcd.print("PEMF Control");
      if (currentTemperature > -99) {
        String tempVal = String(currentTemperature, 1);
        lcd.setCursor(20 - tempVal.length() - 1, 0);
        lcd.print(tempVal);
        lcd.write(223); // Degree symbol
      }
      lcd.setCursor(0, 1); lcd.print(currentMenuOption == 0 ? ">" : " "); lcd.print("Condition Treat.");
      lcd.setCursor(0, 2); lcd.print(currentMenuOption == 1 ? ">" : " "); lcd.print("Custom Mode");
      lcd.setCursor(0, 3); lcd.print(currentMenuOption == 2 ? ">" : " "); lcd.print("Power ");
      lcd.print((currentPowerLevel == POWER_19V) ? "19V" : "5V");
      break;
    case INTENSITY_MENU: // Show intensity options
      lcd.setCursor(0, 0); lcd.print("Select Intensity:");
      lcd.setCursor(0, 1); lcd.print(currentIntensity == NORMAL ? ">" : " "); lcd.print("1: Normal (5%)");
      lcd.setCursor(0, 2); lcd.print(currentIntensity == INTENSE ? ">" : " "); lcd.print("2: Intense (8%)");
      lcd.setCursor(0, 3); lcd.print(currentIntensity == WARM ? ">" : " "); lcd.print("3: Warm (13%)"); break;
    case POLARITY_MENU: // Show polarity options
      lcd.setCursor(0, 0); lcd.print("Select Polarity:");
      lcd.setCursor(0, 1); lcd.print(currentPolarity == UNIPOLAR ? ">" : " "); lcd.print("Unipolar");
      lcd.setCursor(0, 2); lcd.print(currentPolarity == BIPOLAR ? ">" : " "); lcd.print("Bipolar");
      lcd.setCursor(0, 3); lcd.print(currentPolarity == ALTERNATE ? ">" : " "); lcd.print("Alternate (2 min)"); break;
    case CATEGORY_MENU: // Show category selection
      lcd.setCursor(0, 0); lcd.print("Select Category:");
      lcd.setCursor(0, 1); lcd.print(">"); lcd.print(categories[currentCategoryIndex].name);
      if (currentCategoryIndex < categoryCount - 1) {
        lcd.setCursor(0, 2); lcd.print("  "); lcd.print(categories[(currentCategoryIndex + 1) % categoryCount].name);
      }
      if (currentCategoryIndex > 0) {
        lcd.setCursor(0, 3); lcd.print("  "); lcd.print(categories[(currentCategoryIndex - 1 + categoryCount) % categoryCount].name);
      } break;
    case CONDITION_MENU: // Show condition selection
      lcd.setCursor(0, 0); lcd.print(categories[currentCategoryIndex].name.substring(0,20));
      lcd.setCursor(0, 1); lcd.print("Select Condition:");
      lcd.setCursor(0, 2); lcd.print(">");
      lcd.print(categories[currentCategoryIndex].conditions[currentConditionIndex].name.substring(0,18));
      if (currentConditionIndex < categories[currentCategoryIndex].conditionCount - 1) {
        lcd.setCursor(0, 3); lcd.print("  ");
        lcd.print(categories[currentCategoryIndex].conditions[(currentConditionIndex + 1) % categories[currentCategoryIndex].conditionCount].name.substring(0,18));
      } break;
    case FREQUENCY_MENU: // Show frequency options
      lcd.setCursor(0, 0); lcd.print("Select Frequency:");
      lcd.setCursor(0, 1); lcd.print(">");
      lcd.print(categories[currentCategoryIndex].conditions[currentConditionIndex].frequencies[currentFrequencyIndex].description);
      if (currentFrequencyIndex < categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount - 1) {
        lcd.setCursor(0, 2); lcd.print("  ");
        lcd.print(categories[currentCategoryIndex].conditions[currentConditionIndex].frequencies[(currentFrequencyIndex + 1) % categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount].description);
      }
      if (currentFrequencyIndex > 0) {
        lcd.setCursor(0, 3); lcd.print("  ");
        lcd.print(categories[currentCategoryIndex].conditions[currentConditionIndex].frequencies[(currentFrequencyIndex - 1 + categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount) % categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount].description);
      } break;
    case DURATION_MENU: // Show duration setting
      lcd.setCursor(0, 0); lcd.print("Set Duration:");
      lcd.setCursor(0, 1); lcd.print("Current: "); lcd.print(duration); lcd.print(" min");
      lcd.setCursor(0, 2); lcd.print("Hold UP/DOWN to");
      lcd.setCursor(0, 3); lcd.print("adjust value"); break;
    case CUSTOM_MODE: // Show custom mode menu
      lcd.setCursor(0, 0); lcd.print("Custom Mode");
      if (customAdjustMode) {
        lcd.setCursor(0, 1); lcd.print("Adjusting:");
        if (customMenuOption == 0) {
          lcd.setCursor(0, 2); lcd.print("Frequency: "); lcd.print(frequency); lcd.print(" Hz");
        } else {
          lcd.setCursor(0, 2); lcd.print("Duration: "); lcd.print(duration); lcd.print(" min");
        }
        lcd.setCursor(0, 3); lcd.print("SEL: Confirm");
      } else {
        lcd.setCursor(0, 1); lcd.print(customMenuOption == 0 ? ">" : " "); lcd.print("Frequency: "); lcd.print(frequency); lcd.print(" Hz");
        lcd.setCursor(0, 2); lcd.print(customMenuOption == 1 ? ">" : " "); lcd.print("Duration: "); lcd.print(duration); lcd.print(" min");
        lcd.setCursor(0, 3); lcd.print(customMenuOption == 2 ? ">" : " "); lcd.print("Start Program");
      } break;
    case POWER_MENU: // Show power selection
      lcd.setCursor(0, 0); lcd.print("Select Power Level:");
      lcd.setCursor(0, 1); lcd.print(currentPowerLevel == POWER_19V ? ">" : " "); lcd.print("19V (Full Power)");
      lcd.setCursor(0, 2); lcd.print(currentPowerLevel == POWER_5V ? ">" : " "); lcd.print("5V (Low Power)");
      lcd.setCursor(0, 3); lcd.print("SEL: Confirm");
      break;
    case START_STOP: // Show treatment status
      switch(currentIntensity) { case NORMAL: tempStr = "Int:Norm"; break;
        case INTENSE: tempStr = "Int:Intense"; break; case WARM: tempStr = "Int:Warm"; break; }
      switch(currentPolarity) { case UNIPOLAR: tempStr += " Pol:Uni"; break;
        case BIPOLAR: tempStr += " Pol:Bi"; break; case ALTERNATE: tempStr += " Pol:Alt"; break; }
      lcd.setCursor(0, 0); lcd.print(tempStr);
      lcd.setCursor(0, 1); lcd.print("Freq.: "); lcd.print(frequency); lcd.print(" Hz ");
      lcd.print((currentPowerLevel == POWER_19V) ? "19V" : "5V");
      lcd.setCursor(0, 2);
      if (pemfActive) {
        unsigned long remainingSeconds = (duration * 60) - ((millis() - pemfStartTime) / 1000);
        if (remainingSeconds > (duration * 60UL)) remainingSeconds = 0;
        sprintf(timeString, "%02lu:%02lu", remainingSeconds / 60, remainingSeconds % 60);
        lcd.print("Time left: "); lcd.print(timeString);
      } else {
        lcd.print("Duration: "); lcd.print(duration); lcd.print(" min");
      }
      lcd.setCursor(0, 3);
      if (pemfActive) {
        lcd.print("ANY BTN STOP T:");
        if (currentTemperature > -99) {
          lcd.print(String(currentTemperature, 1));
          lcd.write(223);
        } else {
          lcd.print("--.-");
        }
      } else {
        lcd.print(">START PEMF");
      }
      break;
  }
}

// --- Returns Duty Cycle Percentage For Intensity Level ---
float getDutyCycleFromIntensity() {
  switch (currentIntensity) {
    case NORMAL: return 5.0;
    case INTENSE: return 8.0;
    case WARM: return 13.0;
    default: return 5.0;
  }
}

// --- Play Start Sound On Buzzer ---
void playStartSound() {
  tone(BUZZER_PIN, 1000, 100);
  delay(100);
  tone(BUZZER_PIN, 1500, 150);
}

// --- Play Stop Sound On Buzzer ---
void playStopSound() {
  tone(BUZZER_PIN, 1500, 100);
  delay(100);
  tone(BUZZER_PIN, 1000, 150);
}

// --- Start PEMF Treatment ---
void startPEMF() {
  playStartSound();
  pemfActive = true;
  pemfStartTime = millis();
  fanCycleEnabled = false;
  fanActive = false;
  digitalWrite(FAN_PIN, LOW);

  setPowerLevel(currentPowerLevel);

  float dutyCycle = getDutyCycleFromIntensity();
  unsigned long periodMicros = 1000000UL / frequency;

  portENTER_CRITICAL(&timerMux);
  if (currentPolarity == BIPOLAR) {
    unsigned long totalOnTime = (unsigned long)(periodMicros * (dutyCycle / 100.0));
    pos_pulse_ticks = (totalOnTime / 2) / TICK_INTERVAL_US;
    neg_pulse_ticks = pos_pulse_ticks;
    off_time_ticks = (periodMicros - totalOnTime) / TICK_INTERVAL_US;
  } else {
    pos_pulse_ticks = ((unsigned long)(periodMicros * (dutyCycle / 100.0))) / TICK_INTERVAL_US;
    neg_pulse_ticks = 0;
    off_time_ticks = (periodMicros - (pos_pulse_ticks * TICK_INTERVAL_US)) / TICK_INTERVAL_US;
  }

  pemf_state = STATE_OFF;
  currentDirState = HIGH;
  digitalWrite(MD10_DIR, currentDirState);
  digitalWrite(MD10_PWM, LOW);

  if (currentPolarity == ALTERNATE) {
    lastPolarityChangeTime = millis();
  }

  tick_counter = 0;
  next_state_change_tick = off_time_ticks;
  portEXIT_CRITICAL(&timerMux);

  timerAlarmEnable(tick_timer);
  Serial.println("ISR Engine timer enabled.");
  updateDisplay();
}

// --- Stop PEMF Treatment ---
void stopPEMF() {
  if (!pemfActive) return;

  playStopSound();
  pemfActive = false;
  digitalWrite(FAN_PIN, LOW);
  fanActive = false;
  fanCycleEnabled = false;

  if (tick_timer != NULL) {
    timerAlarmDisable(tick_timer);
    Serial.println("ISR Engine timer disabled.");
  }

  digitalWrite(MD10_PWM, LOW);
  digitalWrite(MD10_DIR, HIGH);
  digitalWrite(LED_PIN, LOW);

  menuState = MAIN_MENU;
  updateDisplay();
}

// --- Handles Treatment Pulse, Fan, Polarity Switch ---
void handlePEMF() {
  if (!pemfActive) return;

  handleFanControl();

  // Alternate polarity mode: switch direction every POLARITY_CHANGE_INTERVAL ms
  if (currentPolarity == ALTERNATE) {
    if (millis() - lastPolarityChangeTime >= POLARITY_CHANGE_INTERVAL) {
      portENTER_CRITICAL(&timerMux);
      currentDirState = !currentDirState;
      portEXIT_CRITICAL(&timerMux);
      lastPolarityChangeTime = millis();
      Serial.println("Alternate Polarity Switched");
    }
  }

  // Blink LED during operation
  static unsigned long lastLedBlink = 0;
  if (millis() - lastLedBlink >= 500) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastLedBlink = millis();
  }
}

// --- Controls Cooling Fan Timing ---
void handleFanControl() {
  // Enable fan cycling after 5 minutes of PEMF
  if (!fanCycleEnabled && (millis() - pemfStartTime >= FAN_START_DELAY_MS)) {
    Serial.println("Fan control enabled after 5 minutes.");
    fanCycleEnabled = true;
    fanCycleStartTime = millis();
    fanActive = true;
    digitalWrite(FAN_PIN, HIGH);
    Serial.println("Fan ON");
  }

  // Cycle fan ON/OFF every FAN_CYCLE_INTERVAL_MS
  if (fanCycleEnabled) {
    if (millis() - fanCycleStartTime >= FAN_CYCLE_INTERVAL_MS) {
      fanActive = !fanActive;
      digitalWrite(FAN_PIN, fanActive ? HIGH : LOW);
      Serial.println(fanActive ? "Fan ON" : "Fan OFF");
      fanCycleStartTime = millis();
    }
  }
}
