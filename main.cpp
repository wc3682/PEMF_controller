//FINAL FULL WORKING VERSION 5.0  
//2025-10-05 - Alexandre BERNARD ORECILLA with help of GEMINI 2.5 Pro and Z.AI GLM 4.6 
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_AM2320.h>

// Pin definitions
#define LCD_ADDRESS 0x27
#define AM2320_ADDRESS 0x5C  // Explicitly define Temp sensor address
#define LED_PIN 2
#define BUZZER_PIN 5
#define NAV_UP 25 //NAV_UP
#define NAV_DOWN 26 //NAV_DOWN
#define SEL_UP 32 //Validation
#define SEL_DOWN 33 //Esc
#define MD10_PWM 4
#define MD10_DIR 16
#define FAN_PIN 19
#define RELAY_PIN 27  // New pin for relay control to switch 19V - 5V

LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);
Adafruit_AM2320 am2320 = Adafruit_AM2320();

// Enums
enum IntensityLevel { NORMAL = 1, INTENSE = 2, WARM = 3 };
enum PolarityMode { UNIPOLAR, BIPOLAR, ALTERNATE };
enum PemfState { STATE_OFF, STATE_POS_PULSE, STATE_NEG_PULSE };
enum MenuState { MAIN_MENU, INTENSITY_MENU, POLARITY_MENU, CATEGORY_MENU, CONDITION_MENU,
                 FREQUENCY_MENU, DURATION_MENU, CUSTOM_MODE, START_STOP, POWER_MENU };
enum PowerLevel { POWER_19V, POWER_5V };

MenuState menuState = MAIN_MENU;

// System variables
int frequency = 10;
int duration = 20;
bool pemfActive = false;
unsigned long pemfStartTime = 0;

// Power level variable 
PowerLevel currentPowerLevel = POWER_19V;  // Default to 19V
bool powerLevelInitialized = false;  // Flag to prevent multiple initializations

// Temperature sensor variables 
float currentTemperature = -99.9;
unsigned long lastTempRead = 0;
const unsigned long TEMP_READ_INTERVAL = 20000; // Read temperature every 20 seconds
unsigned long tempSensorRetryCount = 0;
const unsigned long MAX_TEMP_RETRIES = 3;
bool tempSensorDisabled = false;

// Menu and custom mode variables 
int currentMenuOption = 0;  // Will be properly initialized in setup
int currentCategoryIndex = 0, currentConditionIndex = 0,
    currentFrequencyIndex = 0, customMenuOption = 0;
bool customAdjustMode = false;

// Polarity and Intensity variables
int currentIntensity = NORMAL;
volatile PolarityMode currentPolarity = UNIPOLAR;
volatile int currentDirState = HIGH;
unsigned long lastPolarityChangeTime = 0;

// ISR Engine (for timing control)
hw_timer_t *tick_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long tick_counter = 0;
const int TICK_INTERVAL_US = 10;
volatile unsigned long next_state_change_tick = 0;
volatile PemfState pemf_state = STATE_OFF;
volatile unsigned long pos_pulse_ticks = 0, neg_pulse_ticks = 0, off_time_ticks = 0;

// Button handling
struct Button { 
  uint8_t pin; 
  bool lastState; 
  bool currentState; 
  unsigned long lastDebounceTime;
  bool pressed; 
  unsigned long pressTime;
};

Button navUp = {NAV_UP, HIGH, HIGH, 0, false, 0}, 
      navDown = {NAV_DOWN, HIGH, HIGH, 0, false, 0}, 
      selUp = {SEL_UP, HIGH, HIGH, 0, false, 0}, 
      selDown = {SEL_DOWN, HIGH, HIGH, 0, false, 0};

// Timing variables
unsigned long lastButtonCheck = 0, lastDisplayUpdate = 0, stateChangeTime = 0;
unsigned long buttonHoldStartTime = 0;
bool isHolding = false;
const unsigned long HOLD_TO_REPEAT_DELAY = 400;
const unsigned long REPEAT_INTERVAL = 100;
unsigned long lastRepeatTime = 0;

// Fan control
bool fanActive = false, fanCycleEnabled = false;
unsigned long fanCycleStartTime = 0;

// Constants from original stable code
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long STATE_CHANGE_DELAY = 300;
const unsigned long DISPLAY_REFRESH_INTERVAL = 500;
const unsigned long POLARITY_CHANGE_INTERVAL = 120000;
const unsigned long FAN_START_DELAY_MS = 300000;
const unsigned long FAN_CYCLE_INTERVAL_MS = 30000;

// Data Structures
struct FrequencyOption { int value; String description; };
struct Condition { String name; int defaultDuration; FrequencyOption frequencies[5]; int frequencyCount; };
struct Category { String name; Condition conditions[20]; int conditionCount; };

Category categories[7];
int categoryCount = 7;

// Function prototypes
void setupTreatmentData();
void updateButtons();
void handleMenuNavigation();
void handleValueAdjustment();
void updateDisplay();
void readTemperature_robust();
void resetI2CBus();
void setPowerLevel(PowerLevel level); 
void startPEMF();
void stopPEMF();
void handlePEMF();
void playStartSound();
void playStopSound();
void handleFanControl();
float getDutyCycleFromIntensity();

// --- ISR ENGINE ---
void IRAM_ATTR onTimerISR() {
  tick_counter++;
  if (tick_counter >= next_state_change_tick) {
    if (currentPolarity == BIPOLAR) {
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

// --- Full Treatment Database ---
void setupTreatmentData() {
  // Category 1: Musculoskeletal System
  categories[0].name = "Musculoskeletal";
  categories[0].conditionCount = 17;
  categories[0].conditions[0] = {"Fractures", 20, {{10, "10 Hz"}, {20, "20 Hz"}}, 2};
  categories[0].conditions[1] = {"Periostitis", 20, {{6, "6 Hz"}}, 1};
  categories[0].conditions[2] = {"Pseudoarthrosis", 30, {{10, "10 Hz"}, {20, "20 Hz"}}, 2};
  categories[0].conditions[3] = {"Osteoporosis", 20, {{8, "8 Hz"}, {10, "10 Hz"}, {15, "15 Hz"}, {19, "19 Hz"}}, 4};
  categories[0].conditions[4] = {"Osteoarthrosis", 20, {{8, "8 Hz"}, {12, "12 Hz"}, {18, "18 Hz"}}, 3};
  categories[0].conditions[5] = {"Tendinitis", 10, {{8, "8 Hz"}}, 1};
  categories[0].conditions[6] = {"Ligament injuries", 20, {{10, "10 Hz"}, {15, "15 Hz"}}, 2};
  categories[0].conditions[7] = {"Frozen shoulder", 30, {{7, "7 Hz"}, {8, "8 Hz"}}, 2};
  categories[0].conditions[8] = {"Tennis or golf elbow", 10, {{8, "8 Hz"}}, 1};
  categories[0].conditions[9] = {"Dislocations and sprains", 30, {{10, "10 Hz"}}, 1};
  categories[0].conditions[10] = {"Strains", 20, {{11, "11 Hz"}, {15, "15 Hz"}}, 2};
  categories[0].conditions[11] = {"Herniated disc", 30, {{16, "16 Hz"}, {20, "20 Hz"}, {30, "30 Hz"}}, 3};
  categories[0].conditions[12] = {"Rheumatoid arthritis", 20, {{10, "10 Hz"}, {20, "20 Hz"}}, 2};
  categories[0].conditions[13] = {"Psoriatic arthritis", 20, {{18, "18 Hz"}}, 1};
  categories[0].conditions[14] = {"Fibromyalgia", 20, {{18, "18 Hz"}}, 1};
  categories[0].conditions[15] = {"Musculoskeletal pain", 20, {{10, "10 Hz"}}, 1};
  categories[0].conditions[16] = {"Osteonecrosis", 30, {{10, "10 Hz"}, {19, "19 Hz"}, {20, "20 Hz"}}, 3};

  // Category 2: Circulation
  categories[1].name = "Circulation";
  categories[1].conditionCount = 8;
  categories[1].conditions[0] = {"Hypertension", 30, {{1, "1 Hz"}, {5, "5 Hz"}}, 2};
  categories[1].conditions[1] = {"Arrhythmia", 30, {{7, "7 Hz"}, {8, "8 Hz"}}, 2};
  categories[1].conditions[2] = {"Angina pectoris", 30, {{2, "2 Hz"}, {8, "8 Hz"}}, 2};
  categories[1].conditions[3] = {"Arteriosclerosis", 15, {{7, "7 Hz"}, {10, "10 Hz"}}, 2};
  categories[1].conditions[4] = {"Circulatory dysfunction", 15, {{7, "7 Hz"}, {10, "10 Hz"}}, 2};
  categories[1].conditions[5] = {"Poor blood supply", 20, {{2, "2 Hz"}, {6, "6 Hz"}, {20, "20 Hz"}}, 3};
  categories[1].conditions[6] = {"Raynaud's syndrome", 20, {{15, "15 Hz"}}, 1};
  categories[1].conditions[7] = {"Lymphatic disorders", 30, {{18, "18 Hz"}}, 1};

  // Category 3: Nervous System
  categories[2].name = "Nervous System";
  categories[2].conditionCount = 16;
  categories[2].conditions[0] = {"Stroke", 15, {{7, "7 Hz"}, {10, "10 Hz"}, {20, "20 Hz"}}, 3};
  categories[2].conditions[1] = {"Alzheimer's disease", 30, {{2, "2 Hz"}, {8, "8 Hz"}}, 2};
  categories[2].conditions[2] = {"Parkinson's disease", 30, {{20, "20 Hz"}}, 1};
  categories[2].conditions[3] = {"Headache", 15, {{20, "20 Hz"}}, 1};
  categories[2].conditions[4] = {"Tinnitus", 20, {{10, "10 Hz"}}, 1};
  categories[2].conditions[5] = {"Sleep disorders", 20, {{1, "1 Hz"}, {5, "5 Hz"}}, 2};
  categories[2].conditions[6] = {"Carpal tunnel syndrome", 10, {{6, "6 Hz"}, {20, "20 Hz"}}, 2};
  categories[2].conditions[7] = {"Lumbago", 15, {{10, "10 Hz"}, {20, "20 Hz"}}, 2};
  categories[2].conditions[8] = {"Sciatica", 20, {{16, "16 Hz"}, {20, "20 Hz"}}, 2};
  categories[2].conditions[9] = {"Spinal injuries", 20, {{5, "5 Hz"}, {15, "15 Hz"}, {30, "30 Hz"}}, 3};
  categories[2].conditions[10] = {"Multiple sclerosis", 30, {{5, "5 Hz"}, {13, "13 Hz"}, {20, "20 Hz"}}, 3};
  categories[2].conditions[11] = {"Sensitivity to weather fronts", 10, {{11, "11 Hz"}, {15, "15 Hz"}}, 2};
  categories[2].conditions[12] = {"Stress", 15, {{3, "3 Hz"}, {5, "5 Hz"}}, 2};
  categories[2].conditions[13] = {"Depression", 10, {{3, "3 Hz"}, {20, "20 Hz"}}, 2};
  categories[2].conditions[14] = {"Hyperactivity", 20, {{20, "20 Hz"}}, 1};
  categories[2].conditions[15] = {"Nerve pain", 10, {{6, "6 Hz"}}, 1};

  // Category 4: Digestive System
  categories[3].name = "Digestive System";
  categories[3].conditionCount = 6;
  categories[3].conditions[0] = {"Diabetes mellitus", 20, {{12, "12 Hz"}}, 1};
  categories[3].conditions[1] = {"Inflamed liver, pancreas, colon", 30, {{10, "10 Hz"}, {30, "30 Hz"}, {50, "50 Hz"}}, 3};
  categories[3].conditions[2] = {"Crohn's disease", 30, {{13, "13 Hz"}}, 1};
  categories[3].conditions[3] = {"Dental and oral diseases", 30, {{30, "30 Hz"}}, 1};
  categories[3].conditions[4] = {"Stomach/duodenal ulcer (no bleeding!)", 12, {{10, "10 Hz"}, {20, "20 Hz"}}, 2};
  categories[3].conditions[5] = {"Stomach aches", 12, {{10, "10 Hz"}}, 1};

  // Category 5: Respiratory System
  categories[4].name = "Respiratory System";
  categories[4].conditionCount = 5;
  categories[4].conditions[0] = {"Bronchitis", 12, {{4, "4 Hz"}}, 1};
  categories[4].conditions[1] = {"Pneumonia, respiratory diseases", 30, {{4, "4 Hz"}, {12, "12 Hz"}}, 2};
  categories[4].conditions[2] = {"Asthma", 20, {{7, "7 Hz"}, {10, "10 Hz"}, {12, "12 Hz"}, {15, "15 Hz"}}, 4};
  categories[4].conditions[3] = {"Allergy", 10, {{5, "5 Hz"}, {10, "10 Hz"}}, 2};
  categories[4].conditions[4] = {"Tuberculosis (TB)", 12, {{4, "4 Hz"}}, 1};

  // Category 6: Wounds
  categories[5].name = "Wounds";
  categories[5].conditionCount = 4;
  categories[5].conditions[0] = {"Wound healing", 15, {{1, "1 Hz"}, {5, "5 Hz"}}, 2};
  categories[5].conditions[1] = {"Pain associated with wound healing", 15, {{11, "11 Hz"}, {15, "15 Hz"}, {17, "17 Hz"}}, 3};
  categories[5].conditions[2] = {"Bruises", 15, {{10, "10 Hz"}, {14, "14 Hz"}}, 2};
  categories[5].conditions[3] = {"Phantom pain", 15, {{16, "16 Hz"}, {19, "19 Hz"}}, 2};

  // Category 7: Other
  categories[6].name = "Other";
  categories[6].conditionCount = 10;
  categories[6].conditions[0] = {"Psoriasis", 30, {{10, "10 Hz"}}, 1};
  categories[6].conditions[1] = {"Chronic pelvic pain", 20, {{5, "5 Hz"}, {7, "7 Hz"}}, 2};
  categories[6].conditions[2] = {"Menstrual pain", 20, {{5, "5 Hz"}, {7, "7 Hz"}}, 2};
  categories[6].conditions[3] = {"Cystitis", 10, {{5, "5 Hz"}, {8, "8 Hz"}}, 2};
  categories[6].conditions[4] = {"Prostatitis", 15, {{2, "2 Hz"}, {8, "8 Hz"}}, 2};
  categories[6].conditions[5] = {"Erectile dysfunction", 20, {{6, "6 Hz"}}, 1};
  categories[6].conditions[6] = {"Hepatitis", 30, {{18, "18 Hz"}}, 1};
  categories[6].conditions[7] = {"Systemic lupus erythematosus (SLE)", 20, {{5, "5 Hz"}, {22, "22 Hz"}}, 2};
  categories[6].conditions[8] = {"Chronic blepharitis", 30, {{1, "1 Hz"}, {2, "2 Hz"}}, 2};
  categories[6].conditions[9] = {"Glaucoma, atrophy of the optic nerve", 30, {{15, "15 Hz"}}, 1};
  }


void setup() {
  Serial.begin(115200);

  // Initialize relay pin FIRST to prevent random toggling
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Default to 19V (relay OFF)
  delay(100); // Give relay time to settle
  
  // Initialize I2C with delay to prevent conflicts
  delay(200);
  Wire.begin();
  Wire.setClock(100000);
  delay(100); // Give I2C bus time to stabilize
  
  // Initialize LCD with proper timing
  lcd.init();
  delay(100);
  lcd.backlight();
  delay(100);
  
  // Clear LCD and show startup message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ABO PEMF Device V5.0");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);
  
  setupTreatmentData();
  
  // Initialize sensor with error handling
  if (!am2320.begin()) {
    Serial.println("AM2320 not found on boot. Will keep trying.");
    tempSensorDisabled = false;
    tempSensorRetryCount = 0;
  } else {
    Serial.println("AM2320 sensor found!");
    tempSensorDisabled = false;
    tempSensorRetryCount = 0;
  }
  
  // Initialize all pins with proper delays
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
  
  // Initialize menu variables properly
  currentMenuOption = 0;
  currentCategoryIndex = 0;
  currentConditionIndex = 0;
  currentFrequencyIndex = 0;
  customMenuOption = 0;
  customAdjustMode = false;
  
  // Set power level initialization flag
  powerLevelInitialized = true;
  
  Serial.println("Configuring ISR Engine Timer...");
  tick_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(tick_timer, &onTimerISR, true);
  timerAlarmWrite(tick_timer, TICK_INTERVAL_US, true);
  
  // Show initialization complete message
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

void loop() {
  if (millis() - lastButtonCheck > 10) { 
    updateButtons(); 
    lastButtonCheck = millis(); 
  }
  
  if (menuState < START_STOP) {
    (menuState < DURATION_MENU) ? handleMenuNavigation() : handleValueAdjustment();
  } else {
    handleMenuNavigation();
  }
  
  if (pemfActive) {
    handlePEMF();
    if (millis() - pemfStartTime > (duration * 60000UL)) { 
      stopPEMF(); 
    }
  }
  
  if (millis() - lastTempRead > TEMP_READ_INTERVAL && !tempSensorDisabled) {
    lastTempRead = millis();
    readTemperature_robust();
  }
  
  if (millis() - lastDisplayUpdate > DISPLAY_REFRESH_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
}

// NEW CORRECTED FUNCTION
void setPowerLevel(PowerLevel level) {
    // This check is no longer needed here and was causing the bug.
    // The function will now always execute the digitalWrite.
    
    // We still update the global variable to keep the display in sync.
    currentPowerLevel = level; 

    // With transistor: HIGH = ON (5V), LOW = OFF (19V)
    digitalWrite(RELAY_PIN, (level == POWER_5V) ? HIGH : LOW);
    delay(50); // A small delay for the relay to physically switch is good practice.

    // Visual feedback on the built-in LED
    digitalWrite(LED_PIN, (level == POWER_5V) ? HIGH : LOW);
}

void resetI2CBus() {
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(50000);
  Serial.println("I2C bus reset");
}

void readTemperature_robust() {
  if (tempSensorDisabled) {
    return;
  }
  
  unsigned long wakeUpStart = millis();
  bool wakeUpSuccess = false;
  
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

void updateButtons() {
  Button* buttons[] = {&navUp, &navDown, &selUp, &selDown};
  
  for (int i = 0; i < 4; i++) {
    Button* btn = buttons[i];
    bool reading = digitalRead(btn->pin);
    
    if (reading != btn->lastState) {
      btn->lastDebounceTime = millis();
    }
    
    if ((millis() - btn->lastDebounceTime) > DEBOUNCE_DELAY) {
      if (reading != btn->currentState) {
        btn->currentState = reading;
        
        if (btn->currentState == LOW) {
          btn->pressed = true;
          btn->pressTime = millis();
          
          if (pemfActive) {
            stopPEMF();
            for (int j = 0; j < 4; j++) {
              buttons[j]->pressed = false;
            }
          }
        }
      }
      
      if (btn->pressed && (millis() - btn->pressTime > 200)) {
        btn->pressed = false;
      }
    }
    
    btn->lastState = reading;
  }
}

// handleMenuNavigation FUNCTION
void handleMenuNavigation() {
    if (millis() - stateChangeTime < STATE_CHANGE_DELAY) return;

    if (navUp.pressed) {
        navUp.pressed = false;
        switch (menuState) {
            case MAIN_MENU:
                currentMenuOption = (currentMenuOption == 0) ? 2 : currentMenuOption - 1;
                break;
            case INTENSITY_MENU:
                currentIntensity = (currentIntensity - 1);
                if (currentIntensity < NORMAL) currentIntensity = WARM;
                break;
            case POLARITY_MENU:
                currentPolarity = (PolarityMode)((currentPolarity - 1 + 3) % 3);
                break;
            case CATEGORY_MENU:
                currentCategoryIndex = (currentCategoryIndex - 1 + categoryCount) % categoryCount;
                break;
            case CONDITION_MENU:
                currentConditionIndex = (currentConditionIndex - 1 + 
                  categories[currentCategoryIndex].conditionCount) % categories[currentCategoryIndex].conditionCount;
                break;
            case FREQUENCY_MENU:
                currentFrequencyIndex = (currentFrequencyIndex - 1 + 
                  categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount) % 
                  categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount;
                break;
            case POWER_MENU: { // Braces fix the compiler error
                PowerLevel newLevel = (currentPowerLevel == POWER_19V) ? POWER_5V : POWER_19V;
                setPowerLevel(newLevel);
                break;
            }
            default:
                break;
        }
        updateDisplay();
    }

    if (navDown.pressed) {
        navDown.pressed = false;
        switch (menuState) {
            case MAIN_MENU:
                currentMenuOption = (currentMenuOption + 1) % 3;
                break;
            case INTENSITY_MENU:
                currentIntensity = (currentIntensity % 3) + 1;
                break;
            case POLARITY_MENU:
                currentPolarity = (PolarityMode)((currentPolarity + 1) % 3);
                break;
            case CATEGORY_MENU:
                currentCategoryIndex = (currentCategoryIndex + 1) % categoryCount;
                break;
            case CONDITION_MENU:
                // Your original code had a duplicated line here, I've fixed it.
                currentConditionIndex = (currentConditionIndex + 1) % categories[currentCategoryIndex].conditionCount;
                break;
            case FREQUENCY_MENU:
                currentFrequencyIndex = (currentFrequencyIndex + 1) % 
                  categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount;
                break;
            case POWER_MENU: { // Braces fix the compiler error
                PowerLevel newLevel = (currentPowerLevel == POWER_19V) ? POWER_5V : POWER_19V;
                setPowerLevel(newLevel);
                break;
            }
            default:
                break;
        }
        updateDisplay();
    }

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
            // old way for CONDITION_MENU : when just one Freq then no choice nor Freq display 
            /* case CONDITION_MENU:
                if (categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount > 1) {
                    menuState = FREQUENCY_MENU; currentFrequencyIndex = 0;
                } else {
                    menuState = DURATION_MENU;
                    duration = categories[currentCategoryIndex].conditions[currentConditionIndex].defaultDuration;
                }
                break; */
            // The new, modified code for UX consistency
            case CONDITION_MENU:
                menuState = FREQUENCY_MENU;
                currentFrequencyIndex = 0; // Always go to the frequency menu
                break;

            case FREQUENCY_MENU:
                menuState = DURATION_MENU;
                duration = categories[currentCategoryIndex].conditions[currentConditionIndex].defaultDuration;
                break;
            case POWER_MENU: menuState = MAIN_MENU; break;
            case START_STOP: if (!pemfActive) { startPEMF(); } break;
            default: break;
        }
        stateChangeTime = millis(); updateDisplay();
    }

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


void handleValueAdjustment() {
  if (millis() - stateChangeTime < STATE_CHANGE_DELAY) return;
  
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
  
  if (menuState == CUSTOM_MODE && !customAdjustMode) {
    if (navUp.pressed) { navUp.pressed = false; customMenuOption = (customMenuOption - 1 + 3) % 3; updateDisplay(); }
    if (navDown.pressed) { navDown.pressed = false; customMenuOption = (customMenuOption + 1) % 3; updateDisplay(); }
    return;
  }
  
  bool canAdjust = (menuState == DURATION_MENU) || (menuState == CUSTOM_MODE && customAdjustMode);
  if (!canAdjust) { isHolding = false; return; }
  
  if (navUp.currentState == HIGH && navDown.currentState == HIGH) { isHolding = false; return; }
  
  bool adjusted = false;
  bool isUp = (navUp.currentState == LOW);
  bool isDown = (navDown.currentState == LOW);
  
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
        lcd.write(223);
      }
      lcd.setCursor(0, 1); lcd.print(currentMenuOption == 0 ? ">" : " "); lcd.print("Condition Treat.");
      lcd.setCursor(0, 2); lcd.print(currentMenuOption == 1 ? ">" : " "); lcd.print("Custom Mode");
      lcd.setCursor(0, 3); lcd.print(currentMenuOption == 2 ? ">" : " "); 
      lcd.print("Power ");
      lcd.print((currentPowerLevel == POWER_19V) ? "19V" : "5V");
      break;
    case INTENSITY_MENU: lcd.setCursor(0, 0); lcd.print("Select Intensity:"); lcd.setCursor(0, 1);
      lcd.print(currentIntensity == NORMAL ? ">" : " "); lcd.print("1: Normal (5%)");
      lcd.setCursor(0, 2); lcd.print(currentIntensity == INTENSE ? ">" : " "); lcd.print("2: Intense (8%)"); 
      lcd.setCursor(0, 3); lcd.print(currentIntensity == WARM ? ">" : " "); lcd.print("3: Warm (13%)"); break;
    case POLARITY_MENU: lcd.setCursor(0, 0); lcd.print("Select Polarity:"); lcd.setCursor(0, 1);
      lcd.print(currentPolarity == UNIPOLAR ? ">" : " "); lcd.print("Unipolar"); lcd.setCursor(0, 2);
      lcd.print(currentPolarity == BIPOLAR ? ">" : " "); lcd.print("Bipolar"); lcd.setCursor(0, 3);
      lcd.print(currentPolarity == ALTERNATE ? ">" : " "); lcd.print("Alternate (2 min)"); break;
    case CATEGORY_MENU: lcd.setCursor(0, 0); lcd.print("Select Category:"); lcd.setCursor(0, 1);
      lcd.print(">"); lcd.print(categories[currentCategoryIndex].name); if (currentCategoryIndex < categoryCount - 1) { 
        lcd.setCursor(0, 2); lcd.print("  ");
        lcd.print(categories[(currentCategoryIndex + 1) % categoryCount].name); 
      } if (currentCategoryIndex > 0) { 
        lcd.setCursor(0, 3); lcd.print("  ");
        lcd.print(categories[(currentCategoryIndex - 1 + categoryCount) % categoryCount].name); 
      } break;
    case CONDITION_MENU: lcd.setCursor(0, 0);
      lcd.print(categories[currentCategoryIndex].name.substring(0,20)); lcd.setCursor(0, 1);
      lcd.print("Select Condition:"); lcd.setCursor(0, 2); lcd.print(">");
      lcd.print(categories[currentCategoryIndex].conditions[currentConditionIndex].name.substring(0,18)); 
      if (currentConditionIndex < categories[currentCategoryIndex].conditionCount - 1) {
        lcd.setCursor(0, 3); lcd.print("  ");
        lcd.print(categories[currentCategoryIndex].conditions[(currentConditionIndex + 1) % 
          categories[currentCategoryIndex].conditionCount].name.substring(0,18)); 
      } break;
    case FREQUENCY_MENU: lcd.setCursor(0, 0); lcd.print("Select Frequency:"); lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.print(categories[currentCategoryIndex].conditions[currentConditionIndex].frequencies[currentFrequencyIndex].description); 
      if (currentFrequencyIndex < categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount - 1) {
        lcd.setCursor(0, 2); lcd.print("  ");
        lcd.print(categories[currentCategoryIndex].conditions[currentConditionIndex].frequencies[(currentFrequencyIndex + 1) % 
          categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount].description); 
      } if (currentFrequencyIndex > 0) { 
        lcd.setCursor(0, 3); lcd.print("  ");
        lcd.print(categories[currentCategoryIndex].conditions[currentConditionIndex].frequencies[(currentFrequencyIndex - 1 + 
          categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount) % 
          categories[currentCategoryIndex].conditions[currentConditionIndex].frequencyCount].description); 
      } break;
    case DURATION_MENU: lcd.setCursor(0, 0); lcd.print("Set Duration:"); lcd.setCursor(0, 1);
      lcd.print("Current: "); lcd.print(duration); lcd.print(" min"); lcd.setCursor(0, 2);
      lcd.print("Hold UP/DOWN to"); lcd.setCursor(0, 3); lcd.print("adjust value"); break;
    case CUSTOM_MODE: lcd.setCursor(0, 0); lcd.print("Custom Mode"); if (customAdjustMode) {
      lcd.setCursor(0, 1); lcd.print("Adjusting:"); if (customMenuOption == 0) { lcd.setCursor(0, 2);
      lcd.print("Frequency: "); lcd.print(frequency); lcd.print(" Hz"); } else {
      lcd.setCursor(0, 2); lcd.print("Duration: "); lcd.print(duration); lcd.print(" min"); }
      lcd.setCursor(0, 3); lcd.print("SEL: Confirm"); } else { lcd.setCursor(0, 1);
      lcd.print(customMenuOption == 0 ? ">" : " "); lcd.print("Frequency: "); lcd.print(frequency);
      lcd.print(" Hz"); lcd.setCursor(0, 2); lcd.print(customMenuOption == 1 ? ">" : " ");
      lcd.print("Duration: "); lcd.print(duration); lcd.print(" min"); lcd.setCursor(0, 3);
      lcd.print(customMenuOption == 2 ? ">" : " "); lcd.print("Start Program"); } break;
    case POWER_MENU:
      lcd.setCursor(0, 0); lcd.print("Select Power Level:");
      lcd.setCursor(0, 1); lcd.print(currentPowerLevel == POWER_19V ? ">" : " "); 
      lcd.print("19V (Full Power)");
      lcd.setCursor(0, 2); lcd.print(currentPowerLevel == POWER_5V ? ">" : " "); 
      lcd.print("5V (Low Power)");
      lcd.setCursor(0, 3); lcd.print("SEL: Confirm");
      break;
    case START_STOP:
      switch(currentIntensity) { case NORMAL: tempStr = "Int:Norm"; break; 
        case INTENSE: tempStr = "Int:Intense"; break; case WARM: tempStr = "Int:Warm"; break; }
      switch(currentPolarity) { case UNIPOLAR: tempStr += " Pol:Uni"; break; 
        case BIPOLAR: tempStr += " Pol:Bi"; break; case ALTERNATE: tempStr += " Pol:Alt"; break; }
      lcd.setCursor(0, 0); lcd.print(tempStr);
      lcd.setCursor(0, 1); 
      lcd.print("Freq.: "); lcd.print(frequency); lcd.print(" Hz ");
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

float getDutyCycleFromIntensity() {
  switch (currentIntensity) {
    case NORMAL: return 5.0;
    case INTENSE: return 8.0;
    case WARM: return 13.0;
    default: return 5.0;
  }
}

void playStartSound() {
  tone(BUZZER_PIN, 1000, 100);
  delay(100);
  tone(BUZZER_PIN, 1500, 150);
}

void playStopSound() {
  tone(BUZZER_PIN, 1500, 100);
  delay(100);
  tone(BUZZER_PIN, 1000, 150);
}

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

void handlePEMF() {
  if (!pemfActive) return;
  
  handleFanControl();
  
  if (currentPolarity == ALTERNATE) {
    if (millis() - lastPolarityChangeTime >= POLARITY_CHANGE_INTERVAL) {
      portENTER_CRITICAL(&timerMux);
      currentDirState = !currentDirState;
      portEXIT_CRITICAL(&timerMux);
      lastPolarityChangeTime = millis();
      Serial.println("Alternate Polarity Switched");
    }
  }
  
  static unsigned long lastLedBlink = 0;
  if (millis() - lastLedBlink >= 500) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastLedBlink = millis();
  }
}

void handleFanControl() {
  if (!fanCycleEnabled && (millis() - pemfStartTime >= FAN_START_DELAY_MS)) {
    Serial.println("Fan control enabled after 5 minutes.");
    fanCycleEnabled = true;
    fanCycleStartTime = millis();
    fanActive = true;
    digitalWrite(FAN_PIN, HIGH);
    Serial.println("Fan ON");
  }
  
  if (fanCycleEnabled) {
    if (millis() - fanCycleStartTime >= FAN_CYCLE_INTERVAL_MS) {
      fanActive = !fanActive;
      digitalWrite(FAN_PIN, fanActive ? HIGH : LOW);
      Serial.println(fanActive ? "Fan ON" : "Fan OFF");
      fanCycleStartTime = millis();
    }
  }
}