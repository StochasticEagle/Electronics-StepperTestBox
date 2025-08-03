// Stepper Motor Control Test Box

#include <Arduino.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_pinIO.h>
#include "Button.h"

#if defined(ESP32)
    #define ADC_BITS 12
    #define ADC_RANGE ((1 << ADC_BITS) - 1)
    #define VREF 3.3
    #define LED_BUILTIN 2
#elif defined(ARDUINO_ARCH_AVR)
    #define ADC_BITS 10
    #define ADC_RANGE ((1 << ADC_BITS) - 1)
    #define VREF 5.0
#else
    #warning "Using default ADC range (1023) and VREF (5.0). Please verify these values for your board."
    #define ADC_BITS 10
    #define ADC_RANGE ((1 << ADC_BITS) - 1)
    #define VREF 5.0
#endif

// CW and CCW arrow patterns
byte cwArrow[8] = {
    B00010,
    B11111,
    B00010,
    B00000,
    B01100,
    B10010,
    B10010,
    B01100
};

byte ccwArrow[8] = {
    B01000,
    B11111,
    B01000,
    B00000,
    B00110,
    B01001,
    B01001,
    B00110
};

enum eMODE
{
    DM_STOP, DM_RUN,
    DM_NUM_STATES
};

// MS1	MS2	Microstep Resolution	Excitation Mode
// L	L	Full step               2 phase
// H	L	Half step               1-2 phase
// L	H	Quarter step            W1-2 phase
// H	H	Sixteenth step          4W1-2 phase

enum eMICROSTEP
{
    MS_FULL,
    MS_HALF,
    MS_QUARTER,
    MS_SIXTEENTH,
    MS_NUM_STATES
};

#define motorCW true
#define motorCCW false

// Control Logic ////
// Pin Assignments
static const int LCD_RS_PIN = 8, LCD_EN_PIN = 12, LCD_D4_PIN = A0, LCD_D5_PIN = A1, LCD_D6_PIN = A2, LCD_D7_PIN = A3;
static const int LCD_BL_RED_PIN = 9, LCD_BL_GREEN_PIN = 10, LCD_BL_BLUE_PIN = 11;
static const int KNOB_SPEED_PIN = A7;
static const int BTN_STOP_PIN = A5;
static const int BTN_RUN_PIN = A4;
static const int BTN_DIR_PIN = 7;
static const int MTR_STEP_PIN = 6;      // pin to control motor step
static const int MTR_EN_PIN = 5;        // Motor enable pin or step pin
static const int MTR_DIR_PIN = 4;       // Motor direction control pin
static const int MTR_MS1_PIN = 2;       // pin to control motor microstepping
static const int MTR_MS2_PIN = 3;       // pin to control motor microstepping

// Inits and control constants
#define LCD_BL_COMMON_ANODE    // Define if LCD has common anode backlight, otherwise comment out

hd44780_pinIO lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);    // Initialize LCD interface pin with the arduino pin numbers
Button stopButton(BTN_STOP_PIN);
Button runButton(BTN_RUN_PIN);
Button dirButton(BTN_DIR_PIN, true, 50, 500);  // 500ms long press delay for direction button
static const char* microstepStrings[4] = {"1   ", "1/2 ", "1/4 ", "1/16"};
static const int microstepMultiplier[4] = {1, 2, 4, 16};  // Frequency multiplier to keep the same RPM
static const unsigned long intervalLED = 500000;        // 1 Hz heartbeat (us)
static const unsigned long intervalRefresh = 3077;      // 30 Hz screen refresh rate (us)
static const unsigned long btnLongPressTime = 1500000;       // Minimum time to start long press action (us)
static const int speedDelta = 0.025f * ADC_RANGE;
static const int minSpeedThreshold = speedDelta;                        // 2.5% threshold
static const int maxSpeedThreshold = ADC_RANGE - speedDelta;            // 97.5% threshold
static const int speedRange = maxSpeedThreshold - minSpeedThreshold;    // The deltaX for the range
static const int motorfullRPM = 300;                     // 100% Speed for stepper motor testing
static const int motorFullStepsPerRev = 200;            // For 1.8 degrees/step motors
static const unsigned int stepPulseWidthMicros = 5;     // Step pulse HIGH duration in microseconds
static const uint8_t charIdxArr[13][2] = {{12, 0}, {13, 0}, {14, 0}, {2,1}, {3,1}, {4,1}, {5,1}, {6,1}, {7,1}, {8,1}, {9,1}, {10,1}, {11,1}};     // Index array for piecewise LCD updates

// Control variables
eMODE devMode = DM_STOP;
eMICROSTEP microstepMode = MS_FULL;
unsigned long prevTimeLED = 0;              // Keep track of previous event time
unsigned long prevTimeRefresh = 0;          // Keep track of previous event time
unsigned long currentTime = 0;              // Declare global time variable, prevents stack usage of var 
unsigned long lastStepTimeMicros = 0;       // Last timestamp a step pulse was generated
bool stepPulseActive = false;               // Whether the step pin is currently HIGH
bool motorDir = motorCW;
int motorSpeed = 0;
bool prevMotorSpeedZero = true;
unsigned long stepCount = 0;
uint8_t charPos = 0;            // Keep track of which character in the datastream was written.  Used for tiny loop time adjustments
char dataBuf[14];           // Store the character buffer

void setup()
{
    //Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LCD_D4_PIN, OUTPUT);    // LED DATA
    pinMode(LCD_D5_PIN, OUTPUT);    // LED DATA
    pinMode(LCD_D6_PIN, OUTPUT);    // LED DATA
    pinMode(LCD_D7_PIN, OUTPUT);    // LED DATA
    pinMode(LCD_BL_RED_PIN, OUTPUT);
    pinMode(LCD_BL_GREEN_PIN, OUTPUT);
    pinMode(LCD_BL_BLUE_PIN, OUTPUT);

    // Setup pin modes for buttons, input pullup for active low
    pinMode(BTN_STOP_PIN, INPUT_PULLUP);
    pinMode(BTN_RUN_PIN, INPUT_PULLUP);
    pinMode(BTN_DIR_PIN, INPUT_PULLUP);

    // Stop button: Only short press on release, no long press
    stopButton.onShortPressPress = []() {
        //Serial.println("Stop button short press (press)");
        devMode = DM_STOP;
        digitalWrite(MTR_EN_PIN, HIGH);
        setColorMode();
        renderLCD_Data();
    };

    // Run button: Only short press on release, no long press
    runButton.onShortPressPress = []() {
        //Serial.println("Run button short press (press)");
        devMode = DM_RUN;
        digitalWrite(MTR_EN_PIN, LOW);
        setColorMode();
        renderLCD_Data();
    };

    // Direction button:
    dirButton.onShortPressRelease = []() {
        //Serial.println("Direction button short press (release) - toggle motorDir");
        motorDir = !motorDir;
        setDirection();
        renderLCD_UI();
        renderLCD_Data();
    };

    dirButton.onLongPressRelease = []() {
        //Serial.println("Direction button long press triggered - cycle microstepMode");
        microstepMode = (eMICROSTEP)((microstepMode + 1) % MS_NUM_STATES);
        setMicrostepping();
        renderLCD_UI();
        renderLCD_Data();
    };

    setColor(255, 0, 0);            // lcd backlught
    lcd.begin(16, 2);               // set up the LCD's number of columns and rows
    lcd.createChar(0, cwArrow);
    lcd.createChar(1, ccwArrow);
    renderLCD_UI();
    renderLCD_Data();

    // Setup the motor driver
    pinMode(MTR_DIR_PIN, OUTPUT);
    pinMode(MTR_EN_PIN, OUTPUT);
    pinMode(MTR_STEP_PIN, OUTPUT);
    pinMode(MTR_MS1_PIN, OUTPUT);
    pinMode(MTR_MS2_PIN, OUTPUT);
    digitalWrite(MTR_STEP_PIN, LOW);
    digitalWrite(MTR_EN_PIN, HIGH);  // Active-low, disabled on startup
    setMicrostepping();
    setDirection();
}


void loop()
{
    currentTime = micros();
    motorSpeed = pctSpeed(analogRead(KNOB_SPEED_PIN));
    stopButton.update();
    runButton.update();
    dirButton.update();
    softwareStepperControl();

    // LED Heartbeat
    if (currentTime - prevTimeLED >= intervalLED)
    {
        prevTimeLED = currentTime;

        #if defined(ESP32)
            REG_WRITE(GPIO_OUT_REG_LED, REG_READ(GPIO_OUT_REG_LED) ^ GPIO_LED_MASK);
        #elif defined(ARDUINO_ARCH_AVR)
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        #endif
    }

    // LCD screen refresh (called every 0.3077 seconds, for 25 Hz on the entire screen (40/13 ms))
    if (currentTime - prevTimeRefresh >= intervalRefresh)
    {
        prevTimeRefresh = currentTime;
        renderLCD_Data();

        if (motorSpeed == 0)
        {
            if (!prevMotorSpeedZero) setColorMode();
            prevMotorSpeedZero = true;
        }
        else
        {
            if (prevMotorSpeedZero) setColorMode();
            prevMotorSpeedZero = false;
        }
    }
}


void setColor(int redValue, int greenValue, int blueValue)
{
    #ifdef LCD_BL_COMMON_ANODE
        // Inverted for common anode
        analogWrite(LCD_BL_RED_PIN, 255 - redValue);
        analogWrite(LCD_BL_GREEN_PIN, 255 - greenValue);
        analogWrite(LCD_BL_BLUE_PIN, 255 - blueValue);
    #else
        // Straight values for common cathode
        analogWrite(LCD_BL_RED_PIN, redValue);
        analogWrite(LCD_BL_GREEN_PIN, greenValue);
        analogWrite(LCD_BL_BLUE_PIN, blueValue);
    #endif
}

void setColorMode()
{
    switch (devMode)
    {
        case DM_STOP:
            setColor(255, 0, 0);
            break;
        case DM_RUN:
            if (motorSpeed == 0)
            {
                setColor(255, 150, 0);
            }
            else
            {
                setColor(0, 255, 0);
            }
            break;
        default:
            setColor(255, 150, 0);
            break;
    }
}

int pctSpeed(int reading)
{
    if (reading <= minSpeedThreshold) return 0;
    if (reading >= maxSpeedThreshold) return 100;

    // Linearly interpolate between minSpeedThreshold and maxSpeedThreshold to 0..100
    return round((float(reading - minSpeedThreshold) / speedRange) * 100);
}

void renderLCD_UI()
{
    // Draw labels and units
    lcd.setCursor(0, 0);
    lcd.write(228);                                 // Greek 'mu'
    lcd.setCursor(1, 0);
    lcd.print(F("Step:         %"));                // this is shorter because of the "mu" character, and the LCD library doesn't support escape codes
    lcd.setCursor(0, 1);
    lcd.print(F("n:           r: "));
    lcd.setCursor(15, 1);
    lcd.write(byte(motorDir ? 0 : 1));              // Data element, but infrequent change
    lcd.setCursor(7, 0);
    lcd.print(microstepStrings[microstepMode]);     // Data element, but infrequent change
}

void renderLCD_Data()
{
    if (charPos == 0)
    {
        snprintf(dataBuf, sizeof(dataBuf), "%3d%10lu", motorSpeed, stepCount);
    }

    lcd.setCursor(charIdxArr[charPos][0], charIdxArr[charPos][1]);
    lcd.print(dataBuf[charPos]);
    charPos++;
    if (charPos > 12) charPos = 0;
}

void setMicrostepping()
{
    digitalWrite(MTR_MS1_PIN, (microstepMode & 0x01) ? HIGH : LOW);
    digitalWrite(MTR_MS2_PIN, (microstepMode & 0x02) ? HIGH : LOW);
}

void setDirection()
{
    digitalWrite(MTR_DIR_PIN, (motorDir == motorCW) ? LOW : HIGH);
}

static inline unsigned long calcStepMicros()
{
    double actualRPS = (motorfullRPM * motorSpeed) / 6000.0f;
    //int stepsPerRev = motorFullStepsPerRev * microstepMultiplier[int(microstepMode)];
    int stepsPerRev = motorFullStepsPerRev; //disable microstep speed compensation, because the Arduino is not fast enough to implement constant RPM modes effectively.
    double stepsPerSecond = (stepsPerRev * actualRPS);              // Motor speed in Hz, based on percent setting
    return (motorSpeed > 0) ? (unsigned long)(1000000.0f / stepsPerSecond) : 131072;   // Prevent division by zero
}

void softwareStepperControl() {
    static unsigned long pulseStartTime = 0;
    unsigned long now = micros();
    unsigned long stepInterval = calcStepMicros(); // your existing function to get step period

    if (devMode != DM_RUN || motorSpeed == 0) {
        // Motor stopped - ensure step pin is low and reset states
        digitalWrite(MTR_STEP_PIN, LOW);
        stepPulseActive = false;
        return;
    }

    if (stepPulseActive) {
        // Currently pulsing HIGH; check if pulse width elapsed
        if (now - pulseStartTime >= stepPulseWidthMicros) {
            digitalWrite(MTR_STEP_PIN, LOW);
            stepPulseActive = false;
            lastStepTimeMicros = now;
            stepCount++;  // Increment step count on falling edge
        }
        // else: still within pulse, do nothing to keep pin HIGH
    }
    else {
        // Step pulse not active; check if it's time for next pulse
        if (now - lastStepTimeMicros >= stepInterval) {
            digitalWrite(MTR_STEP_PIN, HIGH);
            pulseStartTime = now;
            stepPulseActive = true;
        }
    }
}
