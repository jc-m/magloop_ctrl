// optical encoder code from https://www.instructables.com/id/Improved-Arduino-Rotary-Encoder-Reading/

// include the library code:
#include <LiquidCrystal_PCF8574.h>
#include <AccelStepper.h>

LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display


#define DEBUG 1

#define CW 0
#define CCW 1
#define TUNING 0
#define SETTING 1

// defines pins numbers
// A4988/DRV8825 pins
const int enablePin = 13; // 8
const int ms1Pin = 11; // 7
const int ms2Pin = 10; // 6
const int ms3Pin = 9;  // 5
const int stepPin = 8; // 2
const int dirPin = 7;  // 1

// optical encoder pins
const int pinA = 2;  // need interrupts
const int pinB = 3;  // need interrupts
const int buttonPin  = 4;

int stepCount;
int stepList [6] = { 1, 2, 5, 10, 50, 100};
int stepIndex =2; // start at 10 steps per rotation.

char line2[16];
char* modes[] = {
  "Tuning ",
  "Setting"
};

volatile byte aFlag = 0;
volatile byte bFlag = 0;
volatile byte encoderPos = 0;
volatile int8_t encoderDir = 0;
volatile byte oldEncPos = 0;
volatile byte reading = 0;

unsigned long lastMotorRun = 0;
bool enabled;

byte lastButtonState = HIGH;
byte buttonState = HIGH;
bool menu;
unsigned long lastButtonChange = 0;
unsigned long debounceDelay = 50;    // the debounce time;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {

  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(false, false, true);
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(100);
  stepper.setMinPulseWidth(100);
  
  // Sets the stepper driver microstepping pins as Outputs
  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);
  pinMode(ms3Pin, OUTPUT);

  // Sets the encoder pins as input with pull up resistor
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  attachInterrupt(0, PinA, RISING);
  attachInterrupt(1, PinB, RISING);

   // set the microstepping to 1/16 on A4988 or 1/32 on DRV8825
  digitalWrite(ms1Pin, HIGH);
  digitalWrite(ms2Pin, HIGH);
  digitalWrite(ms3Pin, LOW);

#ifdef DEBUG
  Serial.begin(115200);
#endif
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setBacklight(1);
  
  stepCount = stepList[stepIndex];
  menu = false;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(modes[TUNING]);
  lcd.setCursor(0, 1);
  sprintf(line2, " %4d", stepCount);
  lcd.print(line2);
}

void PinA() {
  cli();
  reading = PIND & 0xC;
  if (reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --;
    encoderDir = -1;
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei();
}

void PinB() {
  cli();
  reading = PIND & 0xC;
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++;
    encoderDir = 1;
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei();
}

void loop() {

  int button = digitalRead(buttonPin);

  if (button != lastButtonState) {
    lastButtonChange = millis();
  }

  if ((millis() - lastButtonChange) > debounceDelay) {
    // if the button state has changed:
    if (button != buttonState) {
      buttonState = button;

      if (buttonState == HIGH) {
#ifdef DEBUG
        Serial.print("Press "); Serial.println(menu);
#endif
        menu = !menu;
        lcd.setCursor(0, 0);
        if (menu) {
          lcd.print(modes[SETTING]);
        } else {
          lcd.print(modes[TUNING]);
        }
      }
    }
  }
  if (oldEncPos != encoderPos) {
    
    char* dir;

    if (encoderDir < 0) {
      if (menu) {
        dir=" ";
        if (stepIndex > 0) {
          stepIndex --;
#ifdef DEBUG
          Serial.print("StepIndex "); Serial.println(stepIndex);
#endif
        }
      } else {
        dir="-";
        stepper.move(stepCount * -1);
#ifdef DEBUG
        Serial.print(dir); Serial.print(encoderPos);Serial.print("="); Serial.println(stepper.targetPosition());
#endif

      }

    } else {
      if (menu) {
        dir=" ";
        if (stepIndex < (sizeof(stepList) / sizeof(stepList[0])) - 1) {
          stepIndex ++;
#ifdef DEBUG
          Serial.print("StepIndex "); Serial.println(stepIndex);
#endif
        }
      } else {
        dir="+";
        stepper.move(stepCount);
#ifdef DEBUG
        Serial.print(dir); Serial.print(encoderPos);Serial.print("="); Serial.println(stepper.targetPosition());
#endif
      }

    }
    stepCount = stepList[stepIndex];
    lcd.setCursor(0, 1);
    sprintf(line2, "%s%4d", dir, stepCount);
    lcd.print(line2);
    oldEncPos = encoderPos;
    
  }
  if (stepper.distanceToGo() != 0) {
    // set this here as this was not reliable for single steps.
    lastMotorRun = millis();
    if (!enabled) {
#ifdef DEBUG
      Serial.println("Waking up ... ");
#endif
      stepper.enableOutputs();
      enabled = true;
    }
  } else {
    if (enabled && (millis() > lastMotorRun + 100)) {
        stepper.disableOutputs();
        enabled = false;
#ifdef DEBUG
        Serial.println("Sleeping ... ");
#endif
    }
  }
  
  stepper.run();
  
  lastButtonState = button;

}
