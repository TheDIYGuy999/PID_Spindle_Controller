// PID Spindle controller for 2020 CNC with GRBL CNC controller
// SparkFun Pro Micro 16MHz board

const float codeVersion = 1.1; // Software revision

//
// =======================================================================================================
// LIBRARIES
// =======================================================================================================
//

#include <PID_v1.h>
#include <PWMFrequency.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//
// =======================================================================================================
// OPTIONS
// =======================================================================================================
//

//#define SERIALPRINT // For debugging only!

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

#define SPEED_TARGET_POT A0
#define SPEED_TARGET_CNC A1
#define SPEED_OVERRIDE_POT A2
#define SPINDLE_TACH_PIN 7 // Spindle Speed Tachometer Input
#define SPINDLE_PWM_OUT 6 // Spindle PWM Output
#define MANUAL_SWITCH 8 // Pulled to GND = Manual Mode

//Define PID Variables
double Setpoint, Input, Output;

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// PID tuning variables-(spindle specific!)-------------------------------------

//Kp: proportional (instantly), Ki: integral (slow, precise), Kd: deriative (speed of difference)
// 0.03, 0.06, 0.0015
// 0.06, 1, 0 (30ms Sample Time)
// 0.05, 1, 0 (20ms Sample Time)
// 0.07, 0.8, 0 (30ms Sample Time, OK!)
double Kp = 0.07, Ki = 0.8, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//
void setup() {

  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.print("PID Controller");
  lcd.setCursor(0, 1);
  lcd.print("Version: ");
  lcd.print(codeVersion);
  delay(3000);
  lcd.clear();
  

  // Spindle Target RPM
  Setpoint = 0;

  //turn the PID controller on
  myPID.SetMode(AUTOMATIC);

  // set PID Sample Time: 30ms instead of 200 ms is required for a smooth spindle operation!
  myPID.SetSampleTime(30);


  // Set PinModes
  pinMode(SPEED_TARGET_POT, INPUT);
  pinMode(SPEED_TARGET_CNC, INPUT);
  pinMode(SPEED_OVERRIDE_POT, INPUT);
  pinMode(SPINDLE_TACH_PIN, INPUT_PULLUP);
  pinMode(MANUAL_SWITCH, INPUT_PULLUP);
  pinMode(SPINDLE_PWM_OUT, OUTPUT);


  // Setup Serial.print
#ifdef SERIALPRINT
  Serial.begin(115200);
#endif

  // Attach RPM Sensor Interrupt
  attachInterrupt(4, interruptSpindleRPM, CHANGE); // pin 2 = interrupt 1, pin 1 (TX0) = interrupt 3, pin 7 = interrupt 4

  // Adjust PWM frequency
  setPWMPrescaler(SPINDLE_PWM_OUT, 4); // Sets 1kHz Frequency ( Divisor 32 )
}

//
// =======================================================================================================
// READ TARGET RPM
// =======================================================================================================
//
void readTarget() {

  int overridePercentage = map( analogRead(SPEED_OVERRIDE_POT), 0, 1023, 50, 150 );

  if (digitalRead(MANUAL_SWITCH)) {
    Setpoint = analogRead(SPEED_TARGET_CNC) / 100 * overridePercentage * 9.7; // CNC Mode
  } else {
    Setpoint = analogRead(SPEED_TARGET_POT) * 9.7; // Manual Mode
  }
  if (Setpoint <= 1500.0) Setpoint = 0.0;
}

//
// =======================================================================================================
// SPINDLE RPM CALCULATION
// =======================================================================================================
//
volatile long spindleRotationCount;  // "volatile" is required because of the interrupt functions!
static unsigned long spindleTimeStart;

// This function calculates the current spindle RPM
int getSpindleRPM() {
  unsigned long rpm = spindleRotationCount * 3750 / ( millis() - spindleTimeStart );
  spindleRotationCount = 0;
  spindleTimeStart = millis();
  return  rpm;
}

//
// =======================================================================================================
// PID SPINDLE CONTROLLER
// =======================================================================================================
//
void spindleController() {
  myPID.Compute();
  if (Setpoint <= 1500.0) analogWrite(SPINDLE_PWM_OUT, 0);
  else analogWrite(SPINDLE_PWM_OUT, Output);
}

//
// =======================================================================================================
// READ RPM SENSOR
// =======================================================================================================
//
void readSensor() {
  Input = getSpindleRPM();
}


//
// =======================================================================================================
// SERIAL PRINT
// =======================================================================================================
//
void serialPrint() {
  Serial.print("Target RPM: ");
  Serial.print(Setpoint);
  Serial.print("   Spindle RPM: ");
  Serial.print(Input);
  Serial.print("   PWM: ");
  Serial.println(Output);
}

//
// =======================================================================================================
// LCD PRINT
// =======================================================================================================
//
void lcdPrint() {

  //lcd.clear(); // We don't want to clear the entire screen every time!

  // Compute formatted outputs
  double OutputPercent = Output / 255 * 100;
  char bufSetpoint[6];
  char bufInput[6];
  char bufOutput[4];
  sprintf(bufSetpoint, "%-5u", (int)Setpoint);
  sprintf(bufInput, "%-5u", (int)Input);
  sprintf(bufOutput, "%-3u", (int)OutputPercent);

  // Target RPM
  lcd.setCursor(0, 0);
  lcd.print("RPM: ");
  lcd.print(bufSetpoint);

  // Spindle RPM
  lcd.setCursor(11, 0);
  lcd.print(bufInput);

  // PWM %
  lcd.setCursor(0, 1);
  lcd.print("PWM%: ");
  lcd.print(bufOutput);

  // Mode
  lcd.setCursor(10, 1);
  if (digitalRead(MANUAL_SWITCH)) {
    lcd.print("M: CNC"); // CNC Mode
  } else {
    lcd.print("M: MAN"); // Manual Mode
  }
}

//
// =======================================================================================================
// SPINDLE RPM SENSOR INTERRUPT FUNCTION
// =======================================================================================================
//
void interruptSpindleRPM() {
  spindleRotationCount ++; // Just count the sensor pulses
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//
void loop() {

  // Every 30 ms:
  static unsigned long lastRead;
  if (millis() - lastRead >= 30) {
    lastRead = millis();

    // Read target RPM
    readTarget();

    // Read RPM sensor
    readSensor();
  }

  // Every 300 ms:
  static unsigned long lastLcd;
  if (millis() - lastLcd >= 300) {
    lastLcd = millis();

    // Print LCD
    lcdPrint();

#ifdef SERIALPRINT
    serialPrint();
#endif
  }

  // Call PID Spindle Controller
  spindleController();
}
