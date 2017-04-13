//
// =======================================================================================================
// PROJECT DESCRIPTION
// =======================================================================================================
//

// PID Spindle controller for 2020 CNC with GRBL CNC controller


//
// =======================================================================================================
// LIBRARIES
// =======================================================================================================
//

#include <PID_v1.h>
#include <PWMFrequency.h>
#include <SimpleTimer.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//
// =======================================================================================================
// OPTIONS
// =======================================================================================================
//

//#define SERIALPRINT

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

//
// Digital 0, 1, Shared as serial pins (Serial1)
// Digital 2 - 13, General Purpose
// Digital 3, 5, 6, 9, 10, 11, and 13 - PWM
// Digital 11 - 13 Excluded on Pro Micro
// Digital 13 has (not Leo) an LED and resistor attached to it
// Digital 14 - 16, Shared with an SPI bus
// Digital 17 - Doubles and RX LED
// Digital Pins can sink 40ma, recommended 470 ohm resistor
// Analog Inputs: A0-A3, Pro Micro excludes A4, A5
// Analog Inputs: A6-A11 (on digital pins 4, 6, 8, 9, 10, and 12)
// Analog Pins can be used as digital ( refer to them as A0, A1, ...)
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
LiquidCrystal_I2C lcd(0x27,16,2);  


// PID tuning variables-(spindle specific!)-------------------------------------

//Kp: proportional (instantly), Ki: integral (slow, precise), Kd: deriative (speed of difference)
// 0.03, 0.06, 0.0015
// 0.06, 1, 0 (30ms Sample Time)
// 0.05, 1, 0 (20ms Sample Time)
// 0.07, 0.8, 0 (30ms Sample Time, OK!)
double Kp = 0.07, Ki = 0.8, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//----TIMER OBJECT
SimpleTimer timer;

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//
void setup() {

  // Initialize the lcd 
  lcd.init(); 
  lcd.backlight();  

  // Spindle Target RPM
  Setpoint = 5000;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // set PID Sample Time: 100ms instead of 200 ms is required for a smooth spindle operation!
  myPID.SetSampleTime(30);


  // Set PinModes
  pinMode(SPEED_TARGET_POT, INPUT);
  pinMode(SPEED_TARGET_CNC, INPUT);
  pinMode(SPEED_OVERRIDE_POT, INPUT);
  pinMode(SPINDLE_TACH_PIN, INPUT_PULLUP);
  pinMode(MANUAL_SWITCH, INPUT_PULLUP);
  pinMode(SPINDLE_PWM_OUT, OUTPUT);


  // Timer, which triggers the Sensor Read function
  timer.setInterval(30, readSensor);
  
  // Timer, which triggers the lcdPrint function
  timer.setInterval(750, lcdPrint);


  // Timer, which triggers the Serial.print function
  #ifdef SERIALPRINT
  Serial.begin(115200);
  timer.setInterval(300, serialPrint);
  #endif

  // Attach RPM Sensor Interrupt
  attachInterrupt(4, interruptSpindleRPM, CHANGE); // pin 2 = interrupt 1, pin 1 (TX0) = interrupt 3, pin 7 = interrupt 4

  // Adjust PWM frequency
  setPWMPrescaler(SPINDLE_PWM_OUT, 4); // Sets 1kHz Frequency ( Divisor 32 )
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//
void loop() {

  // Read Target
  readTarget();

  // Loop the timer
  timer.run();

  // call PID Spindle Controller
  spindleController();
}

//
// =======================================================================================================
// READ TARGET RPM
// =======================================================================================================
//
void readTarget() {
  
  int overridePercentage = map( analogRead(SPEED_OVERRIDE_POT), 0, 1023, 50, 150 );
  //Serial.println(overridePercentage);

  if (digitalRead(MANUAL_SWITCH)) {
    Setpoint = analogRead(SPEED_TARGET_CNC) / 100 * overridePercentage * 9.7; // CNC Mode
  }else{
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
  lcd.clear();
  
  // Target RPM
  lcd.setCursor(0, 0);
  lcd.print("RPM: ");
  lcd.print(int(Setpoint));
  
  // Spindle RPM
  lcd.setCursor(11, 0);
  lcd.print(int(Input));
  
  // PWM %
  lcd.setCursor(0, 1);
  lcd.print("PWM%: ");
  lcd.print(int(Output/255*100));
  
  // Mode
  lcd.setCursor(10, 1);
  if (digitalRead(MANUAL_SWITCH)) {
    lcd.print("M: CNC"); // CNC Mode
  }else{
    lcd.print("M: MAN"); // Manual Mode
  }
}

//
// =======================================================================================================
// SPINDLE RPM SENSOR INTERRUPT FUNCTION
// =======================================================================================================
//
void interruptSpindleRPM() {
  spindleRotationCount ++;
}
