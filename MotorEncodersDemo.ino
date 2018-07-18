//Test comment
// include library codes:
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <Encoder.h>

// These #defines make it easy to set the backlight color
#define RED      0x1
#define GREEN    0x2
#define YELLOW   0x3
#define BLUE     0x4
#define VIOLET   0x5
#define TEAL     0x6
#define WHITE    0x7

// Motor pins
#define E1 9 
#define M1 8 
#define E2 6 
#define M2 7 

// Encoder pins
#define Enc1A 2   // Left white
#define Enc1B 4   // Left yellow
#define Enc2A 3   // Right yellow
#define Enc2B 5   // Right white

// The LCD shield uses the I2C SCL and SDA pins. On the Arduino Uno
// these are Analog pins 4 and 5 so you can't use those for analogRead() any more.
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// Change these pin numbers to the pins connected to the motor encoder.
// Best Performance: both pins have interrupt capability
// Good Performance: only the first pin has interrupt capability
// Low Performance:  neither pin has interrupt capability
Encoder motorLeftEnc(Enc1A, Enc1B);
Encoder motorRightEnc(Enc2A, Enc2B);

uint8_t buttons;
unsigned long last;

// If the Rover was built with different encoders, they might count
// differently.  Use these two floats to calibrate the encoders.
const float leftMult = 1.0;
const float rightMult = 1.0;

// Conversion of Encoders into standard distance units (mm/cm/in)
// are controlled by these two floats.
const float leftConversion = 1.0;
const float rightConversion = 1.0;

// Use these two PWM values to make the Rover drive straight
const char leftPWM = 255;
const char rightPWM = 255;

void setup() {
  Serial.begin(9600);
  
  // Motor driver pins
  pinMode(E1, OUTPUT); 
  pinMode(E2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  // Encoder pins
  pinMode(Enc1A, INPUT);
  pinMode(Enc1B, INPUT);
  pinMode(Enc2A, INPUT);
  pinMode(Enc2B, INPUT);

  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.print("Booting ...");
  lcd.setBacklight(RED);

  // initialize motor drivers
  digitalWrite(E1, LOW);
  digitalWrite(E2, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);

  //User Interface Load Screen
  delay(1000);
  lcd.setBacklight(TEAL);
  delay(1000);
  lcd.clear();
  lcd.print("Press SELECT");
  lcd.setBacklight(VIOLET);
  
  buttons = 0;
  while (buttons == 0)
  {
    buttons = lcd.readButtons() & BUTTON_SELECT;
  }

  lcd.clear();
  lcd.print("Calibrating ... ");
  lcd.setBacklight(YELLOW);
  delay(500);
  
  // Initialize encoders
  motorLeftEnc.write(0);
  motorRightEnc.write(0);
  lcd.clear();
  lcd.print("Ready");
  lcd.setBacklight(GREEN);
  delay(500);
  last = millis();
}

void loop() {

  buttons = lcd.readButtons();

  if (buttons) {
    if (buttons & BUTTON_UP) {
      digitalWrite(M1, LOW);
      digitalWrite(M2, HIGH);
      stopLeft(1000);
    }
    if (buttons & BUTTON_DOWN) {
      digitalWrite(M1, HIGH);
      digitalWrite(M2, LOW);
      stopLeft(1000);
    }
    if (buttons & BUTTON_LEFT) {
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      stopLeft(500);
    }
    if (buttons & BUTTON_RIGHT) {
      digitalWrite(M1, LOW);
      digitalWrite(M2, LOW);
      stopLeft(500);
    }
    if (buttons & BUTTON_SELECT) {
      lcd.print("Reset ");
      motorLeftEnc.write(0);
      motorRightEnc.write(0);
    }
    Serial.print("buttons");
    Serial.print("\t");    
    Serial.print(motorLeftEnc.read());
    Serial.print("\t");
    Serial.println(motorRightEnc.read());
    lcd.setCursor(0,1);
    lcd.print("L");
    lcd.print((leftConversion*motorLeftEnc.read()) / leftMult);
    lcd.print(" R");
    lcd.print((rightConversion*motorRightEnc.read()) / rightMult);
    lcd.print("     ");
  }
  else if (millis()-last > 50) {
    Serial.print("else if");
    Serial.print("\t");    
    Serial.print(motorLeftEnc.read());
    Serial.print("\t");
    Serial.println(motorRightEnc.read());
    lcd.setCursor(0,1);
    lcd.print("L");
    lcd.print((leftConversion*motorLeftEnc.read()) / leftMult);
    lcd.print(" R");
    lcd.print((rightConversion*motorRightEnc.read()) / rightMult);
    lcd.print("     ");
    last = millis();
  }
}

void stopLeft(float lastCount) {
  motorLeftEnc.write(0);
  motorRightEnc.write(0);
  last = millis();
  analogWrite(E1, leftPWM);
  analogWrite(E2, rightPWM);
  while (abs(motorLeftEnc.read() / leftMult) < abs(lastCount)) { 
    Serial.print("stopLeft");
    Serial.print("\t");    
    Serial.print(motorLeftEnc.read());
    Serial.print("\t");
    Serial.println(motorRightEnc.read());
    if (millis()-last > 50) {
      lcd.setCursor(0,1);
      lcd.print("L");
      lcd.print((leftConversion*motorLeftEnc.read()) / leftMult);
      lcd.print(" R");
      lcd.print((rightConversion*motorRightEnc.read()) / rightMult);
      lcd.print("     ");
      last = millis();
    }
  }
  analogWrite(E1, 0);
  analogWrite(E2, 0);
  delay(100);
}


