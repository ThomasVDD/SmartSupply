/* //==\\ ||\\  //||     //\\     ||===\\   ========  //==\\  ||    ||  ||===\\   ||===\\   ||        \\    //
 * ||     || \\// ||    //  \\    ||    ||     ||     ||      ||    ||  ||    ||  ||    ||  ||         \\  //
 * \\==\\ ||  \/  ||   //    \\   ||===//      ||     \\==\\  ||    ||  ||===//   ||===//   ||          \\//
 *     || ||      ||  //======\\  ||   \\      ||         ||  ||    ||  ||        ||        ||           //
 * \\==// ||      || //        \\ ||    \\     ||     \\==//  \\====//  ||        ||        ||======    //
 * 
 * Thomas Van den Dries
 * github.com/ThomasVDD
 * ThomasVDD on Instructables 
 * 
 * Board: arduino pro or pro mini
 * Programmer: AVR ISP / AVRISP MKII
 * For burning the bootloader: https://www.arduino.cc/en/Tutorial/ArduinoToBreadboard
 * If you accidently ordered ATMEGA328-PU instead of ATMEGA328P-PU: https://www.instructables.com/id/Bootload-an-ATmega328/
 */

/* ============================================== LIBRARIES =======================================================*/
#include<SPI.h>
#include<EEPROM.h>
#include<LiquidCrystal.h>
#include<Wire.h>
#include<INA219.h>
LiquidCrystal lcd(8);
INA219 ina219;

/* ============================================== DECLARATIONS =====================================================*/
/* regulator_output is the only value that should be changed to calibrate the powersupply
 * Measure the output voltage of the 5V voltage regulator and put it below
 */
float regulator_output = 4.94;
/* Now, the error can be calculated and used to calibrate */
float regulator_error = regulator_output / 5;
/* make symbols for battery level*/
byte batt0[8] = {B00000, B00000, B00000, B00000, B00000, B00000, B00000,};
byte batt1[8] = {B00000, B00000, B00000, B00000, B00000, B00000, B00111,};
byte batt3[8] = {B00000, B00000, B00000, B00000, B00111, B00111, B00111,};
byte batt4[8] = {B00000, B00000, B00000, B00111, B00111, B00111, B00111,};
byte batt6[8] = {B00000, B00111, B00111, B00111, B00111, B00111, B00111,};
byte batt7[8] = {B00111, B00111, B00111, B00111, B00111, B00111, B00111,};
int batteryLine0[] = {0, 0, 1, 4, 7};
int batteryLine1[] = {3, 6, 7, 7, 7};
int batteryLevel = 0;
int batteryPin = A0;
const int numReadingsB = 10;
int readingsB[numReadingsB];
int readIndexB = 0;
int totalB = 0;
float averageB = 0;
float batteryVoltage;

int boostConverter = 0;
int boostVoltage = 0;
int potentiometerPin = A1;
int chargePump = 5;

int VoutPin = A3;
int IoutPin = A2;

int zeroButton = 6;
boolean outputOnOff = 0;

/*Rotary encoders: 0 = voltage, 1 = current*/
#define encoder0PinA  2
#define encoder1PinA  3
#define encoderPinB  4

volatile unsigned int encoder0Pos;  // keep the position of the voltage rotary encoder (0 - 50000)
volatile unsigned int encoder1Pos;  // keep the position of the current rotary encoder (0 - 50000)
int amount0 = 10;                   // changes the amount counter 0 is increased or decreased
int amount1 = 10;
unsigned long prevTime0 = 0;        // stores the previous time the interrupt was executed
unsigned long prevTime1 = 0;        // unsinged long needed for calculations with millis()!
int ticks0 = 0;                     // stores the number of times the intterupt has been called
int ticks1 = 0;
int prevEncoder0Pos = 0;            // tracks a change in value
int prevEncoder1Pos = 0;
volatile float pos0;                //keep the value of the voltage (0 - 20V mapped to 0-999)
volatile float pos1;                //keep the value of the current (0 - 1A  mapped to 0-999)

/*averages for voltage and current*/
const int numReadingsV = 20;        // number of readings to be averaged
int readingsV[numReadingsV];        // the readings from the analog input
int readIndexV = 0;                 // the index of the current reading
int totalV = 0;                     // the running total
float averageV = 0;                 // the average
float realAverageV = 0;             // the remapped average

const int numReadingsA = 20;
int readingsA[numReadingsA];
int readIndexA = 0;
int totalA = 0;
float averageA = 0;
float realAverageA = 0;

float finalA = 0;                   // holds the most accurate current measurement to be displayed
int currentRange = 320;             // the range of the current measurement, initialized at 320 mA

/* calibration & presets (pushbutton actions)*/
int currentOffset = 0;              // used to calibrate the current measurement
String readString;                  // read serial input
boolean calibrate = 1;              // tracks the current calibrate setting
String message[] = {"CALIB", "TOTAL"}; // calibration messages
float calibration[] = {0, 0};       // keeps the calibrated current
int presetButton = 7;               // switch for presets
int preset;                         // tracks the current preset
int presetV[] = {0, 0, 165, 250, 500, 999}; // voltage presets
int presetA[] = {0, 0, 330, 500, 999, 999}; // current presets
int numberOfPresets = 6;            // length of the array. First preset used for saved value

/* ============================================== SETUP =====================================================*/
void setup() {
  Serial.begin(9600);               // start serial communication
  ina219.begin();                   // initialize ina219
  setupPWM16(regulator_error);                     // initialize the 10 bit pwm

  lcd.begin(16, 2);                 // initialize lcd & make battery symbols
  lcd.createChar(0, batt0);
  lcd.createChar(1, batt1);
  lcd.createChar(3, batt3);
  lcd.createChar(4, batt4);
  lcd.createChar(6, batt6);
  lcd.createChar(7, batt7);

  pinMode(presetButton, INPUT);
  digitalWrite(presetButton, HIGH);   //turn on pullup resistor
  pinMode(zeroButton, INPUT);
  digitalWrite(zeroButton, HIGH);    //turn on pullup resistor
  pinMode(potentiometerPin, OUTPUT);
  digitalWrite(potentiometerPin, HIGH);
  pinMode(chargePump, OUTPUT);
  analogWrite(chargePump, 127);       //50% pwm for charge pump 

  encoder0Pos = EEPROMReadInt(0);                    // retreive the previous voltage value from memory
  pos0 = map(encoder0Pos, 0, 50000, 0, 999);         //map to 0 - 5 V ==> 1mA / step
  presetA[0] = map(encoder0Pos, 0, 50000, 0, 999);
  encoder1Pos = EEPROMReadInt(2);                    // retreive the previous current value from memory
  pos1 = map(encoder1Pos, 0, 50000, 0, 999);         //map to 0 - 5 V ==> 20mV / step
  presetV[0] = map(encoder1Pos, 0, 50000, 0, 999);
  //preset = EEPROMReadInt(4);                       // retreive the previous preset from memory

  //encoder 0
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  attachInterrupt(0, doEncoder0, CHANGE); // encoder pin on interrupt 0 - pin 2
  //encoder 1
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA, HIGH);       // turn on pullup resistor
  attachInterrupt(1, doEncoder1, CHANGE); // encoder pin on interrupt 1 - pin 3
  // common pin  
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);       // turn on pullup resistor

  analogReference(EXTERNAL);             //external voltage reference of 2.048 V
  
  /* initialize all the readings to 0 */
  for (int thisReading = 0; thisReading < numReadingsV; thisReading++) {
    readingsV[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < numReadingsA; thisReading++) {
    readingsA[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < numReadingsB; thisReading++) {
    readingsB[thisReading] = 0;
  }

  /* Show welcome message */
  lcd.setCursor(0, 0);
  lcd.print("  SmartSupply");
  lcd.setCursor(0, 1);
  lcd.print("   ThomasVDD");
  delay(2500);

  /* setup 16x2 lcd display:
     ------------------
     |SET 0.00V 000mA |
     |OUT 0.00V 000mA |
     ------------------
      0123456789111111
                012345
  */
  lcd.setCursor(0, 0);
  lcd.print("SET             ");
  lcd.setCursor(0, 1);
  lcd.print("OUT             ");
  lcd.setCursor(8, 0);
  lcd.print("V");
  lcd.setCursor(8, 1);  
  lcd.print("V");
  lcd.setCursor(13, 0);
  lcd.print("mA");
  lcd.setCursor(13, 1);
  lcd.print("mA");
  lcd.write(byte(0));
}

/* ============================================== LOOP =====================================================*/
/* 1) read serial data from java
 * 2) poll buttons 
 * 3) measure voltage
 * 4) measure current
 * 5) measure current with ina219
 * 6) send serial data to java
 * 7) get battery charge 
 * 8) update screen
 */

void loop() {
  /*read serial input: voltage 0-999, current 1000-1999*/
  while (Serial.available()) {
    char c = Serial.read();         //gets one byte from serial buffer
    readString += c;                //makes the string readString
    delay(2);                       //slow looping to allow buffer to fill with next character
  }
  if (readString.length() > 0) {
    int n = readString.toInt();     //convert readString into a number
    if (n >= 0 && n < 1000) {       //voltage setting
      pos1 = n;
      encoder1Pos = map(pos1, 0, 999, 0, 50000);
      if (outputOnOff){
        analogWrite16(9, pos1);
      }
    }
    else if (n >= 1000 && n <= 1999) { //current setting
      pos0 = n - 1000;
      encoder0Pos = map(pos0, 0, 999, 0, 50000);
      if (outputOnOff){
        analogWrite16(10, pos0);
      }
    }
    else if (n==9001){                // power on/off
      outputOnOff = !outputOnOff;
      if (outputOnOff){
        analogWrite16(9, pos1);
        analogWrite16(10, pos0);
      }
      else{
        analogWrite16(9, 0);
        analogWrite16(10, 0);
      }
    }
    readString = "";                //empty for next input
  }

  /* check for presets & save*/
  if (digitalRead(presetButton) == LOW) {
    delay(1000);
    //Save (long press)
    if (digitalRead(presetButton) == LOW) { 
      /*write new values to memory*/
      EEPROMWriteInt(0, encoder0Pos);
      presetA[0] = map(encoder0Pos, 0, 50000, 0, 999);
      EEPROMWriteInt(2, encoder1Pos);
      presetV[0] = map(encoder1Pos, 0, 50000, 0, 999);
      //EEPROMWriteInt(4, preset);
      lcd.setCursor(0, 0);
      lcd.print("MEM");
      delay(2500);
      lcd.setCursor(0, 0);
      lcd.print("SET ");
    }
    //Presets (short press)
    else {                                   
      preset++;
      if (preset > numberOfPresets - 1) {      //reset to preset 0
        preset = 0;
      }
      pos1 = presetV[preset];
      encoder1Pos = map(pos1, 0, 999, 0, 50000);
      if (outputOnOff){
        analogWrite16(9, pos1);
      }
      else{
        analogWrite16(9, 0);
      }
      pos0 = presetA[preset];
      encoder0Pos = map(pos0, 0, 999, 0, 50000);
      if (outputOnOff){
        analogWrite16(10, pos0);
      }
      else{
        analogWrite16(10, 0);
      }
    }
  }

  /* check for on/off and calibration*/ 
  if (digitalRead(zeroButton) == LOW){
    delay(1000);
    //CALIBRATE (long press)
    if (digitalRead(zeroButton) == LOW){
      calibrate = !calibrate;
      calibration[0] = finalA;
      lcd.setCursor(10, 1);
      lcd.print(message[calibrate]);
      delay(2500);
      lcd.setCursor(13, 1);
      lcd.print("mA");
    }
    //ON/OFF (short press)
    else{
      outputOnOff = !outputOnOff;
      if (outputOnOff){
        analogWrite16(9, pos1);
        analogWrite16(10, pos0);
      }
      else{
        analogWrite16(9, 0);
        analogWrite16(10, 0);
      }    
    }
  }

  /*Measure the voltage & average for lower noise*/
  totalV = totalV - readingsV[readIndexV];      //subtract the last reading
  readingsV[readIndexV] = analogRead(VoutPin);  //read from the sensor
  totalV = totalV + readingsV[readIndexV];      //add the reading to the total
  readIndexV = readIndexV + 1;                  //advance to the next position in the array
  if (readIndexV >= numReadingsV) {             //if we're at the end of the array...
    readIndexV = 0;                             //...wrap around to the beginning
  }
  averageV = totalV / numReadingsV;             //calculate the average
  realAverageV = averageV * 2.048 * 5 / 1023;     //map 1-2.048V range to corresponding value

  /*Measure the current & average for lower noise*/
  totalA = totalA - readingsA[readIndexA];
  readingsA[readIndexA] = analogRead(IoutPin);
  totalA = totalA + readingsA[readIndexA];
  readIndexA = readIndexA + 1;
  if (readIndexA >= numReadingsA) {
    readIndexA = 0;
  }
  averageA = totalA / numReadingsA;
  realAverageA = (averageA * 2.048 * 0.5 / 1.023);

  /*Measure the current with the INA219*/
  if (realAverageA < 300){                      // current < 320 mA can be measured with INA219
    if (realAverageA < 20 && currentRange != 40){
      currentRange = 40;
      ina219.setCalibration_40();
    }
    else if (realAverageA >= 20 && realAverageA < 50 && currentRange != 80){
      currentRange = 80;
      ina219.setCalibration_80();
    }
    else if (realAverageA >= 50 && realAverageA < 150 && currentRange != 160){
      currentRange = 160;
      ina219.setCalibration_160();
    }
    else if (realAverageA >= 150 && realAverageA < 320 && currentRange != 320){
      currentRange = 320;
      ina219.setCalibration_320();
    }    
    finalA = ina219.getCurrent_mA() - calibration[calibrate];
  }
  else{
    finalA = realAverageA - calibration[calibrate];
  }

  /* Send data to JAVA as: Voltage>Current>*/
  Serial.print(realAverageV*2);                   //send measured voltage
  Serial.print(">");
  if (finalA < 0) {                             // current lower than calibrate current
    finalA = 0;
  }
  Serial.print(finalA,2);                       //send measured current
  Serial.print(">");
  if (realAverageA > pos0 || !outputOnOff){     //send current limiting
    Serial.print("L");
  }

  /*  set the boost converter for the desired output*/
  boostVoltage = (pos1+125)*0.02; //add 125 to take the extra 2.5V into account
  //boost converter off for voltages < 5V
  if (pos1<375){
    boostConverter = 0;
  }
  else{
    boostConverter = 260 - ((36955 - 1364*boostVoltage)/(2.2*boostVoltage - 15.25))*0.0255;
    if (boostConverter > 255){
      boostConverter = 255;
    }
  }

  SPI.setClockDivider(SPI_CLOCK_DIV8);          //ensure correct clockdivider (lcd library also uses a clockdivider)
  digitalWrite(potentiometerPin, LOW);          //pull pin low to connect
  SPI.transfer(B00010011);                      //let MCP41010 know we want to set the potentiometer
  SPI.transfer(boostConverter);                 //send actual value
  digitalWrite(potentiometerPin, HIGH);         //pull pin high to disconnect

  /*write to the LCD*/ 
  lcd.setCursor(4, 0);
  if (pos1/50 < 9.994){       // <10 doesn't work because 9.995 < 10, but lcd.print(9.995,2) gives 10,00
    lcd.print(pos1 / 50 , 2);
  }
  else{
    lcd.print(pos1 / 50 , 1);
  }
  
  lcd.setCursor(4, 1);
  realAverageV *= 2; //map to 0-20V range
  if (realAverageV < 9.994){       // <10 doesn't work because 9.995 < 10, but lcd.print(9.995,2) gives 10,00
    lcd.print(realAverageV, 2);
  }
  else{
    lcd.print(realAverageV,1);
  }
  
  if (pos0 < 10){
    lcd.setCursor(11, 0);
    lcd.print("  ");      
  }
  else if (pos0 < 100){
    lcd.setCursor(12, 0);
    lcd.print(" ");  
  }
  if (finalA < 10){
    lcd.setCursor(11, 1);
    lcd.print("  ");      
  }
  else if (finalA < 100){
    lcd.setCursor(12, 1);
    lcd.print(" ");  
  }
  
  lcd.setCursor(10, 0);
  lcd.print(pos0, 0);
  lcd.setCursor(10, 1);
  lcd.print(finalA, 0);

  SOC();  // get the State Of Charge of the battery
  lcd.setCursor(15, 0);
  lcd.write(byte(batteryLine0[batteryLevel]));
  lcd.setCursor(15, 1);
  lcd.write(byte(batteryLine1[batteryLevel]));

  lcd.setCursor(0, 1);
  if (outputOnOff){
    lcd.print("OUT");
  }
  else{
    lcd.print("OFF");
  }

  delay(10);
}

/* ============================================== Functions =====================================================*/

/*interrupts*/
void doEncoder0() { //Current
  if (digitalRead(encoder0PinA) == digitalRead(encoderPinB)) {
    encoder0Pos -= amount0;
  } else {
    encoder0Pos += amount0;
  }

  if (encoder0Pos >= 60000) {   //unsigned int ==> negative numbers start from 65536
    encoder0Pos = 0;
  }
  if (encoder0Pos >= 50000) {
    encoder0Pos = 50000;
  }
  //routine to increase counter speed when rotary encoder is turned consequently
  if (millis() - prevTime0 >= 1000) { //reset ticks and amount when encoder is stationary for more than 1 second
    amount0 = 10;
    ticks0 = 0;
  }
  if (ticks0 >= 100 && millis() - prevTime0 <= 500) { //increase amount after 100 turns
    amount0 += 20;
    ticks0 = 0;
  }

  prevTime0 = millis();
  ticks0 ++;

  pos0 = map(encoder0Pos, 0, 50000, 0, 999); //map to 0 - 5 V ==> 1mA / step
  if (outputOnOff){
     analogWrite16(10, pos0);
  }
  else{
     analogWrite16(10, 0);
  }
}

void doEncoder1() { //Voltage
  if (digitalRead(encoder1PinA) == digitalRead(encoderPinB)) {
    encoder1Pos -= amount1;
  } else {
    encoder1Pos += amount1;
  }

  if (encoder1Pos >= 60000) {
    encoder1Pos = 0;
  }
  if (encoder1Pos >= 50000) {
    encoder1Pos = 50000;
  }
  if (millis() - prevTime1 >= 1000) {
    amount1 = 10;
    ticks1 = 0;
  }

  if (ticks1 >= 100 && millis() - prevTime1 <= 500) {
    amount1 += 20;
    ticks1 = 0;
  }

  prevTime1 = millis();
  ticks1 ++;

  pos1 = map(encoder1Pos, 0, 50000, 0, 999); //map to 0 - 5 V ==> 10mV / step

  if (outputOnOff){
     analogWrite16(9, pos1);
  }
  else{
     analogWrite16(9, 0);
  }
}

/* Configure digital pins 9 and 10 as 16-bit PWM outputs. */
void setupPWM16(float error) {
  DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
           | _BV(WGM11);              /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
           | _BV(CS10);               /* no prescaling */
  ICR1 = 0x3E8 * error;                       /* TOP counter value = 1000; +/- 10 bit resolution @ 15 kHz*/
}

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */
void analogWrite16(uint8_t pin, uint16_t val)
{
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}

/* EEPROM write and read functions*/
//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

/* battery measuring code
   battery voltage: 3V = empty, 4.2V = full ; 5 steps ==> 0.3V / step
   5 steps: 3V, 3.3V, 3.6V, 3.9V, 4.2V
*/
void SOC()
{
  totalB = totalB - readingsB[readIndexB];      //subtract the last reading
  readingsB[readIndexB] = analogRead(batteryPin);//read from the sensor
  totalB = totalB + readingsB[readIndexB];      //add the reading to the total
  readIndexB = readIndexB + 1;                  //advance to the next position in the array
  if (readIndexB >= numReadingsB) {             //if we're at the end of the array...
    readIndexB = 0;                             //...wrap around to the beginning
  }
  averageB = totalB / numReadingsB;             //calculate the average
  batteryVoltage = averageB * 2.048 * 4.09 / 1024;   //map 1-2.048V range to corresponding value: (2.2k + 8.8k)/2.2k = 4.09
  if (batteryVoltage >= 8.2) {
    batteryLevel = 4;
  }
  else if (batteryVoltage >= 7.6) {
    batteryLevel = 3;
  }
  else if (batteryVoltage >= 7.2) {
    batteryLevel = 2;
  }
  else if (batteryVoltage >= 6.6) {
    batteryLevel = 1;
  }
  else {
    batteryLevel = 0;
  }
}

/* ============================================== REFERENCES =====================================================*/
/* References
   Rotary encoder: http://playground.arduino.cc/Main/RotaryEncoders#Example14
   16 bit PWM: http://arduino.stackexchange.com/questions/12718/increase-pwm-bit-resolution
   Smoothing: https://www.arduino.cc/en/Tutorial/Smoothing
   LCD 74HC595: http://playground.arduino.cc/Main/LiquidCrystal
   Serial read: http://forum.arduino.cc/index.php?topic=80254.0
   EEPROM: http://forum.arduino.cc/index.php?topic=37470.0
   INA219: ADAFRUIT library
*/
