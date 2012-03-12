// version dated 3/12/2012

// variable prefixes
// c: constant
// g: global
// p: parameter
// v: local variable

//include EEPROM library
#include <EEPROM.h>
//include libaries for the RTC to talk over i2c 
#include <Wire.h>
#include "RTClib.h"
//include the SD library
#include <SD.h>
//include the LCD library
#include <LiquidCrystal.h>
//include the TimedAction library for event scheduling
#include <TimedAction.h>

// initialize the Real Time Clock object
RTC_DS1307 RTC; 
// initialize the logging file object
File logfile;
// initialize the lcd object with the numbers of the interface pins
LiquidCrystal lcd(8, 7, 5, 4, A3, 2);
// initilize the pressure control schedule object
TimedAction pressureControlEvent = TimedAction(20,pressureControl); 
// initilize the fast display schedule object, SD buffer and Serial terminal
TimedAction fastDisplayEvent = TimedAction(50,fastDisplay); 
// initilize the slow display schedule object, LCD and SD flush
TimedAction slowDisplayEvent = TimedAction(200,slowDisplay); 

// Definitions
//data logger compile time defines
#define ECHO_TO_SERIAL   0 // echo data to serial port
#define WAIT_TO_START    1 // Wait for serial input in setup()

// constant declaration, including pin assignments
const int cBackLightPin = 6;  // 2n2222a transistor driving LCD backlight LED
const int cPressurePin = A0; // Pressure sensor input pin. 1-5v 0-250psi
const int cPumpPin = 9; //connected 212 Fc 2 pole RC low pass into snow stgII controller
const int cRelayPin = 3; //connected to logic level FET, weak pulldown, fires relay/solenoid
const int cUserPin = A1; //connected to 0-2k pot, currently this sets the open loop duty
const int cVoltageDividerPin = A2; //connected to voltage divider for 12v raw
const int cMaxPressure = 120; //maximum system pressure limit in PSI
const int cLimitGain = 15; //gain for max pressure limiting
const int chipSelect = 10;//set the hardware chip select pin for the adafruit dataloggershield
//must be left as an output or the SD library
//functions will not work.

// global variable declaration
float gSysVoltage = readVoltage(); //float value of system voltage
int gPumpDuty=0; //actual 0-100% pump duty cycle output
int gPressurePSI = readPressure(); //system pressure in PSI
int gDesiredDutyCycle = 0; // desired duty cycle
int gDutyCycleBit = 0; //duty cycle bits after linearization 0-255
float gOutVoltage = 0;  //pump controller average analog voltage for display only

//linearizes the snow stage II pump controller
//pass a desiredDuty to dutyCycle function and it
//returns the 8bit analogValue value suitable for analogWrite function
//requires EEPROM sketch to populate addresses 15-70
int dutyCycle(int pDesiredDuty)
{
  int vAnalogValue = 0;
  if(pDesiredDuty < 15){vAnalogValue = 0;}
  else if(pDesiredDuty > 70){vAnalogValue = 255;}
  else
  vAnalogValue = EEPROM.read(pDesiredDuty);
  return vAnalogValue;
}

//calculate actual output duty cycle for display, datalogging
int pumpDC(int pDesiredDuty)
{
  int vDisplayedDutyCycle;
  if(pDesiredDuty < 15){vDisplayedDutyCycle = 0;}
  else if(pDesiredDuty > 70){vDisplayedDutyCycle = 100;}
  else vDisplayedDutyCycle = pDesiredDuty;
  return vDisplayedDutyCycle;
}

//read the pressure sensor
int readPressure()
{
  int vPressureSensorBit;
  int vPressureShiftedBit;
  int vPressurePSI;
  //read pressure sensor
  vPressureSensorBit = analogRead(cPressurePin);
  //offest 1 volt for 1-4v range
  vPressureShiftedBit = vPressureSensorBit - 200;
  //convert shifted bit to pressure in PSI
  vPressurePSI=vPressureShiftedBit*0.31313; //62.5psi per volt,* .0049 volt per bit *(5/4.89) verf
  //make sure pressure doesn't jitter below 0
  if(vPressurePSI <= 0){vPressurePSI=0;};
  return vPressurePSI;
}

//read the system voltage
float readVoltage()
{
 float vSysVoltage;
 //measure system voltage
 vSysVoltage= analogRead(cVoltageDividerPin)*0.0238;
 return vSysVoltage;
}

//error reporting function- can be re-used 
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  while(1); //halts execution on errors
}

//Pressure Controller Frequency 100hz
//read the sensor
//convert the sensor bit to PSI
//ensure gPressurePSI is not less than zero for jitter
//read the user input (disired duty or desired pressure)
//transform ADC 0-1023 to 0-100 duty cycle int
//check the max pressure isn't exceeded
//change the desired duty cycle with kP loop
//linearize the snow stageII controll with EEPROM lookup table
//write the PWM output register
void pressureControl()
{
  //read the pressure sensor
  gPressurePSI=readPressure();
  //measure the user input desired duty cycle
  gDesiredDutyCycle= map(analogRead(cUserPin),0,1023,0,100);
  //check that pump pressure max isn't exceeded
  if(gPressurePSI > cMaxPressure)
    {
      gDesiredDutyCycle=gDesiredDutyCycle-cLimitGain*(gPressurePSI-cMaxPressure);
      if(gDesiredDutyCycle<15){gDesiredDutyCycle=15;}
    }
  //linearize snow stage II pump controller with dutyCycle function.
  gDutyCycleBit = dutyCycle(gDesiredDutyCycle); 
  //output pump bits to pump controller
  analogWrite(cPumpPin,gDutyCycleBit);
}

//Fastest readout frequency (lcd, serial, SD) 10hz
//read the system voltage
//convert to float
//convert the output PWM to average analog voltage (float)
//calculate the pump controller (pump motor) duty cycle
//get the DateTime from the RTC
//print stuff to the sd card buffer and serial terminal if echo serial is 1
void fastDisplay()
{
  //measure system voltage
  gSysVoltage = readVoltage();  
  //convert to output to volts 
  gOutVoltage = gDutyCycleBit*0.0192;
  //calculate pump duty
  gPumpDuty = pumpDC(gDesiredDutyCycle);
  //datalogger section
  DateTime now;
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
  #if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");  
  #endif
 
 // fetch the time
  now = RTC.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
  #if ECHO_TO_SERIAL
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
  #endif //ECHO_TO_SERIAL

  //log the data
  logfile.print(", ");    
  logfile.print(gSysVoltage);
  logfile.print(", ");    
  logfile.print(gPumpDuty);
  logfile.print(", ");    
  logfile.print(gPressurePSI);
  logfile.println();
  #if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(gSysVoltage);
  Serial.print(", ");    
  Serial.print(gPumpDuty);
  Serial.print(", ");    
  Serial.print(gPressurePSI);
  Serial.println();
  #endif //ECHO_TO_SERIAL
  
}

//LCD frequency 5hz
//move the cursors and print display variables
//(labels are from setup)
//flush SD card related ram locations to card
void slowDisplay()
{
  // print the pressure
  lcd.setCursor(6,3);
  lcd.print(gPressurePSI);
  lcd.print("  ");
  // print the system voltage
  lcd.setCursor(6,0);
  lcd.print(gSysVoltage);
  lcd.print("  ");
  // print the average analog output voltage to the controller
  lcd.setCursor(6,2);
  lcd.print(gOutVoltage); 
  lcd.print("  ");
  // print the duty cycle outputted to the pump controller
  lcd.setCursor(6,1);
  lcd.print(gPumpDuty);
  lcd.print("  ");
  
  // syncing data to the card & updating FAT!
  logfile.flush();
}

//This code runs once at startup
void setup() 
{
  pinMode(A3, OUTPUT); // set A3 as an output for LCD wiring
  pinMode(3, OUTPUT); // set 3 as an output for relay/solenoid transistor
  pinMode(10, OUTPUT); // chip select for SD card, required for library to function
  
  // open relay, close the solenoid
  // ensures the transistor does not see intermediate voltage
  digitalWrite(cRelayPin, LOW);
   // set up the LCD's number of columns and rows: 
  lcd.begin(20, 4);
  // turn on the backlight and set brightness
  analogWrite(cBackLightPin,125);
  lcd.clear(); //clear the screen
  lcd.setCursor(0,4);
  lcd.print("  PSI"); // Print pressure PSI label to the LCD
  lcd.setCursor(0,0);
  lcd.print(" Vsys"); // Print the system volts label to the LCD
  lcd.setCursor(0,2);
  lcd.print(" Vout"); // Print output volts label to the LCD
  lcd.setCursor(0,1);
  lcd.print("DCout");   // Print duty cycle label to the LCD
  
  //initialize serial terminal at 9600 baud
  Serial.begin(9600);
  Serial.println();
  
  //wait for the user if WAIT_TO_STATRT define is set
  #if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
  #endif //WAIT_TO_START
  
  // print initialize the SD card to the serial terminal
  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  //throw error if logger can't create file
  if (! logfile) {
    error("couldnt create file");
  }
  //report file being logged to
  Serial.print("Logging to: ");
  Serial.println(filename);
   
  //connect to the RTC
  Wire.begin();  
  if (!RTC.begin())
  {
  logfile.println("RTC failed");
  #if ECHO_TO_SERIAL
  Serial.println("RTC failed");
  #endif  //ECHO_TO_SERIAL
  }
  
  //print the log header to the SD file and serial
  logfile.println("millis,timestamp,time,voltage,duty cycle,pressure");    
  #if ECHO_TO_SERIAL
  Serial.println("millis,timestampe,time,voltage,duty cycle,pressure");
  #endif
   
  //energize relay, open solenoid
  digitalWrite(cRelayPin, HIGH);
}

void loop()
{
  pressureControlEvent.check();
  fastDisplayEvent.check();
  slowDisplayEvent.check();
}



