// version dated 3/23/2012

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
//include the TimedAction library for event scheduling
#include <TimedAction.h>

// constant declaration, including pin assignments
const int cPressurePin = A0; // Pressure sensor input pin. 1-5v 0-250psi
const int cPumpPin = 9; //connected 212 Fc 2 pole RC low pass into snow stgII controller
const int cRelayPin = 3; //connected to logic level FET, weak pulldown, fires relay/solenoid
const int cVoltageDividerPin = A2; //connected to voltage divider for 12v raw
const int chipSelect = 10;//set the hardware chip select pin for the adafruit dataloggershield
//must be left as an output or the SD library
//functions will not work.

// global variable declaration
double gDesiredDutyCycle = 20; // desired duty cycle
float gSysVoltage = readVoltage(); //float value of system voltage
int gPumpDuty=0; //actual 0-100% pump duty cycle output
double gPressurePSI = readPressure(); //system pressure in PSI
int gDutyCycleBit = 0; //duty cycle bits after linearization 0-255
float gOutVoltage = 0;  //pump controller average analog voltage for display only

// initialize the Real Time Clock object
RTC_DS1307 RTC; 
// initialize the logging file object
File logfile;
// initilize the fast display schedule object, SD buffer and Serial terminal
TimedAction fastDisplayEvent = TimedAction(5,fastDisplay); 
// initilize the slow display schedule object, LCD and SD flush
TimedAction slowDisplayEvent = TimedAction(100,slowDisplay); 

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

//Fastest frequency (SD) 10hz
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
  gPressurePSI = readPressure();
  //convert to output to volts 
  //utVoltage = gDutyCycleBit*0.0192;
  //calculate pump duty
  //gPumpDuty = pumpDC(gDesiredDutyCycle);
  //datalogger section
  //DateTime now;
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
 
  //log the data
  //logfile.print(", ");    
  logfile.print(gSysVoltage);
  logfile.print(", ");    
  //logfile.print(gPumpDuty);
  //logfile.print(", ");
  //logfile.print(gDesiredPressure);
  //logfile.print(", ");  
  logfile.print(gPressurePSI);
  logfile.println();
}

//flush SD card related ram locations to card
void slowDisplay()
{
  // syncing data to the card & updating FAT!
  logfile.flush();
}


//This code runs once at startup
void setup() 
{
  pinMode(3, OUTPUT); // set 3 as an output for relay/solenoid transistor
  pinMode(10, OUTPUT); // chip select for SD card, required for library to function
  
  // open relay, close the solenoid
  // ensures the transistor does not see intermediate voltage
  digitalWrite(cRelayPin, LOW);
 
 // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
     while(1); //halts execution on errors
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
   while(1); //halts execution on errors
  }

  //connect to the RTC
  Wire.begin();  
  if (!RTC.begin())
  {
  logfile.println("RTC failed");
  }
  
  //print the log header to the SD file and serial
  logfile.println("millis,voltage,pressure");    
     
  //energize relay, open solenoid
  digitalWrite(cRelayPin, HIGH);
  
  //set the pump duty cycle
  analogWrite(cPumpPin, dutyCycle(gDesiredDutyCycle));
}

void loop()
{
  fastDisplayEvent.check();
  slowDisplayEvent.check();
}



