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
// define the Real Time Clock object
RTC_DS1307 RTC; 

//include the SD library
#include <SD.h>
//set the hardware chip select pin for the adafruit dataloggershield
//must be left as an output or the SD library
//functions will not work.
const int chipSelect = 10;
// the logging file
File logfile;

//include the LCD library
#include <LiquidCrystal.h>
// initialize the lcd with the numbers of the interface pins
LiquidCrystal lcd(8, 7, 5, 4, A3, 2);

//data logger defines
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    1 // Wait for serial input in setup()
// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

// other pin definitions
const int cBackLightPin = 6;  // 2n2222a transistor driving LCD backlight LED
const int cPressurePin = A0; // Pressure sensor input pin. 1-5v 0-250psi
const int cPumpPin = 9; //connected 212 Fc 2 pole RC low pass into snow stgII controller
const int cRelayPin = 3; //connected to logic level FET, weak pulldown, fires relay/solenoid
const int cUserPin = A1; //connected to 0-2k pot, currently this sets the open loop duty
const int cVoltageDividerPin = A2; //connected to voltage divider for 12v raw
const int cMaxPressure = 120; //maximum system pressure limit in PSI
const int cLimitGain = 15; //gain for max pressure limiting

// variable definitions
float gSysVoltage = 0; //float value of system voltage
int gPumpDuty; //actual 0-100% pump duty cycle output
int gPressurePSI = 0; //system pressure in PSI
int gPressureSensorBit = 0; //ADC bit from pressure sensor, 0-1023
int gPressureShiftedBit = 0; //ADC bit after subtracting 1v offset
int gDesiredDutyCycle = 0; // desired duty cycle
int gDutyCycleBit = 0; //duty cycle bits after linearization 0-255
float gOutVoltage = 0;  //pump controller average analog voltage for display only


//linearizes the snow stage II pump controller
//pass a desiredDuty to dutyCycle function and it
//returns the 8bit analogValue value suitable for analogWrite function
//requires EEPROM sketch to populate addresses 15-70
int dutyCycle(int desiredDuty)
{
  int analogValue = 0;
  if(desiredDuty < 15){analogValue = 0;}
  else if(desiredDuty > 70){analogValue = 255;}
  else
  analogValue = EEPROM.read(desiredDuty);
  return analogValue;
}

//calculate actual output duty cycle for display
int pumpDC(int desiredDuty2)
{
  int value;
  if(desiredDuty2 < 15){value = 0;}
  else if(desiredDuty2 > 70){value = 100;}
  else value = desiredDuty2;
  return value;
}

//error reporting function
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
    // red LED indicates error
  //digitalWrite(redLEDpin, HIGH);
  while(1);
}

//run once
void setup() 
{
  // set A3 as an output for LCD wiring
  pinMode(A3, OUTPUT);
  // set 3 as an output for relay/solenoid transistor
  pinMode(3, OUTPUT);
  // open relay, close the solenoid
  // ensures the transistor does not see interediate voltage
  digitalWrite(cRelayPin, LOW);
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(20, 4);
  // turn on the backlight and set brightness
  analogWrite(cBackLightPin,125);
  //clear the screen
  lcd.clear();
  // Print pressure PSI label to the LCD
  lcd.setCursor(0,4);
  lcd.print("  PSI");   
  // Print the system volts label to the LCD
  lcd.setCursor(0,0);
  lcd.print(" Vsys"); 
  // Print output volts label to the LCD
  lcd.setCursor(0,2);
  lcd.print(" Vout"); 
  // Print duty cycle label to the LCD
  lcd.setCursor(0,1);
  lcd.print("DCout");   
  
  //initialize serial terminal at 9600 baud
  Serial.begin(9600);
  Serial.println();
  
  //wait for the user if WAIT_TO_STATRT define is set
  #if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
  #endif //WAIT_TO_START
  
  // print initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
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

//loop
void loop()
{
  //read pressure sensor
  gPressureSensorBit = analogRead(cPressurePin);
  //offest 1 volt for 1-4v range
  gPressureShiftedBit = gPressureSensorBit - 200;
  //convert shifted bit to pressure in PSI
  gPressurePSI=gPressureShiftedBit*0.31313; //62.5psi per volt,* .0049 volt per bit *(5/4.89) verf
  //make sure pressure doesn't jitter below 0
  if(gPressurePSI <= 0){gPressurePSI=0;};
  //measure system voltage
  gSysVoltage= analogRead(cVoltageDividerPin)*0.0238;
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
  //convert to output to volts 
  gOutVoltage = gDutyCycleBit*0.0192;
  //calculate pump duty
  gPumpDuty = pumpDC(gDesiredDutyCycle);
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
  
  //datalogger section
  DateTime now;
  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
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
  
   // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // syncing data to the card & updating FAT!
  logfile.flush();

  
}
