//include EEPROM library
#include <EEPROM.h>
//include the LCD library
#include <LiquidCrystal.h>
// initialize the lcd with the numbers of the interface pins
LiquidCrystal lcd(8, 7, 5, 4, A3, 2);
// other pin definitions
int backLightpin = 6;  // 2n2222a transistor driving backlight LED
int pressurePin = A0; // Pressure sensor input pin. 1-5v 0-250psi
int pumpPin = 9; //connected 212 Fc 2 pole RC low pass, snow stgII controller
int relayPin = 3; //connected to logic level FET, weak pulldown
int userPin = A1; //connected to 0-2k pot
int sysVoltpin = A2; //connected to voltage divider for 12v raw
float sysVolt = 0; //float value of system voltage
int pumpDuty; //actual 0-100% pump duty cycle output
int pressurePSI = 0; //system pressure in PSI
int pressureBit = 0; //used in pressure scaling
int pressureShiftedBit = 0; //used in pressure scaling
int desiredDC = 0; // desired duty cycle
int dcBits = 0; //dc bits after linearization
float outVolts = 0;  //pump controll voltage for display
int maxPressure = 120; //psi
int limitGain = 15; //gain for max pressure limiting


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


//run once
void setup() 
{
  // set A3 as an output
  pinMode(A3, OUTPUT);
  // set 3 as an output
  pinMode(3, OUTPUT);
  // set relay OFF
  digitalWrite(relayPin, LOW);
  // set up the LCD's number of columns and rows: 
  lcd.begin(20, 4);
  // turn on the backlight and set brightness
  analogWrite(backLightpin,125);
  lcd.clear();
  // Print pressure PSI label
  lcd.setCursor(0,4);
  lcd.print("  PSI");   
  // Print the system volts label
  lcd.setCursor(0,0);
  lcd.print(" Vsys"); 
  // Print output volts label
  lcd.setCursor(0,2);
  lcd.print(" Vout"); 
  // Print duty cycle label
  lcd.setCursor(0,1);
  lcd.print("DCout");   
  // Print output label
  //lcd.setCursor(0,2);
  //lcd.print("     Out BIT"); 
  // Print a title to the LCD's fourth row.
  //lcd.setCursor(0,3);
  //lcd.print("ORLANDO ENTERPRISES");
  // Print pressure bit label
  //lcd.setCursor(0,0);
  //lcd.print("PRESSURE BIT");
  
  //open solenoid
  digitalWrite(relayPin, HIGH);
}

//loop
void loop()
{
  //read pressure sensor
  pressureBit = analogRead(pressurePin);
  //offest 1 volt for 1-4v range
  pressureShiftedBit = pressureBit - 200;
  //convert shifted bit to pressure in PSI
  pressurePSI=pressureShiftedBit*0.31313; //62.5psi per volt,* .0049 volt per bit *(5/4.89) verf
  //make sure pressure doesn't jitter below 0
  if(pressurePSI <= 0){pressurePSI=0;};
  //measure system voltage
  sysVolt= analogRead(sysVoltpin)*0.0238;
  //measure the user input desired duty cycle
  desiredDC= map(analogRead(userPin),0,1023,0,100);
  //check that pump pressure max isn't exceeded
  if(pressurePSI > maxPressure)
    {
      desiredDC=desiredDC-limitGain*(pressurePSI-maxPressure);
      if(desiredDC<15){desiredDC=15;}
    }
  //linearize snow stage II pump controller with dutyCycle function.
  dcBits = dutyCycle(desiredDC); 
  //output pump bits to pump controller
  analogWrite(pumpPin,dcBits); 
  //convert to output to volts 
  outVolts = dcBits*0.0192;
  //calculate pump duty
  pumpDuty = pumpDC(desiredDC);
  
  // print the pressure
  lcd.setCursor(6,3);
  lcd.print(pressurePSI);
  lcd.print("  ");
  // print the sysVolts
  lcd.setCursor(6,0);
  lcd.print(sysVolt);
  lcd.print("  ");
  // print the outVolts
  lcd.setCursor(6,2);
  lcd.print(outVolts); 
  lcd.print("  ");
  // print the pumpDuty
  lcd.setCursor(6,1);
  lcd.print(pumpDuty);
  lcd.print("  ");
  
  //set pump controller voltage
  //outBits= map(analogRead(userPin),0,1023,0,255);
  //analogWrite(pumpPin, outBits);
  //outVolts = outBits*0.0192;
  
  //print the pressure bit
  //lcd.setCursor(13,0);
  //lcd.print(pressureBit);
  //lcd.print("  ");
  // print the outBits
  //lcd.setCursor(13,2);
  //lcd.print(outBits);
  //lcd.print("  ");
  
  //turn relay ON
  //digitalWrite(relayPin, HIGH);
  //delay two seconds
  //delay(2000);
  //turn realy OFF
  //digitalWrite(relayPin, LOW);
  //delay 2 seconds
  //delay(2000);
}
