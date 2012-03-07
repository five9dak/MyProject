//This sketch readouts the EEPROM Conents for verification

//include EEPROM library
#include <EEPROM.h>

// include the LCD library:
#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 7, 5, 4, A3, 2);
// LCD LED backlight driver circuit connected to digital pin 6
int backLight = 6;  

//run once
void setup() 
{
  lcd.begin(20, 4);
  // turn on the backlight and set brightness
  analogWrite(backLight,125);
  // Print address label
  lcd.setCursor(0,0);
  lcd.print("EEPROM Address");
   // Print Entry label
  lcd.setCursor(0,1);
  lcd.print("         Entry");  
}

//loop
void loop()
{
   for (int i=0; i <= 70; i++){
       //print the address
       lcd.setCursor(15,0);
       lcd.print(i);
       lcd.print("  ");
       // print the pressure
       lcd.setCursor(15,1);
       lcd.print(EEPROM.read(i));
       lcd.print("  ");
      delay(1000);
   }


}
  
