//This sketch write EEPROM range 15-70 with a lookup table to linearize the snow
//stageII pump controller. 
//address is desired duty cycle, entry is 8 bit analogWrite value
#include <EEPROM.h>

void setup()
{
//Reserved for user preferences
EEPROM.write(0,0);
EEPROM.write(1,0);
EEPROM.write(2,0);
EEPROM.write(3,0);
EEPROM.write(4,0);
EEPROM.write(5,0);
EEPROM.write(6,0);
EEPROM.write(7,0);
EEPROM.write(8,0);
EEPROM.write(9,0);
EEPROM.write(10,0);
EEPROM.write(11,0);
EEPROM.write(12,0);
EEPROM.write(13,0);
EEPROM.write(14,0);
 
//snow stage II pump controller duty cycle linearization 
EEPROM.write(15,35);
EEPROM.write(16,38);
EEPROM.write(17,41);
EEPROM.write(18,44);
EEPROM.write(19,47);
EEPROM.write(20,50);
EEPROM.write(21,54);
EEPROM.write(22,57);
EEPROM.write(23,61);
EEPROM.write(24,64);
EEPROM.write(25,68);
EEPROM.write(26,72);
EEPROM.write(27,76);
EEPROM.write(28,80);
EEPROM.write(29,84);
EEPROM.write(30,88);
EEPROM.write(31,92);
EEPROM.write(32,96);
EEPROM.write(33,100);
EEPROM.write(34,104);
EEPROM.write(35,108);
EEPROM.write(36,112);
EEPROM.write(37,116);
EEPROM.write(38,120);
EEPROM.write(39,124);
EEPROM.write(40,128);
EEPROM.write(41,132);
EEPROM.write(42,135);
EEPROM.write(43,139);
EEPROM.write(44,142);
EEPROM.write(45,146);
EEPROM.write(46,149);
EEPROM.write(47,152);
EEPROM.write(48,156);
EEPROM.write(49,159);
EEPROM.write(50,162);
EEPROM.write(51,165);
EEPROM.write(52,167);
EEPROM.write(53,170);
EEPROM.write(54,173);
EEPROM.write(55,175);
EEPROM.write(56,177);
EEPROM.write(57,179);
EEPROM.write(58,182);
EEPROM.write(59,182);
EEPROM.write(60,184);
EEPROM.write(61,187);
EEPROM.write(62,189);
EEPROM.write(63,190);
EEPROM.write(64,192);
EEPROM.write(65,193);
EEPROM.write(66,194);
EEPROM.write(67,195);
EEPROM.write(68,196);
EEPROM.write(69,197);
EEPROM.write(70,198);
}

void loop()
{
  
}
