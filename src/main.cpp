#include <Arduino.h>
#include "rgb_lcd.h"

rgb_lcd lcd;
int i = 0;
int BP1 = 0, BP2 = 2, BP3 = 12;
int val1=0, val2=0, val3=0;
int pot= 33;
int lecture_pot;

void setup() {

  // Initialise la liaison avec le terminal
  Serial.begin(115200);
  pinMode(BP1,INPUT_PULLUP);
  pinMode(BP2,INPUT_PULLUP);
  pinMode(BP3,INPUT_PULLUP);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);

}

void loop() 
{
val1 = digitalRead(BP1);
val2 = digitalRead(BP2);
val3 = digitalRead(BP3);
lecture_pot=analogRead(pot);
Serial.printf("bp1 %d    bp2 %d     bp3 %d     pot %d\n",val1, val2, val3, lecture_pot);

lcd.setCursor(0,0); 
lcd.printf("pot %d           ",lecture_pot);
delay(1000);

}
