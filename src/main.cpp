#include <Arduino.h>
#include <ESP32Encoder.h>
#include "rgb_lcd.h"
#include "Adafruit_TCS34725.h"
#include <SPI.h>
#include <Wire.h>

#define CLK 23 // CLK ENCODER 
#define DT 19 // DT ENCODER 

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
ESP32Encoder encoder;
rgb_lcd lcd;


int i = 0, e;
int BP1 = 0, BP2 = 2, BP3 = 12;
int val1=0, val2=0, val3=0;
int pot= 33, lecture_pot;

int phase = 26;
int encodeurA= 23, encodeurB= 19;

int vitesse, moteur = 27, pwmChannel = 0;
int frequence = 25000;
int resolution = 11;



void setup() {

  // Initialise la liaison avec le terminal
  Serial.begin(9600);
  pinMode(BP1,INPUT_PULLUP);
  pinMode(BP2,INPUT_PULLUP);
  pinMode(BP3,INPUT_PULLUP);
  pinMode(moteur, OUTPUT);
  pinMode(phase, OUTPUT);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);

  // Init moteur
  ledcSetup(pwmChannel, frequence, resolution);
  ledcAttachPin(moteur, pwmChannel);

  // Init encoder
  encoder.attachHalfQuad ( DT, CLK );
  encoder.setCount ( 0 );

  //Init capteur couleur
   if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}

void loop() 
{
  // Utilisation boutton + pot
val1 = digitalRead(BP1);
val2 = digitalRead(BP2);
val3 = digitalRead(BP3);
lecture_pot=analogRead(pot);

// Affichage
//Serial.printf("bp1 %d    bp2 %d     bp3 %d     pot %d\n",val1, val2, val3, lecture_pot);
lcd.setCursor(0,0); 
lcd.printf("pot %d           ",lecture_pot);
lcd.setCursor(0,1); 
lcd.printf("coder %d        ",e);

// controle moteur
if (val1 == LOW)
  {
    digitalWrite(phase, HIGH);
  }
else
  {
    digitalWrite(phase, LOW);
  } 
vitesse =  analogRead(pot)/2;
ledcWrite(pwmChannel, vitesse);

// encodeur
long newPosition = encoder.getCount();
e = newPosition;

// capteur couleur
uint16_t r, g, b, c, colorTemp, lux;
tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

}