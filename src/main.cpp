#include <Arduino.h>
#include <ESP32Encoder.h>
#include "rgb_lcd.h"
#include "Adafruit_TCS34725.h"
#include <SPI.h>
#include <Wire.h>

#define CLK 23 // CLK ENCODER
#define DT 19  // DT ENCODER

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
ESP32Encoder encoder;
rgb_lcd lcd;

int i = 0;
int v1, v2, vts;
int BP1 = 0, BP2 = 2, BP3 = 12;
int val1 = 0, val2 = 0, val3 = 0;
int pot = 33, lecture_pot;

int phase = 26;
int encodeurA = 23, encodeurB = 19, vitesse;

int moteur = 27, pwmChannel = 0;

int frequence = 25000;
int resolution = 11;
int consigne;

// asservissement
void vTaskPeriodic(void *pvParameters)
{
  //déclaration variables
  int oldPosition = 0;
  int erreur, Erreurtot;
  float k = 100, Ki= 50;
  TickType_t xLastWakeTime;
  
  // Lecture du nombre de ticks quand la tâche commence
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    // calcul vitesse rotation
    int newPosition = encoder.getCount();
    vts = oldPosition - newPosition;
    oldPosition = newPosition;

    //calcul de l'erreur+ erreur intégrale
    Erreurtot = Erreurtot + erreur;
    erreur = consigne - vts;
    vitesse = k * erreur + Ki * Erreurtot;

    //bride de l'erreur intégrale
    if(Erreurtot>30)
      Erreurtot=Erreurtot-10;

    //si vitesse augmente
    if (vitesse > 0)
    {
      if (vitesse > 2047)
        vitesse = 2047;
      digitalWrite(phase, 1);
      ledcWrite(pwmChannel, vitesse);
    }

    //si vitesse diminue
    if (vitesse < 0)
    {
      if (vitesse < -2047)
        vitesse = -2047;
      digitalWrite(phase, 0);
      ledcWrite(pwmChannel, -vitesse);
    }

    // Endort la tâche pendant le temps restant par rapport au réveil,
    // ici 100ms, donc la tâche s'effectue ici toutes les 100ms.
    // xLastWakeTime sera mis à jour avec le nombre de ticks au prochain
    // réveil de la tâche.
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}

void setup()
{
  // Initialise la liaison avec le terminal
  Serial.begin(115200);
  pinMode(BP1, INPUT_PULLUP);
  pinMode(BP2, INPUT_PULLUP);
  pinMode(BP3, INPUT_PULLUP);
  pinMode(moteur, OUTPUT);
  pinMode(phase, OUTPUT);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);

  // Init moteur
  ledcSetup(pwmChannel, frequence, resolution);
  ledcAttachPin(moteur, pwmChannel);

  // Init encoder
  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);

  // Init capteur couleur
  if (tcs.begin())
  {
    Serial.println("Found sensor");
  }
  else
  {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ;
  }

  // Init asservissement
  Serial.printf("Initialisations\n");

  // Création de la tâche périodique
  xTaskCreate(vTaskPeriodic, "vTaskPeriodic", 10000, NULL, 2, NULL);
}

void loop()
{
  // Utilisation boutton + pot
  val1 = digitalRead(BP1);
  val2 = digitalRead(BP2);
  val3 = digitalRead(BP3);
  consigne = analogRead(pot) / 150;

  lcd.setCursor(0, 0);
  lcd.printf("pot %d           ", consigne);

  // capteur couleur
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);

  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
  /*
    Serial.print("Color Temp: ");
    Serial.print(colorTemp, DEC);
    Serial.print(" K - ");
    Serial.print("Lux: ");
    Serial.print(lux, DEC);
    Serial.print(" - ");
    Serial.print("R: ");
    Serial.print(r, DEC);
    Serial.print(" ");
    Serial.print("G: ");
    Serial.print(g, DEC);
    Serial.print(" ");
    Serial.print("B: ");
    Serial.print(b, DEC);
    Serial.print(" ");
    Serial.print("C: ");
    Serial.print(c, DEC);
    Serial.print(" ");
    Serial.println(" ");
  */
 
}