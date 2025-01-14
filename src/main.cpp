git config --global user.email#include <Arduino.h>
#include <ESP32Encoder.h>
#include "rgb_lcd.h"
#include "Adafruit_TCS34725.h"
#include <SPI.h>
#include <Wire.h>
#include <SPI.h>
#include "MFRC522_I2C.h"

#define CLK 23 // CLK ENCODER
#define DT 19  // DT ENCODER

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
ESP32Encoder encoder;
rgb_lcd lcd;

MFRC522 mfrc522(0x28);

int i = 0, tour = 0, bozpos = 0, csg = 0;
int v1, v2, vts;
int BP1 = 0, BP2 = 12, BP3 = 2;
int val1 = 0, val2 = 0, val3 = 0;
int pot = 33, lecture_pot, newpot = 0;
int vbp2;

int phase = 26;
int encodeurA = 23, encodeurB = 19, vitesse;

int moteur = 27, pwmChannel1 = 0, pwmChannel2 = 2;

int frequence = 25000, frequence2 = 50;
int resolution = 11, resolution2 = 16;
int consigne;
int servo = 13;
int posi = 0;
char etat = 0;

// position
void position(long pos)
{
  if ((bozpos < ((pos - 1) * 103 + 75)) && (bozpos > ((pos - 1) * 103 + 65)))
    csg = 0;
  else
  {
    if (bozpos < ((pos - 1) * 103 + 65))
      csg = 1;
    if (bozpos > ((pos - 1) * 103 + 75))
      csg = -1;
  }
}

// fonction asservissement
void vTaskPeriodic(void *pvParameters)
{
  // déclaration variables
  int oldPosition = 0;
  int erreur, Erreurtot;
  float k = 100, Ki = 50;
  TickType_t xLastWakeTime;

  // Lecture du nombre de ticks quand la tâche commence
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    // calcul vitesse rotation
    int newPosition = encoder.getCount();
    posi = encoder.getCount();
    vts = oldPosition - newPosition;
    oldPosition = newPosition;

    // calcul de l'erreur+ erreur intégrale
    Erreurtot = Erreurtot + erreur;
    erreur = consigne - vts;
    vitesse = k * erreur + Ki * Erreurtot;

    // bride de l'erreur intégrale
    if (Erreurtot > 30)
      Erreurtot = Erreurtot - 10;

    // si vitesse augmente
    if (vitesse > 0)
    {
      if (vitesse > 2047)
        vitesse = 2047;
      digitalWrite(phase, 1);
      ledcWrite(pwmChannel1, vitesse);
    }

    // si vitesse diminue
    if (vitesse < 0)
    {
      if (vitesse < -2047)
        vitesse = -2047;
      digitalWrite(phase, 0);
      ledcWrite(pwmChannel1, -vitesse);
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
  ledcSetup(pwmChannel1, frequence, resolution);
  ledcAttachPin(moteur, pwmChannel1);

  // Init servomoteur
  ledcSetup(pwmChannel2, frequence2, resolution2);
  ledcAttachPin(servo, pwmChannel2);

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

  mfrc522.PCD_Init();

  // Init asservissement
  Serial.printf("Initialisations\n");

  // Création de la tâche périodique
  xTaskCreate(vTaskPeriodic, "vTaskPeriodic", 10000, NULL, 2, NULL);
}

void loop()
{
  if (!mfrc522.PICC_IsNewCardPresent() ||
      !mfrc522.PICC_ReadCardSerial())
  {
    delay(200);
    return;
  }

  for (byte i = 0; i < mfrc522.uid.size; i++)
  {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
  }
  Serial.println("");

  // position
  tour = encoder.getCount() / 410;
  bozpos = encoder.getCount() - 410 * tour;
  // Utilisation boutton + pot
  val1 = digitalRead(BP1);
  val2 = digitalRead(BP2);
  val3 = digitalRead(BP3);
  // consigne = analogRead(pot)/150;
  // newpot = map (consigne, 0, 4095, 3000, 7000);

  lcd.setCursor(0, 0);
  lcd.printf("pot %3d           ", consigne);

  // capteur couleur
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);

  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

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

  // servomoteur
  // if(val1 == LOW)
  //   {
  // ledcWrite(pwmChannel2, 5800);
  //   }
  // if(val3 == LOW)
  //   {
  // ledcWrite(pwmChannel2, 4769);
  //   }
  //   if(val2== LOW)
  //   {
  // ledcWrite(pwmChannel2, 3826);
  //   }

  //
  switch (etat)
  {
  case 0:
    ledcWrite(pwmChannel2, 4769);
    consigne = 1;
    if (lux >= 200)
      etat = 2;
    break;

  case 2:
    delay(1500);
    consigne = 0;
    delay(500);
    ledcWrite(pwmChannel2, 3826);
    delay(500);
    etat = 0;
    break;
  }
}
