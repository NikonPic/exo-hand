#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Servo.h"

// Variablen
  // Input-Pin Multiplexer 
  #define muxSIG 36
  // control-Pins Multiplexer
  #define muxS0 12
  #define muxS1 14
  #define muxS2 27
  #define muxS3 26
  // inout pins Kraftsensoren
  #define force1 25
  #define force2 33
  #define force3 32
  #define force4 35

  // Test einlesedauer bestimmen
  int potentiometerValue = 0;
  unsigned long startTime = 0;
  unsigned long endTime = 0;
  unsigned long elapsedTime = 0;

  // Servos Ansteuerung Multiplexer
  Adafruit_PWMServoDriver myServos = Adafruit_PWMServoDriver(0x40);
  // Servo Ansteuerung direkt
  Servo servo;


// Skalierung des gemessenen Signals auf die Spannung
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// logische Schaltung des Multiplexers
int readMux(int channel){
  int controlPin[] = {muxS0, muxS1, muxS2, muxS3};

  int muxChannel[16][4]={
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
  };

  // Schleife über 4 Signale
  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  // Wert am Signal-Pin lesen
  int val = analogRead(muxSIG);

  // Wert in Winkel [°] umrechnen (Annahme: Winkelbereich Poti 360°)
  float valGrad = map(val, 0, 4095, 0, 360);
  
  // Wert zurück geben
  return valGrad;
}

void setup() {

// // Pinbelegung 
//   // Input-Pin Multiplexer 
//   pinMode(muxSIG, INPUT);
//   // control-Pins Multiplexer 
//   pinMode(muxS0, OUTPUT);
//   pinMode(muxS1, OUTPUT);
//   pinMode(muxS2, OUTPUT);
//   pinMode(muxS3, OUTPUT);
//   // input pins Kraftsensoren
//   pinMode(force1, INPUT);
//   pinMode(force2, INPUT);
//   pinMode(force3, INPUT);
//   pinMode(force4, INPUT);

   Serial.begin(9600);
  // servo.attach(13);
  myServos.begin();
  myServos.setPWMFreq(60);
  Serial.println("Servo test setup!");
  delay(10);

}

void loop() {
  
// // Startzeit speichern
// startTime = micros();

// // Sensorwerte auslesen und auf Seriellen Monitor schreiben
// for(int i=0;i<50;i=i+1){

//   // Potentiometerwert auslesen
//   for (int j=0;j<16;j=j+1){
//   potentiometerValue = readMux(j);
//   Serial.println(potentiometerValue);
//   }

// // Potentiometerwerte auslesen

//   // // Serial.print("Poti Zeigefinger\n");
//   // Serial.println(readMux(0)); // Grundgelenk
//   // Serial.println(readMux(1)); // Mittelgelenk
//   // Serial.println(readMux(2)); // Endgelenk
//   // Serial.println(readMux(3)); // Andere Drehachse

//   // // // Serial.print("Poti Mittelfinger\n");
//   // Serial.println(readMux(4)); // Grundgelenk
//   // Serial.println(readMux(5)); // Mittelgelenk
//   // Serial.println(readMux(6)); // Endgelenk
//   // Serial.println(readMux(7)); // Andere Drehachse

//   // // // Serial.print("Poti Ringfinger\n");
//   // Serial.println(readMux(8)); // Grundgelenk
//   // Serial.println(readMux(9)); // Mittelgelenk
//   // Serial.println(readMux(10)); // Endgelenk
//   // Serial.println(readMux(11)); // Andere Drehachse

//   // // // Serial.print("Poti kleiner Finger\n");
//   // Serial.println(readMux(12)); // Grundgelenk
//   // Serial.println(readMux(13)); // Mittelgelenk
//   // Serial.println(readMux(14)); // Endgelenk
//   // Serial.println(readMux(15)); // Andere Drehachse
//   // Serial.print("\n\n");


// // Kraftwerte auslesen, aktueller Widerstand: 10k Ohm

//   // // Serial.print("Kraftsensor Zeigefinger \n");
//   // Serial.print(analogRead(force1));

//   // // Serial.print("Kraftsensor Mittelfinger \n");
//   // Serial.print(analogRead(force2));

//   // // Serial.print("Kraftsensor Ringfinger \n");
//   // Serial.print(analogRead(force3));

//   // // Serial.print("Kraftsensor kleiner Finger \n");
//   // Serial.print(analogRead(force4));


// //delay(2000); 
// }

// // Endzeit speichern
//   endTime = micros();

//   // Verstrichene Zeit berechnen
//   elapsedTime = endTime - startTime;

//   // Ergebnis ausgeben
//   //Serial.print(potentiometerValue);
//   Serial.print("Zeit: ");
//   Serial.print(elapsedTime);
//   Serial.println(" us \n");

// while(1);

    // Ansteuerung Servos über Multiplexer
    myServos.setPWM(0, 0, 150);
    delay(2000);
    myServos.setPWM(0, 0, 300);
    delay(2000);
  

  // direkte Ansteuerung Servo
  // servo.write(85);
  // delay(3000);
  // servo.write(90);
  // delay(3000);
  // servo.write(95);
  // delay(3000);
  // servo.write(90);

  Serial.println("Servo test loop!");
  

  while(1);
  
}