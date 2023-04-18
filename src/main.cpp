#include <Arduino.h>

// Variablen
  // Input-Pin Multiplexer 
  const int muxSIG = 36;
  // control-Pins Multiplexer
  const int muxS0 = 26;
  const int muxS1 = 25;
  const int muxS2 = 33;
  const int muxS3 = 32;

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

  // Wert in Winkel [°] umrechnen (Annahme: Winkelbereich Poti 310°)
  float valGrad = map(val, 0, 4095, 0, 360);
  
  // Wert zurück geben
  return valGrad;
}

void setup() {

// Pinbelegung 
  // Input-Pin Multiplexer 
  pinMode(muxSIG, INPUT);
  // control-Pins Multiplexer 
  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  
// Winkelwerte auslesen und auf Seriellen Monitor schreiben
for(int i=0;i<160;i=i+1){
//Serial.print("Potentiometer Zeigefinger\n");
Serial.println(readMux(0)); // Grundgelenk
Serial.println(readMux(1)); // Mittelgelenk
Serial.println(readMux(2)); // Endgelenk
// Serial.println(readMux(4)); // Andere Drehachse

// Kraftwert Zeigefinger auslesen, aktueller Widerstand: 10kOhm
// Serial.print("Kraftsensor Zeigefinger \n");
// Serial.println(analogRead(39));
// Serial.print("\n");

//Serial.print("Potentiometer Ringfinger\n");
//Serial.println(readMux(6)); // Grundgelenk
//Serial.println(readMux(7)); // Mittelgelenk
//Serial.println(readMux(3)); // Endgelenk
//Serial.println(readMux()); // Andere Drehachse
//Serial.print("\n\n");

delay(100);
}
while(1);
}