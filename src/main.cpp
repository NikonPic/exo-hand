#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

// BLE Server (von chatgpt)
#include <BLEDevice.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

int sensorValue = 0; // replace this with your actual sensor value

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();

        // process incoming data from client if needed
    }
};


// BluetoothSerial SerialBT;


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

  // Variablendeklaration
  int potentiometerValue = 0;
  int forceValue[4]= {0,0,0,0};
  unsigned long startTime = 0;
  int count = 0;

  // Servos Ansteuerung Multiplexer
  Adafruit_PWMServoDriver myServos = Adafruit_PWMServoDriver(0x40);


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

// Pinbelegung 
  // Input-Pin Multiplexer 
  pinMode(muxSIG, INPUT);
  // control-Pins Multiplexer 
  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);
  // input pins Kraftsensoren
  pinMode(force1, INPUT);
  pinMode(force2, INPUT);
  pinMode(force3, INPUT);
  pinMode(force4, INPUT);

   Serial.begin(9600);

    // SerialBT.begin("ESP32test"); //Name des ESP32
    // Serial.println("The device started, now you can pair it with bluetooth!");

    Wire.begin();
    myServos.begin();
    myServos.setPWMFreq(50);
    delay(10);

    // initialize BLE
    BLEDevice::init("ESP32 Sensor Export");
    pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                    );

    pCharacteristic->setCallbacks(new MyCallbacks());

    // set initial sensor value
    pCharacteristic->setValue(String(sensorValue).c_str());

    pService->start();

    // start advertising
    pServer->getAdvertising()->addServiceUUID(pService->getUUID());
    pServer->getAdvertising()->start();
    
}

void loop() {
  // startTime=millis();
 
  // // Für 10 Sekunden Sensorwerte auslesen (und auf Seriellen Monitor schreiben)
  
  // while (millis()-startTime < 10000){
  // //   Potentiometerwerte auslesen
  // // readMux(0) - readMux(3) = Zeigefinger
  // // readMux(0): Grundgelenk
  // // readMux(1): Mittelgelenk
  // // readMux(2): Endgelenk
  // // readMux(3): Andere Drehachse, ...
   
  //   for (int j=0;j<16;j++){
  //   potentiometerValue = readMux(j);
  //   // Serial.print(potentiometerValue);
  //   }


  // // Kraftwerte auslesen, aktueller Widerstand: 10k Ohm
  //   forceValue[0] = analogRead(force1); // Kraftsensor Zeigefinger
  //   forceValue[1] = analogRead(force2); // Kraftsensor Mittelfinger
  //   forceValue[2] = analogRead(force3); // Kraftsensor Ringfinger
  //   forceValue[3] = analogRead(force4); // Kraftsensor kleiner Finger
  //   // for (int k = 0; k<4; k++){
  //   //   Serial.print(forceValue[k]);
  //   // }

  //   myServos.setPWM(0, 0, 360 + (count/100));
  //   myServos.setPWM(4, 0, 360 + (count/100));

  //   count++;
  // }
  // Serial.print("Durchläufe: \n");
  // Serial.println(count);

  // myServos.setPWM(0, 0, 375);
  // myServos.setPWM(4, 0, 375);
  // while(1);


  //   Ansteuerung Servos über Multiplexer, bei 50 Hz sollte der Servo bei 375 (1500 / 4) still stehen
  //   myServos.setPWM(0, 0, 360);
  //   myServos.setPWM(4, 0, 360);
  //   delay(2000);

  //   myServos.setPWM(0, 0, 375);
  //   myServos.setPWM(4, 0, 375);
  //   delay(2000);

  //   myServos.setPWM(0, 0, 390);
  //   myServos.setPWM(4, 0, 390);
  //   delay(2000);

  //   myServos.setPWM(0, 0, 375);
  //   myServos.setPWM(4, 0, 375);
  //   delay(2000);
  

  //  while(1);


  // Kommunikation über Bluetooth
  // if (Serial.available()) {
  //   SerialBT.write(Serial.read());
  // }
  // if (SerialBT.available()) {
  //   Serial.write(SerialBT.read());
  // }
  // delay(25);



    // Bluetooth Server
   // update sensor value here
    sensorValue = readMux(0);

    Serial.println(sensorValue);

    // update characteristic value
    pCharacteristic->setValue(String(sensorValue).c_str());

    // notify connected clients
    pCharacteristic->notify();

    delay(1000);
  
}