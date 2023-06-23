#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


// #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
// #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// BLEServer* pServer = NULL;
// BLECharacteristic* pCharacteristic = NULL;
// class MyCallbacks : public BLECharacteristicCallbacks {
//     void onWrite(BLECharacteristic *pCharacteristic) {
//         std::string value = pCharacteristic->getValue(); // process incoming data from client if needed
//     }
// };

// Variablen
// Input-Pin Multiplexer
#define muxSIG 36
// control-Pins Multiplexer
#define muxS0 12
#define muxS1 14
#define muxS2 27
#define muxS3 26
// inout pins Kraftsensoren
#define force2 35
#define force3 32
#define force4 33
#define force5 25

// Variablendeklaration
int potentiometerValue = 0;
int forceValue[4] = {0, 0, 0, 0};
int sensorValue = 0;
unsigned long startTime = 0;
int count = 0;
float sensor_array[20];
// Kraftregelung
int dir_array[4] = {0, 0, 0, 0};
int F_upper = 2500;
int F_lower = -500;
int angle_2 = 0;
int angle_3 = 0;
int angle_4 = 0;
// Zeit
int Startzeit = 0;
int Dauer = 10000;


// Servos Ansteuerung Multiplexer
Adafruit_PWMServoDriver myServos = Adafruit_PWMServoDriver(0x40);


// logische Schaltung des Multiplexers (Potentiometer)
int readMux(int channel)
{
    int controlPin[] = {muxS0, muxS1, muxS2, muxS3};

    int muxChannel[16][4] = {
        {0, 0, 0, 0}, // channel 0
        {1, 0, 0, 0}, // channel 1
        {0, 1, 0, 0}, // channel 2
        {1, 1, 0, 0}, // channel 3
        {0, 0, 1, 0}, // channel 4
        {1, 0, 1, 0}, // channel 5
        {0, 1, 1, 0}, // channel 6
        {1, 1, 1, 0}, // channel 7
        {0, 0, 0, 1}, // channel 8
        {1, 0, 0, 1}, // channel 9
        {0, 1, 0, 1}, // channel 10
        {1, 1, 0, 1}, // channel 11
        {0, 0, 1, 1}, // channel 12
        {1, 0, 1, 1}, // channel 13
        {0, 1, 1, 1}, // channel 14
        {1, 1, 1, 1}  // channel 15
    };

    // Schleife über 4 Signale
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }

    int val = analogRead(muxSIG);              // Wert am Signal-Pin lesen
    float valGrad = map(val, 0, 4095, 0, 360); // Wert in Winkel [°] umrechnen (Annahme: Winkelbereich Poti 360°)
    return valGrad;                            // Wert zurück geben
}


// Motor Regelung
    // Ansteuerung Servos über Multiplexer, bei 50 Hz sollte der Servo bei 375 (1500 / 4) still stehen (+-11,25)
    // Regelung der Servos: swich case für jeden Finger, jeweils obere und untere Kraftgrenze definiert Richtung des Servos (dir)
    // Geschwindigkeit: Je größer die Differenz zum Grenzwert, desto höher die Geschwindkeit, bei Annäherung Verringerung
void control (int pin, int dir, int force)
{
    switch (dir){
        case 0:
            if (force < 1800){
            myServos.setPWM(pin, 0, 395 - (force-1000)/100);
            delay(20);
            }
            else{
                dir_array[pin] = 1;
            }
        break;
        case 1:
            if (force>1500){
            myServos.setPWM(pin, 0, 365 - (force-1000)/100);
            delay(20);
            }
            else{
                dir_array[pin]=0;
            }
        break;
        }
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
    pinMode(force2, INPUT);
    pinMode(force3, INPUT);
    pinMode(force4, INPUT);
    pinMode(force5, INPUT);

    Serial.begin(9600);
    

// initialize servos
    Wire.begin();
    myServos.begin();
    myServos.setPWMFreq(50);
    delay(10);


// initialize BLE
    // BLEDevice::init("ESP32 Sensor Export");
    // pServer = BLEDevice::createServer();
    // BLEService *pService = pServer->createService(SERVICE_UUID);
    // pCharacteristic = pService->createCharacteristic(
    //     CHARACTERISTIC_UUID,
    //     BLECharacteristic::PROPERTY_READ |
    //         BLECharacteristic::PROPERTY_NOTIFY);
    // pCharacteristic->setCallbacks(new MyCallbacks());
    // // set initial sensor value
    // pCharacteristic->setValue(String(sensorValue).c_str());
    // pService->start();
    // // start advertising
    // pServer->getAdvertising()->addServiceUUID(pService->getUUID());
    // pServer->getAdvertising()->start();


Startzeit=millis();
}

void loop() {

if (millis() - Startzeit <= Dauer){

    // Potentiometerwerte einlesen
        
        // readMux(0) - readMux(3) = Zeigefinger
        // readMux(0): MCP
        // readMux(1): PIP
        // readMux(2): DIP
        // readMux(3): Andere Drehachse, ...
    
        for (int j=0;j<16;j++){
        potentiometerValue = readMux(j);
        sensor_array[j]=potentiometerValue;
        Serial.print(sensor_array[j]);
        Serial.print(",");
        }

    
        // Offset der Winkel abziehen
        angle_2 = sensor_array[0] - 73;
        angle_3 = sensor_array[4] - 73;
        angle_4 = sensor_array[8] - 73;


    // Kraftwerte einlesen, Widerstand Spannungsteiler: 10k Ohm

        sensor_array[16] = analogRead(force2); // Kraftsensor Zeigefinger
        sensor_array[17] = analogRead(force3); // Kraftsensor Mittelfinger
        sensor_array[18] = analogRead(force4); // Kraftsensor Ringfinger
        sensor_array[19] = analogRead(force5); // Kraftsensor kleiner Finger
    
        Serial.print(sensor_array[16]);
        Serial.print(",");
        Serial.print(sensor_array[17]);
        Serial.print(",");
        Serial.print(sensor_array[18]);
        Serial.print(",");
        Serial.print(sensor_array[19]);
        Serial.print("\n");
    
    // Ansteuerung Servos: dir_array[0] = Zeigefinger, dir_array[1] = Mittelfinger, ...

        control(0, dir_array[0], sensor_array[16]);
        // control(1, dir_array[1], sensor_array[17]);
        // control(2, dir_array[2], sensor_array[18]);
        // control(4, dir_array[3], sensor_array[19]);
        

    

    // // Bluetooth Übertragung Server

        // String arrayData = "";
        // for (int z = 0; z < 20; z++) {
        //   arrayData += String(sensor_array[z]) + ",";
        // }

        // // update characteristic value
        // pCharacteristic->setValue(arrayData.c_str());

        // // notify connected clients
        // pCharacteristic->notify();
}
else{
    while(1);
    }
  
}