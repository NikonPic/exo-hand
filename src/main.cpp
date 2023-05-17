#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;


class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue(); // process incoming data from client if needed
    }
};

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
int forceValue[4] = {0, 0, 0, 0};
int sensorValue = 0;
unsigned long startTime = 0;
int count = 0;
float array[26];

// Servos Ansteuerung Multiplexer
Adafruit_PWMServoDriver myServos = Adafruit_PWMServoDriver(0x40);

Adafruit_MPU6050 mpu;

// Skalierung des gemessenen Signals auf die Spannung (für Potentiometer)
float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
    

// initialize servos
    Wire.begin();
    // myServos.begin();
    // myServos.setPWMFreq(50);
    // delay(10);


  // initialize IMU
    if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
        yield();
        }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);


// initialize BLE
    BLEDevice::init("ESP32 Sensor Export");
    pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);

    pCharacteristic->setCallbacks(new MyCallbacks());

    // set initial sensor value
    pCharacteristic->setValue(String(sensorValue).c_str());

    pService->start();

    // start advertising
    pServer->getAdvertising()->addServiceUUID(pService->getUUID());
    pServer->getAdvertising()->start();
}

void loop() {
//   startTime=millis();
//   // Für 10 Sekunden Sensorwerte auslesen (und auf Seriellen Monitor schreiben)
//   while (millis()-startTime < 10000){

  // //   Potentiometerwerte auslesen
  // // readMux(0) - readMux(3) = Zeigefinger
  // // readMux(0): MCP
  // // readMux(1): PIP
  // // readMux(2): DIP
  // // readMux(3): Andere Drehachse, ...
   
    for (int j=0;j<16;j++){
    potentiometerValue = readMux(j);
    // Serial.print(potentiometerValue);
    array[j]=potentiometerValue;
    }


  // // Kraftwerte auslesen, aktueller Widerstand: 10k Ohm
    forceValue[0] = analogRead(force1); // Kraftsensor Zeigefinger
    forceValue[1] = analogRead(force2); // Kraftsensor Mittelfinger
    forceValue[2] = analogRead(force3); // Kraftsensor Ringfinger
    forceValue[3] = analogRead(force4); // Kraftsensor kleiner Finger
    for (int k = 0; k<4; k++){
    array[16 + k]=forceValue[k];
    //   Serial.print(forceValue[k]);
    }


  //   Ansteuerung Servos über Multiplexer, bei 50 Hz sollte der Servo bei 375 (1500 / 4) still stehen
    // myServos.setPWM(0, 0, 390);
    // myServos.setPWM(4, 0, 390);
    // myServos.setPWM(8, 0, 390);
    // delay(1000);

    // myServos.setPWM(0, 0, 375);
    // myServos.setPWM(4, 0, 375);
    // myServos.setPWM(8, 0, 375);
    // delay(2000);

    // myServos.setPWM(0, 0, 360);
    // myServos.setPWM(4, 0, 360);
    // myServos.setPWM(8, 0, 360);
    // delay(1000);

    // myServos.setPWM(0, 0, 375);
    // myServos.setPWM(4, 0, 375);
    // myServos.setPWM(8, 0, 375);
    // delay(2000);


// read IMU
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // save sensor values to array
    array[20]= a.acceleration.x;
    array[21]= a.acceleration.y;
    array[22]= a.acceleration.z;
    array[23]= g.gyro.x;
    array[24]= g.gyro.y;
    array[25]= g.gyro.z;
    /* Print out the values */
    //   Serial.print("Acceleration X: ");
    //   Serial.print(a.acceleration.x);
    //   Serial.print(", Y: ");
    //   Serial.print(a.acceleration.y);
    //   Serial.print(", Z: ");
    //   Serial.print(a.acceleration.z);
    //   Serial.println(" m/s^2");
    //   Serial.print("Rotation X: ");
    //   Serial.print(g.gyro.x);
    //   Serial.print(", Y: ");
    //   Serial.print(g.gyro.y);
    //   Serial.print(", Z: ");
    //   Serial.print(g.gyro.z);
    //   Serial.println(" rad/s");
    //   Serial.print("Temperature: ");
    //   Serial.print(temp.temperature);
    //   Serial.println(" degC");


    // print array values on serial monitor
    // for (int u = 0; u<26; u++){
    //         Serial.print("Element ");
    //         Serial.print(u);
    //         Serial.print(": ");
    //         Serial.println(array[u]);
    //     }
    
    
    // Bluetooth Server
    String arrayData = "";
    for (int z = 0; z < 26; z++) {
      arrayData += String(array[z]) + ",";
    }

    // update characteristic value
    pCharacteristic->setValue(arrayData.c_str());

    // notify connected clients
    pCharacteristic->notify();

    delay(3000);

//     count++;
//   }
//   Serial.print("Durchläufe: \n");
//   Serial.println(count);

//   while(1);
  
}