#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>


// #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
// #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// BLEServer* pServer = NULL;
// BLECharacteristic* pCharacteristic = NULL;
// class MyCallbacks : public BLECharacteristicCallbacks {
//     void onWrite(BLECharacteristic *pCharacteristic) {
//         std::string value = pCharacteristic->getValue(); // process incoming data from client if needed
//     }
// };


// variable declaration

// input-pin multiplexer
#define muxSIG 36
// control-pins multiplexer
#define muxS0 12
#define muxS1 14
#define muxS2 27
#define muxS3 26
// input pins force sensors
#define force2 35
#define force3 32
#define force4 33
#define force5 25
// sensors
int sensor_value = 0;
float sensor_array[20];
float sensor_array_cal[20];
// servo control
int dir_array[4] = {0, 0, 0, 0};
int F_upper = 2500;
int F_lower = -500;
// timer
int start_time = 0;
int duration = 10000;


// servo control multiplexer
Adafruit_PWMServoDriver myServos = Adafruit_PWMServoDriver(0x40);


// logical gate multiplexer (to read rotatory sensors)
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

    // loop for 4 signals
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }

    int val = analogRead(muxSIG);              // read signal-pin
    float val_deg = map(val, 0, 4095, 0, 360); // convert value in degree [°] 
    return val_deg;                            // return value
}


// servo control
    // servos controlled by multiplexer, at 50 Hz and with the pwm value 375 (1500 / 4) the servo should not move (+-11,25)
    // control: upper and lower force limit defines direction of the servos (dir = 0: pull)
    // velocity: Higher delta of force limit and actual force = higher velocity, slowing down when measured force is getting closer to force limit
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

// pins
    // input-Pin multiplexer
    pinMode(muxSIG, INPUT);
    // control-Pins multiplexer
    pinMode(muxS0, OUTPUT);
    pinMode(muxS1, OUTPUT);
    pinMode(muxS2, OUTPUT);
    pinMode(muxS3, OUTPUT);
    // input pins force sensors
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
    // pCharacteristic->setValue(String(sensor_value).c_str());
    // pService->start();
    // // start advertising
    // pServer->getAdvertising()->addServiceUUID(pService->getUUID());
    // pServer->getAdvertising()->start();


start_time=millis();
}

void loop() {

if (millis() - start_time <= duration){

    // read rotatory sensors
        
        // readMux(0) - readMux(3) = pointing finger
        // readMux(0): MCP
        // readMux(1): PIP
        // readMux(2): DIP
        // readMux(3): MCP vertikal rotation axis, ...
    
        for (int j=0;j<16;j++){
        sensor_array[j] = readMux(j);
        }


    // read force sensors, resistance voltage divider: 10k Ohm

        sensor_array[16] = analogRead(force2); // force sensor pointing finger
        sensor_array[17] = analogRead(force3); // force sensor middle finger
        sensor_array[18] = analogRead(force4); // force sensor ring finger
        sensor_array[19] = analogRead(force5); // force sensor little finger


    // array with calibrated sensor values

        sensor_array_cal[0] = sensor_array[0] - 73;
        sensor_array_cal[1] = sensor_array[1] - 70;
        sensor_array_cal[2] = sensor_array[2] - 70;
        sensor_array_cal[3] = sensor_array[3] - 175;
        sensor_array_cal[4] = sensor_array[4] - 77;
        sensor_array_cal[5] = sensor_array[5] - 70;
        sensor_array_cal[6] = sensor_array[6] - 70;
        sensor_array_cal[7] = sensor_array[7] - 158;
        sensor_array_cal[8] = sensor_array[8] - 72;
        sensor_array_cal[9] = sensor_array[9] - 65;
        sensor_array_cal[10] = sensor_array[10] - 70;
        sensor_array_cal[11] = sensor_array[11] - 122;
        sensor_array_cal[12] = sensor_array[12] - 94;
        sensor_array_cal[13] = sensor_array[13] - 64;
        sensor_array_cal[14] = sensor_array[14] - 70;
        sensor_array_cal[15] = sensor_array[15] - 187;

    
    // print sensor values

        for (int k=0;k<20;k++){
            Serial.print(sensor_array_cal[k]);
            Serial.print(",");
        }
        Serial.print("\n");
    
    // control servos: dir_array[0] = pointing finger, dir_array[1] = middle finger, ...

        control(0, dir_array[0], sensor_array_cal[16]);
        // control(1, dir_array[1], sensor_array[17]);
        // control(2, dir_array[2], sensor_array[18]);
        // control(4, dir_array[3], sensor_array[19]);
        

    

    // // bluetooth transmission server

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