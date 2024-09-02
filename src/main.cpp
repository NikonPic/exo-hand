#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <cstring> // For std::memcpy

// include BLE libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// include personal libraries
#include "read_mux.h"

// choose modus
// int modus= 1; // normal modus
int modus = 2; // BLE modus

// variables declaration

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
// pins servos
#define servo2 1
#define servo3 2
#define servo4 3
#define servo5 4

// sensors
int sensor_value = 0;
float sensor_array[20];
float sensor_array_cal[20];
int16_t sensor_array_raw[21];
int firstRun = 1;
int forceValue[4];
int count[4] = {0};

// servo control
Adafruit_PWMServoDriver myServos = Adafruit_PWMServoDriver(0x40);
int dir_array[4] = {0};

// force and servo definitions
const int F_upper = 4;
const int F_lower = 1.9;

// timer
const int duration = 30000;
int start_time = 0;

// BLE
int turnOn = 0;
int cur_run = 0;

float meanForceI = 0;
float meanForceM = 0;
float meanForceR = 0;
float meanForceS = 0;
float meanForce = 0;

int packagesSent = 0;

// servo control function
// servos controlled by multiplexer, at 50 Hz and with the pwm value 375 (1500 / 4) the servo should not move (+-11,25)
// control: upper and lower force limit defines direction of the servos (dir = 0: pull)
// velocity: Higher delta of force limit and actual force = higher velocity, slowing down when measured force is getting closer to force limit

/*
void stopAllServos()
{
    myServos.setPWM(servo2, 0, stopServo);
    myServos.setPWM(servo3, 0, stopServo);
    myServos.setPWM(servo4, 0, stopServo);
    myServos.setPWM(servo5, 0, stopServo);
}
*/

void stopAllServos()
{
    // Setting the PWM to 0 will stop sending PWM signals, effectively deactivating the servos
    myServos.setPWM(servo2, 0, 0);
    myServos.setPWM(servo3, 0, 0);
    myServos.setPWM(servo4, 0, 0);
    myServos.setPWM(servo5, 0, 0);
}

/*
void control(int pin, int dir, int force)
{
    float highServo = 390;
    float lowServo = 360;
    float stopServo = 375;

    float minServo = 15;
    float maxServo = 30;


    float diffLow = force - F_lower;
    float diffHigh = F_upper - force;
    float genForceDiff = F_upper - F_lower;
    

    switch (dir)
    {
    case 0:
        if (force < F_upper)
        {
            myServos.setPWM(pin, 0, highServo);
        }
        else
        {
            dir_array[pin - 1] = 1;
            myServos.setPWM(pin, 0, lowServo);
        }
        break;
    case 1:
        if (force > F_lower)
        {
            myServos.setPWM(pin, 0, lowServo);
        }
        else
        {
            dir_array[pin - 1] = 0;
            myServos.setPWM(pin, 0, highServo);
        }
        break;
    }
}
*/


void control(int pin, int dir, int force)
{
    float stopServo = 375; // Neutral position for the servo
    float minServo = 10;   // Minimum PWM value to add/subtract from stopServo
    float maxServo = 20;   // Maximum PWM value to add/subtract from stopServo

    float pwmValue; // PWM value to be calculated based on force

    switch (dir)
    {
    case 0: // Pull direction
        maxServo += 10;
        if (force < F_upper)
        {
            // Scale the PWM value based on the difference from F_upper
            float diffHigh = F_upper - force;
            pwmValue = stopServo + map(diffHigh, 0, F_upper - F_lower, minServo, maxServo);
            pwmValue = constrain(pwmValue, stopServo, stopServo + maxServo); // Ensure PWM is within bounds
            myServos.setPWM(pin, 0, pwmValue);
        }
        else
        {
            dir_array[pin - 1] = 1; // Switch direction to push
            myServos.setPWM(pin, 0, stopServo - minServo);
        }
        break;

    case 1: // Push direction
        if (force > F_lower)
        {
            // Scale the PWM value based on the difference from F_lower
            float diffLow = force - F_lower;
            pwmValue = stopServo - map(diffLow, 0, F_upper - F_lower, minServo, maxServo);
            pwmValue = constrain(pwmValue, stopServo - maxServo, stopServo); // Ensure PWM is within bounds
            myServos.setPWM(pin, 0, pwmValue);
        }
        else
        {
            dir_array[pin - 1] = 0; // Switch direction to pull
            myServos.setPWM(pin, 0, stopServo + minServo);
        }
        break;
    }
}




void control_simple(int pin, int dir, float force, float meanForce)
{
    switch (dir)
    {
    case 0:
        if (force < meanForce * 1.1)
        {
            myServos.setPWM(pin, 0, 390);
        }
        else
        {
            dir_array[pin - 1] = 1;
            myServos.setPWM(pin, 0, 360);
        }
        break;
    case 1:
        if (force > meanForce * 0.9)
        {
            myServos.setPWM(pin, 0, 360);
        }
        else
        {
            dir_array[pin - 1] = 0;
            myServos.setPWM(pin, 0, 390);
        }
        break;
    }
}

void control_ringfinger(int pin, int dir, int force)
{
    switch (dir)
    {
    case 0:
        if (force < 3500)
        {
            //                     myServos.setPWM(pin, 0, 387 + (F_upper-force)/100);
            myServos.setPWM(pin, 0, 390);
        }
        else
        {
            dir_array[pin - 1] = 1;
            myServos.setPWM(pin, 0, 360);
        }
        break;
    case 1:
        if (force > 700)
        {
            //                     myServos.setPWM(pin, 0, 363 - (force-F_lower)/100);
            myServos.setPWM(pin, 0, 360);
        }
        else
        {
            dir_array[pin - 1] = 0;
            myServos.setPWM(pin, 0, 390);
        }
        break;
    }
}

// initialize BLE
BLECharacteristic *pControlCharacteristic;
BLECharacteristic *pSensorCharacteristic;
bool deviceConnected = false;
bool controlState = false;

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        Serial.print("connected\n");
    }

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.print("disconencted\n");
        pServer->getAdvertising()->start(); // Restart advertising after disconnection
        Serial.println("Advertising restarted");
    }
};

class ControlCharacteristicCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string value = pCharacteristic->getValue();
        Serial.println(value.c_str());

        if (!value.empty())
        {
            if (value == "Start")
            {
                turnOn = 1;
                Serial.println("Starting!");
                start_time = millis();
            }
            if (value == "Game")
            {
                turnOn = 2;
                stopAllServos();
                Serial.println("Starting!");
                start_time = millis();
            }
            if (value == "Stop")
            {
                turnOn = 0;
                Serial.println("Stopping!");
            }
        }
    }
};

void int16sToBytes(int16_t *ints, uint8_t *bytes, int numInts)
{
    for (int i = 0; i < numInts; i++)
    {
        // Convert each int16_t to bytes and store them in the byte array
        std::memcpy(&bytes[i * sizeof(int16_t)], &ints[i], sizeof(int16_t));
    }
}

void setup()
{

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

    Serial.begin(57600);

    // initialize servos
    Wire.begin();
    myServos.begin();
    myServos.setPWMFreq(50);
    delay(10);

    // initialize BLE

    BLEDevice::init("exo-hand");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pControlService = pServer->createService(BLEUUID((uint16_t)0x180F));
    BLEService *pSensorService = pServer->createService(BLEUUID((uint16_t)0x180A));

    pControlCharacteristic = pControlService->createCharacteristic(
        BLEUUID((uint16_t)0x2A19),
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    pControlCharacteristic->addDescriptor(new BLE2902());
    pControlCharacteristic->setCallbacks(new ControlCharacteristicCallbacks());
    pControlService->start();

    pSensorCharacteristic = pSensorService->createCharacteristic(
        BLEUUID((uint16_t)0x2A29),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    BLE2902 *pDescriptor = new BLE2902();
    pDescriptor->setNotifications(true);
    pSensorCharacteristic->addDescriptor(pDescriptor);
    pSensorCharacteristic->setValue("test");
    pSensorService->start();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();

    // set temporary variable, used to filter spikes and ignore the first run
    int firstRun = 1;

    // variable for timer
    int start_time = millis();

    Serial.print("setup complete\n");
}

int16_t getElapsedTimeInCentiSeconds()
{
    unsigned long current_time = millis();
    unsigned long elapsed_time = current_time - start_time;
    int16_t elapsed_time_in_centi_seconds = elapsed_time / 10;
    return elapsed_time_in_centi_seconds;
}



void loop()
{

    // Connection to App via BLE
    if (deviceConnected)

    {
        if (turnOn > 0) // if signal "1 or 2" is transmitted via BLE
        {
            // read rotatory sensors
            // read_mux(0) to readMux(3): index finger (4-7: middle finger, ...)
            // read_mux(0): MCP
            // read_mux(1): PIP
            // read_mux(2): DIP
            // read_mux(3): MCP vertikal rotation axis
            for (int j = 0; j < 16; j++)
            {
                // if ((abs(read_mux(j)- sensor_array[j]) < 40) || firstRun){
                int res = read_mux(j);
                sensor_array[j] = res;
                sensor_array_raw[j] = res;
                //}
            }
            // filter for spikes
            firstRun = 0;

            // read force sensors, resistance voltage divider: 10k Ohm
            forceValue[0] = analogRead(force2); // index finger
            forceValue[1] = analogRead(force3); // middle finger
            forceValue[2] = analogRead(force4); // ring finger
            forceValue[3] = analogRead(force5); // little finger

            // determine whether the force value is constant to change servo direction
            for (int n = 0; n < 4; n++)
            {
                if (abs(forceValue[n] - sensor_array[n + 16]) < 30)
                {
                    count[n]++;
                }
                else
                {
                    count[n] = 0;
                }
                if (count[n] > 20)
                {
                    // dir_array[n] = dir_array[n] ^ 1; // change the direction of the correspondig servo if the force did not change significantly during 15 following time steps
                    dir_array[n] = 0; // change direction to pull
                    count[n] = 0;
                }
            }

            // write force sensor values into sensor array
            sensor_array[16] = forceValue[0]; // index finger
            sensor_array[17] = forceValue[1]; // middle finger
            sensor_array[18] = forceValue[2]; // ring finger
            sensor_array[19] = forceValue[3]; // little finger

            sensor_array_raw[16] = forceValue[0]; // index finger
            sensor_array_raw[17] = forceValue[1]; // middle finger
            sensor_array_raw[18] = forceValue[2]; // ring finger
            sensor_array_raw[19] = forceValue[3]; // little finger

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

            /*
            sensor_array_cal[16] = pow(sensor_array[16], 2) * 0.004938 - sensor_array[16] * 6.967 + 2648;
            sensor_array_cal[17] = pow(sensor_array[17], 2) * 0.003234 - sensor_array[17] * 2.163 - 56.16;
            sensor_array_cal[18] = pow(sensor_array[18], 2) * 0.0006528 + sensor_array[18] * 8.069 - 2487;
            sensor_array_cal[19] = pow(sensor_array[19], 2) * 0.004539 - sensor_array[19] * 2.147 - 531.1;
            */

            // calibration
            sensor_array_cal[16] = 0.005 * sensor_array[16] - 3;
            sensor_array_cal[17] = 0.005 * sensor_array[17] - 1.3;
            sensor_array_cal[18] = 0.010 * sensor_array[18] - 2.55;
            sensor_array_cal[19] = 0.007 * sensor_array[19] - 2;

            double tfac = 0.9;
            meanForceI = tfac * meanForceI + (1-tfac) * sensor_array_cal[16];
            meanForceM = tfac * meanForceM + (1-tfac) * sensor_array_cal[17];
            meanForceR = tfac * meanForceR + (1-tfac) * sensor_array_cal[18];
            meanForceS = tfac * meanForceS + (1-tfac) * sensor_array_cal[19];

            meanForce = tfac * meanForce + (1-tfac) * (meanForceI + meanForceM + meanForceR + meanForceS) / 4.0;

            sensor_array_raw[20] = getElapsedTimeInCentiSeconds();

            cur_run++;
            if (deviceConnected && turnOn && cur_run > 0)
            {
                cur_run = 0;
                // Prepare a byte array large enough to hold all int values in bytes
                uint8_t sensorDataBytes[sizeof(sensor_array) / sizeof(sensor_array[0]) * sizeof(int)];

                // Convert int array to byte array
                int16sToBytes(sensor_array_raw, sensorDataBytes, sizeof(sensor_array_raw) / sizeof(sensor_array_raw[0]));

                // Send the byte array over BLE
                pSensorCharacteristic->setValue(sensorDataBytes, sizeof(sensorDataBytes));
                pSensorCharacteristic->notify(); // Notify connected client

                packagesSent++;

                // Print the size of the data being sent
                if (false)
                {
                    Serial.print("Sending BLE data of length: ");
                    Serial.println(sizeof(sensorDataBytes));
                    Serial.println("sensor_array_raw contents:");
                    for (size_t i = 0; i < sizeof(sensor_array_raw) / sizeof(sensor_array_raw[0]); ++i)
                    {
                        Serial.print(sensor_array_raw[i]);
                        Serial.print(", ");
                    }
                    Serial.println("");
                }
            }

            // Sensor value
            String arrayData = "";
            for (int z = 0; z < 20; z++)
            {
                arrayData += String(sensor_array_cal[z]) + ",";
            }

            // update characteristic value
            pSensorCharacteristic->setValue(arrayData.c_str());
            pSensorCharacteristic->notify();

            // only loop in here if the value is 1-> "Measure + Motor" in the app
            if (turnOn == 1)
            {
                control(servo2, dir_array[0], meanForceI);
                control(servo3, dir_array[1], meanForceM);
                control(servo4, dir_array[2], meanForceR);
                control(servo5, dir_array[3], meanForceS);
            }
            else {
                stopAllServos();
            }
            
            Serial.print(dir_array[0]);
            Serial.print(", ");
            Serial.print(dir_array[1]);
            Serial.print(", ");
            Serial.print(dir_array[2]);
            Serial.print(", ");
            Serial.println(dir_array[3]);
            float hz = (float)packagesSent / ((float)sensor_array_raw[20] / 100);
            Serial.println(dir_array[3]);
        }
        else
        {
            // stop all servos
            stopAllServos();
        }
        delay(10);
    }
}
