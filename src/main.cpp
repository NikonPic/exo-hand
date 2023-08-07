#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// include BLE libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// include personal libraries
#include "read_mux.h"


// choose modus
    int modus= 1; // normal modus
    // int modus= 2; // BLE modus


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
        int firstRun = 1;
        int forceValue[4];
        int count[4] = {0};

    // servo control
        Adafruit_PWMServoDriver myServos = Adafruit_PWMServoDriver(0x40);
        int dir_array[4] = {0};
        const int F_upper = 3000;
        const int F_lower = 700;
        #define stopServo 375

    // timer
        const int duration = 10000;
        int start_time = 0;
        
    // BLE
        int turnOn = 0;

// servo control function
    // servos controlled by multiplexer, at 50 Hz and with the pwm value 375 (1500 / 4) the servo should not move (+-11,25)
    // control: upper and lower force limit defines direction of the servos (dir = 0: pull)
    // velocity: Higher delta of force limit and actual force = higher velocity, slowing down when measured force is getting closer to force limit
    
    void control (int pin, int dir, int force)
    {
        switch (dir){
            case 0:
                if (force < F_upper){
                    // myServos.setPWM(pin, 0, 387 + (F_upper-force)/100);
                    myServos.setPWM(pin, 0, 390);
                }
                else{
                    dir_array[pin-1] = 1;
                    myServos.setPWM(pin, 0, 360);
                }
            break;
            case 1:
                if (force> F_lower){
                    // myServos.setPWM(pin, 0, 363 - (force-F_lower)/100);
                    myServos.setPWM(pin, 0, 360);
                }
                else{
                    dir_array[pin-1]=0;
                    myServos.setPWM(pin, 0, 390);
                }
            break;
            }
    }


 void control_ringfinger (int pin, int dir, int force)
    {
        switch (dir){
            case 0:
                if (force < 3500){
//                     myServos.setPWM(pin, 0, 387 + (F_upper-force)/100);
                    myServos.setPWM(pin, 0, 390);
                }
                else{
                    dir_array[pin-1] = 1;
                    myServos.setPWM(pin, 0, 360);
                }
            break;
            case 1:
                if (force> 700){
//                     myServos.setPWM(pin, 0, 363 - (force-F_lower)/100);
                    myServos.setPWM(pin, 0, 360);
                }
                else{
                    dir_array[pin-1]=0;
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

    class MyServerCallbacks : public BLEServerCallbacks {
      void onConnect(BLEServer* pServer) {
        deviceConnected = true;
      }

      void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
      }
    };

    class ControlCharacteristicCallbacks : public BLECharacteristicCallbacks {
      void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();

        if (value.length() > 0) {
          if (value[0] == '1') {
            turnOn=1;
            
          } else {
            turnOn=0;
        
          }
        }
      }
    };




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
                                    BLECharacteristic::PROPERTY_WRITE
                                );

        pControlCharacteristic->addDescriptor(new BLE2902());
        pControlCharacteristic->setCallbacks(new ControlCharacteristicCallbacks());

        pSensorCharacteristic = pSensorService->createCharacteristic(
                                    BLEUUID((uint16_t)0x2A29),
                                    BLECharacteristic::PROPERTY_READ
                                );
        pSensorCharacteristic->setValue("0");

        pControlService->start();
        pSensorService->start();

        BLEAdvertising *pAdvertising = pServer->getAdvertising();
        pAdvertising->start();


    // set temporary variable, used to filter spikes and ignore the first run
        int firstRun = 1;

    // variable for timer
        int start_time = millis();
}

void loop() {

    if (modus == 1){

        // timer (control servos for duration, than stop servos)
        if (millis() - start_time <= duration){

        // read rotatory sensors
            // read_mux(0) to readMux(3): index finger (4-7: middle finger, ...)
            // read_mux(0): MCP
            // read_mux(1): PIP
            // read_mux(2): DIP
            // read_mux(3): MCP vertikal rotation axis
            for (int j=0;j<16;j++){
                //if ((abs(read_mux(j)- sensor_array[j]) < 40) || firstRun){
                    sensor_array[j] = read_mux(j);
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
            for (int n=0;n<4;n++){
                if (abs(forceValue[n]-sensor_array[n+16])<15){
                    count[n]++;
                }
                else{
                    count[n] = 0;
                }
                if (count[n]>30){
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
            // first calibration
            // sensor_array_cal[16] = pow(sensor_array[16],2)*0.0068 - sensor_array[16]*15.04 + 8256;
            // sensor_array_cal[17] = pow(sensor_array[17],2)*0.003352 - sensor_array[17]*3.638 + 971,3;
            // sensor_array_cal[18] = -pow(sensor_array[18],2)*0.001564 + sensor_array[18]*6.722 - 3181;
            // sensor_array_cal[19] = pow(sensor_array[19],2)*0.003045 + sensor_array[19]*2.009 - 3750 ;
            // second calibration
            sensor_array_cal[16] = pow(sensor_array[16],2)*0.004938 - sensor_array[16]*6.967 + 2648;
            sensor_array_cal[17] = pow(sensor_array[17],2)* 0.003234 - sensor_array[17]*2.163 - 56.16;
            sensor_array_cal[18] = pow(sensor_array[18],2)*0.0006528 + sensor_array[18]*8.069 - 2487 ;
            sensor_array_cal[19] = pow(sensor_array[19],2)*0.004539 - sensor_array[19]*2.147 - 531.1 ;

        // print calibrated sensor values
            for (int k=0;k<20;k++){
                Serial.print(sensor_array_cal[k]);
                Serial.print(",");
            }
            for (int k=0;k<4;k++){
                Serial.print(dir_array[k]);
                Serial.print(",");
            }
            Serial.print("\n");

            // Serial.println(sensor_array[18]);
            // Serial.println(sensor_array_cal[8]);
            // Serial.print("\n");
            // Serial.println(sensor_array[19]);
            // Serial.println(sensor_array_cal[9]);
            // Serial.print("\n\n");
            //delay(100);



        // control servos: 
            // servo2 = pin for servo that controls index finger, ...
            // dir_array[0] = index finger, dir_array[1] = middle finger, ..., defines direction for each servo
            // sensor_array_cal[16] = measured force for index finger (used to calculate servo speed and to decide for change of direction)
            
            control(servo2, dir_array[0], sensor_array_cal[16]);
            control(servo3, dir_array[1], sensor_array_cal[17]);
            control(servo4, dir_array[2], sensor_array_cal[18]);
            control_ringfinger(servo5, dir_array[3], sensor_array_cal[19]);
        }
    else{

        // stop all servos
            myServos.setPWM(servo2, 0, stopServo);
            myServos.setPWM(servo3, 0, stopServo);
            myServos.setPWM(servo4, 0, stopServo);
            myServos.setPWM(servo5, 0, stopServo);
            while(1);
        }
    }








    // BLE modus
    else if (modus == 2){

        // Connection to App via BLE
        if (deviceConnected) 
            {
            if(turnOn) // if signal "1" is transmitted via BLE
                {
                // read rotatory sensors
                    // read_mux(0) to readMux(3): index finger (4-7: middle finger, ...)
                    // read_mux(0): MCP
                    // read_mux(1): PIP
                    // read_mux(2): DIP
                    // read_mux(3): MCP vertikal rotation axis
                    for (int j=0;j<16;j++){
                        //if ((abs(read_mux(j)- sensor_array[j]) < 40) || firstRun){
                            sensor_array[j] = read_mux(j);
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
                    for (int n=0;n<4;n++){
                        if (abs(forceValue[n]-sensor_array[n+16])<15){
                            count[n]++;
                        }
                        else{
                            count[n] = 0;
                        }
                        if (count[n]>30){
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
                    // first calibration
                    // sensor_array_cal[16] = pow(sensor_array[16],2)*0.0068 - sensor_array[16]*15.04 + 8256;
                    // sensor_array_cal[17] = pow(sensor_array[17],2)*0.003352 - sensor_array[17]*3.638 + 971,3;
                    // sensor_array_cal[18] = -pow(sensor_array[18],2)*0.001564 + sensor_array[18]*6.722 - 3181;
                    // sensor_array_cal[19] = pow(sensor_array[19],2)*0.003045 + sensor_array[19]*2.009 - 3750 ;
                    // second calibration
                    sensor_array_cal[16] = pow(sensor_array[16],2)*0.004938 - sensor_array[16]*6.967 + 2648;
                    sensor_array_cal[17] = pow(sensor_array[17],2)* 0.003234 - sensor_array[17]*2.163 - 56.16;
                    sensor_array_cal[18] = pow(sensor_array[18],2)*0.0006528 + sensor_array[18]*8.069 - 2487 ;
                    sensor_array_cal[19] = pow(sensor_array[19],2)*0.004539 - sensor_array[19]*2.147 - 531.1 ;

                // print calibrated sensor values
                    for (int k=0;k<20;k++){
                        Serial.print(sensor_array_cal[k]);
                        Serial.print(",");
                    }
                    for (int k=0;k<4;k++){
                        Serial.print(dir_array[k]);
                        Serial.print(",");
                    }
                    Serial.print("\n");


         
                // Sensor value

                    String arrayData = "";
                    for (int z = 0; z < 20; z++) {
                        arrayData += String(sensor_array_cal[z]) + ",";
                    }

                    update characteristic value
                    pSensorCharacteristic->setValue(arrayData.c_str());
                    pSensorCharacteristic->notify();

                // control servos: 
                    // servo2 = pin for servo that controls index finger, ...
                    // dir_array[0] = index finger, dir_array[1] = middle finger, ..., defines direction for each servo
                    // sensor_array_cal[16] = measured force for index finger (used to calculate servo speed and to decide for change of direction)
                            
                    control(servo2, dir_array[0], sensor_array_cal[16]);
                    control(servo3, dir_array[1], sensor_array_cal[17]);
                    control(servo4, dir_array[2], sensor_array_cal[18]);
                    control(servo5, dir_array[3], sensor_array_cal[19]);
                }
            else
                {

                // stop all servos
                    myServos.setPWM(servo2, 0, stopServo);
                    myServos.setPWM(servo3, 0, stopServo);
                    myServos.setPWM(servo4, 0, stopServo);
                    myServos.setPWM(servo5, 0, stopServo);
                }

            delay(25);
        }
    }

}


