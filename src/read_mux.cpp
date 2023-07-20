// read_mux.cpp

#include <Arduino.h>
#include "read_mux.h"

// logical gate multiplexer (to read rotatory sensors)
    int read_mux(int channel)
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
        // float val_deg = (val/4095) *360;
        return val_deg;                            // return value
    }